#include "predictor.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <boost/optional.hpp>

#include <stack>

#include <iomanip>

constexpr double stay_probability = 4.0 / 5.0;

namespace {

class recursive_predictor : public predictor {
public:
  recursive_predictor(map const* m, unsigned cutoff)
    : map_(m)
    , cutoff_(cutoff)
  {}

  void update_obstacles(world const&) override;
  double predict_obstacle(position_time) override;

  std::unordered_map<position_time, double> field() const override {
    return obstacles_;
  }

private:
  std::unordered_map<position_time, double> obstacles_;
  tick_t last_update_time_ = 0;
  map const* map_ = nullptr;
  unsigned cutoff_ = 0;
};

}

std::unique_ptr<predictor>
make_recursive_predictor(map const& m, unsigned cutoff) {
  return std::make_unique<recursive_predictor>(&m, cutoff);
}

void
recursive_predictor::update_obstacles(world const& w) {
  assert(map_ == w.map().get());

  if (w.tick() == last_update_time_)
    return;

  obstacles_.clear();

  for (auto pos_obstacle : w.obstacles())
    obstacles_[{std::get<0>(pos_obstacle), w.tick()}] = 1.0;

  last_update_time_ = w.tick();
}

double
recursive_predictor::predict_obstacle(position_time where) {
  assert(where.time >= last_update_time_);

  if (cutoff_ && where.time - last_update_time_ > cutoff_)
    where.time = last_update_time_ + cutoff_;

  std::stack<position_time> stack;
  stack.push(where);

  while (!stack.empty()) {
    position_time pt = stack.top();

    if (obstacles_.count(pt)) {
      stack.pop();
      continue;
    }

    if (pt.time == last_update_time_ ||
        map_->get(pt.x, pt.y) == tile::wall) {
      obstacles_[pt] = 0.0;
      stack.pop();

    } else {
      bool have_neighbours = true;
      double result = 0.0;
      double next = 1.0;

      auto previous = obstacles_.find({pt.x, pt.y, pt.time - 1});
      if (previous == obstacles_.end()) {
        stack.push({pt.x, pt.y, pt.time - 1});
        have_neighbours = false;
      } else {
        result = previous->second * stay_probability;
        next = 1 - result;
      }

      for (direction d : all_directions) {
        position p = translate({pt.x, pt.y}, d);
        position_time neighbour_pt{p, pt.time - 1};

        if (map_->get(p) == tile::wall)
          continue;

        auto neighbour = obstacles_.find(neighbour_pt);
        if (neighbour == obstacles_.end()) {
          stack.push({translate({pt.x, pt.y}, d), pt.time - 1});
          have_neighbours = false;
        }

        if (!have_neighbours)
          continue;

        unsigned neighbour_options = 0;
        for (direction e : all_directions) {
          position q = translate(p, e);
          position_time q_pt{q, pt.time - 1};
          if (map_->get(q) != tile::wall)
            ++neighbour_options;
        }

        if (neighbour_options == 0)
          continue;

        double probability =
          neighbour->second *
          (1 - stay_probability) *
          (1 / (double) neighbour_options);

        result += next * probability;
        next *= 1 - probability;
      }

      if (have_neighbours) {
        assert(result <= 1.0);

        obstacles_[pt] = result;
        stack.pop();
      }
    }
  }

  assert(obstacles_.count(where));
  assert(obstacles_[where] >= 0.0);
  assert(obstacles_[where] <= 1.0);

  return obstacles_[where];
}

namespace {

using transition_matrix_type = Eigen::SparseMatrix<double>;
using obstacle_state_vector_type = Eigen::VectorXd;

class markov_predictor : public predictor {
public:
  markov_predictor(map const&, unsigned cutoff);

  void update_obstacles(world const&) override;
  double predict_obstacle(position_time) override;
  std::unordered_map<position_time, double> field() const override;

private:
  transition_matrix_type transition_;
  std::vector<obstacle_state_vector_type> states_;
  tick_t last_update_time_ = 0;
  map::coord_type width_ = 0;
  unsigned cutoff_ = 0;

  position::coord_type
  linear(position p) const { return p.y * width_ + p.x; }
};

}

static transition_matrix_type
make_transition_matrix(map const& m) {
  map::coord_type map_size = m.width() * m.height();
  auto linear = [&] (position p) { return p.y * m.width() + p.x; };

  std::vector<Eigen::Triplet<double>> transitions;
  transitions.reserve(5 * map_size);

  for (position::coord_type from_y = 0; from_y < m.height(); ++from_y)
    for (position::coord_type from_x = 0; from_x < m.width(); ++from_x) {
      position from{from_x, from_y};
      if (m.get(from) == tile::wall)
        continue;

      std::vector<position> neighbours;
      neighbours.reserve(4);
      for (direction d : all_directions) {
        position to = translate(from, d);
        if (!in_bounds(to, m))
          continue;

        neighbours.push_back(to);
      }

      if (neighbours.empty()) {
        transitions.emplace_back(linear(from), linear(from), 1.0);
        continue;
      } else
        transitions.emplace_back(linear(from), linear(from), stay_probability);

      double const transition_probability =
        (1 - stay_probability) * (1.0 / neighbours.size());

      for (position to : neighbours)
        transitions.emplace_back(linear(to), linear(from),
                                 transition_probability);
    }

  transition_matrix_type result(map_size, map_size);
  result.setFromTriplets(transitions.begin(), transitions.end());

#ifndef NDEBUG
  for (auto i = 0; i < map_size; ++i)
    assert(std::abs(result.col(i).sum() - 1.0) < 1e-6 ||
           std::abs(result.col(i).sum() - 0.0) < 1e-6);
#endif

  return result;
}

std::unique_ptr<predictor>
make_markov_predictor(map const& m, unsigned cutoff) {
  return std::make_unique<markov_predictor>(m, cutoff);
}

markov_predictor::markov_predictor(map const& m, unsigned cutoff)
  : transition_(make_transition_matrix(m))
  , width_(m.width())
  , cutoff_(cutoff)
{}

void
markov_predictor::update_obstacles(world const& w) {
  if (last_update_time_ == w.tick())
    return;

  states_.clear();

  obstacle_state_vector_type known_state(w.map()->width() * w.map()->height());
  known_state.fill(0.0);
  for (auto const& pos_obstacle : w.obstacles())
    known_state(linear(pos_obstacle.first)) = 1.0;

  states_.push_back(std::move(known_state));
  last_update_time_ = w.tick();

  assert(std::abs(states_.back().sum() - w.obstacles().size()) < 1e-6);
}

double
markov_predictor::predict_obstacle(position_time pt) {
  if (cutoff_ && pt.time - last_update_time_ > cutoff_)
    pt.time = last_update_time_ + cutoff_;

  while (pt.time - last_update_time_ >= states_.size())
    states_.push_back(transition_ * states_.back());

  return states_[pt.time - last_update_time_](linear({pt.x, pt.y}));
}

std::unordered_map<position_time, double>
markov_predictor::field() const {
  std::unordered_map<position_time, double> result;

  for (std::size_t t = 0; t < states_.size(); ++t)
    for (auto i = 0; i < states_[t].size(); ++i) {
      if (std::abs(states_[t](i)) < 1e-6)
        continue;

      result[{i % width_,
              i / width_,
              last_update_time_ + (tick_t) t}] = states_[t](i);
    }

  return result;
}
