#include "predictor.hpp"

#include <Eigen/Dense>
#include <Eigen/SparseCore>

#include <boost/optional.hpp>

#include <memory>
#include <stack>

namespace {

enum class movement : std::size_t {
  north = 0, east, south, west, stay
};

constexpr unsigned num_moves = 5;

class movement_estimator {
public:
  explicit
  movement_estimator(world const&);

  void update(world const&);
  double estimate(movement) const;

private:
  using obstacle_positions = std::unordered_map<obstacle::id_type, position>;

  void clear();
  void store_positions(world const&);

  obstacle_positions last_obstacles_;
  std::array<unsigned, num_moves> move_count_;
  unsigned num_moves_ = 0;

#ifndef NDEBUG
  tick_t last_tick_ = 0;
#endif
};

} // anonymous namespace

static movement
direction_to_movement(direction d) {
  return static_cast<movement>(static_cast<unsigned>(d));
}

movement_estimator::movement_estimator(world const& w) {
  clear();
  store_positions(w);
}

void
movement_estimator::update(world const& w) {
  for (auto const& pos_obstacle : w.obstacles()) {
    obstacle::id_type id = std::get<1>(pos_obstacle).id();
    auto last = last_obstacles_.find(id);

    if (last == last_obstacles_.end())
      continue;

    position current = std::get<0>(pos_obstacle);

    if (current == last->second)
      ++move_count_[static_cast<std::size_t>(movement::stay)];
    else
      ++move_count_[static_cast<std::size_t>(
        direction_to_movement(direction_to(last->second, current))
      )];

    ++num_moves_;
  }

  store_positions(w);

#ifndef NDEBUG
  double sum = 0.0;
  for (movement m = movement::north; m <= movement::stay; ++(size_t&) m)
    sum += estimate(m);
  assert(std::abs(sum - 1.0) < 1e-6);
#endif
}

double
movement_estimator::estimate(movement m) const {
  std::size_t move = static_cast<std::size_t>(m);
  if (num_moves_ > 0)
    return (double) move_count_[move] / (double) num_moves_;
  else
    return m == movement::stay ? 1.0 : 0.0;
}

void
movement_estimator::clear() {
  std::fill(move_count_.begin(), move_count_.end(), 0);
  num_moves_ = 0;
}

void
movement_estimator::store_positions(world const& w) {
  assert(last_tick_ == 0 || w.tick() == last_tick_ + 1);

  last_obstacles_.clear();

  for (auto const& pos_obstacle : w.obstacles())
    last_obstacles_.insert(
      {std::get<1>(pos_obstacle).id(), std::get<0>(pos_obstacle)}
    );

#ifndef NDEBUG
  last_tick_ = w.tick();
#endif
}

namespace {

class recursive_predictor : public predictor {
public:
  recursive_predictor(world const& w, unsigned cutoff)
    : world_(&w)
    , cutoff_(cutoff)
    , estimator_(w)
  {}

  void update_obstacles(world const&) override;
  double predict_obstacle(position_time) override;

  std::unordered_map<position_time, double> field() const override {
    return obstacles_;
  }

private:
  std::unordered_map<position_time, double> obstacles_;
  tick_t last_update_time_ = 0;
  world const* world_ = nullptr;
  unsigned cutoff_ = 0;
  movement_estimator estimator_;
};

}

std::unique_ptr<predictor>
make_recursive_predictor(world const& w, unsigned cutoff) {
  return std::make_unique<recursive_predictor>(w, cutoff);
}

void
recursive_predictor::update_obstacles(world const& w) {
  assert(world_->map() == w.map());

  if (w.tick() == last_update_time_)
    return;

  obstacles_.clear();

  for (auto pos_obstacle : w.obstacles())
    obstacles_[{std::get<0>(pos_obstacle), w.tick()}] = 1.0;

  last_update_time_ = w.tick();

  estimator_.update(w);
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

    if (pt.time == last_update_time_
        || world_->get({pt.x, pt.y}) == tile::wall
        || world_->get({pt.x, pt.y}) == tile::agent) {
      obstacles_[pt] = 0.0;
      stack.pop();

    } else {
      bool have_neighbours = true;
      double complementary_prob = 1.0;

      double stay_probability = estimator_.estimate(movement::stay);

      auto previous = obstacles_.find({pt.x, pt.y, pt.time - 1});
      if (previous == obstacles_.end()) {
        stack.push({pt.x, pt.y, pt.time - 1});
        have_neighbours = false;
      } else
        complementary_prob = 1 - previous->second * stay_probability;

      for (direction d : all_directions) {
        position p = translate({pt.x, pt.y}, d);
        position_time neighbour_pt{p, pt.time - 1};

        if (world_->get(p) == tile::wall || world_->get(p) == tile::agent)
          continue;

        auto neighbour = obstacles_.find(neighbour_pt);
        if (neighbour == obstacles_.end()) {
          stack.push({translate({pt.x, pt.y}, d), pt.time - 1});
          have_neighbours = false;
        }

        if (!have_neighbours)
          continue;

        double probability =
          neighbour->second *
          estimator_.estimate(direction_to_movement(
            direction_to(p, {pt.x, pt.y})
          ));

        complementary_prob *= 1 - probability;
      }

      if (have_neighbours) {
        assert(complementary_prob >= 0.0);
        assert(complementary_prob <= 1.0);

        obstacles_[pt] = 1 - complementary_prob;
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

class matrix_predictor : public predictor {
public:
  matrix_predictor(world const&, unsigned cutoff, tick_t update_frequency);

  void update_obstacles(world const&) override;
  double predict_obstacle(position_time) override;
  std::unordered_map<position_time, double> field() const override;

private:
  movement_estimator estimator_;
  transition_matrix_type transition_;
  std::vector<obstacle_state_vector_type> states_;
  tick_t last_update_time_ = 0;
  map::coord_type width_ = 0;
  unsigned cutoff_ = 0;
  tick_t matrix_update_frequency_;

  position::coord_type
  linear(position p) const { return p.y * width_ + p.x; }
};

}

static transition_matrix_type
make_transition_matrix(map const& m, movement_estimator const& estimator) {
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

      double const stay_probability = estimator.estimate(movement::stay);
      double leftover = 1.0 - stay_probability;

      if (neighbours.empty()) {
        transitions.emplace_back(linear(from), linear(from), 1.0);
        continue;
      }

      for (position to : neighbours) {
        double const transition_probability =
          estimator.estimate(direction_to_movement(
            direction_to(from, to)
          ));

        transitions.emplace_back(linear(to), linear(from),
                                 transition_probability);

        leftover -= transition_probability;
      }

      // It is possible that not all neighbours are traversable, and that we
      // therefore have some leftover probability. We'll account this leftover
      // to the stay probability as well, in order to make sure our transitions
      // sum up to 1.0.
      transitions.emplace_back(linear(from), linear(from),
                               stay_probability + leftover);
    }

  transition_matrix_type result(map_size, map_size);
  result.setFromTriplets(transitions.begin(), transitions.end());

#ifndef NDEBUG
  for (auto i = 0; i < map_size; ++i) {
    double sum = result.col(i).sum();
    assert(std::abs(sum - 1.0) < 1e-6
           || std::abs(sum - 0.0) < 1e-6);
  }
#endif

  return result;
}

std::unique_ptr<predictor>
make_matrix_predictor(world const& w, unsigned cutoff,
                      tick_t update_frequency) {
  return std::make_unique<matrix_predictor>(w, cutoff, update_frequency);
}

matrix_predictor::matrix_predictor(world const& w, unsigned cutoff,
                                   tick_t update_frequency)
  : estimator_(w)
  , transition_(make_transition_matrix(*w.map(), estimator_))
  , width_(w.map()->width())
  , cutoff_(cutoff)
  , matrix_update_frequency_(update_frequency)
{}

void
matrix_predictor::update_obstacles(world const& w) {
  if (last_update_time_ == w.tick())
    return;

  estimator_.update(w);
  states_.clear();

  if (w.tick() % matrix_update_frequency_ == 0)
    transition_ = make_transition_matrix(*w.map(), estimator_);

  obstacle_state_vector_type known_state(w.map()->width() * w.map()->height());
  known_state.fill(0.0);
  for (auto const& pos_obstacle : w.obstacles())
    known_state(linear(pos_obstacle.first)) = 1.0;

  states_.push_back(std::move(known_state));
  last_update_time_ = w.tick();

  assert(std::abs(states_.back().sum() - w.obstacles().size()) < 1e-6);
}

double
matrix_predictor::predict_obstacle(position_time pt) {
  if (cutoff_ && pt.time - last_update_time_ > cutoff_)
    pt.time = last_update_time_ + cutoff_;

  while (pt.time - last_update_time_ >= states_.size())
    states_.push_back(transition_ * states_.back());

  return states_[pt.time - last_update_time_](linear({pt.x, pt.y}));
}

std::unordered_map<position_time, double>
matrix_predictor::field() const {
  std::unordered_map<position_time, double> result;

  for (std::size_t t = 0; t < states_.size(); ++t)
    for (auto i = 0; i < states_[t].size(); ++i)
      result[{i % width_,
              i / width_,
              last_update_time_ + (tick_t) t}] = states_[t](i);

  return result;
}

double
predicted_cost::operator () (position_time pt, unsigned) const {
  if (!predictor_)
    return 1.0;

  return
    1.0
    + predictor_->predict_obstacle({pt.x, pt.y, start_tick_ + pt.time})
    * obstacle_penalty_;
}

double
predicted_cost::operator () (position p, unsigned distance) const {
  if (!predictor_)
    return 1.0;

  return
    1.0
    + predictor_->predict_obstacle({p.x, p.y, start_tick_ + distance})
    * obstacle_penalty_;
}
