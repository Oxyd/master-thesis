#include "predictor.hpp"

#include <boost/optional.hpp>

#include <stack>

constexpr double stay_probability = 4.0 / 5.0;

namespace {

class recursive_predictor : public predictor {
public:
  explicit recursive_predictor(map const* m)
    : map_(m)
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
};

}

std::unique_ptr<predictor>
make_recursive_predictor(map const& m) {
  return std::make_unique<recursive_predictor>(&m);
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
