#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <memory>
#include <unordered_map>

#include "world.hpp"

class predictor {
public:
  virtual void update_obstacles(world const&) = 0;
  virtual double predict_obstacle(position_time) = 0;
  virtual std::unordered_map<position_time, double> field() const = 0;
};

std::unique_ptr<predictor>
make_recursive_predictor(world const&, unsigned cutoff);

std::unique_ptr<predictor>
make_matrix_predictor(world const&, unsigned cutoff, tick_t update_frequency);

struct predicted_cost {
  predicted_cost(predictor* p, tick_t start_tick, unsigned obstacle_penalty)
    : predictor_(p)
    , start_tick_(start_tick)
    , obstacle_penalty_(obstacle_penalty)
  { }

  double
  operator () (position_time from, position_time to, unsigned) const;

  double
  operator () (position from, position to, unsigned distance) const;

private:
  predictor* predictor_;
  tick_t start_tick_;
  unsigned obstacle_penalty_;
};

#endif
