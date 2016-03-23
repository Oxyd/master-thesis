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
make_recursive_predictor(map const&);

std::unique_ptr<predictor>
make_markov_predictor(map const&);

#endif
