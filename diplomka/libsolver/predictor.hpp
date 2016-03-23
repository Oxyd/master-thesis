#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <boost/optional.hpp>

#include <unordered_map>

#include "world.hpp"

class predictor {
public:
  void update_obstacles(world const&);
  double predict_obstacle(position_time);

  std::unordered_map<position_time, double> field() const { return obstacles_; }

private:
  std::unordered_map<position_time, double> obstacles_;
  tick_t last_update_time_ = 0;
  map const* map_ = nullptr;
};

#endif
