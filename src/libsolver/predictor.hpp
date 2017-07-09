#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <memory>
#include <unordered_map>

#include "world.hpp"

// Type for representing obstacle movements. Unlike direction, this also allows
// the stay move.
enum class movement : std::size_t {
  north = 0, east, south, west, stay
};

constexpr unsigned num_moves = 5;

// Estimates obstacle movement based on observed past movements using
// maximum-likelihood estimation.
class movement_estimator {
public:
  using estimates_type = std::array<double, num_moves>;

  explicit
  movement_estimator(world const&);

  // Observe the movement of obstacles and update the estimate.
  void update(world const&);

  // Estimate the probability of an obstacle making the given move.
  double estimate(movement) const;

  // Current estimates of all possible moves.
  estimates_type estimates() const { return estimate_; }

private:
  using obstacle_positions = std::unordered_map<obstacle::id_type, position>;

  void clear();
  void store_positions(world const&);

  obstacle_positions last_obstacles_;
  std::array<unsigned, num_moves> move_count_;
  std::array<double, num_moves> estimate_{0.0, 0.0, 0.0, 0.0, 1.0};
  unsigned num_moves_ = 0;

#ifndef NDEBUG
  tick_t last_tick_ = 0;
#endif
};

// Predicts obstacle movement.
class predictor {
public:
  virtual void update_obstacles(world const&) = 0;
  virtual double predict_obstacle(position_time) = 0;
  virtual std::unordered_map<position_time, double> field() const = 0;
};

// Make a predictor that uses the recursive algorithm for prediction.
std::unique_ptr<predictor>
make_recursive_predictor(world const&, unsigned cutoff);

// Make a predictor that predicts by multiplying a transition matrix.
std::unique_ptr<predictor>
make_matrix_predictor(world const&, unsigned cutoff);

// Step-cost for a_star that adds the obstacle probability to the cost.
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
