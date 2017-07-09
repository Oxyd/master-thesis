#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "action.hpp"
#include "world.hpp"

#include <atomic>
#include <random>
#include <string>
#include <unordered_map>
#include <vector>

class predictor;
class log_sink;

// Is the scenario finished? I.e. are all agents at their goals?
bool
solved(world const& w);

using obstacle_field_type = std::unordered_map<position_time, double>;

// Interface for the planning algorithm. This is called each time-step to
// provide a joint action for the agents.
class solver {
public:
  virtual
  ~solver() { }

  // Move the agents in the world.
  virtual void step(world& w, std::default_random_engine&) = 0;

  // Name of this solver.
  virtual std::string name() const = 0;

  // Names for values in the stat_values() vector.
  virtual std::vector<std::string> stat_names() const { return {}; }

  // Statistics about the algorithm performance.
  virtual std::vector<std::string> stat_values() const { return {}; };

  // Get the path for an agent.
  virtual std::vector<position> get_path(agent::id_type) const { return {}; }

  // Get the predicted obstacle field, if any.
  virtual obstacle_field_type get_obstacle_field() const {
    return {};
  }

  // Change the window of the algorithm. If the algorithm doesn't use a window,
  // doesn't do anything.
  virtual void window(unsigned /*new_window*/) {}

  // Interrupt the algorithm execution. This should be called from another
  // thread whilst step is running in order to force it to fail.
  void kill() { should_stop_ = true; }

protected:
  std::atomic<bool> should_stop_{false};
};

std::unique_ptr<solver>
make_greedy();

std::unique_ptr<solver>
make_lra(log_sink&,
         unsigned rejoin_limit,
         std::unique_ptr<predictor> predictor,
         unsigned obstacle_penalty, double obstacle_threshold);

std::unique_ptr<solver>
make_whca(log_sink& log, unsigned window, unsigned rejoin_limit,
          std::unique_ptr<predictor> predictor, unsigned obstacle_penalty,
          double obstacle_threshold);

std::unique_ptr<solver>
make_od(unsigned window,
        std::unique_ptr<predictor> predictor, unsigned obstacle_penalty,
        double obstacle_threshold);

#endif // SOLVERS_HPP
