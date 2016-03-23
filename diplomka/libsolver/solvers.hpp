#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "a_star.hpp"
#include "action.hpp"
#include "predictor.hpp"
#include "world.hpp"

#include <boost/functional/hash.hpp>
#include <boost/optional.hpp>

#include <array>
#include <functional>
#include <map>
#include <random>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

class log_sink;

bool
solved(world const& w);

class solver {
public:
  virtual
  ~solver() { }

  virtual joint_action get_action(world w, std::default_random_engine&) = 0;
  virtual std::string name() const = 0;
  virtual std::vector<std::string> stat_names() const { return {}; }
  virtual std::vector<std::string> stat_values() const { return {}; };
  virtual std::vector<position> get_path(agent::id_type) const { return {}; }
  virtual std::unordered_map<position_time, double> get_obstacle_field() const {
    return {};
  }
  virtual void window(unsigned /*new_window*/) {}
};

std::unique_ptr<solver>
make_greedy();

std::unique_ptr<solver>
make_lra(log_sink&);

std::unique_ptr<solver>
make_whca(log_sink& log, unsigned window, unsigned rejoin_limit,
          bool avoid_obstacles, unsigned obstacle_penalty,
          double obstacle_threshold);

#endif // SOLVERS_HPP
