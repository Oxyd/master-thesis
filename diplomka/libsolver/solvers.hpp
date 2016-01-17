#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "a_star.hpp"
#include "action.hpp"
#include "world.hpp"

#include <boost/functional/hash.hpp>

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
};

using solver_description = std::tuple<
  std::string,
  std::function<std::unique_ptr<solver>(log_sink&, world const&)>
>;

extern
std::array<solver_description, 3>
solvers;

class greedy : public solver {
public:
  joint_action get_action(world w, std::default_random_engine&) override;
  std::string name() const override { return "Greedy"; }
};

class separate_paths_solver : public solver {
public:
  explicit
  separate_paths_solver(log_sink& log);

  joint_action
  get_action(world, std::default_random_engine&) override;

  std::vector<std::string>
  stat_names() const override {
    return {"Path not found", "Recalculations", "Path invalid",
            "Nodes expanded"};
  }

  std::vector<std::string>
  stat_values() const override;

protected:
  log_sink& log_;
  unsigned times_without_path_ = 0;
  unsigned recalculations_ = 0;
  unsigned path_invalid_ = 0;
  unsigned nodes_ = 0;

private:
  std::unordered_map<position, path> paths_;

  path
  recalculate(position, world const&, std::default_random_engine&);

  virtual path
  find_path(position, world const&, std::default_random_engine&) = 0;
};

class lra : public separate_paths_solver {
public:
  explicit lra(log_sink& log) : separate_paths_solver(log) { }
  std::string name() const override { return "LRA*"; }

private:
  struct agent_data {
    tick_t last_recalculation = 0;
    double agitation = 0;
  };

  std::unordered_map<agent::id_type, agent_data> data_;

  path find_path(position, world const&, std::default_random_engine&) override;
};

struct position_time {
  position::coord_type x, y;
  tick_t time;

  position_time(position::coord_type x, position::coord_type y,
                tick_t time)
    : x(x), y(y), time(time) { }

  position_time(position p, tick_t time)
    : x(p.x), y(p.y), time(time) { }
};

inline bool
operator == (position_time lhs, position_time rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y && lhs.time == rhs.time;
}

inline bool
operator != (position_time lhs, position_time rhs) {
  return !operator == (lhs, rhs);
}

namespace std {
template <>
struct hash<position_time> {
  using argument_type = position_time;
  using result_type = std::size_t;

  result_type
  operator () (argument_type pt) const {
    std::size_t seed{};
    boost::hash_combine(seed, pt.x);
    boost::hash_combine(seed, pt.y);
    boost::hash_combine(seed, pt.time);

    return seed;
  }
};
}  // namespace std

class cooperative_a_star : public separate_paths_solver {
public:
  cooperative_a_star(log_sink& log, world const& w);
  std::string name() const override { return "HCA*"; }

private:
  struct permanent_reservation {
    agent::id_type agent_id;
    tick_t start;
  };

  using reservation_table_type =
    std::unordered_map<position_time, agent::id_type>;
  using permanent_reservation_table_type =
    std::unordered_map<position, permanent_reservation>;
  using heuristic_search_type = a_star<>;
  using heuristic_map_type = std::map<agent::id_type, heuristic_search_type>;

  reservation_table_type reservations_;
  permanent_reservation_table_type permanent_reservations_;
  heuristic_map_type heuristic_map_;

  path find_path(position, world const&, std::default_random_engine&) override;
  void unreserve(agent const&);
};

#endif // SOLVERS_HPP
