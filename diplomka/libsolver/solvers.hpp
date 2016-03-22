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
};

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
    return {"Path not found", "Recalculations", "Path invalid"};
  }

  std::vector<std::string>
  stat_values() const override;

  std::vector<position>
  get_path(agent::id_type) const override;

protected:
  log_sink& log_;
  unsigned times_without_path_ = 0;
  unsigned recalculations_ = 0;
  unsigned path_invalid_ = 0;

private:
  std::unordered_map<agent::id_type, path> paths_;

  path
  recalculate(position, world const&, std::default_random_engine&,
              boost::optional<path const&> old_path = {});

  boost::optional<position>
  next_step(position, world const&, std::default_random_engine&,
            boost::optional<path const&> old_path = {});

  virtual path
  find_path(position, world const&, std::default_random_engine&,
            boost::optional<path const&> old_path) = 0;
};

class lra : public separate_paths_solver {
public:
  explicit lra(log_sink& log) : separate_paths_solver(log) { }
  std::string name() const override { return "LRA*"; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

private:
  struct agent_data {
    tick_t last_recalculation = 0;
    double agitation = 0;
  };

  struct passable_not_immediate_neighbour {
    position from;
    bool operator () (position p, position, world const& w, unsigned) {
      return w.get(p) == tile::free || !neighbours(p, from);
    }
  };

  struct agitated_distance {
    position destination;
    double agitation;
    std::default_random_engine& rng;

    double operator () (position from, world const&, unsigned) const;
  };

  std::unordered_map<agent::id_type, agent_data> data_;
  unsigned nodes_ = 0;

  path find_path(position, world const&, std::default_random_engine&,
                 boost::optional<path const&>) override;
};

class cooperative_a_star : public separate_paths_solver {
public:
  cooperative_a_star(log_sink& log, unsigned window, unsigned rejoin_limit,
                     bool avoid_obstacles, unsigned obstacle_penalty);
  std::string name() const override { return "WHCA*"; }
  void window(unsigned new_window) { window_ = new_window; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::unordered_map<position_time, double>
  get_obstacle_field() const override;

private:
  using heuristic_search_type = a_star<>;
  using heuristic_map_type = std::map<agent::id_type, heuristic_search_type>;

  struct hierarchical_distance {
    hierarchical_distance(heuristic_search_type& h_search, predictor& p,
                          bool penalise_obstacles, unsigned obstacle_penalty)
      : h_search_(h_search)
      , predictor_(p)
      , penalise_obstacles_(penalise_obstacles)
      , obstacle_penalty_(obstacle_penalty)
    { }

    double operator () (position from, world const& w,
                        unsigned distance_so_far);

  private:
    heuristic_search_type& h_search_;
    predictor& predictor_;
    bool penalise_obstacles_ = false;
    unsigned obstacle_penalty_ = 100;
  };

  predictor predictor_;
  heuristic_map_type heuristic_map_;
  unsigned window_;
  unsigned nodes_primary_ = 0;
  unsigned nodes_heuristic_ = 0;
  unsigned nodes_rejoin_ = 0;
  unsigned rejoin_limit_ = 0;
  unsigned rejoin_attempts_ = 0;
  unsigned rejoin_successes_ = 0;
  bool avoid_obstacles_ = false;
  unsigned obstacle_penalty_ = 100;

  path find_path(position, world const&, std::default_random_engine&,
                 boost::optional<path const&> old_path) override;
  boost::optional<path> rejoin_path(position from, world const& w,
                                    path const& old_path);
};

#endif // SOLVERS_HPP
