#ifndef SEPARATE_PATHS_SOLVER_HPP
#define SEPARATE_PATHS_SOLVER_HPP

#include "solvers.hpp"

// Base class for decoupled solvers: LRA* and WHCA*. This calls the derived
// class for finding a plan for a single agent and then puts those plans
// together into joint actions. It also implements the path rejoining heuristic.
template <typename Derived>
class separate_paths_solver : public solver {
public:
  explicit
  separate_paths_solver(log_sink& log,
                        unsigned rejoin_limit,
                        std::unique_ptr<predictor> predictor,
                        unsigned obstacle_penalty,
                        double obstacle_threshold);

  void
  step(world&, std::default_random_engine&) override;

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::vector<position>
  get_path(agent::id_type) const override;

  std::unordered_map<position_time, double>
  get_obstacle_field() const override;

protected:
  log_sink& log_;
  unsigned times_without_path_ = 0;
  unsigned recalculations_ = 0;
  unsigned path_invalid_ = 0;
  unsigned nodes_rejoin_ = 0;
  unsigned rejoin_limit_ = 0;
  unsigned rejoin_attempts_ = 0;
  unsigned rejoin_successes_ = 0;

  std::unique_ptr<predictor> predictor_;
  unsigned obstacle_penalty_ = 100;
  double obstacle_threshold_ = 0.1;

private:
  using paths_map_type = std::unordered_map<agent::id_type, path<>>;

  paths_map_type paths_;

  Derived*
  derived() { return static_cast<Derived*>(this); }

  path<>
  recalculate(position, world const&, agent::id_type,
              std::default_random_engine&,
              boost::optional<path<> const&> old_path = {});

  boost::optional<position>
  next_step(position, world const&, std::default_random_engine&,
            boost::optional<path<> const&> old_path = {});

  virtual void
  on_path_invalid(agent::id_type) { }

  virtual void
  on_path_found(agent::id_type, path<> const&, world const&) { }

  virtual bool
  path_valid(path<> const&, world const&) const { return true; }

  virtual path<>
  find_path(position, world const&, std::default_random_engine&) = 0;

  boost::optional<path<>> rejoin_path(position from, world const& w,
                                      path<> const& old_path);
};

class whca;
class lra;

extern template class separate_paths_solver<whca>;
extern template class separate_paths_solver<lra>;

#endif
