#ifndef SEPARATE_PATHS_SOLVER_HPP
#define SEPARATE_PATHS_SOLVER_HPP

#include "solvers.hpp"

class separate_paths_solver : public solver {
public:
  explicit
  separate_paths_solver(log_sink& log,
                        std::unique_ptr<predictor> predictor,
                        unsigned obstacle_penalty,
                        double obstacle_threshold);

  void
  step(world&, std::default_random_engine&) override;

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

  std::unique_ptr<predictor> predictor_;
  unsigned obstacle_penalty_ = 100;
  double obstacle_threshold_ = 0.1;

private:
  std::unordered_map<agent::id_type, path<>> paths_;

  path<>
  recalculate(position, world const&, std::default_random_engine&,
              boost::optional<path<> const&> old_path = {});

  boost::optional<position>
  next_step(position, world const&, std::default_random_engine&,
            boost::optional<path<> const&> old_path = {});

  virtual path<>
  find_path(position, world const&, std::default_random_engine&,
            boost::optional<path<> const&> old_path) = 0;
};

#endif
