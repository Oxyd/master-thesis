#ifndef LRA_HPP
#define LRA_HPP

#include "separate_paths_solver.hpp"

class lra : public separate_paths_solver {
public:
  explicit lra(log_sink& log,
               std::unique_ptr<predictor> predictor,
               unsigned obstacle_penalty,
               double obstacle_threshold)
    : separate_paths_solver(log)
    , predictor_(std::move(predictor))
    , obstacle_penalty_(obstacle_penalty)
    , obstacle_threshold_(obstacle_threshold)
  { }

  void step(world& w, std::default_random_engine& rng) override;

  std::string name() const override { return "LRA*"; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::unordered_map<position_time, double>
  get_obstacle_field() const override;

private:
  struct agent_data {
    tick_t last_recalculation = 0;
    double agitation = 0;
  };

  struct passable_not_immediate_neighbour {
    position from;
    bool operator () (position p, position, world const& w, unsigned) const {
      return w.get(p) == tile::free || !neighbours(p, from);
    }
  };

  struct passable_if_not_predicted_obstacle {
    passable_if_not_predicted_obstacle(position from,
                                       predictor* predictor,
                                       double threshold)
      : not_neighbour_{from}
      , predictor_(predictor)
      , threshold_(threshold)
    { }

    bool operator () (position where, position from, world const& w,
                      unsigned distance) const;

  private:
    passable_not_immediate_neighbour not_neighbour_;
    predictor* predictor_;
    double threshold_;
  };

  struct agitated_distance {
    position destination;
    double agitation;
    std::default_random_engine& rng;

    double operator () (position from, world const&) const;
  };

  std::unordered_map<agent::id_type, agent_data> data_;
  unsigned nodes_ = 0;
  std::unique_ptr<predictor> predictor_;
  unsigned obstacle_penalty_;
  double obstacle_threshold_;

  path<> find_path(position, world const&, std::default_random_engine&,
                   boost::optional<path<> const&>) override;
};

#endif
