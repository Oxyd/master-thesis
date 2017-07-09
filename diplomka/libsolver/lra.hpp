#ifndef LRA_HPP
#define LRA_HPP

#include "a_star.hpp"
#include "separate_paths_solver.hpp"

class predicted_cost;

class lra : public separate_paths_solver<lra> {
  struct passable_if_not_predicted_obstacle;

public:
  using rejoin_search_type = a_star<
    position,
    position_successors,
    passable_if_not_predicted_obstacle,
    manhattan_distance_heuristic,
    predicted_cost,
    space_time_coordinate
  >;

  lra(log_sink& log,
      unsigned rejoin_limit,
      std::unique_ptr<predictor> predictor,
      unsigned obstacle_penalty,
      double obstacle_threshold);

  std::string name() const override { return "LRA*"; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::unordered_map<position_time, double>
  get_obstacle_field() const override;

  std::unique_ptr<rejoin_search_type>
  make_rejoin_search(position from, position to, world const& w,
                     agent const& agent);

private:
  struct agent_data {
    boost::optional<tick_t> last_recalculation;
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

  path<> find_path(position, world const&,
                   std::default_random_engine&) override;
};

#endif
