#ifndef WHCA_HPP
#define WHCA_HPP

#include "separate_paths_solver.hpp"

#include "a_star.hpp"

class predicted_cost;

class whca : public separate_paths_solver<whca> {
  class passable_if_not_predicted_obstacle;

public:
  using rejoin_search_type = a_star<
    position,
    position_successors,
    passable_if_not_predicted_obstacle,
    manhattan_distance_heuristic,
    predicted_cost,
    space_time_coordinate
  >;

  whca(log_sink& log, unsigned window, unsigned rejoin_limit,
       std::unique_ptr<predictor> predictor,
       unsigned obstacle_penalty,
       double obstacle_threshold);

  std::string name() const override { return "WHCA*"; }
  void window(unsigned new_window) override { window_ = new_window; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::unique_ptr<rejoin_search_type>
  make_rejoin_search(position from, position to, world const& w,
                     agent const& agent) const;

private:
  struct reservation_table_record {
    ::agent::id_type agent;
    boost::optional<position> from;
  };

  using reservation_table_type =
    std::unordered_map<position_time, reservation_table_record>;

  using heuristic_search_type = a_star<>;
  using heuristic_map_type = std::map<agent::id_type, heuristic_search_type>;

  class passable_if_not_reserved {
  public:
    passable_if_not_reserved(reservation_table_type const& reservations,
                             agent const& agent,
                             position from);
    bool operator () (position where, position from, world const& w,
                      unsigned distance);

  private:
    reservation_table_type const& reservations_;
    agent const& agent_;
    position from_;
  };

  void reserve(agent::id_type for_agent, path<> const&, tick_t from);
  void unreserve(agent::id_type);

  struct hierarchical_distance {
    hierarchical_distance(heuristic_search_type& h_search)
      : h_search_(h_search)
    { }

    double operator () (position from, world const& w);

  private:
    heuristic_search_type& h_search_;
  };

  struct passable_if_not_predicted_obstacle {
    passable_if_not_predicted_obstacle(predictor* p,
                                       passable_if_not_reserved pnr,
                                       double threshold)
      : not_reserved_(pnr)
      , predictor_(p)
      , threshold_(threshold)
    { }

    bool operator () (position where, position from, world const& w,
                      unsigned distance);

  private:
    passable_if_not_reserved not_reserved_;
    predictor*predictor_;
    double threshold_;
  };

  reservation_table_type agent_reservations_;
  heuristic_map_type heuristic_map_;
  unsigned window_;
  unsigned nodes_primary_ = 0;
  unsigned nodes_heuristic_ = 0;

  path<> find_path(position, world const&,
                   std::default_random_engine&) override;

  void
  on_path_invalid(agent::id_type) override;

  void
  on_path_found(agent::id_type, path<> const&, world const&) override;

  bool
  path_valid(path<> const&, world const&) const override;
};

#endif
