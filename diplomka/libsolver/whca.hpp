#ifndef WHCA_HPP
#define WHCA_HPP

#include "separate_paths_solver.hpp"

#include "a_star.hpp"

class whca : public separate_paths_solver {
public:
  whca(log_sink& log, unsigned window, unsigned rejoin_limit,
       std::unique_ptr<predictor> predictor,
       unsigned obstacle_penalty,
       double obstacle_threshold);
  void step(world& w, std::default_random_engine& rng) override;

  std::string name() const override { return "WHCA*"; }
  void window(unsigned new_window) override { window_ = new_window; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

  std::unordered_map<position_time, double>
  get_obstacle_field() const override;

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

  std::unique_ptr<predictor> predictor_;
  reservation_table_type agent_reservations_;
  heuristic_map_type heuristic_map_;
  unsigned window_;
  unsigned nodes_primary_ = 0;
  unsigned nodes_heuristic_ = 0;
  unsigned nodes_rejoin_ = 0;
  unsigned rejoin_limit_ = 0;
  unsigned rejoin_attempts_ = 0;
  unsigned rejoin_successes_ = 0;

  path<> find_path(position, world const&, std::default_random_engine&,
                   boost::optional<path<> const&> old_path) override;
  boost::optional<path<>> rejoin_path(position from, world const& w,
                                      path<> const& old_path);
};

#endif
