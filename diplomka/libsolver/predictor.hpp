#ifndef PREDICTOR_HPP
#define PREDICTOR_HPP

#include <boost/optional.hpp>

#include <unordered_map>

#include "world.hpp"

class predictor {
  struct reservation_table_record {
    ::agent::id_type agent;
    boost::optional<position> from;
  };

  using reservation_table_type =
    std::unordered_map<position_time, reservation_table_record>;

public:
  class impassable_reserved {
  public:
    impassable_reserved(reservation_table_type const& reservations,
                        agent const& agent,
                        position from);
    bool operator () (position where, position from, world const& w,
                      unsigned distance);

  private:
    reservation_table_type const& reservations_;
    agent const& agent_;
    position from_;
  };

  void reserve(agent::id_type for_agent, path const&, tick_t from);
  void unreserve(agent::id_type);
  impassable_reserved impassable_predicate(agent const&, position from) const;

  void update_obstacles(world const&);
  double predict_obstacle(position_time);

  std::unordered_map<position_time, double> field() const { return obstacles_; }

private:
  reservation_table_type agent_reservations_;
  std::unordered_map<position_time, double> obstacles_;
  tick_t last_update_time_ = 0;
  map const* map_ = nullptr;
};

#endif
