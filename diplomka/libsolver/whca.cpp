#include "whca.hpp"

#include "predictor.hpp"

whca::whca(log_sink& log, unsigned window,
           unsigned rejoin_limit,
           std::unique_ptr<predictor> predictor,
           unsigned obstacle_penalty,
           double obstacle_threshold)
  : separate_paths_solver(log, rejoin_limit, std::move(predictor),
                          obstacle_penalty, obstacle_threshold)
  , window_(window)
{ }

std::vector<std::string>
whca::stat_names() const {
  std::vector<std::string> result = separate_paths_solver::stat_names();
  result.insert(result.end(),
                {"Primary nodes expanded", "Heuristic nodes expanded",
                 "Total nodes expanded"});
  return result;
}

std::vector<std::string>
whca::stat_values() const {
  std::vector<std::string> result = separate_paths_solver::stat_values();
  result.insert(
    result.end(),
    {
      std::to_string(nodes_primary_),
      std::to_string(nodes_heuristic_),
      std::to_string(nodes_primary_ + nodes_heuristic_),
    }
  );
  return result;
}

auto
whca::make_rejoin_search(position from, position to, world const& w,
                         agent const& agent) const
  -> std::unique_ptr<rejoin_search_type> {
  return std::make_unique<rejoin_search_type>(
    from, to, w,
    manhattan_distance_heuristic{to},
    predicted_cost(predictor_.get(), w.tick(), obstacle_penalty_),
    passable_if_not_predicted_obstacle(
      predictor_.get(),
      passable_if_not_reserved(agent_reservations_, agent, from),
      predictor_ ? obstacle_threshold_ : 1.0
    )
  );
}

whca::passable_if_not_reserved::passable_if_not_reserved(
  reservation_table_type const& reservations,
  agent const& agent,
  position from
)
  : reservations_(reservations)
  , agent_(agent)
  , from_(from)
{ }

bool
whca::passable_if_not_reserved::operator () (
  position where, position from, world const& w, unsigned distance
) {
  if (reservations_.count(position_time{where, w.tick() + distance}))
    return false;

  auto vacated = reservations_.find(
    position_time{from, w.tick() + distance}
  );
  if (vacated != reservations_.end() &&
      vacated->second.from &&
      *vacated->second.from == where)
    return false;

  return w.get(where) == tile::free || !neighbours(where, from_);
}

void
whca::reserve(agent::id_type a_id, path<> const& path,
              tick_t from) {
  for (tick_t distance = 0; distance < path.size(); ++distance) {
    position const p = path[path.size() - distance - 1];
    position_time const pt{p, from + distance};

    assert(!agent_reservations_.count(pt));

    if (distance > 0)
      agent_reservations_[pt] = {a_id, path[path.size() - distance]};
    else
      agent_reservations_[pt] = {a_id, boost::none};
  }
}

void
whca::unreserve(agent::id_type a_id) {
  auto it = agent_reservations_.begin();
  while (it != agent_reservations_.end())
    if (it->second.agent == a_id)
      it = agent_reservations_.erase(it);
    else
      ++it;
}

double
whca::hierarchical_distance::operator () (
  position from,
  world const& w
) {
  if (from == h_search_.from())
    return 0.0;

  return h_search_.find_distance(from, w);
}

bool
whca::passable_if_not_predicted_obstacle::operator () (
  position where, position from, world const& w, unsigned distance
) {
  return
    not_reserved_(where, from, w, distance) &&
    (!predictor_
     || where == from
     || predictor_->predict_obstacle({where, w.tick() + distance})
        <= threshold_);
}

path<>
whca::find_path(position from, world const& w, std::default_random_engine&) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  heuristic_map_.erase(a.id());
  heuristic_search_type& h_search = heuristic_map_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(a.id()),
    std::forward_as_tuple(a.target, from, w, manhattan_distance_heuristic{from},
                          predicted_cost{predictor_.get(), w.tick(),
                                         obstacle_penalty_})
  ).first->second;
  unsigned const old_h_search_nodes = h_search.nodes_expanded();

  using search_type = a_star<
    position,
    position_successors,
    passable_if_not_predicted_obstacle,
    hierarchical_distance,
    unitary_step_cost,
    space_time_coordinate
  >;
  search_type as(
    from, a.target, w,
    hierarchical_distance(h_search),
    unitary_step_cost{},
    passable_if_not_predicted_obstacle(
      predictor_.get(),
      passable_if_not_reserved(agent_reservations_, a, from),
      predictor_ ? obstacle_threshold_ : 1.0
    )
  );
  path<> new_path = as.find_path(w, window_);

  nodes_primary_ += as.nodes_expanded();
  nodes_heuristic_ += h_search.nodes_expanded() - old_h_search_nodes;

  return new_path;
}

void
whca::on_path_invalid(agent::id_type agent) {
  unreserve(agent);
}

void
whca::on_path_found(agent::id_type agent, path<> const& path, world const& w) {
  reserve(agent, path, w.tick());
}

bool
whca::path_valid(path<> const& path, world const& w) const {
  for (tick_t distance = 0; distance < path.size(); ++distance) {
    position const p = path[path.size() - distance - 1];
    position_time const pt{p, w.tick() + distance};

    if (agent_reservations_.count(pt))
      return false;
  }

  return true;
}
