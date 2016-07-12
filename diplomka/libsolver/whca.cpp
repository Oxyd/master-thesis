#include "whca.hpp"

#include "predictor.hpp"

whca::whca(log_sink& log, unsigned window,
           unsigned rejoin_limit,
           std::unique_ptr<predictor> predictor,
           unsigned obstacle_penalty,
           double obstacle_threshold)
  : separate_paths_solver(log, std::move(predictor), obstacle_penalty,
                          obstacle_threshold)
  , window_(window)
  , rejoin_limit_(rejoin_limit)
{ }

void whca::step(world& w, std::default_random_engine& rng) {
  if (predictor_)
    predictor_->update_obstacles(w);

  separate_paths_solver::step(w, rng);
}

std::vector<std::string>
whca::stat_names() const {
  std::vector<std::string> result = separate_paths_solver::stat_names();
  result.insert(result.end(),
                {"Primary nodes expanded", "Heuristic nodes expanded",
                 "Rejoin nodes expanded", "Total nodes expanded",
                 "Rejoin attempts", "Rejoin successes", "Rejoin success rate"});
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
      std::to_string(nodes_rejoin_),
      std::to_string(nodes_primary_ + nodes_heuristic_ + nodes_rejoin_),
      std::to_string(rejoin_attempts_),
      std::to_string(rejoin_successes_),
      rejoin_attempts_ > 0
        ? std::to_string((double) rejoin_successes_ / (double) rejoin_attempts_)
        : "0"
    }
  );
  return result;
}

std::unordered_map<position_time, double>
whca::get_obstacle_field() const {
  if (predictor_)
    return predictor_->field();
  else
    return {};
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
    (!predictor_ ||
     predictor_->predict_obstacle({where, w.tick() + distance}) <= threshold_);
}

path<>
whca::find_path(position from, world const& w,
                              std::default_random_engine&,
                              boost::optional<path<> const&> old_path) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  unreserve(a.id());

  path<> new_path;
  if (rejoin_limit_ > 0 && old_path)
    if (auto p = rejoin_path(from, w, *old_path))
      return new_path = std::move(*p);

  if (new_path.empty()) {
    heuristic_search_type& h_search = heuristic_map_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(a.id()),
      std::forward_as_tuple(a.target, from, w)
    ).first->second;
    unsigned const old_h_search_nodes = h_search.nodes_expanded();

    using search_type = a_star<
      position,
      position_successors,
      passable_if_not_predicted_obstacle,
      hierarchical_distance,
      predicted_cost,
      space_time_coordinate
    >;
    search_type as(
      from, a.target, w,
      hierarchical_distance(h_search),
      predicted_cost(predictor_.get(), w.tick(), obstacle_penalty_),
      passable_if_not_predicted_obstacle(
        predictor_.get(),
        passable_if_not_reserved(agent_reservations_, a, from),
        predictor_ ? obstacle_threshold_ : 1.0
      )
    );
    new_path = as.find_path(w, window_);

    nodes_primary_ += as.nodes_expanded();
    nodes_heuristic_ += h_search.nodes_expanded() - old_h_search_nodes;
  }

  reserve(a.id(), new_path, w.tick());

  return new_path;
}

std::ostream&
operator << (std::ostream& out, path<> const& p) {
  for (auto point = p.rbegin(); point != p.rend(); ++point) {
    if (point != p.rbegin())
      out << " -> ";
    out << *point;
  }

  return out;
}

boost::optional<path<>>
whca::rejoin_path(position from, world const& w,
                  path<> const& old_path) {
  if (old_path.empty())
    return {};

  boost::optional<position> to = boost::none;
  std::unordered_map<position, path<>::const_reverse_iterator> target_positions;
  for (auto point = old_path.rbegin(); point != old_path.rend(); ++point)
    if (w.get(*point) == tile::free) {
      if (!to)
        to = *point;
      target_positions.insert({*point, point});
    }

  if (!to)
    return {};

  ++rejoin_attempts_;

  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  using search_type = a_star<
    position,
    position_successors,
    passable_if_not_predicted_obstacle,
    manhattan_distance_heuristic,
    predicted_cost,
    space_time_coordinate
  >;
  search_type as(from, *to, w,
                 manhattan_distance_heuristic{},
                 predicted_cost(predictor_.get(), w.tick(), obstacle_penalty_),
                 passable_if_not_predicted_obstacle(
                   predictor_.get(),
                   passable_if_not_reserved(agent_reservations_, a, from),
                   predictor_ ? obstacle_threshold_ : 1.0
                 ));

  path<> join_path = as.find_path(
    w,
    [&] (position p) { return target_positions.count(p); },
    rejoin_limit_
  );

  nodes_rejoin_ += as.nodes_expanded();

  if (join_path.empty())
    return {};

  assert(target_positions.count(join_path.front()));
  path<>::const_reverse_iterator rejoin_point =
    target_positions[join_path.front()];

  path<> result;

  auto old_end = rejoin_point.base() - 1;
  auto new_begin = join_path.begin();

  result.insert(result.end(), old_path.begin(), old_end);
  result.insert(result.end(), new_begin, join_path.end());

  ++rejoin_successes_;
  return result;
}
