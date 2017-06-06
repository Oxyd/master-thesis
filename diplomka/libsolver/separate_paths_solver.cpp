#include "separate_paths_solver.hpp"

#include "log_sinks.hpp"
#include "lra.hpp"
#include "predictor.hpp"
#include "whca.hpp"

template <typename Derived>
separate_paths_solver<Derived>::separate_paths_solver(
  log_sink& log,
  unsigned rejoin_limit,
  std::unique_ptr<predictor> predictor,
  unsigned obstacle_penalty,
  double obstacle_threshold
)
  : log_(log)
  , rejoin_limit_(rejoin_limit)
  , predictor_(std::move(predictor))
  , obstacle_penalty_(obstacle_penalty)
  , obstacle_threshold_(obstacle_threshold)
{ }

static bool
is_waiting_at_goal(agent a, path<> const& path) {
  return std::all_of(path.begin(), path.end(),
                     [&] (position p) { return p == a.target; });
}

template <typename Derived>
void
separate_paths_solver<Derived>::step(
  world& w, std::default_random_engine& rng
) {
  if (predictor_)
    predictor_->update_obstacles(w);

  std::unordered_map<agent::id_type, position> agents;
  std::vector<agent::id_type> agent_order;
  std::unordered_set<agent::id_type> finished_agents;

  for (auto const& pos_agent : w.agents()) {
    agent const& a = std::get<1>(pos_agent);
    agents.insert({a.id(), std::get<0>(pos_agent)});
    agent_order.push_back(a.id());

    if (is_waiting_at_goal(a, paths_[a.id()]))
      finished_agents.insert(a.id());
  }

  std::shuffle(agent_order.begin(), agent_order.end(), rng);

  // Agents standing at the goal can be difficult for agents still trying to
  // reach the goal, especially if the waiting agent is in the way. So, we'll
  // look if we need to recalculate any non-finished agent at all -- if we do,
  // we'll discard the plans for the finished agents to force them to
  // cooperate. We'll also find paths for the finished agents after all
  // non-finished agents, so that the non-finished ones have priority.

  bool need_recalculate = false;
  for (auto const& id_path : paths_)
    if (std::get<1>(id_path).size() < 2
        && !finished_agents.count(std::get<0>(id_path))) {
      need_recalculate = true;
      break;
    }

  if (need_recalculate) {
    std::stable_partition(
      agent_order.begin(), agent_order.end(),
      [&] (agent::id_type id) { return !finished_agents.count(id); }
    );

    for (agent::id_type id : finished_agents)
      paths_[id].clear();
  }

  for (agent::id_type id : agent_order) {
    position const pos = agents[id];
    agent const& agent = *w.get_agent(pos);

    boost::optional<position> maybe_next = next_step(pos, w, rng);
    if (!maybe_next) {
      if (pos != agent.target) {
        log_ << "No path for " << pos << '\n';
        ++times_without_path_;
      }

      continue;
    }

    if (pos == *maybe_next)
      continue;

    direction dir = direction_to(pos, *maybe_next);
    if (!valid(action{pos, dir}, w)) {
      log_ << "Path invalid for " << pos << '\n';
      ++path_invalid_;

      path<> old_path = std::move(paths_[agent.id()]);
      paths_.erase(agent.id());
      maybe_next = next_step(pos, w, rng, old_path);
    }

    if (!maybe_next || pos == *maybe_next)
      continue;

    dir = direction_to(pos, *maybe_next);
    action a{pos, dir};
    w = apply(a, std::move(w));
  }
}

template <typename Derived>
std::vector<std::string>
separate_paths_solver<Derived>::stat_names() const {
  return {"Path not found", "Recalculations", "Path invalid",
          "Rejoin nodes expanded", "Rejoin attempts", "Rejoin successes",
          "Rejoin success rate"};
}

template <typename Derived>
std::vector<std::string>
separate_paths_solver<Derived>::stat_values() const {
  return {
    std::to_string(times_without_path_),
    std::to_string(recalculations_),
    std::to_string(path_invalid_),
    std::to_string(nodes_rejoin_),
    std::to_string(rejoin_attempts_),
    std::to_string(rejoin_successes_),
    rejoin_attempts_ > 0
      ? std::to_string((double) rejoin_successes_ / (double) rejoin_attempts_)
      : "0"
  };
}

template <typename Derived>
std::vector<position>
separate_paths_solver<Derived>::get_path(agent::id_type a) const {
  auto p_it = paths_.find(a);
  if (p_it != paths_.end())
    return p_it->second;
  else
    return {};
}

template <typename Derived>
std::unordered_map<position_time, double>
separate_paths_solver<Derived>::get_obstacle_field() const {
  if (predictor_)
    return predictor_->field();
  else
    return {};
}

template <typename Derived>
path<>
separate_paths_solver<Derived>::recalculate(
  position from, world const& w, agent::id_type agent_id,
  std::default_random_engine& rng, boost::optional<path<> const&> old_path
) {
  on_path_invalid(agent_id);

  path<> new_path;

  if (rejoin_limit_ > 0 && old_path) {
    ++rejoin_attempts_;

    if (auto p = rejoin_path(from, w, *old_path))
      if (path_valid(*p, w)) {
        ++rejoin_successes_;
        new_path = std::move(*p);
      }
  }

  if (new_path.empty()) {
    log_ << "Recalculating for " << w.get_agent(from)->id()
         << '@' << from << '\n';
    ++recalculations_;

    new_path = find_path(from, w, rng);
  }

  if (new_path.empty())
    log_ << "Found no path for " << from << '\n';
  else
    on_path_found(agent_id, new_path, w);

  return new_path;
}

template <typename Derived>
boost::optional<position>
separate_paths_solver<Derived>::next_step(
  position from, world const& w, std::default_random_engine& rng,
  boost::optional<path<> const&> old_path
) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (paths_[a.id()].size() < 2)
    paths_[a.id()] = recalculate(from, w, a.id(), rng, old_path);

  if (paths_[a.id()].size() < 2)
    return {};

  assert(paths_[a.id()].back() == from);
  paths_[a.id()].pop_back();
  return paths_[a.id()].back();
}

template <typename Derived>
boost::optional<path<>>
separate_paths_solver<Derived>::rejoin_path(position from, world const& w,
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

  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  auto as = derived()->make_rejoin_search(from, *to, w, a);
  path<> join_path = as->find_path(
    w,
    [&] (position p) { return target_positions.count(p); },
    rejoin_limit_
  );

  nodes_rejoin_ += as->nodes_expanded();

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

  return result;
}

template class separate_paths_solver<whca>;
template class separate_paths_solver<lra>;
