#include "solvers.hpp"

#include "a_star.hpp"
#include "log_sinks.hpp"

#include <algorithm>
#include <array>
#include <random>
#include <unordered_map>
#include <unordered_set>
#include <tuple>
#include <queue>

static direction
random_dir(std::default_random_engine& rng) {
  std::uniform_int_distribution<unsigned> d(0, 3);
  return static_cast<direction>(d(rng));
}

static void
make_random_action(position from, world& w, joint_action& actions,
                   std::default_random_engine& rng)
{
  direction const d = random_dir(rng);
  action const a{from, d};

  if (valid(a, w)) {
    actions.add(a);
    w = apply(a, w);
  }
}

bool
solved(world const& w) {
  for (auto const& pos_agent : w.agents()) {
    position const& pos = std::get<0>(pos_agent);
    agent const& agent = std::get<1>(pos_agent);

    if (pos != agent.target)
      return false;
  }

  return true;
}

joint_action
greedy::get_action(world temp_world, std::default_random_engine& rng) {
  std::vector<std::tuple<position, agent>> agents(temp_world.agents().begin(),
                                                  temp_world.agents().end());
  std::shuffle(agents.begin(), agents.end(), rng);

  std::discrete_distribution<bool> random_move{0.99, 0.01};

  joint_action result;

  for (auto const& pos_agent : agents) {
    position const& pos = std::get<0>(pos_agent);
    agent const& agent = std::get<1>(pos_agent);

    position const& goal = agent.target;
    if (pos == goal)
      continue;

    if (random_move(rng)) {
      make_random_action(pos, temp_world, result, rng);
    } else {
      int const dx = goal.x - pos.x;
      int const dy = goal.y - pos.y;

      direction d;
      if (std::abs(dx) > std::abs(dy))
        d = dx > 0 ? direction::east : direction::west;
      else
        d = dy > 0 ? direction::south : direction::north;

      action const a{pos, d};
      if (valid(a, temp_world)) {
        result.add(a);
        temp_world = apply(a, temp_world);
      } else {
        make_random_action(pos, temp_world, result, rng);
      }
    }
  }

  return result;
}

separate_paths_solver::separate_paths_solver(log_sink& log)
  : log_(log) { }

joint_action
separate_paths_solver::get_action(
  world w, std::default_random_engine& rng
) {
  std::unordered_map<agent::id_type, position> agents;
  std::vector<agent::id_type> agent_order;

  for (auto const& pos_agent : w.agents()) {
    agents.insert({std::get<1>(pos_agent).id(), std::get<0>(pos_agent)});
    agent_order.push_back(std::get<1>(pos_agent).id());
  }

  std::shuffle(agent_order.begin(), agent_order.end(), rng);

  joint_action result;

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

      path old_path = std::move(paths_[agent.id()]);
      paths_.erase(agent.id());
      maybe_next = next_step(pos, w, rng, old_path);
    }

    if (!maybe_next || pos == *maybe_next)
      continue;

    dir = direction_to(pos, *maybe_next);
    action a{pos, dir};
    result.add(a);
    w = apply(a, w);
  }

  return result;
}

std::vector<std::string>
separate_paths_solver::stat_values() const {
  return {
    std::to_string(times_without_path_),
    std::to_string(recalculations_),
    std::to_string(path_invalid_)
  };
}

std::vector<position>
separate_paths_solver::get_path(agent::id_type a) const {
  auto p_it = paths_.find(a);
  if (p_it != paths_.end())
    return p_it->second;
  else
    return {};
}

path
separate_paths_solver::recalculate(position from, world const& w,
                                   std::default_random_engine& rng,
                                   boost::optional<path const&> old_path) {
  log_ << "Recalculating for " << w.get_agent(from)->id()
       << '@' << from << '\n';
  ++recalculations_;

  path new_path = find_path(from, w, rng, old_path);

  if (new_path.empty())
    log_ << "Found no path for " << from << '\n';

  return new_path;
}

boost::optional<position>
separate_paths_solver::next_step(position from, world const& w,
                                 std::default_random_engine& rng,
                                 boost::optional<path const&> old_path) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (paths_[a.id()].size() < 2)
    paths_[a.id()] = recalculate(from, w, rng, old_path);

  if (paths_[a.id()].size() < 2)
    return {};

  assert(paths_[a.id()].back() == from);
  paths_[a.id()].pop_back();
  return paths_[a.id()].back();
}

std::vector<std::string>
lra::stat_names() const {
  std::vector<std::string> result = separate_paths_solver::stat_names();
  result.push_back("Nodes expanded");
  return result;
}

std::vector<std::string>
lra::stat_values() const {
  std::vector<std::string> result = separate_paths_solver::stat_values();
  result.push_back(std::to_string(nodes_));
  return result;
}

double
lra::agitated_distance::operator () (position from, world const&, unsigned) const {
  std::uniform_real_distribution<> agit(0.0, agitation);
  return distance(from, destination) + agit(rng);
}

path
lra::find_path(position from, world const& w, std::default_random_engine& rng,
               boost::optional<path const&>) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (from == a.target)
    return {};

  tick_t const recalc_interval = w.tick() - data_[a.id()].last_recalculation;
  assert(recalc_interval > 0);
  if (recalc_interval < 5)
    data_[a.id()].agitation += 5 / recalc_interval;
  else
    data_[a.id()].agitation = 0.0;

  a_star<passable_not_immediate_neighbour, agitated_distance> as(
    from, a.target, w,
    agitated_distance{a.target, data_[a.id()].agitation, rng},
    passable_not_immediate_neighbour{from}
  );
  path new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  data_[a.id()].last_recalculation = w.tick();

  return new_path;
}

cooperative_a_star::cooperative_a_star(log_sink& log, unsigned window,
                                       unsigned rejoin_limit,
                                       bool avoid_obstacles,
                                       unsigned obstacle_penalty)
  : separate_paths_solver(log)
  , window_(window)
  , rejoin_limit_(rejoin_limit)
  , avoid_obstacles_(avoid_obstacles)
  , obstacle_penalty_(obstacle_penalty)
{ }

std::vector<std::string>
cooperative_a_star::stat_names() const {
  std::vector<std::string> result = separate_paths_solver::stat_names();
  result.insert(result.end(),
                {"Primary nodes expanded", "Heuristic nodes expanded",
                 "Rejoin nodes expanded", "Total nodes expanded",
                 "Rejoin attempts", "Rejoin successes", "Rejoin success rate"});
  return result;
}

std::vector<std::string>
cooperative_a_star::stat_values() const {
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
cooperative_a_star::get_obstacle_field() const {
  return predictor_.field();
}

double
cooperative_a_star::hierarchical_distance::operator () (
  position from,
  world const& w,
  unsigned distance_so_far
) {
  if (from == h_search_.from())
    return 0.0;

  unsigned h_distance = h_search_.find_distance(from, w);
  double obstacle_prob =
    penalise_obstacles_
    ? predictor_.predict_obstacle({from, w.tick() + distance_so_far})
    : 0.0;

  return h_distance + obstacle_prob * obstacle_penalty_;
}

path
cooperative_a_star::find_path(position from, world const& w,
                              std::default_random_engine&,
                              boost::optional<path const&> old_path) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (avoid_obstacles_)
    predictor_.update_obstacles(w);
  predictor_.unreserve(a.id());

  path new_path;
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
      predictor::passable_not_reserved,
      hierarchical_distance,
      space_time_coordinate
    >;
    search_type as(
      from, a.target, w,
      hierarchical_distance(h_search, predictor_,
                            avoid_obstacles_, obstacle_penalty_),
      predictor_.passable_predicate(a, from)
    );
    new_path = as.find_path(w, window_);

    nodes_primary_ += as.nodes_expanded();
    nodes_heuristic_ += h_search.nodes_expanded() - old_h_search_nodes;
  }

  predictor_.reserve(a.id(), new_path, w.tick());

  return new_path;
}

std::ostream&
operator << (std::ostream& out, path const& p) {
  for (auto point = p.rbegin(); point != p.rend(); ++point) {
    if (point != p.rbegin())
      out << " -> ";
    out << *point;
  }

  return out;
}

boost::optional<path>
cooperative_a_star::rejoin_path(position from, world const& w,
                                path const& old_path) {
  if (old_path.empty())
    return {};

  ++rejoin_attempts_;

  boost::optional<position> to;
  std::unordered_map<position, path::const_iterator> target_positions;
  for (auto point = old_path.rbegin(); point != old_path.rend(); ++point)
    if (w.get(*point) == tile::free) {
      if (!to)
        to = *point;
      target_positions.insert({*point, point.base()});
    }

  if (!to)
    return {};

  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  using search_type = a_star<
    predictor::passable_not_reserved,
    manhattan_distance_heuristic,
    space_time_coordinate
  >;
  search_type as(from, *to, w, predictor_.passable_predicate(a, from));

  path join_path = as.find_path(
    w,
    [&] (position p) { return target_positions.count(p); },
    rejoin_limit_
  );

  nodes_rejoin_ += as.nodes_expanded();

  if (join_path.empty())
    return {};

  assert(target_positions.count(join_path.front()));
  path::const_iterator rejoin_point = target_positions[join_path.front()];

  path result;
  if (rejoin_point != old_path.begin())
    result.insert(result.end(), old_path.begin(), std::prev(rejoin_point));
  result.insert(result.end(), join_path.begin(), join_path.end());

  ++rejoin_successes_;
  return result;
}
