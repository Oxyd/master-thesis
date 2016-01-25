#include "solvers.hpp"

#include "a_star.hpp"
#include "log_sinks.hpp"

#include <algorithm>
#include <array>
#include <random>
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

std::array<solver_description, 3>
solvers{{
  solver_description{
    "WHCA*",
    [] (log_sink& log, world const& w) {
      return std::make_unique<cooperative_a_star>(log, w);
    }
  },
  solver_description{
    "LRA*",
    [] (log_sink& log, world const&) {
      return std::make_unique<lra>(log);
    }
  },
  solver_description{
    "Greedy",
    [] (log_sink&, world const&) {
      return std::make_unique<greedy>();
    }
  }
}};

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
  joint_action result;

  auto agents = w.agents();
  for (auto const& pos_agent : agents) {
    position const pos = std::get<0>(pos_agent);
    agent const& agent = std::get<1>(pos_agent);

    if (pos == agent.target)
      continue;

    boost::optional<position> maybe_next = next_step(pos, w, rng);
    if (!maybe_next) {
      log_ << "No path for " << pos << '\n';
      ++times_without_path_;
      continue;
    }

    direction dir = direction_to(pos, *maybe_next);
    if (!valid(action{pos, dir}, w)) {
      log_ << "Path invalid for " << pos << '\n';
      ++path_invalid_;

      paths_.erase(agent.id());
      maybe_next = next_step(pos, w, rng);
    }

    if (!maybe_next)
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
    std::to_string(path_invalid_),
    std::to_string(nodes_)
  };
}

path
separate_paths_solver::recalculate(position from, world const& w,
                                   std::default_random_engine& rng) {
  log_ << "Recalculating for " << w.get_agent(from)->id()
       << '@' << from << '\n';
  ++recalculations_;

  path new_path = find_path(from, w, rng);

  if (new_path.empty())
    log_ << "Found no path for " << from << '\n';

  return new_path;
}

boost::optional<position>
separate_paths_solver::next_step(position from, world const& w,
                                 std::default_random_engine& rng) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (paths_[a.id()].size() < 2)
    paths_[a.id()] = recalculate(from, w, rng);

  if (paths_[a.id()].size() < 2)
    return {};

  assert(paths_[a.id()].back() == from);
  paths_[a.id()].pop_back();
  return paths_[a.id()].back();
}

path
lra::find_path(position from, world const& w, std::default_random_engine& rng) {
  assert(w.get_agent(from));

  struct impassable_immediate_neighbour {
    position from;
    bool operator () (position p, world const& w, unsigned) {
      return w.get(p) == tile::free || !neighbours(p, from);
    }
  };

  struct agitated_distance {
    position destination;
    double agitation;
    std::default_random_engine& rng;

    unsigned operator () (position from, world const&) const {
      std::uniform_real_distribution<> agit(0.0, agitation);
      return distance(from, destination) + agit(rng);
    }
  };

  agent const& a = *w.get_agent(from);

  tick_t const recalc_interval = w.tick() - data_[a.id()].last_recalculation;
  assert(recalc_interval > 0);
  if (recalc_interval < 5)
    data_[a.id()].agitation += 5 / recalc_interval;
  else
    data_[a.id()].agitation = 0.0;

  a_star<impassable_immediate_neighbour, agitated_distance> as(
    from, a.target, w,
    agitated_distance{a.target, data_[a.id()].agitation, rng},
    impassable_immediate_neighbour{from}
  );
  path new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  data_[a.id()].last_recalculation = w.tick();

  return new_path;
}

cooperative_a_star::cooperative_a_star(log_sink& log, world const& w)
  : separate_paths_solver(log)
{
  for (auto const& pos_agent : w.agents())
    permanent_reservations_.insert({pos_agent.first,
                                    {pos_agent.second.id(), w.tick()}});
}

path
cooperative_a_star::find_path(position from, world const& w,
                              std::default_random_engine&) {
  struct impassable_reserved {
    impassable_reserved(
      reservation_table_type const& reservations,
      permanent_reservation_table_type const& permanent_reservations,
      agent const& agent,
      position from
    )
      : reservations_(reservations)
      , permanent_reservations_(permanent_reservations)
      , agent_(agent)
      , from_(from) { }

    bool operator () (position p, world const& w, unsigned distance) {
      if (reservations_.count(position_time{p, w.tick() + distance}))
        return false;

      auto permanent = permanent_reservations_.find(p);
      if (permanent != permanent_reservations_.end() &&
          permanent->second.start <= w.tick() + distance)
        return false;

      return w.get(p) == tile::free || !neighbours(p, from_);
    }

  private:
    reservation_table_type const& reservations_;
    permanent_reservation_table_type const& permanent_reservations_;
    agent const& agent_;
    position from_;
  };

  struct distance_heuristic {
    explicit
    distance_heuristic(heuristic_search_type& h_search)
      : h_search_(h_search) { }

    unsigned operator () (position from, world const& w) {
      return h_search_.find_distance(from, w);
    }

  private:
    heuristic_search_type& h_search_;
  };

  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  unreserve(a);

  heuristic_search_type& h_search = heuristic_map_.emplace(
    std::piecewise_construct,
    std::forward_as_tuple(a.id()),
    std::forward_as_tuple(a.target, from, w)
  ).first->second;
  unsigned const old_h_search_nodes = h_search.nodes_expanded();

  a_star<impassable_reserved, distance_heuristic> as(
    from, a.target, w,
    distance_heuristic(h_search),
    impassable_reserved(reservations_, permanent_reservations_, a, from)
  );
  path new_path = as.find_path(w, 5);
  nodes_ += as.nodes_expanded();
  nodes_ += h_search.nodes_expanded() - old_h_search_nodes;

  for (tick_t distance = 1; distance <= new_path.size(); ++distance) {
    position const p = new_path[new_path.size() - distance];
    position_time const pt{p, w.tick() + distance - 1};

    assert(!reservations_.count(pt));
    reservations_[pt] = a.id();
  }
  if ((!new_path.empty() && new_path.front() == a.target) ||
      (new_path.empty() && from == a.target))
    permanent_reservations_.insert({
      new_path.front(), {a.id(), w.tick() + (tick_t) new_path.size()}
    });

  return new_path;
}

void
cooperative_a_star::unreserve(agent const& a) {
  auto it = reservations_.begin();
  while (it != reservations_.end())
    if (it->second == a.id())
      it = reservations_.erase(it);
    else
      ++it;

  auto perm = permanent_reservations_.begin();
  while (perm != permanent_reservations_.end())
    if (perm->second.agent_id == a.id())
      perm = permanent_reservations_.erase(perm);
    else
      ++perm;
}
