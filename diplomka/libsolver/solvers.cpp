#include "solvers.hpp"
#include "log_sinks.hpp"

#include <boost/heap/fibonacci_heap.hpp>

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

using path = std::vector<direction>;

static unsigned
distance(position a, position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static constexpr double
infinity = std::numeric_limits<double>::max();

static constexpr double agent_penalty = 100;
static constexpr double obstacle_penalty = 150;

namespace {

struct always_passable {
  constexpr bool operator () (position, world const&, unsigned) const {
    return true;
  }
};

template <typename Passable = always_passable>
class a_star {
public:
  a_star(position from, position to, Passable passable = Passable{});
  path find_path(world const& w);

  unsigned nodes_expanded() const { return expanded_; }

private:
  struct node {
    position pos;
    unsigned g = infinity;
    unsigned h = 0;
    double f() const { return g + h; }
  };

  struct node_comparator {
    bool
    operator () (node x, node y) const {
      return x.f() > y.f();
    }
  };

  using heap_type = boost::heap::fibonacci_heap<
    node, boost::heap::compare<node_comparator>
  >;
  using handle_type = typename heap_type::handle_type;

  position from_;
  position to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  std::unordered_map<position, handle_type> open_;
  std::unordered_set<position> closed_;
  std::unordered_map<position, position> come_from_;
  Passable passable_;
};

}

template <typename Passable>
a_star<Passable>::a_star(position from, position to, Passable passable)
  : from_(from)
  , to_(to)
  , passable_(std::move(passable))
{
  handle_type h = heap_.push({from, 0, distance(from, to)});
  open_.insert({from, h});
}

template <typename Passable>
path
a_star<Passable>::find_path(world const& w) {
  while (!heap_.empty()) {
    node current = heap_.top();
    heap_.pop();
    open_.erase(current.pos);
    closed_.insert(current.pos);

    if (current.pos == to_) {
      path result;
      position current = to_;

      while (current != from_) {
        position previous = come_from_[current];
        result.push_back(direction_to(previous, current));
        current = previous;
      }

      return result;
    }

    ++expanded_;

    for (direction d : all_directions) {
      position const neighbour = translate(current.pos, d);
      if (!in_bounds(neighbour, *w.map()) ||
          w.get(neighbour) == tile::wall)
        continue;

      if (closed_.count(neighbour))
        continue;

      if (!passable_(neighbour, w, current.g + 1))
        continue;

      double step_cost = 0;
#if 0
      if (w.get(neighbour) == tile::agent)
        step_cost += agent_penalty / (current.g + 1);
      if (w.get(neighbour) == tile::obstacle)
        step_cost += obstacle_penalty / (current.g + 1);
#endif

      auto n = open_.find(neighbour);
      if (n != open_.end()) {
        handle_type neighbour_handle = n->second;
        if ((*neighbour_handle).g > current.g + 1) {
          (*neighbour_handle).g = current.g + 1;
          come_from_[neighbour] = current.pos;
          heap_.decrease(neighbour_handle);
        }

      } else {
        handle_type h = heap_.push(
          {neighbour, current.g + 1,
           (unsigned) (distance(neighbour, to_) + step_cost)}
        );
        come_from_[neighbour] = current.pos;
        open_.insert({neighbour, h});
      }
    }
  }

  return path{};
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
    "CA*", [] (log_sink& log) { return std::make_unique<cooperative_a_star>(log); }
  },
  solver_description{
    "LRA*", [] (log_sink& log) { return std::make_unique<lra>(log); }
  },
  solver_description{
    "Greedy", [] (log_sink&) { return std::make_unique<greedy>(); }
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
  world w, std::default_random_engine&
) {
  joint_action result;

  std::vector<std::tuple<position, agent>> agents(w.agents().begin(),
                                                  w.agents().end());
  for (auto pos_agent : agents) {
    position const pos = std::get<0>(pos_agent);
    agent const& agent = std::get<1>(pos_agent);

    if (pos == agent.target)
      continue;

    auto path_it = paths_.find(pos);
    path p;

    if (path_it != paths_.end()) {
      if (path_it->second.empty()) {
        assert(pos == agent.target);
        paths_.erase(path_it);
        continue;
      }

      if (!valid(action{pos, path_it->second.back()}, w))
        p = recalculate(pos, w);
      else
        p = std::move(path_it->second);
    } else
      p = recalculate(pos, w);

    if (p.empty()) {
      log_ << "No path for " << pos << '\n';
      ++times_without_path_;
      continue;
    }

    direction d = p.back();
    position new_pos = translate(pos, d);

    action a{pos, d};
    if (!valid(a, w)) {
      log_ << "Path invalid for " << pos << '\n';
      ++path_invalid_;
      continue;
    }

    result.add(a);
    w = apply(a, w);
    p.pop_back();

    if (path_it != paths_.end())
      paths_.erase(path_it);
    paths_.emplace(new_pos, std::move(p));
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
separate_paths_solver::recalculate(position from, world const& w) {
  log_ << "Recalculating for " << from << '\n';
  ++recalculations_;

  path new_path = find_path(from, w);

  if (new_path.empty())
    log_ << "A* found no path for " << from << '\n';

  return new_path;
}

path
lra::find_path(position from, world const& w) {
  assert(w.get_agent(from));

  struct impassable_immediate_neighbour {
    position from;
    bool operator () (position p, world const& w, unsigned) {
      return w.get(p) == tile::free || !neighbours(p, from);
    }
  };

  a_star<impassable_immediate_neighbour> as(
    from, w.get_agent(from)->target,
    impassable_immediate_neighbour{from}
  );
  path new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  return new_path;
}

path
cooperative_a_star::find_path(position from, world const& w) {
  struct impassable_reserved {
    impassable_reserved(reservation_table_type const& reservations,
                        agent const& agent,
                        position from)
      : reservations_(reservations)
      , agent_(agent)
      , from_(from) { }

    bool operator () (position p, world const& w, unsigned distance) {
      if (reservations_.count(position_time{p, w.tick() + distance}))
        return false;
      else
        return w.get(p) == tile::free || !neighbours(p, from_);
    }

  private:
    reservation_table_type const& reservations_;
    agent const& agent_;
    position from_;
  };

  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  unreserve(a);

  a_star<impassable_reserved> as(
    from, a.target, impassable_reserved(reservations_, a, from)
  );
  path new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  position p = from;
  for (tick_t distance = 0; distance < new_path.size(); ++distance) {
    p = translate(p, new_path[new_path.size() - distance - 1]);
    position_time const pt{p, w.tick() + distance + 1};

    assert(!reservations_.count(pt));
    reservations_[pt] = a.id();
  }

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
}
