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

using path = std::stack<direction>;

static double
distance(position a, position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

static constexpr double
infinity = std::numeric_limits<double>::max();

static constexpr double agent_penalty = 100;
static constexpr double obstacle_penalty = 150;

namespace {

struct always_passable {
  constexpr bool operator () (position, world const&) const { return true; }
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
    double g = infinity;
    double h = 0;
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
        result.push(direction_to(previous, current));
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

      if (!passable_(neighbour, w))
        continue;

      double step_cost = 1;
      if (w.get(neighbour) == tile::agent)
        step_cost += agent_penalty / (current.g + 1);
      if (w.get(neighbour) == tile::obstacle)
        step_cost += obstacle_penalty / (current.g + 1);

      auto neighbour_g = current.g + step_cost;
      auto n = open_.find(neighbour);
      if (n != open_.end()) {
        handle_type neighbour_handle = n->second;
        if ((*neighbour_handle).g > neighbour_g) {
          (*neighbour_handle).g = neighbour_g;
          come_from_[neighbour] = current.pos;
          heap_.decrease(neighbour_handle);
        }

      } else {
        handle_type h = heap_.push(
          {neighbour, neighbour_g, distance(neighbour, to_)}
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

std::array<solver_description, 2>
solvers{{
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

lra::lra()
  : log_(null_log_sink) { }

lra::lra(log_sink& log)
  : log_(log) { }

joint_action
lra::get_action(world w, std::default_random_engine&) {
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

      if (!valid(action{pos, path_it->second.top()}, w))
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

    direction d = p.top();
    position new_pos = translate(pos, d);

    action a{pos, d};
    if (!valid(a, w)) {
      log_ << "Path invalid for " << pos << '\n';
      ++path_invalid_;
      continue;
    }

    result.add(a);
    w = apply(a, w);
    p.pop();

    if (path_it != paths_.end())
      paths_.erase(path_it);
    paths_.emplace(new_pos, std::move(p));
  }

  return result;
}

std::vector<std::string>
lra::stat_values() const {
  return {
    std::to_string(times_without_path_),
    std::to_string(recalculations_),
    std::to_string(path_invalid_),
    std::to_string(nodes_)
  };
}

path
lra::recalculate(position from, world const& w) {
  log_ << "Recalculating for " << from << '\n';
  ++recalculations_;

  assert(w.get_agent(from));

  struct impassable_immediate_neighbour {
    position from;
    bool operator () (position p, world const& w) {
      return w.get(p) != tile::agent || neighbours(p, from);
    }
  };

  a_star<impassable_immediate_neighbour> as(
    from, w.get_agent(from)->target,
    impassable_immediate_neighbour{from}
  );
  path new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  if (new_path.empty())
    log_ << "A* found no path for " << from << '\n';

  return new_path;
}
