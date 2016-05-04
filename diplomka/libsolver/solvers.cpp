#include "solvers.hpp"

#include "a_star.hpp"
#include "hash.hpp"
#include "log_sinks.hpp"
#include "predictor.hpp"

#include <algorithm>
#include <array>
#include <list>
#include <queue>
#include <random>
#include <tuple>
#include <unordered_map>
#include <unordered_set>

static direction
random_dir(std::default_random_engine& rng) {
  std::uniform_int_distribution<unsigned> d(0, 3);
  return static_cast<direction>(d(rng));
}

static void
make_random_action(position from, world& w, std::default_random_engine& rng)
{
  direction const d = random_dir(rng);
  action const a{from, d};

  if (valid(a, w))
    w = apply(a, std::move(w));
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

namespace {

class greedy : public solver {
public:
  void step(world& w, std::default_random_engine&) override;
  std::string name() const override { return "Greedy"; }
};

}

std::unique_ptr<solver>
make_greedy() {
  return std::make_unique<greedy>();
}

void
greedy::step(world& world, std::default_random_engine& rng) {
  std::vector<std::tuple<position, agent>> agents(world.agents().begin(),
                                                  world.agents().end());
  std::shuffle(agents.begin(), agents.end(), rng);

  std::discrete_distribution<bool> random_move{0.99, 0.01};

  for (auto const& pos_agent : agents) {
    position const& pos = std::get<0>(pos_agent);
    agent const& agent = std::get<1>(pos_agent);

    position const& goal = agent.target;
    if (pos == goal)
      continue;

    if (random_move(rng)) {
      make_random_action(pos, world, rng);
    } else {
      int const dx = goal.x - pos.x;
      int const dy = goal.y - pos.y;

      direction d;
      if (std::abs(dx) > std::abs(dy))
        d = dx > 0 ? direction::east : direction::west;
      else
        d = dy > 0 ? direction::south : direction::north;

      action const a{pos, d};
      if (valid(a, world)) {
        world = apply(a, std::move(world));
      } else {
        make_random_action(pos, world, rng);
      }
    }
  }
}

namespace {

class separate_paths_solver : public solver {
public:
  explicit
  separate_paths_solver(log_sink& log);

  void
  step(world&, std::default_random_engine&) override;

  std::vector<std::string>
  stat_names() const override {
    return {"Path not found", "Recalculations", "Path invalid"};
  }

  std::vector<std::string>
  stat_values() const override;

  std::vector<position>
  get_path(agent::id_type) const override;

protected:
  log_sink& log_;
  unsigned times_without_path_ = 0;
  unsigned recalculations_ = 0;
  unsigned path_invalid_ = 0;

private:
  std::unordered_map<agent::id_type, path<>> paths_;

  path<>
  recalculate(position, world const&, std::default_random_engine&,
              boost::optional<path<> const&> old_path = {});

  boost::optional<position>
  next_step(position, world const&, std::default_random_engine&,
            boost::optional<path<> const&> old_path = {});

  virtual path<>
  find_path(position, world const&, std::default_random_engine&,
            boost::optional<path<> const&> old_path) = 0;
};

}

separate_paths_solver::separate_paths_solver(log_sink& log)
  : log_(log) { }

void
separate_paths_solver::step(
  world& w, std::default_random_engine& rng
) {
  std::unordered_map<agent::id_type, position> agents;
  std::vector<agent::id_type> agent_order;

  for (auto const& pos_agent : w.agents()) {
    agents.insert({std::get<1>(pos_agent).id(), std::get<0>(pos_agent)});
    agent_order.push_back(std::get<1>(pos_agent).id());
  }

  std::shuffle(agent_order.begin(), agent_order.end(), rng);

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

path<>
separate_paths_solver::recalculate(position from, world const& w,
                                   std::default_random_engine& rng,
                                   boost::optional<path<> const&> old_path) {
  log_ << "Recalculating for " << w.get_agent(from)->id()
       << '@' << from << '\n';
  ++recalculations_;

  path<> new_path = find_path(from, w, rng, old_path);

  if (new_path.empty())
    log_ << "Found no path for " << from << '\n';

  return new_path;
}

boost::optional<position>
separate_paths_solver::next_step(position from, world const& w,
                                 std::default_random_engine& rng,
                                 boost::optional<path<> const&> old_path) {
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

namespace {

class lra : public separate_paths_solver {
public:
  explicit lra(log_sink& log) : separate_paths_solver(log) { }
  std::string name() const override { return "LRA*"; }

  std::vector<std::string>
  stat_names() const override;

  std::vector<std::string>
  stat_values() const override;

private:
  struct agent_data {
    tick_t last_recalculation = 0;
    double agitation = 0;
  };

  struct passable_not_immediate_neighbour {
    position from;
    bool operator () (position p, position, world const& w, unsigned) {
      return w.get(p) == tile::free || !neighbours(p, from);
    }
  };

  struct agitated_distance {
    position destination;
    double agitation;
    std::default_random_engine& rng;

    double operator () (position from, world const&, unsigned) const;
  };

  std::unordered_map<agent::id_type, agent_data> data_;
  unsigned nodes_ = 0;

  path<> find_path(position, world const&, std::default_random_engine&,
                   boost::optional<path<> const&>) override;
};

}

std::unique_ptr<solver>
make_lra(log_sink& log) {
  return std::make_unique<lra>(log);
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

path<>
lra::find_path(position from, world const& w, std::default_random_engine& rng,
               boost::optional<path<> const&>) {
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

  a_star<
    position, position_successors,
    passable_not_immediate_neighbour, agitated_distance
  > as(
    from, a.target, w,
    agitated_distance{a.target, data_[a.id()].agitation, rng},
    passable_not_immediate_neighbour{from}
  );
  path<> new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  data_[a.id()].last_recalculation = w.tick();

  return new_path;
}

namespace {

class cooperative_a_star : public separate_paths_solver {
public:
  cooperative_a_star(log_sink& log, unsigned window, unsigned rejoin_limit,
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
    hierarchical_distance(heuristic_search_type& h_search, predictor* p,
                          unsigned obstacle_penalty)
      : h_search_(h_search)
      , predictor_(p)
      , obstacle_penalty_(obstacle_penalty)
    { }

    double operator () (position from, world const& w,
                        unsigned distance_so_far);

  private:
    heuristic_search_type& h_search_;
    predictor* predictor_;
    unsigned obstacle_penalty_ = 100;
  };

  struct predicted_manhattan_distance {
    predicted_manhattan_distance(position destination, predictor* p,
                                 unsigned obstacle_penalty)
      : destination_(destination)
      , predictor_(p)
      , obstacle_penalty_(obstacle_penalty)
    { }

    double operator () (position from, world const& w,
                        unsigned distance_so_far);

  private:
    position destination_;
    predictor* predictor_;
    unsigned obstacle_penalty_;
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
  unsigned obstacle_penalty_ = 100;
  double obstacle_threshold_ = 0.1;

  path<> find_path(position, world const&, std::default_random_engine&,
                   boost::optional<path<> const&> old_path) override;
  boost::optional<path<>> rejoin_path(position from, world const& w,
                                      path<> const& old_path);
};

}

std::unique_ptr<solver>
make_whca(log_sink& log, unsigned window, unsigned rejoin_limit,
          std::unique_ptr<predictor> predictor, unsigned obstacle_penalty,
          double obstacle_threshold) {
  return std::make_unique<cooperative_a_star>(
    log, window, rejoin_limit, std::move(predictor), obstacle_penalty,
    obstacle_threshold
  );
}

cooperative_a_star::cooperative_a_star(log_sink& log, unsigned window,
                                       unsigned rejoin_limit,
                                       std::unique_ptr<predictor> predictor,
                                       unsigned obstacle_penalty,
                                       double obstacle_threshold)
  : separate_paths_solver(log)
  , predictor_(std::move(predictor))
  , window_(window)
  , rejoin_limit_(rejoin_limit)
  , obstacle_penalty_(obstacle_penalty)
  , obstacle_threshold_(obstacle_threshold)
{ }

void cooperative_a_star::step(world& w, std::default_random_engine& rng) {
  if (predictor_)
    predictor_->update_obstacles(w);

  separate_paths_solver::step(w, rng);
}

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
  if (predictor_)
    return predictor_->field();
  else
    return {};
}

cooperative_a_star::passable_if_not_reserved::passable_if_not_reserved(
  reservation_table_type const& reservations,
  agent const& agent,
  position from
)
  : reservations_(reservations)
  , agent_(agent)
  , from_(from)
{ }

bool
cooperative_a_star::passable_if_not_reserved::operator () (
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
cooperative_a_star::reserve(agent::id_type a_id, path<> const& path,
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
cooperative_a_star::unreserve(agent::id_type a_id) {
  auto it = agent_reservations_.begin();
  while (it != agent_reservations_.end())
    if (it->second.agent == a_id)
      it = agent_reservations_.erase(it);
    else
      ++it;
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
    predictor_
    ? predictor_->predict_obstacle({from, w.tick() + distance_so_far})
    : 0.0;

  return h_distance + obstacle_prob * obstacle_penalty_;
}

double
cooperative_a_star::predicted_manhattan_distance::operator () (
  position from,
  world const& w,
  unsigned distance_so_far
) {
  if (from == destination_)
    return 0.0;

  unsigned md = distance(from, destination_);
  double obstacle_prob =
    predictor_
    ? predictor_->predict_obstacle({from, w.tick() + distance_so_far})
    : 0.0;

  return md + obstacle_prob * obstacle_penalty_;
}

bool
cooperative_a_star::passable_if_not_predicted_obstacle::operator () (
  position where, position from, world const& w, unsigned distance
) {
  return
    not_reserved_(where, from, w, distance) &&
    (!predictor_ ||
     predictor_->predict_obstacle({where, w.tick() + distance}) <= threshold_);
}

path<>
cooperative_a_star::find_path(position from, world const& w,
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
      space_time_coordinate
    >;
    search_type as(
      from, a.target, w,
      hierarchical_distance(h_search, predictor_.get(), obstacle_penalty_),
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
cooperative_a_star::rejoin_path(position from, world const& w,
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
    predicted_manhattan_distance,
    space_time_coordinate
  >;
  search_type as(from, *to, w,
                 predicted_manhattan_distance(
                   *to, predictor_.get(), obstacle_penalty_
                 ),
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

namespace {

enum class agent_action : unsigned {
  north = 0, east, south, west, stay, unassigned
};

struct agent_state_record {
  ::position position;  // Post-move position.
  agent::id_type id;
  agent_action action = agent_action::unassigned;
};

struct agents_state {
  std::vector<agent_state_record> agents;
  std::size_t next_agent = 0;
};

static bool
operator == (agent_state_record const& lhs, agent_state_record const& rhs) {
  return lhs.position == rhs.position &&
         lhs.id == rhs.id &&
         lhs.action == rhs.action;
}

static bool
operator == (agents_state const& lhs, agents_state const& rhs) {
  return lhs.next_agent == rhs.next_agent && lhs.agents == rhs.agents;
}

static bool
operator != (agents_state const& lhs, agents_state const& rhs) {
  return !operator == (lhs, rhs);
}

struct state_successors {
  static std::vector<agents_state>
  get(agents_state const& state, world const& w);
};

class operator_decomposition : public solver {
public:
  operator_decomposition(unsigned window,
                         std::unique_ptr<predictor> predictor,
                         unsigned obstacle_penalty,
                         double obstacle_threshold)
    : window_(window)
    , predictor_(std::move(predictor))
    , obstacle_penalty_(obstacle_penalty)
    , obstacle_threshold_(obstacle_threshold)
  { }

  void step(world&, std::default_random_engine&) override;
  std::string name() const override { return "OD"; }

  std::vector<std::string>
  stat_names() const override {
    return {"Replans", "Plan invalid", "Nodes primary", "Nodes heuristic"};
  }

  std::vector<std::string>
  stat_values() const override {
    return {
      std::to_string(replans_),
      std::to_string(plan_invalid_),
      std::to_string(nodes_primary_),
      std::to_string(nodes_heuristic_)
    };
  }

private:
  using plan = path<agents_state>;

  struct group {
    operator_decomposition::plan plan;
    std::vector<position> starting_positions;
  };
  using group_list = std::list<group>;

  using group_id = group_list::iterator;

  struct reservation_table_record {
    group_id group;
    boost::optional<position> from;
  };

  using reservation_table_type = std::unordered_map<
    position_time,
    reservation_table_record
  >;

  struct combined_heuristic_distance {
    combined_heuristic_distance(
      std::unordered_map<agent::id_type, a_star<>>& h_searches,
      predictor* predictor,
      unsigned obstacle_penalty
    );

    unsigned
    operator () (agents_state const& state, world const& w,
                 unsigned distance_so_far) const;

  private:
    std::unordered_map<agent::id_type, a_star<>>& h_searches_;
    predictor* predictor_;
    unsigned obstacle_penalty_;
  };

  struct passable_not_immediate_neighbour {
    agents_state const& from;
    predictor* predictor_;
    double threshold_ = 1.0;

    bool
    operator () (agents_state const& state, agents_state const&, world const& w,
                 unsigned);
  };

  std::unordered_map<agent::id_type, a_star<>> hierarchical_distances_;
  group_list groups_;
  reservation_table_type reservation_table_;
  unsigned window_;
  std::unique_ptr<predictor> predictor_;
  unsigned obstacle_penalty_ = 100;
  double obstacle_threshold_ = 0.5;

  unsigned replans_ = 0;
  unsigned plan_invalid_ = 0;
  unsigned nodes_primary_ = 0;
  unsigned nodes_heuristic_ = 0;

  void
  replan(world const& w);

  bool
  replan_groups(world const& w);

  plan
  replan_group(world const& w, group const& group);

  enum class admissibility {
    admissible = 0,
    incomplete,
    invalid
  };

  admissibility
  plans_admissible(world const& w) const;

  bool
  final(agents_state const& state, world const& w) const;

  void
  merge_groups(std::vector<group_id> const& groups);

  void
  reserve(plan const&, group_id, tick_t start);

  void
  unreserve(group_id);

  void
  make_hierarchical_distances(world const&);
};

} // anonymous namespace

static direction
agent_action_to_direction(agent_action aa) {
  assert(aa != agent_action::stay);
  assert(aa != agent_action::unassigned);

  return static_cast<direction>(static_cast<unsigned>(aa));
}

static agent_action
direction_to_action(direction d) {
  return static_cast<agent_action>(static_cast<unsigned>(d));
}

static void
make_full(agents_state& state) {
  assert(state.next_agent == 0);

  for (agent_state_record& a : state.agents)
    a.action = agent_action::unassigned;
}

std::vector<agents_state>
state_successors::get(agents_state const& state, world const& w) {
  std::vector<agents_state> result;

  auto add = [&] (agent_action action, position dest) {
    result.push_back(state);
    result.back().agents[state.next_agent].action = action;
    result.back().agents[state.next_agent].position = dest;
    result.back().next_agent =
    (result.back().next_agent + 1) % result.back().agents.size();

#ifndef NDEBUG
    for (agent_state_record const& agent_a : state.agents)
      for (agent_state_record const& agent_b : state.agents)
        assert(agent_a.action == agent_action::unassigned ||
               agent_b.action == agent_action::unassigned ||
               agent_a.id == agent_b.id ||
               agent_a.position != agent_b.position);
#endif

    if (result.back().next_agent == 0)
      make_full(result.back());
  };

  agent_state_record const& agent = state.agents[state.next_agent];
  bool needs_vacate = false;

  for (direction d : all_directions) {
    position const destination = translate(agent.position, d);
    if (!in_bounds(destination, *w.map()) || w.get(destination) == tile::wall)
      continue;

    bool possible = true;

    for (agent_state_record const& other_agent : state.agents) {
      if (other_agent.action == agent_action::unassigned)
        break;

      if (other_agent.position == agent.position)
        needs_vacate = true;

      if (other_agent.action == agent_action::stay) {
        if (destination == other_agent.position) {
          possible = false;
          break;
        }
      } else {
        position came_from = translate(
          other_agent.position,
          inverse(agent_action_to_direction(other_agent.action))
        );

        if (destination == other_agent.position ||
            (destination == came_from &&
             other_agent.position == agent.position)) {
          possible = false;
          break;
        }
      }
    }

    if (possible)
      add(direction_to_action(d), destination);
  }

  if (!needs_vacate)
    add(agent_action::stay, agent.position);

  return result;
}

operator_decomposition::
combined_heuristic_distance::combined_heuristic_distance(
  std::unordered_map<agent::id_type, a_star<>>& h_searches,
  predictor* predictor,
  unsigned obstacle_penalty
)
  : h_searches_(h_searches)
  , predictor_(predictor)
  , obstacle_penalty_(obstacle_penalty)
{}

unsigned
operator_decomposition::combined_heuristic_distance::operator () (
  agents_state const& state, world const& w, unsigned distance_so_far
) const {
  unsigned result = 0;
  for (agent_state_record const& agent : state.agents) {
    auto h_search = h_searches_.find(agent.id);
    assert(h_search != h_searches_.end());

    result += h_search->second.find_distance(agent.position, w);

    if (predictor_)
      result += obstacle_penalty_ * predictor_->predict_obstacle(
        {agent.position, w.tick() + distance_so_far}
      );
  }

  return result;
}


namespace std {

template <>
struct hash<agent_state_record> {
  using argument_type = agent_state_record;
  using result_type = std::size_t;

  result_type
  operator () (argument_type agent) const {
    std::size_t result = 0;
    hash_combine(result, agent.position);
    hash_combine(result, agent.id);
    hash_combine(result, static_cast<unsigned>(agent.action));
    return result;
  }
};

template <>
struct hash<agents_state> {
  using argument_type = agents_state;
  using result_type = std::size_t;

  result_type
  operator () (argument_type const& state) const {
    std::size_t result = 0;
    for (agent_state_record agent : state.agents)
      hash_combine(result, agent);

    hash_combine(result, state.next_agent);

    return result;
  }
};

} // namespace std

static joint_action
make_action(agents_state const& from, agents_state const& to) {
  assert(from.agents.size() == to.agents.size());

  joint_action result;
  for (std::size_t i = 0; i < from.agents.size(); ++i) {
    assert(from.agents[i].id == to.agents[i].id);

    if (from.agents[i].position != to.agents[i].position)
      result.add(action{from.agents[i].position,
                        direction_to(from.agents[i].position,
                                     to.agents[i].position)});
  }

  return result;
}

std::unique_ptr<solver>
make_od(unsigned window,
        std::unique_ptr<predictor> predictor, unsigned obstacle_penalty,
        double obstacle_threshold) {
  return std::make_unique<operator_decomposition>(window,
                                                  std::move(predictor),
                                                  obstacle_penalty,
                                                  obstacle_threshold);
}

void
operator_decomposition::step(world& w, std::default_random_engine&) {
  if (predictor_)
    predictor_->update_obstacles(w);

  admissibility ad = plans_admissible(w);
  if (groups_.empty() || ad != admissibility::admissible) {
    if (ad == admissibility::invalid)
      ++plan_invalid_;

    replan(w);
  }

  joint_action result;
  for (group& group : groups_) {
    if (group.plan.size() < 2)
      continue;

    agents_state current = group.plan.back();
    group.plan.pop_back();
    result.extend(make_action(current, group.plan.back()));
  }

  w = apply(result, std::move(w));
}

bool
operator_decomposition::passable_not_immediate_neighbour::operator () (
  agents_state const& state, agents_state const&, world const& w,
  unsigned distance
) {
  for (agent_state_record const& agent : state.agents) {
    if (predictor_ &&
        predictor_->predict_obstacle(
          {agent.position, w.tick() + distance}
        ) > threshold_)
      return false;

    if (w.get(agent.position) == tile::obstacle)
      for (agent_state_record const& from_agent : from.agents)
        if (from_agent.id == agent.id) {
          if (neighbours(from_agent.position, agent.position))
            return false;
          else
            break;
        }
  }

  return true;
}

void
operator_decomposition::replan(world const& w) {
  ++replans_;
  groups_.clear();
  reservation_table_.clear();

  make_hierarchical_distances(w);

  unsigned old_nodes_heuristic = 0;
  for (auto const& id_search : hierarchical_distances_)
    old_nodes_heuristic += std::get<1>(id_search).nodes_expanded();

  for (auto const& pos_agent : w.agents())
    groups_.push_back(group{{}, {std::get<0>(pos_agent)}});

  bool conflicted;
  do
    conflicted = replan_groups(w);
  while (conflicted);

  unsigned new_nodes_heuristic = 0;
  for (auto const& id_search : hierarchical_distances_)
    new_nodes_heuristic += std::get<1>(id_search).nodes_expanded();

  nodes_heuristic_ += new_nodes_heuristic - old_nodes_heuristic;
}

bool
operator_decomposition::replan_groups(world const& w) {
  for (group_id group = groups_.begin(); group != groups_.end(); ++group) {
    if (!group->plan.empty())
      continue;

    group->plan = replan_group(w, *group);

    tick_t time = w.tick();
    std::vector<group_id> conflicts;

    for (auto state = group->plan.rbegin();
         state != group->plan.rend();
         ++state)
    {
      for (std::size_t i = 0; i < state->agents.size(); ++i) {
        agent_state_record const& agent = state->agents[i];

        boost::optional<group_id> conflicting_group;

        auto conflict = reservation_table_.find({agent.position, time});
        if (conflict != reservation_table_.end())
          conflicting_group = conflict->second.group;

        if (!conflicting_group && state != group->plan.rbegin()) {
          assert(std::prev(state)->agents[i].id == agent.id);

          boost::optional<position> from = std::prev(state)->agents[i].position;
          if (from) {
            auto vacated = reservation_table_.find({*from, time});
            if (vacated != reservation_table_.end() &&
                vacated->second.from &&
                *vacated->second.from == agent.position)
              conflicting_group = vacated->second.group;
          }
        }

        assert(!conflicting_group || group != *conflicting_group);

        if (conflicting_group &&
            std::find(conflicts.begin(), conflicts.end(),
                      *conflicting_group) == conflicts.end())
          conflicts.push_back(*conflicting_group);
      }

      ++time;
    }

    if (conflicts.empty())
      reserve(group->plan, group, w.tick());
    else {
      conflicts.push_back(group);
      merge_groups(conflicts);

      return true;
    }
  }

  return false;
}

path<agents_state>
operator_decomposition::replan_group(world const& w,
                                     group const& group) {
  agents_state current_state;
  agents_state goal_state;

  for (auto const& member_pos : group.starting_positions) {
    assert(w.get_agent(member_pos));
    agent const& a = *w.get_agent(member_pos);

    current_state.agents.push_back({member_pos, a.id()});
    goal_state.agents.push_back({a.target, a.id()});
  }

  using search_type = a_star<
    agents_state,
    state_successors,
    passable_not_immediate_neighbour,
    combined_heuristic_distance,
    space_coordinate<agents_state>,
    no_distance_storage
  >;
  search_type search(
    current_state,
    goal_state,
    w,
    combined_heuristic_distance(
      hierarchical_distances_, predictor_.get(), obstacle_penalty_
    ),
    passable_not_immediate_neighbour{current_state, predictor_.get()}
  );

  path<agents_state> result;
  if (window_)
    result = search.find_path_to_goal_or_window(
      w, (unsigned) (window_ * group.starting_positions.size())
    );
  else
    result = search.find_path(w);

  assert(result.empty() || result.back().next_agent == 0);

  nodes_primary_ += search.nodes_expanded();

  result.erase(std::remove_if(result.begin(), result.end(),
                              [] (agents_state const& state) {
                                return state.next_agent != 0;
                              }),
               result.end());

#ifndef NDEBUG
  for (agents_state const& state : result) {
    assert(state.agents.size() == group.starting_positions.size());
    for (agent_state_record const& agent_a : state.agents)
      for (agent_state_record const& agent_b : state.agents)
        assert(agent_a.id == agent_b.id ||
               agent_a.position != agent_b.position);
  }
#endif

  return result;
}

auto
operator_decomposition::plans_admissible(world const& w) const -> admissibility {
  admissibility result = admissibility::admissible;

  for (group const& group : groups_) {
    if (group.plan.size() < 2) {
      if (group.plan.empty() || !final(group.plan.front(), w)) {
        result = std::max(result, admissibility::incomplete);
        continue;
      }
      else
        continue;
    }

    plan::const_iterator next_state = std::prev(std::prev(group.plan.end()));
    for (agent_state_record const& agent : next_state->agents)
      if (w.get(agent.position) == tile::obstacle)
        return admissibility::invalid;
  }

  return result;
}

bool
operator_decomposition::final(agents_state const& state, world const& w) const {
  for (agent_state_record const& agent : state.agents) {
    assert(w.get_agent(agent.position));
    if (w.get_agent(agent.position)->target != agent.position)
      return false;
  }

  return true;
}

void
operator_decomposition::merge_groups(std::vector<group_id> const& groups) {
  assert(!groups.empty());
  group_id target = groups.front();
  unreserve(target);
  target->plan = {};

  for (auto g = std::next(groups.begin()); g != groups.end(); ++g) {
    unreserve(*g);

    target->starting_positions.insert(target->starting_positions.end(),
                                      (**g).starting_positions.begin(),
                                      (**g).starting_positions.end());
    groups_.erase(*g);
  }
}

void
operator_decomposition::reserve(plan const& plan, group_id group,
                                tick_t start) {
  tick_t time = start;
  for (auto state = plan.rbegin(); state != plan.rend(); ++state) {
    for (std::size_t i = 0; i < state->agents.size(); ++i) {
      boost::optional<position> from;

      if (state != plan.rbegin()) {
        assert(std::prev(state)->agents[i].id == state->agents[i].id);
        from = std::prev(state)->agents[i].position;
      }

      reservation_table_[{state->agents[i].position, time}] = {group, from};
    }

    ++time;
  }
}

void
operator_decomposition::unreserve(group_id group) {
  for (auto it = reservation_table_.begin(); it != reservation_table_.end();)
    if (it->second.group == group)
      it = reservation_table_.erase(it);
    else
      ++it;
}

void
operator_decomposition::make_hierarchical_distances(world const& w) {
  for (auto const& pos_agent : w.agents()) {
    position from = std::get<0>(pos_agent);
    agent const& a = std::get<1>(pos_agent);

    hierarchical_distances_.emplace(
      std::piecewise_construct,
      std::forward_as_tuple(a.id()),
      std::forward_as_tuple(a.target, from, w)
    );
  }
}
