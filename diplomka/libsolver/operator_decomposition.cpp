#include "operator_decomposition.hpp"

#include "predictor.hpp"

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

  for (direction d : all_directions) {
    position const destination = translate(agent.position, d);
    if (!in_bounds(destination, *w.map()) || w.get(destination) == tile::wall)
      continue;

    bool possible = true;

    for (agent_state_record const& other_agent : state.agents) {
      if (other_agent.action == agent_action::unassigned)
        break;

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

  bool needs_vacate = false;
  for (agent_state_record const& other_agent : state.agents) {
    if (other_agent.action == agent_action::unassigned)
      break;

    if (other_agent.position == agent.position) {
      needs_vacate = true;
      break;
    }
  }

  if (!needs_vacate)
    add(agent_action::stay, agent.position);

  return result;
}

operator_decomposition::
combined_heuristic_distance::combined_heuristic_distance(
  std::unordered_map<agent::id_type, a_star<>>& h_searches
)
  : h_searches_(h_searches)
{}

unsigned
operator_decomposition::combined_heuristic_distance::operator () (
  agents_state const& state, world const& w
) const {
  unsigned result = 0;
  for (agent_state_record const& agent : state.agents) {
    auto h_search = h_searches_.find(agent.id);
    assert(h_search != h_searches_.end());

    result += h_search->second.find_distance(agent.position, w);
  }

  return result;
}

double
operator_decomposition::predicted_step_cost::operator () (
  agents_state const& from,
  agents_state const& to,
  tick_t time
) const {
  if (!predictor_
      || from.agents[from.next_agent].position
         == to.agents[from.next_agent].position)
    return 1.0;

  return
    1.0
    + predictor_->predict_obstacle({to.agents[from.next_agent].position,
                                    start_time_ + time})
    * obstacle_penalty_;
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

std::vector<position>
operator_decomposition::get_path(agent::id_type agent_id) const {
  std::vector<position> result;

  for (group const& g : groups_)
    for (agents_state const& state : g.plan)
      for (agent_state_record const& agent : state.agents)
        if (agent.id == agent_id)
          result.push_back(agent.position);

  return result;
}

std::unordered_map<position_time, double>
operator_decomposition::get_obstacle_field() const {
  if (predictor_)
    return predictor_->field();
  else
    return {};
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
  permanent_reservation_table_.clear();
  last_nonpermanent_reservation_ = 0;

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

        boost::optional<group_id> conflicting_group = find_conflict(
          agent.position,
          state != group->plan.rbegin()
            ? boost::optional<position>{std::prev(state)->agents[i].position}
            : boost::none,
          time
        );
        assert(!conflicting_group || group != *conflicting_group);

        if (!conflicting_group && std::next(state) == group->plan.rend())
          conflicting_group = find_permanent_conflict(agent.position, time);

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
  max_group_size_ = std::max(max_group_size_,
                             (unsigned) group.starting_positions.size());

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
    predicted_step_cost,
    space_coordinate<agents_state>,
    no_distance_storage
  >;
  search_type search(
    current_state,
    goal_state,
    w,
    combined_heuristic_distance(hierarchical_distances_),
    predicted_step_cost(predictor_.get(), obstacle_penalty_, w.tick()),
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
      last_nonpermanent_reservation_ =
        std::max(time, last_nonpermanent_reservation_);
    }

    ++time;
  }

  if (plan.empty())
    return;

  agents_state const& final_state = plan.front();
  for (agent_state_record const& agent : final_state.agents) {
    assert(!permanent_reservation_table_.count(agent.position));
    permanent_reservation_table_[agent.position] = {group, time};
  }
}

void
operator_decomposition::unreserve(group_id group) {
  auto do_it = [&] (auto& table) {
    for (auto it = table.begin(); it != table.end();)
      if (it->second.group == group)
        it = table.erase(it);
      else
        ++it;
  };

  do_it(reservation_table_);
  do_it(permanent_reservation_table_);
}

auto
operator_decomposition::find_conflict(position to,
                                      boost::optional<position> from,
                                      tick_t time) const
  -> boost::optional<group_id>
{
  boost::optional<group_id> conflicting_group;

  auto conflict = reservation_table_.find({to, time});
  if (conflict != reservation_table_.end())
    conflicting_group = conflict->second.group;

  if (!conflicting_group && from) {
    auto vacated = reservation_table_.find({*from, time});
    if (vacated != reservation_table_.end() &&
        vacated->second.from &&
        *vacated->second.from == to)
      conflicting_group = vacated->second.group;
  }

  if (!conflicting_group) {
    auto permanent_conflict =
      permanent_reservation_table_.find(to);
    if (permanent_conflict != permanent_reservation_table_.end() &&
        permanent_conflict->second.from_time <= time)
      conflicting_group = permanent_conflict->second.group;
  }

  return conflicting_group;
}

auto
operator_decomposition::find_permanent_conflict(position pos,
                                                tick_t since) const
  -> boost::optional<group_id>
{
  for (tick_t t = since; t < last_nonpermanent_reservation_; ++t) {
    auto conflict = reservation_table_.find({pos, t});
    if (conflict != reservation_table_.end())
      return conflict->second.group;
  }

  return boost::none;
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
