#include "action.hpp"

#include <cassert>

position
translate(position p, direction d) {
  switch (d) {
  case direction::north: return {p.x, p.y - 1};
  case direction::east:  return {p.x + 1, p.y};
  case direction::south: return {p.x, p.y + 1};
  case direction::west:  return {p.x - 1, p.y};
  }
}

bool
valid(action a, world const& w) {
  if (!w.get_agent(a.from()))
    return false;

  position const dest = translate(a.from(), a.where());
  if (!in_bounds(dest, *w.map()))
    return false;

  if (w.get_agent(dest))
    return false;

  if (!traversable(w.map()->get(dest)))
    return false;

  return true;
}

world
apply(action a, world w) {
  assert(valid(a, w));

  agent agent = *w.get_agent(a.from());
  w.remove_agent(a.from());
  w.put_agent(translate(a.from(), a.where()), agent);

  return w;
}

void
joint_action::add(action a) {
  assert(actions_.find(a.from()) == actions_.end());
  actions_.insert({a.from(), a.where()});
}

boost::optional<action>
joint_action::action_for(position p) const {
  auto it = actions_.find(p);
  if (it != actions_.end())
    return action{p, it->second};
  else
    return {};
}

world
apply(joint_action const& a, world const& w) {
  world result{w.map()};

  for (auto const& pos_agent : w.agents()) {
    position const p = pos_agent.first;
    agent const& agent = pos_agent.second;

    if (auto action = a.action_for(p)) {
      position const d = translate(p, action->where());
      result.put_agent(d, agent);
    } else {
      result.put_agent(p, agent);
    }
  }

  return result;
}
