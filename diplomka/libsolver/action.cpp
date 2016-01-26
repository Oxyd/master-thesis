#include "action.hpp"

#include <cassert>

std::ostream&
operator << (std::ostream& out, action a) {
  return out << "{" << a.from() << " -> "
             << translate(a.from(), a.where()) << "}";
}

bool
valid(action a, world const& w) {
  if (!w.get_agent(a.from()))
    return false;

  position const dest = translate(a.from(), a.where());
  if (!in_bounds(dest, *w.map()))
    return false;

  return w.get(dest) == tile::free;
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

void
joint_action::show(std::ostream& out) const {
  out << "{";
  bool first = true;
  for (auto const pd : actions_) {
    if (!first)
      out << ", ";
    out << action{pd.first, pd.second};
    first = false;
  }
}

std::ostream&
operator << (std::ostream& out, joint_action const& a) {
  a.show(out);
  return out;
}

world
apply(joint_action const& a, world const& w) {
  world result{w.map(), w.obstacle_settings(), w.agent_settings(),
               w.obstacles(), w.tick()};

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
