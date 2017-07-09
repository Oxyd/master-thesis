#ifndef ACTION_HPP
#define ACTION_HPP

#include "world.hpp"

#include <boost/optional.hpp>

#include <iostream>
#include <unordered_map>

// Action of a single agent.
class action {
public:
  action(position p, direction d)
    : from_(p)
    , where_(d)
  { }

  position  from() const  { return from_; }
  direction where() const { return where_; }

private:
  position from_;
  direction where_;
};

std::ostream&
operator << (std::ostream&, action);

// Can the given action be applied to the world?
bool
valid(action, world const&);

// Apply the action to the world and return the world that results from the
// application. That action has to be valid for the world.
world
apply(action, world);

// Simultaneous action of many agents. This is not a mere sequence of single
// actions -- a joint_action can contain actions that will put an agent at an
// occupied position, provided the agent at the destination also has an action
// that will move it away.
//
// This type only represents actions that cause the agents to change position --
// i.e. the stay action is not represented here.
class joint_action {
public:
  void
  add(action a);

  // Can this action be extended by appending the other action? This checks
  // whether there is at most one action for each agent and whether two agents
  // swap positions. This does not guarantee that the resulting action will be
  // valid: Since stay actions aren't represented here, it is possible that the
  // result will be an action that places an agent at a position occupied by
  // another agent who doesn't make a move in this joint action.
  bool
  can_extend(joint_action const& other) const;

  // Append other action to this action.
  void
  extend(joint_action const& other);

  // Get the action for the agent currently at the given position. Returns none
  // if there either is no such agent or if the agent has no action assigned.
  boost::optional<action>
  action_for(position) const;

  void
  show(std::ostream&) const;

private:
  std::unordered_map<position, direction> actions_;
};

std::ostream&
operator << (std::ostream&, joint_action const&);

// Apply the joint action to the world and return the result of the application.
world
apply(joint_action const&, world const&);

#endif // ACTION_HPP

