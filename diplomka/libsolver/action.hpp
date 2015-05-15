#ifndef ACTION_HPP
#define ACTION_HPP

#include "world.hpp"

#include <boost/optional.hpp>

#include <iostream>
#include <unordered_map>

enum class direction {
  north = 0, east, south, west
};

position
translate(position, direction);

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

bool
valid(action, world const&);

world
apply(action, world);

// Simultaneous action of many agents.
class joint_action {
public:
  void
  add(action a);

  boost::optional<action>
  action_for(position) const;

  void
  show(std::ostream&) const;

private:
  std::unordered_map<position, direction> actions_;
};

std::ostream&
operator << (std::ostream&, joint_action const&);

world
apply(joint_action const&, world const&);

#endif // ACTION_HPP

