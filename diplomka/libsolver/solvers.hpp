#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "action.hpp"
#include "world.hpp"

#include <random>

bool
solved(world const& w);

joint_action
greedy_action(world w, team_type team, std::default_random_engine&);

#endif // SOLVERS_HPP
