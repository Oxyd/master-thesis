#include <algorithm>
#include <vector>

#include "greedy.hpp"

static direction
random_dir(std::default_random_engine& rng) {
  std::uniform_int_distribution<unsigned> d(0, 3);
  return static_cast<direction>(d(rng));
}

static void
make_random_action(position from, world& w, std::default_random_engine& rng) {
  direction const d = random_dir(rng);
  action const a{from, d};

  if (valid(a, w))
    w = apply(a, std::move(w));
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
