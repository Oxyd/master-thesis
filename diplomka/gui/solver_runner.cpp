#include "solver_runner.hpp"

#include "solvers.hpp"
#include "world.hpp"

#include <QMutexLocker>

solver_runner::solver_runner(world w, solver& s,
                             std::default_random_engine& rng)
  : world_{std::move(w)}
  , solver_{s}
  , rng_{rng}
{ }

void
solver_runner::interrupt() {
  solver_.kill();
}

world
solver_runner::move_result() {
  QMutexLocker lock{&mutex_};

  return std::move(world_);
}

void
solver_runner::run() {
  QMutexLocker lock{&mutex_};

  world_.next_tick(rng_);
  solver_.step(world_, rng_);
}
