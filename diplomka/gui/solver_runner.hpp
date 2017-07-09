#ifndef SOLVER_RUNNER_HPP
#define SOLVER_RUNNER_HPP

#include "world.hpp"

#include <QMutex>
#include <QThread>

#include <random>
#include <utility>

class solver;

// Runs solver::step in a separate thread. Wrapping this in a QThread allows the
// caller to listen for the finished signal.
class solver_runner : public QThread {
  Q_OBJECT

public:
  solver_runner(world, solver&, std::default_random_engine&);

  void interrupt();
  world move_result();

protected:
  void run() override;

private:
  world world_;
  solver& solver_;
  std::default_random_engine& rng_;
  QMutex mutex_;
};

#endif
