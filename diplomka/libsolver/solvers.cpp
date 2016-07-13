#include "solvers.hpp"

#include "a_star.hpp"
#include "greedy.hpp"
#include "log_sinks.hpp"
#include "lra.hpp"
#include "operator_decomposition.hpp"
#include "predictor.hpp"
#include "separate_paths_solver.hpp"
#include "whca.hpp"

#include <random>
#include <tuple>

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

std::unique_ptr<solver>
make_greedy() {
  return std::make_unique<greedy>();
}

std::unique_ptr<solver>
make_lra(log_sink& log,
         unsigned rejoin_limit,
         std::unique_ptr<predictor> predictor,
         unsigned obstacle_penalty, double obstacle_threshold) {
  return std::make_unique<lra>(log, rejoin_limit, std::move(predictor),
                               obstacle_penalty, obstacle_threshold);
}

std::unique_ptr<solver>
make_whca(log_sink& log, unsigned window, unsigned rejoin_limit,
          std::unique_ptr<predictor> predictor, unsigned obstacle_penalty,
          double obstacle_threshold) {
  return std::make_unique<whca>(
    log, window, rejoin_limit, std::move(predictor), obstacle_penalty,
    obstacle_threshold
  );
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
