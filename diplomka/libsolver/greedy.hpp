#ifndef GREEDY_HPP
#define GREEDY_HPP

#include <random>
#include <string>

#include "solvers.hpp"

class greedy : public solver {
public:
  void step(world& w, std::default_random_engine&) override;
  std::string name() const override { return "Greedy"; }
};

#endif
