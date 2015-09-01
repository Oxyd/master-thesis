#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "action.hpp"
#include "world.hpp"

#include <random>
#include <string>

bool
solved(world const& w);

class solver {
public:
  virtual
  ~solver() { }

  virtual joint_action
  get_action(world w, std::default_random_engine&) = 0;

  virtual std::string
  name() const = 0;
};

class greedy : public solver {
public:
  joint_action
  get_action(world w, std::default_random_engine&) override;

  std::string
  name() const override { return "Greedy"; }
};

class lra : public solver {
public:
  joint_action
  get_action(world w, std::default_random_engine& rng) override {
    return greedy_.get_action(w, rng);
  }

  std::string
  name() const override { return "LRA*"; }

private:
  greedy greedy_;
};

#endif // SOLVERS_HPP
