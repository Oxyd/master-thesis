#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "action.hpp"
#include "world.hpp"

#include <random>
#include <stack>
#include <string>
#include <unordered_map>

class log_sink;

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
  using path = std::stack<direction>;

public:
  lra();

  explicit
  lra(log_sink& log);

  joint_action
  get_action(world w, std::default_random_engine& rng) override;

  std::string
  name() const override { return "LRA*"; }

private:
  std::unordered_map<position, path> paths_;
  log_sink& log_;

  path
  recalculate(position, world const&);
};

#endif // SOLVERS_HPP
