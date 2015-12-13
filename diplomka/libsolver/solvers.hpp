#ifndef SOLVERS_HPP
#define SOLVERS_HPP

#include "action.hpp"
#include "world.hpp"

#include <random>
#include <stack>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

class log_sink;

bool
solved(world const& w);

class solver {
public:
  virtual
  ~solver() { }

  virtual joint_action get_action(world w, std::default_random_engine&) = 0;
  virtual std::string name() const = 0;
  virtual std::vector<std::string> stat_names() const { return {}; }
  virtual std::vector<std::string> stat_values() const { return {}; };
};

class greedy : public solver {
public:
  joint_action get_action(world w, std::default_random_engine&) override;
  std::string name() const override { return "Greedy"; }
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

  std::vector<std::string>
  stat_names() const override {
    return {"Times without path", "Recalculations"};
  }

  std::vector<std::string>
  stat_values() const override;

private:
  std::unordered_map<position, path> paths_;
  log_sink& log_;
  unsigned times_without_path_ = 0;
  unsigned recalculations_ = 0;

  path
  recalculate(position, world const&);
};

#endif // SOLVERS_HPP
