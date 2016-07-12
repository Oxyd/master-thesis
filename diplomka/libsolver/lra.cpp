#include "lra.hpp"

#include "a_star.hpp"
#include "predictor.hpp"

void
lra::step(world& w, std::default_random_engine& rng) {
  if (predictor_)
    predictor_->update_obstacles(w);

  separate_paths_solver::step(w, rng);
}

std::vector<std::string>
lra::stat_names() const {
  std::vector<std::string> result = separate_paths_solver::stat_names();
  result.push_back("Nodes expanded");
  return result;
}

std::vector<std::string>
lra::stat_values() const {
  std::vector<std::string> result = separate_paths_solver::stat_values();
  result.push_back(std::to_string(nodes_));
  return result;
}

std::unordered_map<position_time, double>
lra::get_obstacle_field() const {
  if (predictor_)
    return predictor_->field();
  else
    return {};
}

double
lra::agitated_distance::operator () (position from, world const&) const {
  std::uniform_real_distribution<> agit(0.0, agitation);
  return distance(from, destination) + agit(rng);
}

bool
lra::passable_if_not_predicted_obstacle::operator () (
  position where,position from, world const& w, unsigned distance
) const {
  return
    not_neighbour_(where, from, w, distance)
    && (!predictor_
        || (predictor_->predict_obstacle({where, w.tick() + distance})
            <= threshold_));
}


path<>
lra::find_path(position from, world const& w, std::default_random_engine& rng,
               boost::optional<path<> const&>) {
  assert(w.get_agent(from));
  agent const& a = *w.get_agent(from);

  if (from == a.target)
    return {};

  tick_t const recalc_interval = w.tick() - data_[a.id()].last_recalculation;
  assert(recalc_interval > 0);
  if (recalc_interval < 5)
    data_[a.id()].agitation += 5 / recalc_interval;
  else
    data_[a.id()].agitation = 0.0;

  a_star<
    position, position_successors,
    passable_if_not_predicted_obstacle, agitated_distance,
    predicted_cost
  > as(
    from, a.target, w,
    agitated_distance{a.target, data_[a.id()].agitation, rng},
    predicted_cost(predictor_.get(), w.tick(), obstacle_penalty_),
    passable_if_not_predicted_obstacle{
      from, predictor_.get(), obstacle_threshold_
    }
  );
  path<> new_path = as.find_path(w);
  nodes_ += as.nodes_expanded();

  data_[a.id()].last_recalculation = w.tick();

  return new_path;
}
