#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "world.hpp"

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/optional.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool.hpp>

#include <atomic>
#include <limits>
#include <unordered_map>
#include <unordered_set>

struct always_passable {
  template <typename State>
  constexpr bool operator () (State const&, State const&,
                              world const&, unsigned) const {
    return true;
  }
};

struct manhattan_distance_heuristic {
  position destination;

  explicit
  manhattan_distance_heuristic(position destination)
    : destination(destination)
  { }

  double operator () (position from, world const&) const {
    return distance(from, destination);
  }
};

struct unitary_step_cost {
  template <typename T>
  double
  operator () (T, T, unsigned) const { return 1.0; }
};

template <typename StateT = position>
struct space_coordinate {
  using type = StateT;

  static type
  make(StateT p, unsigned) { return p; }
};

struct space_time_coordinate {
  using type = position_time;

  static type
  make(position p, unsigned g) { return position_time{p, g}; }
};

template <typename State, typename Node>
struct no_distance_storage {
  void
  store(State, Node) { }

  void
  get() const { }
};

template <typename State, typename Node>
struct position_distance_storage {
  using storage_type = std::unordered_map<State, Node>;

  storage_type storage;

  void
  store(State s, Node n) {
    storage.insert({s, n});
  }

  storage_type const&
  get() const { return storage; }
};

struct position_successors {
  static std::vector<position>
  get(position p, world const& w) {
    std::vector<position> result;

    for (direction d : all_directions) {
      position const n = translate(p, d);
      if (in_bounds(n, *w.map()) && w.get(n) != tile::wall)
        result.push_back(n);
    }

    return result;
  }
};

template <typename Coord>
struct always_close {
  static bool
  get(Coord const&) { return true; }
};

template <typename Coord, typename Handle>
struct coord_open_set {
  using type = std::unordered_map<Coord, Handle>;
};

constexpr unsigned
infinity = std::numeric_limits<unsigned>::max();

// Implementation of the A* search algorithm. This algorithm is used in many
// variants in this work, so we have a generic implementation for all those
// variants.
//
// Template parameters:
//   * State: The state-space the algorithm will search. This can be `position`
//            for plain A* search or `agents_state` for operator decomposition.
//   * SuccessorsFunc: Policy type for getting the successors of a State. Either
//                     `position_successors` or `state_successors`.
//   * Passable: Policy type for determining whether to add a successor to the
//               heap or whether to consider it impassable. Can be used to
//               consider obstacles in the immediate neighbourhood to be
//               impassable, or to consider predicted obstacles to be
//               impassable.
//   * Distance: Distance heuristic. Can be either plain Manhattan distance,
//               agitated distance (with LRA*), heuristic distance (WHCA*) or
//               combined Manhattan distance of all agents (OD).
//   * StepCost: Cost of taking a step. Can be either unitary cost or cost that
//               depends on predictor results.
//   * Coordinate: State coordinates the algorithm will use. Can either be the
//                 State or a State-time combination, where "time" is the number
//                 of steps from the start of the search. Using a State-time
//                 coordinate means that the algorithm will consider empty moves
//                 since even though the state is the same, the time is
//                 different, so the post-move coordinate of an empty move is
//                 not the same as the pre-move coordinate.
//   * DistanceStorage: Policy for storing distances from start of search to any
//                      node in the closed set.
//   * ShouldClosePred: Whether a node should be inserted into the closed set.
//                      Used with OD to close only full states.
//   * OpenSetType: Type of the open set.
template <
  typename State = position,
  typename SuccessorsFunc = position_successors,
  typename Passable = always_passable,
  typename Distance = manhattan_distance_heuristic,
  typename StepCost = unitary_step_cost,
  typename Coordinate = space_coordinate<State>,
  template <typename, typename> class DistanceStorage =
    position_distance_storage,
  typename ShouldClosePred = always_close<typename Coordinate::type>,
  template <typename, typename> class OpenSetType =
    coord_open_set
>
class a_star {
public:
  a_star(State const& from, State const& to, world const& w,
         std::atomic<bool>& stop_flag, Passable passable = Passable{})
    : a_star(from, to, w, stop_flag, Distance{to}, StepCost{}, passable) { }

  a_star(State const& from, State const& to, world const& w,
         std::atomic<bool>& stop_flag,
         Distance distance, StepCost step_cost,
         Passable passable = Passable{})
    : from_(from)
    , to_(to)
    , passable_(std::move(passable))
    , distance_(std::move(distance))
    , step_cost_(std::move(step_cost))
    , stop_flag_{&stop_flag}
  {
    node* start = node_pool_.construct(
      node(from, 0.0, distance_(from, w), 0u)
    );
    handle_type h = heap_.push(start);
    open_.insert({Coordinate::make(from, 0), h});
  }

  a_star(a_star const&) = delete;
  void operator = (a_star const&) = delete;

  a_star(a_star&&) = default;
  a_star& operator = (a_star&&) = default;

  // Find path from the starting position to the goal position the search was
  // initialised with.
  path<State>
  find_path(world const& w) {
    return do_find_path(w, [&] (node const* n) { return n->pos == to_; });
  }

  // Find path either to the goal or to any coordinate that is `window` steps
  // away from the starting position.
  path<State>
  find_path_to_goal_or_window(world const& w, unsigned window) {
    return do_find_path(
      w, [&] (node const* n) {
        return n->pos == to_ || n->steps_distance == window;
      }
    );
  }

  // Find path to any node that is `window` steps away from the starting
  // position.
  path<State>
  find_path(world const& w, unsigned window) {
    return do_find_path(
      w, [&] (node const* n) {
        return n->steps_distance == window;
      }
    );
  }

  // Find goal to a position that satisfies the given predicate. `limit` is the
  // furthest number of steps the algorithm will search before returning the
  // empty path.
  template <typename PositionPred>
  path<State>
  find_path(world const& w, PositionPred goal,
            unsigned limit = std::numeric_limits<unsigned>::max()) {
    return do_find_path(w, [&] (node const* n) { return goal(n->pos); }, limit);
  }

  // Find distance from start to the given position. If the given position is
  // not closed, the algorithm is run until the position becomes closed.
  double
  find_distance(State const& p, world const& w) {
    auto const& shortest_paths = distance_storage_.get();

    auto it = shortest_paths.find(p);
    if (it != shortest_paths.end())
      return it->second->g;

    expand_until([&] (node const* n) { return n->pos == p; }, w);

    it = shortest_paths.find(p);
    if (it != shortest_paths.end())
      return it->second->g;
    else
      return infinity;
  }

  unsigned nodes_expanded() const { return expanded_; }

  State const& from() const { return from_; }
  State const& to() const { return to_; }

  // Call a function on each node in the closed set.
  template <typename F>
  void
  foreach_closed(F&& f) {
    for (coordinate_type const& position : closed_)
      f(position);
  }

private:
  struct node {
    State pos;
    double g; // Sum of step costs.
    double h;
    unsigned steps_distance; // Number of nodes from start to this node.

    node* come_from = nullptr;

    node(State const& pos, double g, double h, unsigned steps_distance)
      : pos(pos), g(g), h(h), steps_distance(steps_distance) { }

    double f() const { return g + h; }
  };

  struct node_comparator {
    bool
    operator () (node* x, node* y) const {
      return x->f() > y->f();
    }
  };

  using heap_type = boost::heap::fibonacci_heap<
    node*, boost::heap::compare<node_comparator>
  >;
  using handle_type = typename heap_type::handle_type;

  using pool_type =
    boost::object_pool<node, boost::default_user_allocator_new_delete>;

  using coordinate_type = typename Coordinate::type;

  using open_set_type =
    typename OpenSetType<coordinate_type, handle_type>::type;

  State from_;
  State to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  pool_type node_pool_;
  open_set_type open_;
  std::unordered_set<coordinate_type> closed_;
  DistanceStorage<State, node*> distance_storage_;
  Passable passable_;
  Distance distance_;
  StepCost step_cost_;
  std::atomic<bool>* stop_flag_;

  template <typename EndPred>
  path<State>
  do_find_path(world const& w, EndPred goal,
               unsigned limit = std::numeric_limits<unsigned>::max()) {
    path<State> result;

    node* current = expand_until(goal, w, limit);
    if (!current)
      return {};

    do {
      result.push_back(current->pos);
      current = current->come_from;
    } while (current);

    return result;
  }

  template <typename EndF>
  node*
  expand_until(EndF end, world const& w,
               unsigned limit = std::numeric_limits<unsigned>::max()) {
    while (!heap_.empty()) {
      if (stop_flag_ && *stop_flag_)
        return nullptr;

      node* const current = heap_.top();
      coordinate_type const current_coord =
        Coordinate::make(current->pos, current->steps_distance);

      assert(open_.count(current_coord));
      assert(!closed_.count(current_coord));

      heap_.pop();
      open_.erase(current_coord);

      if (ShouldClosePred::get(current_coord))
        closed_.insert({current_coord});

      distance_storage_.store(current->pos, current);

      ++expanded_;

      if (current->steps_distance == limit)
        return nullptr;

      std::vector<State> neighbours = SuccessorsFunc::get(current->pos, w);

      if (Coordinate::make(current->pos, current->steps_distance + 1)
          != current_coord)
        neighbours.push_back(current->pos);

      for (State const& neighbour : neighbours) {
        coordinate_type const neighbour_coord =
          Coordinate::make(neighbour, current->steps_distance + 1);

        if (closed_.count(neighbour_coord))
          continue;

        if (!passable_(neighbour, current->pos, w, current->steps_distance + 1))
          continue;

        double step_cost = step_cost_(current_coord, neighbour_coord,
                                      current->steps_distance + 1);
        auto n = open_.find(neighbour_coord);
        if (n != open_.end()) {
          handle_type neighbour_handle = n->second;
          if ((**neighbour_handle).g > current->g + step_cost) {
            (**neighbour_handle).g = current->g + step_cost;
            (**neighbour_handle).come_from = current;
            (**neighbour_handle).steps_distance = current->steps_distance + 1;
            heap_.decrease(neighbour_handle);
          }

        } else {
          node* neighbour_node = node_pool_.construct(node(
            neighbour, current->g + step_cost,
            distance_(neighbour, w),
            current->steps_distance + 1
          ));
          handle_type h = heap_.push(neighbour_node);
          neighbour_node->come_from = current;
          open_.insert({neighbour_coord, h});
        }
      }

      if (end(current))
        return current;
    }

    return nullptr;
  }
};

#endif
