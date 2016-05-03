#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "world.hpp"

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/optional.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool.hpp>

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

  double operator () (position from, world const&, unsigned) const {
    return distance(from, destination);
  }
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

constexpr unsigned
infinity = std::numeric_limits<unsigned>::max();

template <
  typename State = position,
  typename SuccessorsFunc = position_successors,
  typename Passable = always_passable,
  typename Distance = manhattan_distance_heuristic,
  typename Coordinate = space_coordinate<State>,
  template <typename, typename> class DistanceStorage =
    position_distance_storage
>
class a_star {
public:
  a_star(State const& from, State const& to, world const& w,
         Passable passable = Passable{})
    : a_star(from, to, w, Distance{to}, passable) { }

  a_star(State const& from, State const& to, world const& w,
         Distance distance,
         Passable passable = Passable{})
    : from_(from)
    , to_(to)
    , passable_(std::move(passable))
    , distance_(std::move(distance))
  {
    node* start = node_pool_.construct(from, 0, distance_(from, w, 0));
    handle_type h = heap_.push(start);
    open_.insert({Coordinate::make(from, 0), h});
  }

  a_star(a_star const&) = delete;
  void operator = (a_star const&) = delete;

  a_star(a_star&&) = default;
  a_star& operator = (a_star&&) = default;

  path<State>
  find_path(world const& w) {
    return do_find_path(w, [&] (node const* n) { return n->pos == to_; });
  }

  path<State>
  find_path(world const& w, unsigned window) {
    return do_find_path(w, [&] (node const* n) { return n->g == window; });
  }

  template <typename PositionPred>
  path<State>
  find_path(world const& w, PositionPred goal,
            unsigned limit = std::numeric_limits<unsigned>::max()) {
    return do_find_path(w, [&] (node const* n) { return goal(n->pos); }, limit);
  }

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

private:
  struct node {
    State pos;
    unsigned g;
    double h;

    node* come_from = nullptr;

    node(State const& pos, unsigned g, double h)
      : pos(pos), g(g), h(h) { }

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

  State from_;
  State to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  pool_type node_pool_;
  std::unordered_map<coordinate_type, handle_type> open_;
  std::unordered_set<coordinate_type> closed_;
  DistanceStorage<State, node*> distance_storage_;
  Passable passable_;
  Distance distance_;

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
      node* const current = heap_.top();
      coordinate_type const current_coord =
        Coordinate::make(current->pos, current->g);

      assert(open_.count(current_coord));
      assert(!closed_.count(current_coord));

      heap_.pop();
      open_.erase(current_coord);
      closed_.insert({current_coord});

      distance_storage_.store(current->pos, current);

      ++expanded_;

      if (current->g == limit)
        return nullptr;

      std::vector<State> neighbours = SuccessorsFunc::get(current->pos, w);

      if (Coordinate::make(current->pos, current->g + 1) != current_coord)
        neighbours.push_back(current->pos);

      for (State const& neighbour : neighbours) {
        coordinate_type const neighbour_coord =
          Coordinate::make(neighbour, current->g + 1);

        if (closed_.count(neighbour_coord))
          continue;

        if (!passable_(neighbour, current->pos, w, current->g + 1))
          continue;

        auto n = open_.find(neighbour_coord);
        if (n != open_.end()) {
          handle_type neighbour_handle = n->second;
          if ((**neighbour_handle).g > current->g + 1) {
            (**neighbour_handle).g = current->g + 1;
            (**neighbour_handle).come_from = current;
            heap_.decrease(neighbour_handle);
          }

        } else {
          node* neighbour_node = node_pool_.construct(
            neighbour, current->g + 1,
            distance_(neighbour, w, current->g + 1)
          );
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
