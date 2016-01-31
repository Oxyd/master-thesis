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
  constexpr bool operator () (position, position,
                              world const&, unsigned) const {
    return true;
  }
};

struct distance_heuristic {
  position destination;

  unsigned operator () (position from, world const&) const {
    return distance(from, destination);
  }
};

struct space_coordinate {
  using type = position;

  static type
  make(position p, unsigned) { return p; }
};

struct space_time_coordinate {
  using type = position_time;

  static type
  make(position p, unsigned g) { return position_time{p, g}; }
};

using path = std::vector<position>;

constexpr unsigned
infinity = std::numeric_limits<unsigned>::max();

template <typename Passable = always_passable,
          typename Distance = distance_heuristic,
          typename Coordinate = space_coordinate>
class a_star {
public:
  a_star(position from, position to, world const& w,
         Passable passable = Passable{})
    : a_star(from, to, w, Distance{to}, passable) { }

  a_star(position from, position to, world const& w,
         Distance distance,
         Passable passable = Passable{})
    : from_(from)
    , to_(to)
    , passable_(std::move(passable))
    , distance_(std::move(distance))
  {
    node* start = node_pool_.construct(from, 0, distance_(from, w));
    handle_type h = heap_.push(start);
    open_.insert({Coordinate::make(from, 0), h});
  }

  a_star(a_star const&) = delete;
  void operator = (a_star const&) = delete;

  a_star(a_star&&) = default;
  a_star& operator = (a_star&&) = default;

  path
  find_path(world const& w, boost::optional<unsigned> window = {}) {
    path result;

    node* current;
    if (!window)
      current = expand_until(to_, w);
    else
      current = expand_until({}, w, window);

    if (!current)
      return {};

    do {
      result.push_back(current->pos);
      current = current->come_from;
    } while (current);

    return result;
  }

  unsigned
  find_distance(position p, world const& w) {
    auto it = shortest_paths_.find(p);
    if (it != shortest_paths_.end())
      return it->second->g;

    expand_until(p, w);

    it = shortest_paths_.find(p);
    if (it != shortest_paths_.end())
      return it->second->g;
    else
      return infinity;
  }

  unsigned nodes_expanded() const { return expanded_; }

private:
  struct node {
    position pos;
    unsigned g;
    unsigned h;

    node* come_from = nullptr;

    node(position pos, unsigned g, unsigned h)
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

  position from_;
  position to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  pool_type node_pool_;
  std::unordered_map<coordinate_type, handle_type> open_;
  std::unordered_set<coordinate_type> closed_;
  std::unordered_map<position, node*> shortest_paths_;
  Passable passable_;
  Distance distance_;

  node*
  expand_until(boost::optional<position> goal, world const& w,
               boost::optional<unsigned> window = {}) {
    while (!heap_.empty()) {
      node* const current = heap_.top();
      coordinate_type const current_coord =
        Coordinate::make(current->pos, current->g);

      assert(open_.count(current_coord));
      assert(!closed_.count(current_coord));

      heap_.pop();
      open_.erase(current_coord);
      closed_.insert({current_coord});

      shortest_paths_.insert({current->pos, current});

      ++expanded_;

      std::vector<position> neighbours;
      for (direction d : all_directions) {
        position const n = translate(current->pos, d);
        if (in_bounds(n, *w.map()) && w.get(n) != tile::wall)
          neighbours.push_back(n);
      }

      if (Coordinate::make(current->pos, current->g + 1) != current_coord)
        neighbours.push_back(current->pos);

      for (position neighbour : neighbours) {
        coordinate_type const neighbour_coord =
          Coordinate::make(neighbour, current->g + 1);

        if (!in_bounds(neighbour, *w.map()) ||
            w.get(neighbour) == tile::wall)
          continue;

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
          node* neighbour_node = node_pool_.construct(neighbour, current->g + 1,
                                                      distance_(neighbour, w));
          handle_type h = heap_.push(neighbour_node);
          neighbour_node->come_from = current;
          open_.insert({neighbour_coord, h});
        }
      }

      if ((goal && current->pos == *goal) || (window && current->g == *window))
        return current;
    }

    return nullptr;
  }
};

#endif
