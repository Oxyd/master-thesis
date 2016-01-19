#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "world.hpp"

#include <boost/heap/fibonacci_heap.hpp>
#include <boost/pool/object_pool.hpp>
#include <boost/pool/pool.hpp>

#include <limits>
#include <unordered_map>
#include <unordered_set>

struct always_passable {
  constexpr bool operator () (position, world const&, unsigned) const {
    return true;
  }
};

struct distance_heuristic {
  position destination;

  unsigned operator () (position from, world const&) const {
    return distance(from, destination);
  }
};

using path = std::vector<position>;

constexpr unsigned
infinity = std::numeric_limits<unsigned>::max();

template <typename Passable = always_passable,
          typename Distance = distance_heuristic>
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
    open_.insert({from, h});
  }

  a_star(a_star const&) = delete;
  void operator = (a_star const&) = delete;

  a_star(a_star&&) = default;
  a_star& operator = (a_star&&) = default;

  path
  find_path(world const& w) {
    expand_until(to_, w);

    auto final_node = closed_.find(to_);
    if (final_node == closed_.end())
      return path{};

    path result;
    node* current = final_node->second;

    do {
      result.push_back(current->pos);
      current = current->come_from;
    } while (current);

    return result;
  }

  unsigned
  find_distance(position p, world const& w) {
    auto it = closed_.find(p);
    if (it != closed_.end())
      return it->second->g;

    expand_until(p, w);

    it = closed_.find(p);
    if (it != closed_.end())
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

  position from_;
  position to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  pool_type node_pool_;
  std::unordered_map<position, handle_type> open_;
  std::unordered_map<position, node*> closed_;
  Passable passable_;
  Distance distance_;

  void
  expand_until(position p, world const& w) {
    while (!heap_.empty()) {
      node* current = heap_.top();

      assert(open_.count(current->pos));
      assert(!closed_.count(current->pos));

      heap_.pop();
      open_.erase(current->pos);
      closed_.insert({current->pos, current});

      if (current->pos == p)
        return;

      ++expanded_;

      for (direction d : all_directions) {
        position const neighbour = translate(current->pos, d);
        if (!in_bounds(neighbour, *w.map()) ||
            w.get(neighbour) == tile::wall)
          continue;

        if (closed_.count(neighbour))
          continue;

        if (!passable_(neighbour, w, current->g + 1))
          continue;

        auto n = open_.find(neighbour);
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
          open_.insert({neighbour, h});
        }
      }
    }
  }
};

#endif
