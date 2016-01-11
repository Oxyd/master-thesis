#ifndef A_STAR_HPP
#define A_STAR_HPP

#include "world.hpp"

#include <boost/heap/fibonacci_heap.hpp>

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

using path = std::vector<direction>;

constexpr unsigned
infinity = std::numeric_limits<unsigned>::max();

template <typename Passable = always_passable,
          typename Distance = distance_heuristic>
class a_star {
public:
  a_star(position from, position to, world const& w,
         Passable passable = Passable{});
  a_star(position from, position to, world const& w,
         Distance distance,
         Passable passable = Passable{});
  path find_path(world const& w);
  unsigned find_distance(position p, world const& w);

  unsigned nodes_expanded() const { return expanded_; }

private:
  struct node {
    position pos;
    unsigned g = infinity;
    unsigned h = 0;
    double f() const { return g + h; }
  };

  struct node_comparator {
    bool
    operator () (node x, node y) const {
      return x.f() > y.f();
    }
  };

  using heap_type = boost::heap::fibonacci_heap<
    node, boost::heap::compare<node_comparator>
  >;
  using handle_type = typename heap_type::handle_type;

  position from_;
  position to_;
  heap_type heap_;
  unsigned expanded_ = 0;
  std::unordered_map<position, handle_type> open_;
  std::unordered_map<position, unsigned> closed_;
  std::unordered_map<position, position> come_from_;
  Passable passable_;
  Distance distance_;

  void expand_until(position p, world const& w);
};

template <typename Passable, typename Distance>
a_star<Passable, Distance>::a_star(position from, position to,
                                   world const& w,
                                   Passable passable)
  : a_star(from, to, w, Distance{to}, passable)
{ }

template <typename Passable, typename Distance>
a_star<Passable, Distance>::a_star(position from, position to,
                                   world const& w,
                                   Distance distance, Passable passable)
  : from_(from)
  , to_(to)
  , passable_(std::move(passable))
  , distance_(std::move(distance))
{
  handle_type h = heap_.push({from, 0, distance_(from, w)});
  open_.insert({from, h});
}

template <typename Passable, typename Distance>
path
a_star<Passable, Distance>::find_path(world const& w) {
  expand_until(to_, w);

  if (!closed_.count(to_))
    return path{};

  path result;
  position current = to_;

  while (current != from_) {
    position previous = come_from_[current];
    result.push_back(direction_to(previous, current));
    current = previous;
  }

  return result;
}

template <typename Passable, typename Distance>
unsigned
a_star<Passable, Distance>::find_distance(position p, world const& w) {
  auto it = closed_.find(p);
  if (it != closed_.end())
    return it->second;

  expand_until(p, w);

  it = closed_.find(p);
  if (it != closed_.end())
    return it->second;
  else
    return infinity;
}

template <typename Passable, typename Distance>
void
a_star<Passable, Distance>::expand_until(position p, world const& w) {
  while (!heap_.empty()) {
    node current = heap_.top();

    assert(open_.count(current.pos));
    assert(!closed_.count(current.pos));

    heap_.pop();
    open_.erase(current.pos);
    closed_.insert({current.pos, current.g});

    if (current.pos == p)
      return;

    ++expanded_;

    for (direction d : all_directions) {
      position const neighbour = translate(current.pos, d);
      if (!in_bounds(neighbour, *w.map()) ||
          w.get(neighbour) == tile::wall)
        continue;

      if (closed_.count(neighbour))
        continue;

      if (!passable_(neighbour, w, current.g + 1))
        continue;

      auto n = open_.find(neighbour);
      if (n != open_.end()) {
        handle_type neighbour_handle = n->second;
        if ((*neighbour_handle).g > current.g + 1) {
          (*neighbour_handle).g = current.g + 1;
          come_from_[neighbour] = current.pos;
          heap_.decrease(neighbour_handle);
        }

      } else {
        handle_type h = heap_.push(
          {neighbour, current.g + 1, distance_(neighbour, w)}
        );
        come_from_[neighbour] = current.pos;
        open_.insert({neighbour, h});
      }
    }
  }
}

#endif
