#ifndef MAP_HPP
#define MAP_HPP

#include <boost/functional/hash.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/optional.hpp>

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

enum class tile : bool {
  passable,
  impassable
};

bool
traversable(tile);

struct position {
  using coord_type = std::size_t;
  coord_type x, y;
};

inline bool
operator == (position lhs, position rhs) {
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline bool
operator != (position lhs, position rhs) {
  return !operator == (lhs, rhs);
}

namespace std {
template <>
struct hash<position> {
  using argument_type = position;
  using result_type = std::size_t;

  result_type
  operator () (argument_type p) const {
    std::size_t seed{};
    boost::hash_combine(seed, p.x);
    boost::hash_combine(seed, p.y);
    return seed;
  }
};
}

class map {
public:
  using coord_type = position::coord_type;

  struct value_type {
    coord_type x, y;
    tile tile;
  };

  class iterator : public boost::iterator_facade<
      iterator, value_type const, boost::random_access_traversal_tag,
      value_type const
  > {
  public:
    iterator();

  private:
    friend class boost::iterator_core_access;
    friend class map;

    coord_type i_;
    map const* map_;

    iterator(map const* m, coord_type i) : i_{i}, map_{m} { }

    value_type const
    dereference() const {
      coord_type const x = i_ % map_->width();
      coord_type const y = i_ / map_->width();
      return {x, y, map_->get(x, y)};
    }

    bool
    equal(iterator other) const {
      return map_ == other.map_ && i_ == other.i_;
    }

    void increment() { ++i_; }
    void decrement() { --i_; }
    void advance(std::size_t n) { i_ += n; }

    difference_type
    distance_to(iterator other) const { return other.i_ - i_; }
  };

  explicit
  map(coord_type width, coord_type height)
    : tiles_(width * height, tile::passable)
    , width_{width}
    , height_{height}
  { }

  tile
  get(coord_type x, coord_type y) const { return tiles_[y * width_ + x]; }

  tile
  get(position p) const { return get(p.x, p.y); }

  coord_type width() const  { return width_; }
  coord_type height() const { return height_; }

  void
  put(coord_type x, coord_type y, tile t) {
    tiles_[y * width_ + x] = t;
  }

  void
  put(position p, tile t) {
    put(p.x, p.y, t);
  }

  iterator begin() const { return {this, 0}; }
  iterator end() const   { return {this, width_ * height_}; }

private:
  std::vector<tile> tiles_;
  coord_type width_, height_;
};

enum class team_type {
  attacker, defender
};

constexpr std::size_t team_count = 2;

class agent {
public:
  agent(boost::optional<position> target, team_type team)
    : target_(target)
    , team_{team}
  { }

  boost::optional<::position>
  target() const { return target_; }

  team_type
  team() const { return team_; }

private:
  boost::optional<::position> target_;
  team_type team_;
};

class world {
  using agents_list = std::unordered_map<position, agent>;

public:
  explicit
  world(map m);

  boost::optional<agent>
  get_agent(position p) const;

  void
  put_agent(position p, agent a);  // Throws std::logic_error if p not empty.

  void
  remove_agent(position p);  // Throws std::logic_error if p empty.

  map const&
  map() const { return map_; }

  agents_list const&
  agents() const { return agents_; }

private:
  ::map map_;
  agents_list agents_;
};

struct bad_world_format : std::runtime_error {
  bad_world_format() : std::runtime_error{"Bad map file format"} { }

  explicit
  bad_world_format(std::string const& e)
    : std::runtime_error{std::string{"Bad map file format: "} + e}
  { }
};

world
load_world(std::string const& filename);

#endif // MAP_HPP

