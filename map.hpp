#ifndef MAP_HPP
#define MAP_HPP

#include <cstdlib>
#include <stdexcept>
#include <string>
#include <vector>

enum class tile : char {
  passable,
  out_of_bounds,
  tree,
  swamp,
  water
};

class map {
public:
  using coord_type = std::size_t;

  explicit
  map(coord_type width, coord_type height)
    : tiles_(width * height, tile::passable)
    , width_{width}
  { }

  tile
  get(coord_type x, coord_type y) const { return tiles_[y * width_ + x]; }

  coord_type
  width() const { return width_; }

  void
  put(coord_type x, coord_type y, tile t) {
    tiles_[y * width_ + x] = t;
  }

private:
  std::vector<tile> tiles_;
  coord_type width_;
};

struct map_format_error : std::runtime_error {
  map_format_error() : std::runtime_error{"Bad map file format"} { }

  explicit
  map_format_error(std::string const& e)
    : std::runtime_error{std::string{"Bad map file format: "} + e}
  { }
};

map
load(std::string const& filename);

#endif // MAP_HPP

