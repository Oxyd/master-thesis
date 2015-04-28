#include "map.hpp"

#include <fstream>
#include <string>

static void
expect_word(std::ifstream& in, std::string word) {
  using namespace std::string_literals;

  std::string buffer;
  in >> buffer;
  if (!in || buffer != word)
    throw map_format_error{"Expected "s + word};
}

static map::coord_type
expect_dim(std::ifstream& in) {
  map::coord_type x;
  if (!(in >> x))
    throw map_format_error{"Expected dimension"};
  return x;
}

static bool
is_tile_char(char c) {
  return
      c == '.' || c == 'G' || c == '@' || c == 'O' || c == 'T' ||
      c == 'S' || c == 'W';
}

static tile
char_to_tile(char c) {
  switch (c) {
  case '.': case 'G': return tile::passable;
  case '@': case 'O': return tile::out_of_bounds;
  case 'T': return tile::tree;
  case 'S': return tile::swamp;
  case 'W': return tile::water;
  default: throw std::logic_error{"Not a valid tile char"};
  }
}

map
load(std::string const& filename) {
  using namespace std::string_literals;

  std::ifstream in{filename};
  std::string buffer;

  if (!in)
    throw map_format_error{"Could not open file"};

  if (!std::getline(in, buffer) || buffer != "type octile")
    throw map_format_error{"Expected 'type octile'"};

  expect_word(in, "height");
  map::coord_type const height = expect_dim(in);
  expect_word(in, "width");
  map::coord_type const width = expect_dim(in);

  expect_word(in, "map");

  map result(width, height);

  map::coord_type i = 0;
  map::coord_type max = width * height;

  char c;
  while (in && (c = in.get()) != std::ifstream::traits_type::eof()) {
    if (c == '\n')
      continue;

    if (!is_tile_char(c))
      throw map_format_error{"Not a valid tile character: "s + c};

    if (i >= max)
      throw map_format_error{"Too many tiles"};

    result.put(i % width, i / width, char_to_tile(c));
    ++i;
  }

  return result;
}
