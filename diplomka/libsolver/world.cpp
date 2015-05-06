#include "world.hpp"

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>

#include <cassert>
#include <fstream>
#include <sstream>
#include <string>

static void
expect_word(std::istream& in, std::string word) {
  using namespace std::string_literals;

  std::string buffer;
  in >> buffer;
  if (!in || buffer != word)
    throw bad_world_format{"Expected "s + word};
}

static unsigned
expect_num(std::istream& in, std::string name = "number") {
  using namespace std::string_literals;

  unsigned x;
  if (!(in >> x))
    throw bad_world_format{"Expected "s + name};
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

bool
traversable(tile t) {
  switch (t) {
  case tile::passable:
  case tile::swamp:
  case tile::water:
    return true;

  default:
    return false;
  }
}

world::world(::map m)
  : map_(m)
{ }

boost::optional<agent>
world::get_agent(position p) const {
  auto const a = agents_.find(p);
  if (a != agents_.end())
    return a->second;
  else
    return {};
}

void
world::put_agent(position p, agent a) {
  if (agents_.find(p) != agents_.end())
    throw std::logic_error{"put_agent: Position not empty"};

  agents_.insert({p, a});
}

void
world::remove_agent(position p) {
  auto const a = agents_.find(p);
  if (a == agents_.end())
    throw std::logic_error{"remove_agent: Tile empty"};
  agents_.erase(a);
}

static map
load_map(std::string const& filename) {
  using namespace std::string_literals;

  std::ifstream in{filename};

  if (!in)
    throw bad_world_format{"Could not open "s + filename};

  std::string buffer;
  if (!std::getline(in, buffer) || buffer != "type octile")
    throw bad_world_format{"Expected 'type octile'"};

  expect_word(in, "height");
  map::coord_type const height = expect_num(in, "map height");
  expect_word(in, "width");
  map::coord_type const width = expect_num(in, "map width");

  expect_word(in, "map");

  map result(width, height);

  map::coord_type i = 0;
  map::coord_type max = width * height;

  char c;
  while (in && (c = in.get()) != std::ifstream::traits_type::eof()) {
    if (c == '\n')
      continue;

    if (!is_tile_char(c))
      throw bad_world_format{"Not a valid tile character: "s + c};

    if (i >= max)
      throw bad_world_format{"Too many tiles"};

    result.put(i % width, i / width, char_to_tile(c));
    ++i;
  }

  return result;
}

world
load_world(std::string const& filename) {
  using namespace std::string_literals;

  std::ifstream in{filename};
  if (!in)
    throw bad_world_format{"Could not open "s + filename};

  expect_word(in, "map");
  std::string map_filename;
  if (!std::getline(in, map_filename))
    throw bad_world_format{"Expected map filename"};

  using boost::filesystem::path;
  using boost::algorithm::trim_copy;

  path const map_path = path{filename}.parent_path() / trim_copy(map_filename);
  world world{load_map(map_path.string())};

  std::string line_buffer;
  while (std::getline(in, line_buffer)) {
    std::istringstream line{line_buffer};
    expect_word(line, "agent");

    boost::optional<position> pos;
    boost::optional<position> target;
    boost::optional<team_type> team;

    std::string keyword;
    while (line >> keyword) {
      if (keyword == "position"s || keyword == "goal"s) {
        position::coord_type const x = expect_num(line, "X coordinate");
        position::coord_type const y = expect_num(line, "Y coordinate");

        if (x >= world.map().width())
          throw bad_world_format{"X coordinate out of bounds"};
        if (y >= world.map().height())
          throw bad_world_format{"Y coordinate out of bounds"};

        if (keyword == "position"s) {
          if (pos)
            throw bad_world_format{"Duplicate position"};
          pos = position{x, y};
        } else {
          if (target)
            throw bad_world_format{"Duplicate goal"};
          target = position{x, y};
        }
      } else if (keyword == "team"s) {
        if (team)
          throw bad_world_format{"Duplicate team number"};
        unsigned const team_num = expect_num(line, "team number");
        switch (team_num) {
        case 0: team = team_type::attacker; break;
        case 1: team = team_type::defender; break;
        default:
          throw bad_world_format{"Bad team number"};
        }
      } else {
        throw bad_world_format{"Unrecognised keyword "s + keyword};
      }
    }

    if (!pos)
      throw bad_world_format{"Missing position"};
    if (!team)
      throw bad_world_format{"Missing team number"};

    world.put_agent(*pos, agent{target, *team});
  }

  return world;
}
