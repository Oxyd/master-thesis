#include "world.hpp"

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

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
  case '.': case 'G':
    return tile::free;
  case '@': case 'O': case 'T': case 'S': case 'W':
    return tile::wall;
  default:
    throw std::logic_error{"Not a valid tile char"};
  }
}

bool
traversable(tile t) {
  return t == tile::free;
}

std::ostream&
operator << (std::ostream& out, position p) {
  return out << "[" << p.x << ", " << p.y << "]";
}

bool
in_bounds(position p, map const& m) {
  return p.x >= 0 && p.y >= 0
      && p.x < m.width() && p.y < m.height();
}

world::world(const std::shared_ptr<::map const>& m,
             obstacle_list obstacles, tick_t tick)
  : map_(m)
  , obstacles_(std::move(obstacles))
  , tick_(tick)
{ }

static void
flip(obstacle& o, tick_t tick, std::default_random_engine& rng) {
  o.active = !o.active;
  o.next_change = tick + std::max(tick_t{1}, (tick_t) o.duration(rng));
}

void
world::next_tick(std::default_random_engine& rng) {
  ++tick_;

  for (auto& pos_obst : obstacles_) {
    position const& pos = pos_obst.first;
    obstacle& obstacle = pos_obst.second;

    if (obstacle.next_change == tick_ && (obstacle.active || !get_agent(pos)))
      flip(obstacle, tick_, rng);
  }
}

tile
world::get(position p) const {
  if (agents_.count(p))
    return tile::agent;
  else {
    auto it = obstacles_.find(p);
    if (it != obstacles_.end() && it->second.active)
      return tile::obstacle;
  }

  return map_->get(p);
}

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
  if (get(p) != tile::free)
    throw std::logic_error{"put_agent: Position not empty"};

  agents_.insert({p, a});
}

void
world::remove_agent(position p) {
  auto const a = agents_.find(p);
  if (a == agents_.end())
    throw std::logic_error{"remove_agent: Agent not found"};
  agents_.erase(a);
}

void
world::put_obstacle(position p, obstacle o) {
  if (obstacles_.count(p))
    throw std::logic_error{"put_obstacle: obstacle already there"};
  obstacles_.insert({p, o});
}

void
world::remove_obstacle(position p) {
  auto const it = obstacles_.find(p);
  if (it == obstacles_.end())
    throw std::logic_error{"remove_obstacle: Obstacle not found"};
  obstacles_.erase(it);
}

static std::shared_ptr<map>
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

  auto result = std::make_shared<map>(width, height);

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

    result->put(i % width, i / width, char_to_tile(c));
    ++i;
  }

  return result;
}

static position
read_pos(boost::property_tree::ptree const& tree) {
  if (tree.count("") != 2)
    throw bad_world_format{"Coordinates must have exactly two components"};

  position result;
  unsigned i = 0;
  for (auto const& coord : tree)
    result[i++] = coord.second.get_value<unsigned>();

  return result;
}

static std::normal_distribution<>
parse_normal(boost::property_tree::ptree const& p) {
  auto const& params = p.get_child("parameters");
  if (params.count("") != 2)
    throw bad_world_format{"Invalid normal distribution parameters"};

  std::array<double, 2> distrib_params;
  auto par = distrib_params.begin();
  for (auto const& param: params)
    *par++ = param.second.get_value<double>();

  return std::normal_distribution<>(distrib_params[0], distrib_params[1]);
}

static void
make_obstacles(world& w, boost::property_tree::ptree const& prop,
               std::default_random_engine& rng) {
  double const tile_probability = prop.get<double>("tile_probability");
  auto rand = [&] { return std::generate_canonical<double, 32>(rng); };
  std::normal_distribution<> duration =
    parse_normal(prop.get_child("obstacle_duration"));

  for (map::value_type const& tile: *w.map())
    if (tile.tile == ::tile::free && rand() < tile_probability)
    {
      obstacle o{duration};
      if (rand() < 0.5)
        o.active = true;
      o.next_change = w.tick() + std::max(tick_t{1}, (tick_t) o.duration(rng));

      w.put_obstacle({tile.x, tile.y}, std::move(o));
    }
}

world
load_world(std::string const& filename, std::default_random_engine& rng) try {
  namespace pt = boost::property_tree;
  pt::ptree tree;
  pt::read_json(filename, tree);

  using boost::filesystem::path;
  using boost::algorithm::trim_copy;

  std::string const map_filename = tree.get<std::string>("map");
  path const map_path = path{filename}.parent_path() / trim_copy(map_filename);
  world world{load_map(map_path.string())};

  for (auto const& a : tree.get_child("agents")) {
    position const pos = read_pos(a.second.get_child("position"));

    boost::optional<position> goal;
    if (auto const g = a.second.get_child_optional("goal"))
      goal = read_pos(*g);

    if (!goal)
      goal = pos;

    world.put_agent(pos, agent{*goal});
  }

  if (auto const obstacles = tree.get_child_optional("obstacles"))
    make_obstacles(world, *obstacles, rng);

  return world;

} catch (boost::property_tree::json_parser::json_parser_error& e) {
  throw bad_world_format{e.what()};
} catch (boost::property_tree::ptree_error& e) {
  throw bad_world_format{e.what()};
}
