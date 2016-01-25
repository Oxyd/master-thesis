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

std::ostream&
operator << (std::ostream& out, direction d) {
  switch (d) {
  case direction::north: return out << "north";
  case direction::south: return out << "south";
  case direction::east: return out << "east";
  case direction::west: return out << "west";
  }
  assert(!"Can't get here");
  return out;
}

bool
traversable(tile t) {
  return t == tile::free;
}

std::ostream&
operator << (std::ostream& out, position p) {
  return out << "[" << p.x << ", " << p.y << "]";
}

position
translate(position p, direction d) {
  switch (d) {
  case direction::north: return {p.x, p.y - 1};
  case direction::east:  return {p.x + 1, p.y};
  case direction::south: return {p.x, p.y + 1};
  case direction::west:  return {p.x - 1, p.y};
  }
  assert(!"Unreachable");
  return {};
}

direction
direction_to(position from, position to) {
  int dx = to.x - from.x;
  int dy = to.y - from.y;

  assert((std::abs(dx) == 1 && dy == 0) ||
         (std::abs(dy) == 1 && dx == 0));
  if (dx == 1) return direction::east;
  if (dx == -1) return direction::west;
  if (dy == 1) return direction::south;
  if (dy == -1) return direction::north;

  assert(!"Can't get here");
  return direction::north;
}

bool
neighbours(position a, position b) {
  bool const x = std::abs(a.x - b.x) == 1;
  bool const y = std::abs(a.y - b.y) == 1;
  return x != y;
}

unsigned
distance(position a, position b) {
  return std::abs(a.x - b.x) + std::abs(a.y - b.y);
}

std::ostream&
operator << (std::ostream& out, position_time pt) {
  return out << "[" << pt.x << ", " << pt.y << "]@" << pt.time;
}

std::shared_ptr<map>
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

  auto result = std::make_shared<map>(width, height, filename);

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

bool
in_bounds(position p, map const& m) {
  return p.x >= 0 && p.y >= 0
      && p.x < m.width() && p.y < m.height();
}

bool
in_bounds(int x, int y, map const& m) {
  return in_bounds({x, y}, m);
}

world::world(const std::shared_ptr<::map const>& m,
             ::obstacle_settings settings,
             obstacle_list obstacles, tick_t tick)
  : map_(m)
  , obstacles_(std::move(obstacles))
  , tick_(tick)
  , obstacle_settings_(settings)
{ }

static std::vector<position>
valid_directions(position p, world const& w) {
  std::vector<position> result;
  for (auto d : all_directions) {
    position q = translate(p, d);
    if (in_bounds(q, *w.map()) && traversable(w.get(q)))
      result.push_back(q);
  }

  return result;
}

static void
move(obstacle o, position pos, world& w, std::default_random_engine& rng) {
  std::vector<position> choices = valid_directions(pos, w);
  if (choices.empty())
    return;

  std::uniform_int_distribution<std::size_t> i(0, choices.size() - 1);
  position p = choices[i(rng)];

  w.remove_obstacle(pos);
  o.next_move = w.tick() + std::max(tick_t{1}, (tick_t) o.move_distrib(rng));
  w.put_obstacle(p, o);
}

void
world::next_tick(std::default_random_engine& rng) {
  ++tick_;

  auto obstacles = obstacles_;
  for (auto& pos_obst : obstacles) {
    position const& pos = pos_obst.first;
    obstacle& obstacle = pos_obst.second;

    if (obstacle.next_move == tick_)
      move(obstacle, pos, *this, rng);
  }
}

tile
world::get(position p) const {
  if (agents_.count(p))
    return tile::agent;
  else if (obstacles_.count(p))
    return tile::obstacle;
  else
    return map_->get(p);
}

boost::optional<agent&>
world::get_agent(position p) {
  auto const a = agents_.find(p);
  if (a != agents_.end())
    return a->second;
  else
    return {};
}

agent
world::create_agent(position goal) {
  return agent{goal, next_agent_id_++};
}

boost::optional<agent const&>
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
  if (get(p) != tile::free)
    throw std::logic_error{"put_obstacle: Position not empty"};
  obstacles_.insert({p, o});
}

void
world::remove_obstacle(position p) {
  auto const it = obstacles_.find(p);
  if (it == obstacles_.end())
    throw std::logic_error{"remove_obstacle: Obstacle not found"};
  obstacles_.erase(it);
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

static normal_distribution
parse_normal(boost::property_tree::ptree const& p) {
  auto const& params = p.get_child("parameters");
  if (params.count("") != 2)
    throw bad_world_format{"Invalid normal distribution parameters"};

  std::array<double, 2> distrib_params;
  auto par = distrib_params.begin();
  for (auto const& param: params)
    *par++ = param.second.get_value<double>();

  return normal_distribution{distrib_params[0], distrib_params[1]};
}

static std::normal_distribution<>
make_normal(normal_distribution d) {
  return std::normal_distribution<>(d.mean, d.std_dev);
}

static obstacle_settings
parse_obstacle_settings(boost::property_tree::ptree const& p) {
  obstacle_settings result;
  result.tile_probability = p.get<double>("tile_probability");
  result.move_probability =
    parse_normal(p.get_child("obstacle_movement.move_probability"));
  return result;
}

static void
make_obstacles(world& w, obstacle_settings settings,
               std::default_random_engine& rng) {
  auto rand = [&] { return std::generate_canonical<double, 32>(rng); };
  std::normal_distribution<> time_to_move =
    make_normal(settings.move_probability);

  for (map::value_type const& tile: *w.map())
    if (w.get({tile.x, tile.y}) == ::tile::free &&
        rand() < settings.tile_probability)
    {
      obstacle o{time_to_move};
      o.next_move = w.tick() + std::max(tick_t{1},
                                        (tick_t) o.move_distrib(rng));

      w.put_obstacle({tile.x, tile.y}, std::move(o));
    }
}

static std::tuple<world, boost::property_tree::ptree>
load_world_partial(std::string const& filename) try {
  namespace pt = boost::property_tree;
  pt::ptree tree;
  pt::read_json(filename, tree);

  using boost::filesystem::path;
  using boost::algorithm::trim_copy;

  std::string const map_filename = tree.get<std::string>("map");
  path const map_path = path{filename}.parent_path() / map_filename;
  world world{load_map(map_path.string()),
              parse_obstacle_settings(tree.get_child("obstacles"))};

  for (auto const& a : tree.get_child("agents")) {
    position const pos = read_pos(a.second.get_child("position"));

    boost::optional<position> goal;
    if (auto const g = a.second.get_child_optional("goal"))
      goal = read_pos(*g);

    if (!goal)
      goal = pos;

    world.put_agent(pos, world.create_agent(*goal));
  }

  return std::make_tuple(std::move(world), std::move(tree));

} catch (boost::property_tree::json_parser::json_parser_error& e) {
  throw bad_world_format{e.what()};
} catch (boost::property_tree::ptree_error& e) {
  throw bad_world_format{e.what()};
}

static boost::filesystem::path
make_relative(boost::filesystem::path p, boost::filesystem::path relative_to) {
  using namespace boost::filesystem;

  assert(is_directory(relative_to));

  path::iterator p_it = p.begin();
  path::iterator r_it = relative_to.begin();

  while (p_it != p.end() && r_it != relative_to.end() && *p_it == *r_it) {
    ++p_it;
    ++r_it;
  }

  path result;
  while (r_it != relative_to.end()) {
    result /= "..";
    ++r_it;
  }

  while (p_it != p.end())
    result /= *p_it++;

  return result;
}

world
load_world(std::string const& filename) {
  return std::get<0>(load_world_partial(filename));
}

world
load_world(std::string const& filename, std::default_random_engine& rng) {
  namespace pt = boost::property_tree;

  auto result = load_world_partial(filename);
  world& world = std::get<0>(result);
  pt::ptree& tree = std::get<1>(result);

  if (auto const obstacles = tree.get_child_optional("obstacles"))
    make_obstacles(world, parse_obstacle_settings(*obstacles), rng);

  return world;
}

static boost::property_tree::ptree
position_to_ptree(position p) {
  using namespace boost::property_tree;

  ptree x;
  x.put("", p.x);
  ptree y;
  y.put("", p.y);

  ptree result;
  result.push_back({"", x});
  result.push_back({"", y});

  return result;
}

void
save_world(world const& world, std::string const& filename) {
  namespace pt = boost::property_tree;

  pt::ptree tree;

  boost::filesystem::path scenario_path{filename};
  boost::filesystem::path map_path{world.map()->original_filename()};
  boost::filesystem::path relative_map_path =
    make_relative(map_path, scenario_path.parent_path());
  tree.put("map", relative_map_path.string());

  pt::ptree obstacles;
  obstacles.add("mode", "random");
  obstacles.add("tile_probability", world.obstacle_settings().tile_probability);

  pt::ptree move_probability;
  move_probability.add("distribution", "normal");

  pt::ptree mean;
  mean.put("", world.obstacle_settings().move_probability.mean);

  pt::ptree std_dev;
  std_dev.put("", world.obstacle_settings().move_probability.std_dev);

  pt::ptree parameters;
  parameters.push_back({"", mean});
  parameters.push_back({"", std_dev});
  move_probability.add_child("parameters", parameters);

  pt::ptree obstacle_movement;
  obstacle_movement.add_child("move_probability", move_probability);

  obstacles.add_child("obstacle_movement", obstacle_movement);

  tree.add_child("obstacles", obstacles);

  pt::ptree agents;
  for (auto pos_agent : world.agents()) {
    position const pos = std::get<0>(pos_agent);
    agent const a = std::get<1>(pos_agent);

    pt::ptree agent_tree;
    agent_tree.add_child("position", position_to_ptree(pos));
    agent_tree.add_child("goal", position_to_ptree(a.target));

    agents.push_back({"", agent_tree});
  }

  tree.add_child("agents", agents);

  pt::write_json(filename, tree);
}
