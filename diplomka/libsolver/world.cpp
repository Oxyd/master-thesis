#include "world.hpp"

#include <boost/algorithm/string/trim.hpp>
#include <boost/filesystem.hpp>
#include <boost/optional.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <algorithm>
#include <cassert>
#include <fstream>
#include <iterator>
#include <limits>
#include <queue>
#include <sstream>
#include <string>
#include <unordered_set>

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

direction
inverse(direction d) {
  switch (d) {
  case direction::north: return direction::south;
  case direction::east: return direction::west;
  case direction::south: return direction::north;
  case direction::west: return direction::east;
  }

  assert(!"Can't get here");
  return d;
}

bool
traversable(tile t) {
  return t == tile::free;
}

std::ostream&
operator << (std::ostream& out, position p) {
  return out << "[" << p.x << ", " << p.y << "]";
}

std::string
to_string(position p) {
  std::ostringstream os;
  os << p;
  return os.str();
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
  return
    (std::abs(a.x - b.x) == 1 && a.y == b.y)
    || (std::abs(a.y - b.y) == 1 && a.x == b.x);
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
             ::agent_settings agents,
             obstacle_list obstacles, tick_t tick)
  : map_(m)
  , obstacles_(std::move(obstacles))
  , tick_(tick)
  , obstacle_settings_(std::move(settings))
  , agent_settings_(agents)
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
move_random(obstacle o, position pos, world& w, std::default_random_engine& rng) {
  std::vector<position> choices = valid_directions(pos, w);
  if (choices.empty())
    return;

  std::uniform_int_distribution<std::size_t> i(0, choices.size() - 1);
  position p = choices[i(rng)];

  w.remove_obstacle(pos);
  o.next_move = w.tick() + std::max(1, (int) o.move_distrib(rng));
  w.put_obstacle(p, o);
}

static void
move_to_goal(obstacle o, position pos, world& w,
             std::unordered_set<position> const& goal_points,
             std::default_random_engine& rng) {
  if (goal_points.empty())
    throw std::runtime_error{
      "Move-to-goal mode selected, but no goal points defined"
    };

  if (goal_points.count(pos)) {
    w.remove_obstacle(pos);
    return;
  }

  position nearest_goal = *goal_points.begin();
  unsigned nearest_distance = std::numeric_limits<unsigned>::max();
  for (position g : goal_points)
    if (distance(pos, g) < nearest_distance) {
      nearest_goal = g;
      nearest_distance = distance(pos, g);
    }

  int dx = nearest_goal.x - pos.x;
  int dy = nearest_goal.y - pos.y;

  auto sgn = [] (int x) { return x > 0 ? 1 : x < 0 ? -1 : 0; };

  position next = pos;
  if (std::abs(dx) > std::abs(dy))
    next.x += sgn(dx);
  else
    next.y += sgn(dy);

  if (w.get(next) != tile::free)
    next = pos;

  w.remove_obstacle(pos);
  o.next_move = w.tick() + std::max(tick_t{1}, (tick_t) o.move_distrib(rng));
  w.put_obstacle(next, o);
}

static std::normal_distribution<>
make_normal(normal_distribution d) {
  return std::normal_distribution<>(d.mean, d.std_dev);
}

static void
make_obstacles(world& w, obstacle_settings settings,
               std::default_random_engine& rng) {
  auto rand = [&] { return std::generate_canonical<double, 32>(rng); };
  std::normal_distribution<> time_to_move =
    make_normal(settings.move_probability);

  std::unordered_set<position> spawn_candidates = settings.spawn_points;
  if (spawn_candidates.empty())
    for (map::value_type const& t : *w.map())
      if (w.get({t.x, t.y}) == tile::free)
        spawn_candidates.insert(position{t.x, t.y});

  for (position p : spawn_candidates)
    if (w.get(p) == ::tile::free && rand() < settings.tile_probability)
    {
      obstacle o = w.create_obstacle(time_to_move);
      o.next_move = w.tick() + std::max(tick_t{1},
                                        (tick_t) o.move_distrib(rng));

      w.put_obstacle(p, std::move(o));
    }
}

void
world::next_tick(std::default_random_engine& rng) {
#ifndef NDEBUG
  for (agent::id_type id = 0; id < agent_id_end(); ++id)
    assert(std::find_if(agents_.begin(), agents_.end(),
                        [&] (auto pos_agent) {
                          return std::get<1>(pos_agent).id() == id;
                        }) != agents_.end());
#endif

  ++tick_;

  if (obstacle_settings_.mode == obstacle_mode::spawn_to_goal)
    make_obstacles(*this, obstacle_settings_, rng);

  auto obstacles = obstacles_;
  for (auto& pos_obst : obstacles) {
    position const& pos = pos_obst.first;
    obstacle& obstacle = pos_obst.second;

    if (obstacle.next_move == tick_) {
      if (obstacle_settings_.mode == obstacle_mode::random)
        move_random(obstacle, pos, *this, rng);
      else
        move_to_goal(obstacle, pos, *this, obstacle_settings_.goal_points, rng);
    }
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

agent const&
world::get_agent(agent::id_type id) const {
  for (auto const& pos_agent : agents_)
    if (std::get<1>(pos_agent).id() == id)
      return std::get<1>(pos_agent);

  assert(!"Invalid ID");
  static agent invalid{{}, {}};
  return invalid;
}

agent
world::create_agent(position goal) {
  return agent{goal, next_agent_id_++};
}

obstacle
world::create_obstacle(std::normal_distribution<> move_distribution) {
  return obstacle{std::move(move_distribution), next_obstacle_id_++};
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
    throw std::logic_error{"put_agent: Position not empty: " + to_string(p)};

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

static obstacle_mode
parse_obstacle_mode(std::string const& mode) {
  if (mode == "random")
    return obstacle_mode::random;
  else if (mode == "spawn_to_goal")
    return obstacle_mode::spawn_to_goal;
  else
    throw bad_world_format{"Invalid obstacle mode: " + mode};
}

static std::string
obstacle_mode_to_str(obstacle_mode mode) {
  switch (mode) {
  case obstacle_mode::random: return "random";
  case obstacle_mode::spawn_to_goal: return "spawn_to_goal";
  default:
    assert(!"Won't get here");
    return "";
  }
}

static obstacle_settings
parse_obstacle_settings(boost::property_tree::ptree const& p) {
  obstacle_settings result;
  result.mode = parse_obstacle_mode(p.get<std::string>("mode"));
  result.tile_probability = p.get<double>("tile_probability");
  result.move_probability =
    parse_normal(p.get_child("obstacle_movement.move_probability"));

  if (p.count("spawn_points"))
    for (auto point : p.get_child("spawn_points"))
      result.spawn_points.insert(read_pos(point.second));

  if (p.count("goal_points"))
    for (auto point : p.get_child("goal_points"))
      result.goal_points.insert(read_pos(point.second));

  return result;
}

static agent_settings
parse_agent_settings(boost::property_tree::ptree const& p) {
  agent_settings result;
  result.random_agent_number = p.get<unsigned>("random_agents");

  if (auto mode = p.get_child_optional("spawn_mode")) {
    if (mode->get_value<std::string>() == "uniform")
      result.spawn_mode = agent_settings::random_spawn_mode::uniform;
    else if (mode->get_value<std::string>() == "pack")
      result.spawn_mode = agent_settings::random_spawn_mode::pack;
    else
      throw bad_world_format{"Invalid agent spawn mode"};
  }

  return result;
}

static void
place_uniform_agents(world& w, agent_settings settings,
                     std::default_random_engine& rng) {
  auto rand_pos = [&] (auto bad_pos) {
    using uniform_coord = std::uniform_int_distribution<position::coord_type>;
    uniform_coord x(0, w.map()->width() - 1);
    uniform_coord y(0, w.map()->height() - 1);

    position result;
    do
      result = position{x(rng), y(rng)};
    while (w.get(result) == tile::wall || bad_pos(result));

    return result;
  };

  std::unordered_set<position> goals;
  for (unsigned i = 0; i < settings.random_agent_number; ++i) {
    position goal = rand_pos([&] (position p) { return goals.count(p); });
    agent a = w.create_agent(goal);

    position pos = rand_pos([&] (position p) {
      return w.get(p) == tile::agent || w.get(p) == tile::obstacle;
    });

    w.put_agent(pos, a);
    goals.insert(goal);
  }
}

static void
place_pack_agents(world& w, agent_settings settings,
                  std::default_random_engine& rng) {
  std::vector<position> agent_candidates;
  std::vector<position> goal_candidates;

  for (position::coord_type y = 0; y < w.map()->height(); ++y)
    for (position::coord_type x = 0; x < w.map()->width(); ++x) {
      if (w.get({x, y}) == tile::free)
        agent_candidates.emplace_back(position{x, y});

      if (w.get({x, y}) != tile::wall)
        goal_candidates.emplace_back(position{x, y});
    }

  if (agent_candidates.size() < settings.random_agent_number
      || goal_candidates.size() < settings.random_agent_number)
    throw bad_world_format{"Not enough free spaces for random agents"};

  std::uniform_int_distribution<std::size_t> agent_index_distrib(
    0, agent_candidates.size() - 1
  );
  std::uniform_int_distribution<std::size_t> goal_index_distrib(
    0, goal_candidates.size() - 1
  );

  position agent_start = agent_candidates[agent_index_distrib(rng)];

  position goal_start;
  do
    goal_start = goal_candidates[goal_index_distrib(rng)];
  while (distance(agent_start, goal_start) <= 2 * settings.random_agent_number);

  unsigned to_place = settings.random_agent_number;
  std::queue<position> agent_queue{{agent_start}};
  std::queue<position> goal_queue{{goal_start}};
  std::unordered_set<position> used_goals;
  while (to_place > 0) {
    if (agent_queue.empty())
      throw bad_world_format{"Can't place random agent"};

    position agent_pos = agent_queue.front();
    agent_queue.pop();

    if (goal_queue.empty())
      throw bad_world_format{"Can't select random agent's goal"};

    position agent_goal = goal_queue.front();
    goal_queue.pop();

    if (w.get(agent_pos) == tile::free && !used_goals.count(agent_goal)) {
      w.put_agent(agent_pos, w.create_agent(agent_goal));
      --to_place;

      used_goals.insert(agent_goal);
    }

    for (direction d : all_directions) {
      position new_agent_pos = translate(agent_pos, d);
      if (w.get(new_agent_pos) == tile::free)
        agent_queue.push(new_agent_pos);

      position new_goal_pos = translate(agent_goal, d);
      if (w.get(new_goal_pos) != tile::wall && !used_goals.count(new_goal_pos))
        goal_queue.push(new_goal_pos);
    }
  }
}

static void
make_agents(world& w, agent_settings settings,
            std::default_random_engine& rng) {
  switch (settings.spawn_mode) {
  case agent_settings::random_spawn_mode::uniform:
    place_uniform_agents(w, settings, rng);
    break;

  case agent_settings::random_spawn_mode::pack:
    place_pack_agents(w, settings, rng);
    break;
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

  agent_settings as;
  if (tree.count("agent_settings"))
    as = parse_agent_settings(tree.get_child("agent_settings"));

  std::shared_ptr<map> m = load_map(map_path.string());
  obstacle_settings os = parse_obstacle_settings(tree.get_child("obstacles"));
  world world(std::move(m), std::move(os), as);

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
  namespace fs = boost::filesystem;

  assert(fs::is_directory(relative_to));

  fs::path::iterator p_it = p.begin();
  fs::path::iterator r_it = relative_to.begin();

  while (p_it != p.end() && r_it != relative_to.end() && *p_it == *r_it) {
    ++p_it;
    ++r_it;
  }

  fs::path result;
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

  if (auto const rand_agents = tree.get_child_optional("agent_settings"))
    make_agents(world, parse_agent_settings(*rand_agents), rng);

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
  obstacles.add("mode", obstacle_mode_to_str(world.obstacle_settings().mode));
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

  pt::ptree obstacle_spawn_points;
  for (position p : world.obstacle_settings().spawn_points)
    obstacle_spawn_points.push_back({"", position_to_ptree(p)});

  obstacles.add_child("spawn_points", obstacle_spawn_points);

  pt::ptree obstacle_goal_points;
  for (position p : world.obstacle_settings().goal_points)
    obstacle_goal_points.push_back({"", position_to_ptree(p)});

  obstacles.add_child("goal_points", obstacle_goal_points);

  tree.add_child("obstacles", obstacles);

  pt::ptree agent_settings;

  pt::ptree rand_agent_num;
  rand_agent_num.put("", world.agent_settings().random_agent_number);
  agent_settings.add_child("random_agents", rand_agent_num);

  pt::ptree rand_agent_mode;
  switch (world.agent_settings().spawn_mode) {
  case agent_settings::random_spawn_mode::uniform:
    rand_agent_mode.put("", "uniform");
    break;

  case agent_settings::random_spawn_mode::pack:
    rand_agent_mode.put("", "pack");
    break;
  }

  agent_settings.add_child("spawn_mode", rand_agent_mode);

  tree.add_child("agent_settings", agent_settings);

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
