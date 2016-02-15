#include "log_sinks.hpp"
#include "solvers.hpp"
#include "world.hpp"

#include <boost/algorithm/string/predicate.hpp>
#include <boost/program_options.hpp>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

static std::unique_ptr<solver>
make_solver(std::string const& name,
            boost::program_options::variables_map const& vm) {
  using boost::algorithm::iequals;

  if (iequals(name, "whca") || iequals(name, "whca*")) {
    unsigned window = vm.count("window") ? vm["window"].as<unsigned>() : 10;
    unsigned rejoin_limit =
      vm.count("rejoin") ? vm["rejoin"].as<unsigned>() : 0;
    bool avoid_obstacles = vm.count("avoid");
    return std::make_unique<cooperative_a_star>(null_log_sink, window,
                                                rejoin_limit, avoid_obstacles);
  }

  if (iequals(name, "lra") || iequals(name, "lra*"))
    return std::make_unique<lra>(null_log_sink);

  throw std::runtime_error{std::string{"Unknown solver type: "} + name};
}

int
main(int argc, char** argv) try {
  namespace po = boost::program_options;

  po::options_description desc;
  desc.add_options()
    ("help,h", "This text")
    ("scenario,s", po::value<std::string>(), "Scenario to run")
    ("algorithm,a", po::value<std::string>(), "Algorithm to use")
    ("limit,l", po::value<unsigned>(), "Tick limit")
    ("output,o", po::value<std::string>(), "Output file")
    ("window,w", po::value<unsigned>(), "WHCA* window size")
    ("rejoin,r", po::value<unsigned>()->implicit_value(10),
     "Allow path rejoining, for at most N steps")
    ("avoid,v", po::bool_switch(), "Avoid predicted obstacles")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help")) {
    std::cout << desc << '\n';
    return 0;
  }

  if (!vm.count("scenario")) {
    std::cerr << "The --scenario parameter is mandatory\n";
    return 1;
  }

  if (!vm.count("algorithm")) {
    std::cerr << "The --algorithm parameter is mandatory\n";
    return 1;
  }

  std::default_random_engine rng;
  world w = load_world(vm["scenario"].as<std::string>(), rng);
  auto solver = make_solver(vm["algorithm"].as<std::string>(), vm);

  unsigned const limit = vm.count("limit") ? vm["limit"].as<unsigned>() : 0;

  using clock_type = std::chrono::steady_clock;
  auto start = clock_type::now();

  while (!solved(w)) {
    w.next_tick(rng);
    joint_action action = solver->get_action(w, rng);
    w = apply(action, w);

    if (limit > 0 && w.tick() >= limit)
      break;
  }

  auto end = clock_type::now();

  namespace pt = boost::property_tree;
  pt::ptree results;
  results.add("ticks", w.tick());
  results.add(
    "time_ms",
    std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()
  );
  results.add("success", solved(w));

  pt::ptree algo_stats;
  auto names = solver->stat_names();
  auto values = solver->stat_values();
  assert(names.size() == values.size());

  for (std::size_t i = 0; i < names.size(); ++i)
    algo_stats.add(names[i], values[i]);

  results.add_child("algorithm_statistics", algo_stats);

  if (vm.count("output")) {
    std::ofstream out(vm["output"].as<std::string>());
    pt::write_json(out, results);
  } else
    pt::write_json(std::cout, results);

} catch (bad_world_format& e) {
  std::cerr << "World format error: " << e.what() << '\n';
  return 1;
} catch (boost::program_options::error& e) {
  std::cerr << "Bad option: " << e.what() << '\n';
  return 1;
} catch (std::runtime_error& e) {
  std::cerr << "Error: " << e.what() << '\n';
  return 2;
} catch (std::logic_error& e) {
  std::cerr << "Bug: " << e.what() << '\n';
  return 3;
} catch (...) {
  std::cerr << "Unknown error\n";
  return 2;
}
