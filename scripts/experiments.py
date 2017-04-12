#!/usr/bin/env python3

from pathlib import Path
import argparse
import json
import subprocess
import sys
import threading
import queue

impls = [
  ('lra', ['--algorithm', 'lra']),
  ('whca-5', ['--algorithm', 'whca', '--window', '5']),
  ('whca-10', ['--algorithm', 'whca', '--window', '10']),
  ('whca-15', ['--algorithm', 'whca', '--window', '15']),
  ('whca-20', ['--algorithm', 'whca', '--window', '20'])
]

timeout = 5
threads = 8

all_configs = [
  # Num agents | Obstacle probability
  (5, 0.01),
  (5, 0.1),
  (50, 0.01),
  (50, 0.1)
]

small_configs = [
  (1, 0.01),
  (5, 0.01),
  (10, 0.01),
  (15, 0.01),
  (20, 0.01),
  (30, 0.01),
  (40, 0.01),
  (50, 0.01),
  (100, 0.01),
  (200, 0.01)
]

set_configs = {
  'full': all_configs,
  'first': all_configs,
  'small': small_configs
}

solver_path = Path('../bin/opt/cli')
maps_path = Path('../da-maps')
tmp_path = Path('../tmp')


def make_scenario(map_info, map_path, num_agents, obstacles_prob):
  '''Create a scenario for the given map.'''

  tiles = int(map_info['passable_tiles'])
  if tiles > 20000: return None

  return {
    'map': str(map_path),
    'obstacles': {
      'mode': 'random',
      'tile_probability': obstacles_prob,
      'obstacle_movement': {
        'move_probability': {
          'distribution': 'normal',
          'parameters': [5, 1]
        }
      }
    },
    'agent_settings': {
      'random_agents': min(int(num_agents), tiles)
    },
    'agents': []
  }

def make_relative(path, relative_to):
  '''Make path relative to the other given path.'''

  path = path.resolve().parts
  relative_to = relative_to.resolve().parts

  i = 0
  while i < max(len(path), len(relative_to)) and path[i] == relative_to[i]:
    i += 1

  result = Path()
  for _ in range(i, len(relative_to)): result /= '..'

  return result.joinpath(*path[i :])

def substitute_scenario(solver_args, scenario_path):
  '''Replace {} in solver_args with scenario_path and return the resulting
  list.
  '''

  def subst(arg):
    if arg == '{}': return str(scenario_path)
    else: return arg

  return list(map(subst, solver_args))

jobs = queue.Queue()

def reset_jobs():
  global jobs
  jobs = queue.Queue()


def worker():
  '''Worker thread that will run experiments off the job queue.'''

  while True:
    item = jobs.get()
    if item is None: break

    (name, args, result_path, info) = item

    print('{} ...'.format(name))

    try:
      result = subprocess.run(args,
                              stdout=subprocess.PIPE,
                              universal_newlines=True,
                              timeout=timeout)
      result_data = json.loads(result.stdout)
      completed = True

      print('... {} done'.format(name))
    except subprocess.TimeoutExpired:
      print('... {} timed out '.format(name))

      result_data = {}
      completed = False

    result_path.open(mode='w').write(json.dumps(
      {'map_info': info, 'result': result_data, 'completed': completed},
      indent=2
    ))

    jobs.task_done()


def do_experiments(maps, num_agents, obstacle_prob, scenarios, solver_args,
                   dry):
  '''Run the experiments for one implementation and one configuration on all
  available scenarios.
  '''

  reset_jobs()

  for (map_path, info) in maps:
    conf_name = '{}-agents-{}-obst'.format(num_agents, obstacle_prob)
    scenario_dir = scenarios / conf_name
    scenario_dir.mkdir(parents=True, exist_ok=True)
    scenario = make_scenario(info, make_relative(map_path, scenario_dir),
                             num_agents, obstacle_prob)
    if scenario is None: continue

    scenario_path = scenario_dir / (map_path.stem + '.json')
    scenario_path.open(mode='w').write(json.dumps(scenario, indent=2))
    result_path = scenario_dir / (map_path.stem + '.result.json')

    jobs.put((map_path.stem + ' ' + conf_name,
              [str(solver_path.resolve()),
               '--scenario', str(scenario_path.resolve())]
              + solver_args,
              result_path,
              info))

  if dry:
    return

  print('Starting worker threads')

  workers = [threading.Thread(target=worker) for _ in range(threads)]
  for w in workers: w.start()

  jobs.join()

  for _ in range(threads): jobs.put(None)
  for w in workers: w.join()


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--set', type=str, default='all',
                      help='Set name to run. Defaults to "all"')
  parser.add_argument('--dry', action='store_true')
  args = parser.parse_args()

  all_maps = []
  small_maps = []

  for f in maps_path.glob('*.json'):
    map_path = f.parent / (f.stem + '.map')
    if not map_path.exists():
      print('No matching map file for {}'.format(f.name))
      continue

    info = json.load(f.open())

    if not info['connected']:
      print('{} not connected; skipping'.format(f.stem))
      continue

    all_maps.append((map_path, info))

    if 5000 <= info['passable_tiles'] < 6000:
      small_maps.append((map_path, info))

  sets = {
    'full': all_maps,
    'first': [all_maps[0]],
    'none': [],
    'small': small_maps
  }
  all_sets = ['full']

  sets_to_run = []
  if args.set == 'all':
    sets_to_run = all_sets
  else:
    if args.set not in sets:
      print('Unknown set {}'.format(args.set))
      return

    sets_to_run = [args.set]

  for set_name in sets_to_run:
    if set_name not in set_configs:
      print('No config for set {}'.format(set_name))
      continue

    print('====== {} ======'.format(set_name))

    for name, impl_args in impls:
      print('=== {} ==='.format(name))

      for agents, obstacles in set_configs[set_name]:
        do_experiments(sets[set_name], agents, obstacles,
                       tmp_path / set_name / name, impl_args, args.dry)

if __name__ == '__main__':
  main()
