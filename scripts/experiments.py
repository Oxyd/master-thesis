#!/usr/bin/env python3

from collections import namedtuple
from pathlib import Path
import argparse
import json
import subprocess
import sys
import threading
import queue

threads = 8

Implementation = namedtuple(
  'Implementation',
  ['hierarchy', # List of tuples (name, pretty name)
   'args']      # List of strings that will be passed to the binary
)

def impls(extra_args, hierarchy):
  whca_impls = [
    Implementation(hierarchy
                   + [('whca-{}'.format(n), 'WHCA* ({})'.format(n))],
                   ['--algorithm', 'whca', '--window', '{}'.format(n)]
                   + extra_args)
    for n in (5, 10, 15, 20)
  ]

  return [Implementation(hierarchy + [('lra', 'LRA*')],
                         ['--algorithm', 'lra'] + extra_args)] + whca_impls

set_impls = {
  'full': impls([], []),
  'first': impls([], []),
  'algos_small': impls([], []),
  'rejoin_small':
    impls([], [('none', 'No rejoin')])
    + [i
       for imps in (impls(['--rejoin', str(n)],
                          [('rejoin-{}'.format(n), '{} steps'.format(n))])
                    for n in (1, 2, 5, 10, 20))
       for i in imps],
  'predict_penalty':
    impls([], [('recursive', 'recursive'), ('none', 'No predictor')])
    + impls([], [('matrix', 'matrix'), ('none', 'No predictor')])
    + [i
       for imps in (
           impls(
             ['--avoid', predictor,
              '--obstacle-penalty', str(penalty),
              '--obstacle-threshold', '1.0'],
             [(predictor, predictor),
              ('avoid-{}'.format(penalty), 'Penalty {}'.format(penalty))]
           )
           for penalty in (1, 2, 3, 4, 5, 10)
           for predictor in ('recursive', 'matrix')
       )
       for i in imps]
}

all_configs = [
  # Timeout | Num agents | Obstacle probability
  (5, 5, 0.01),
  (5, 5, 0.1),
  (5, 50, 0.01),
  (5, 50, 0.1)
]

small_configs = [
  (10, agents, obstacles)
  for obstacles in (0.01, 0.05, 0.1, 0.2)
  for agents in (1, 5, 10, 15, 20, 30)
]

set_configs = {
  'full': all_configs,
  'first': all_configs,
  'algos_small': small_configs,
  'rejoin_small': [(10, 10, 0.1)],
  'predict_penalty': [(10, 10, 0.1)]
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

    (name, args, result_path, info, timeout) = item

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

    except json.decoder.JSONDecodeError as e:
      print('Invalid output: {}'.format(e), file=sys.stderr)
      print('Command: {}'.format(' '.join('"{}"'.format(a) for a in args)),
            file=sys.stderr)
      print('Output:', file=sys.stderr)
      print(file=sys.stderr)
      print(result.stdout, file=sys.stderr)
      sys.exit(1)

    result_path.open(mode='w').write(json.dumps(
      {'map_info': info, 'result': result_data, 'completed': completed},
      indent=2
    ))

    jobs.task_done()


def do_experiments(maps, num_agents, obstacle_prob, scenarios, solver_args,
                   timeout, dry):
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
              result_path, info, timeout))

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
    'algos_small': small_maps,
    'rejoin_small': small_maps,
    'predict_penalty': small_maps
  }
  all_sets = ['full', 'algos_small', 'rejoin_small']

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

    if set_name not in set_impls:
      print('No implementation for set {}'.format(set_name))
      continue

    print('====== {} ======'.format(set_name))

    for impl in set_impls[set_name]:
      print('=== {} ==='.format(
        ' / '.join(pretty_name for (_, pretty_name) in impl.hierarchy)
      ))
      name = '/'.join(name for (name, _) in impl.hierarchy)

      parent = tmp_path / set_name
      for name, pretty_name in impl.hierarchy:
        (parent / name).mkdir(parents=True, exist_ok=True)

        with (parent / name / 'meta.json').open(mode='w') as out:
          json.dump({'name': pretty_name}, out)

        parent = parent / name

      for timeout, agents, obstacles in set_configs[set_name]:
        do_experiments(sets[set_name], agents, obstacles,
                       parent, impl.args, timeout, args.dry)

if __name__ == '__main__':
  main()
