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

Run = namedtuple(
  'Run',
  ['timeout', 'agents', 'obstacles',
   'args',       # List of tuples (name, pretty name)
   'hierarchy']  # List of strings that will be passed to the binary
)

def runs(args):
  extra_args = args.get('args', [])
  hierarchy = args.get('hierarchy', [])
  timeout = args.get('timeout', 5)
  agents = args.get('agents', 10)
  obstacles = args.get('obstacles', 0.1)

  whca_runs = [
    Run(timeout=timeout, agents=agents, obstacles=obstacles,
        hierarchy=hierarchy + [('whca-{}'.format(n), 'WHCA* ({})'.format(n))],
        args=['--algorithm', 'whca', '--window', '{}'.format(n)] + extra_args)
    for n in (5, 10, 15, 20)
  ]

  lra_run = Run(timeout=timeout, agents=agents, obstacles=obstacles,
                hierarchy=hierarchy + [('lra', 'LRA*')],
                args=['--algorithm', 'lra'] + extra_args)

  return [lra_run] + whca_runs


def product(f, *args):
  def do(bound, unbound):
    if len(unbound) == 0:
      return runs(f(*bound))

    l = []
    for value in unbound[0]:
      l.extend(do(bound + [value], unbound[ 1:]))

    return l

  return do([], args)


def join(*args):
  result = []

  for l in args:
    result.extend(l)

  return result


set_runs = {
  'algos_small':
    product(
      lambda agents, obstacles: {
        'agents': agents, 'obstacles': obstacles,
        'hierarchy': [('{}-agents'.format(agents), '{} agents'.format(agents)),
                      ('{}-obst'.format(obstacles),
                       '{} obstacles'.format(obstacles))],
      },
      (1, 5, 10, 15, 20, 30),
      (0.01, 0.05, 0.1, 0.2)
    ),
  'rejoin_small':
    join(
      runs({'hierarchy': [('none', 'No rejoin')]}),
      product(
        lambda n: {'args': ['--rejoin', str(n)],
                   'hierarchy': [('rejoin-{}'.format(n), '{} steps'.format(n))],
                   'timeout': 10},
        (1, 2, 5, 10, 20)
      )
    ),
  'predict_penalty':
    join(
      product(
        lambda predictor: {
          'hierarchy': [(predictor, predictor), ('none', 'No predictor')]
        },
        ('recursive', 'matrix')
      ),
      product(
        lambda penalty, predictor: {
          'args': ['--avoid', predictor,
                   '--obstacle-penalty', str(penalty),
                   '--obstacle-threshold', '1.0'],
          'hierarchy': [
            (predictor, predictor),
            ('avoid-{}'.format(penalty), 'Penalty {}'.format(penalty))
          ],
          'timeout': 10
        },
        (1, 2, 3, 4, 5, 10),
        ('recursive', 'matrix')
      )
    ),
  'predict_threshold':
    join(
      product(
        lambda predictor: {
          'hierarchy': [(predictor, predictor), ('none', 'No predictor')]
        },
        ('recursive', 'matrix')
      ),
      product(
        lambda threshold, predictor: {
          'args': ['--avoid', predictor,
                   '--obstacle-penalty', '3',
                   '--obstacle-threshold', str(threshold)],
          'hierarchy': [
            (predictor, predictor),
            ('avoid-{}'.format(threshold), 'Threshold {}'.format(threshold))
          ],
          'timeout': 10
        },
        (0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0),
        ('recursive', 'matrix')
      )
    )
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
    scenario_dir = scenarios
    scenario_dir.mkdir(parents=True, exist_ok=True)
    scenario = make_scenario(info, make_relative(map_path, scenario_dir),
                             num_agents, obstacle_prob)
    if scenario is None: continue

    assert int(scenario['agent_settings']['random_agents']) == int(num_agents)

    scenario_path = scenario_dir / (map_path.stem + '.json')
    scenario_path.open(mode='w').write(json.dumps(scenario, indent=2))
    result_path = scenario_dir / (map_path.stem + '.result.json')

    jobs.put((map_path.stem,
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
    'predict_penalty': small_maps,
    'predict_threshold': small_maps
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
    if set_name not in set_runs:
      print('No run for set {}'.format(set_name))
      continue

    print('====== {} ======'.format(set_name))

    for run in set_runs[set_name]:
      print('=== {} ==='.format(
        ' / '.join(pretty_name for (_, pretty_name) in run.hierarchy)
      ))
      name = '/'.join(name for (name, _) in run.hierarchy)

      parent = tmp_path / set_name
      for name, pretty_name in run.hierarchy:
        (parent / name).mkdir(parents=True, exist_ok=True)

        with (parent / name / 'meta.json').open(mode='w') as out:
          json.dump({'name': pretty_name}, out)

        parent = parent / name

      do_experiments(sets[set_name], run.agents, run.obstacles,
                     parent, run.args, run.timeout, args.dry)

if __name__ == '__main__':
  main()
