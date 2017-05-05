#!/usr/bin/env python3

from pathlib import Path
import argparse
import json
import subprocess
import sys
import threading
import queue

threads = 8

def impls(extra_args, suffix='', heuristic_name=''):
  return [
    ('lra' + suffix, 'LRA*', heuristic_name,
     ['--algorithm', 'lra'] + extra_args),
    ('whca-5' + suffix, 'WHCA* (5)', heuristic_name,
     ['--algorithm', 'whca', '--window', '5'] + extra_args),
    ('whca-10' + suffix, 'WHCA* (10)', heuristic_name,
     ['--algorithm', 'whca', '--window', '10'] + extra_args),
    ('whca-15' + suffix, 'WHCA* (15)', heuristic_name,
     ['--algorithm', 'whca', '--window', '15'] + extra_args),
    ('whca-20' + suffix, 'WHCA* (20)', heuristic_name,
     ['--algorithm', 'whca', '--window', '20'] + extra_args)
  ]

set_impls = {
  'full': impls([]),
  'first': impls([]),
  'algos_small': impls([]),
  'rejoin_small':
    impls([], heuristic_name='No rejoin')
    + [i
       for imps in (impls(['--rejoin', str(n)],
                          '-rejoin-{}'.format(n),
                          '{} steps'.format(n))
                    for n in (1, 2, 5, 10, 20))
       for i in imps],
  'predict_recursive_depth':
    impls([], heuristic_name='No predictor')
    + [i
       for imps in (impls(['--avoid', 'recursive',
                           '--predictor-cutoff', str(n)],
                          suffix='-predict-{}'.format(n),
                          heuristic_name='Predict {} steps'.format(n))
                    for n in range(1, 8))
       for i in imps],
  'predict_matrix_depth':
    impls([], heuristic_name='No predictor')
    + [i
       for imps in (impls(['--avoid', 'matrix',
                           '--predictor-cutoff', str(n)],
                          suffix='-predict-{}'.format(n),
                          heuristic_name='Predict {} steps'.format(n))
                    for n in range(1, 8))
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
  for agents in (1, 5, 10, 15, 20, 30, 40, 50, 100, 200)
]

set_configs = {
  'full': all_configs,
  'first': all_configs,
  'algos_small': small_configs,
  'rejoin_small': [(10, 10, 0.1)],
  'predict_recursive_depth': [(10, 10, 0.1)],
  'predict_matrix_depth': [(10, 10, 0.1)]
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
    'predict_recursive_depth': small_maps,
    'predict_matrix_depth': small_maps
  }
  all_sets = ['full', 'algos_small', 'rejoin_small', 'predict_recursive_depth',
              'predict_matrix_depth']

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

    for name, pretty_name, heuristic_name, impl_args in set_impls[set_name]:
      if len(heuristic_name) > 0:
        print('=== {} ({}) ==='.format(pretty_name, heuristic_name))
      else:
        print('=== {} ==='.format(pretty_name))

      for timeout, agents, obstacles in set_configs[set_name]:
        do_experiments(sets[set_name], agents, obstacles,
                       tmp_path / set_name / name, impl_args, timeout, args.dry)

        with (tmp_path / set_name / name / 'meta.json').open(mode='w') as out:
          json.dump({'name': pretty_name,
                     'heuristic_name': heuristic_name}, out)

if __name__ == '__main__':
  main()
