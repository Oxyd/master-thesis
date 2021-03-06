#!/usr/bin/env python3

from collections import namedtuple
from pathlib import Path
import argparse
import json
import queue
import random
import subprocess
import sys
import threading

threads = 8

def seeds(num):
  '''Get a list of `num' seeds. This list is always the same for reproducibility
  reasons.
  '''

  r = random.Random(0)
  return tuple(r.randint(0, 2**32 - 1) for _ in range(num))


Run = namedtuple(
  'Run',
  ['timeout', 'agents', 'obstacles', 'obstacle_move_freq_distr',
   'agents_spawn_mode',
   'args',       # List of strings that will be passed to the binary
   'hierarchy']  # List of tuples (name, pretty name)
)

def runs(args):
  '''Build a list of runs with given parameters.'''

  extra_args = args.get('args', [])
  hierarchy = args.get('hierarchy', [])
  timeout = args.get('timeout', 1)
  agents = args.get('agents', 5)
  obstacles = args.get('obstacles', 0.1)
  obstacle_move_freq_distr = args.get('obstacle_move_freq_distr', (5, 1))
  agents_spawn_mode = args.get('agents_spawn_mode', 'uniform')

  do_od = args.get('do_od', False)
  do_full_od = args.get('do_full_od', False)

  def run(hierarchy, args):
    return Run(timeout=timeout, agents=agents, obstacles=obstacles,
               obstacle_move_freq_distr=obstacle_move_freq_distr,
               agents_spawn_mode=agents_spawn_mode,
               hierarchy=hierarchy, args=args)

  whca_runs = [
    run(hierarchy=hierarchy + [('whca-{}'.format(n), 'WHCA* ({})'.format(n))],
        args=['--algorithm', 'whca', '--window', '{}'.format(n)] + extra_args)
    for n in (5, 10, 20)
  ]

  if do_od:
    od_runs = [
      run(hierarchy=hierarchy + [('od-{}'.format(n), 'OD/ID ({})'.format(n))],
          args=['--algorithm', 'od', '--window', '{}'.format(n)] + extra_args)
      for n in (5, 10)
    ]

    if do_full_od:
      od_runs += [
        run(hierarchy=hierarchy + [('od-0', 'OD/ID (∞)')],
            args=['--algorithm', 'od'] + extra_args)
      ]
  else:
    od_runs = []

  lra_run = run(hierarchy=hierarchy + [('lra', 'LRA*')],
                args=['--algorithm', 'lra'] + extra_args)

  return [lra_run] + whca_runs + od_runs


def product(f, *args):
  '''Given a function f(x1, x2, ..., xn) and n lists l1, l2, ..., ln, produce
  the list [f(v1, v2, ..., vn) for each vi in li].
  '''

  def do(bound, unbound):
    if len(unbound) == 0:
      return runs(f(*bound))

    l = []
    for value in unbound[0]:
      l.extend(do(bound + [value], unbound[ 1:]))

    return l

  return do([], args)


def join(*args):
  '''Given a list of lists [L1, L2, ..., LN], return the list that is the
  concatenation L1 + L2 + ... + LN.
  '''

  result = []

  for l in args:
    result.extend(l)

  return result

# Definitions of experiment sets.
set_runs = {
  'standard':
    product(
      lambda agents, obstacles, seed: {
        'agents': agents, 'obstacles': obstacles,
        'hierarchy': [('{}-agents'.format(agents), '{} agents'.format(agents)),
                      ('{}-obst'.format(obstacles),
                       '{} obstacles'.format(obstacles)),
                      ('seed-{}'.format(seed), ('Seed {}'.format(seed)))],
        'args': ['--seed', str(seed)],
        'do_od': True,
        'do_full_od': True
      },
      range(1, 10),
      (0.01, 0.2),
      seeds(3)
    ),
  'pack':
    product(
      lambda agents, obstacles, seed: {
        'agents': agents, 'obstacles': obstacles,
        'hierarchy': [('{}-agents'.format(agents), '{} agents'.format(agents)),
                      ('{}-obst'.format(obstacles),
                       '{} obstacles'.format(obstacles)),
                      ('seed-{}'.format(seed), ('Seed {}'.format(seed)))],
        'args': ['--seed', str(seed)],
        'agents_spawn_mode': 'pack',
        'do_od': True,
        'do_full_od': True
      },
      range(1, 10),
      (0.01, 0.2),
      seeds(3)
    ),
  'rejoin':
    join(
      product(
        lambda seed: {
          'args': ['--seed', str(seed)],
          'hierarchy': [('none', 'No rejoin'),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))]
        },
        seeds(3)
      ),
      product(
        lambda n, seed: {
          'args': ['--rejoin', str(n),
                   '--seed', str(seed)],
          'hierarchy': [('rejoin-{}'.format(n), '{} steps'.format(n)),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))]
        },
        (5, 10, 20, 40, 80),
        seeds(3)
      )
    ),
  'penalty':
    join(
      product(
        lambda seed: {
          'args': ['--seed', str(seed),
                   '--predictor-cutoff', '10'],
          'hierarchy': [('none', 'No predictor'),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True,
          'timeout': 1
        },
        seeds(3)
      ),
      product(
        lambda penalty, predictor, seed: {
          'args': ['--avoid', predictor,
                   '--obstacle-penalty', str(penalty),
                   '--obstacle-threshold', '0.75',
                   '--seed', str(seed)],
          'hierarchy': [
            ('{}-avoid-{}'.format(predictor, penalty),
             'Penalty {}; {}'.format(penalty, predictor)),
            ('seed-{}'.format(seed), 'Seed {}'.format(seed))
          ],
          'do_od': True,
          'timeout': 1
        },
        (5, 10, 15, 20),
        ('recursive', 'matrix'),
        seeds(3)
      )
    ),
  'cutoff':
    join(
      product(
        lambda predictor, seed: {
          'hierarchy': [('none', 'No predictor'),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True
        },
        ('recursive', 'matrix'),
        seeds(3)
      ),
      product(
        lambda cutoff, predictor, seed: {
          'args': ['--avoid', predictor,
                   '--obstacle-penalty', '10',
                   '--obstacle-threshold', '0.75',
                   '--predictor-cutoff', str(cutoff)],
          'hierarchy': [
            ('{}-cutoff-{}'.format(predictor, cutoff),
             '{} steps; {}'.format(cutoff, predictor)),
            ('seed-{}'.format(seed), 'Seed {}'.format(seed))
          ],
          'do_od': True
        },
        (1, 5, 10, 15, 20),
        ('recursive', 'matrix'),
        seeds(3)
      )
    ),
  'choices':
    join(
      product(
        lambda seed: {
          'args': ['--seed', str(seed)],
          'hierarchy': [('none', 'No predictor'),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True
        },
        seeds(100)
      ),
      product(
        lambda predictor, seed: {
          'args': ['--seed', str(seed),
                   '--avoid', predictor,
                   '--obstacle-penalty', '10',
                   '--obstacle-threshold', '0.75',
                   '--predictor-cutoff', '5'],
          'hierarchy': [(predictor, predictor),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True
        },
        ('recursive', 'matrix'),
        seeds(100)
      )
    ),

  'traffic':
    join(
      product(
        lambda seed: {
          'args': ['--seed', str(seed)],
          'hierarchy': [('none', 'No predictor'),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True,
          'timeout': 3
        },
        seeds(100)
      ),
      product(
        lambda predictor, seed: {
          'args': ['--seed', str(seed),
                   '--avoid', predictor,
                   '--obstacle-penalty', '10',
                   '--obstacle-threshold', '0.75',
                   '--predictor-cutoff', '5'],
          'hierarchy': [(predictor, predictor),
                        ('seed-{}'.format(seed), 'Seed {}'.format(seed))],
          'do_od': True,
          'timeout': 3
        },
        ('recursive', 'matrix'),
        seeds(100)
      )
    )
}

solver_path = Path('../bin/opt/cli')
maps_path = Path('../da-maps')
experiments_path = Path('../experiments')
fixed_scenarios = Path('../experiments-scenarios')

def make_scenario(map_info, map_path, run):
  '''Create a scenario for the given map.'''

  tiles = int(map_info['passable_tiles'])
  if tiles > 20000: return None

  return {
    'map': str(map_path),
    'obstacles': {
      'mode': 'random',
      'tile_probability': run.obstacles,
      'obstacle_movement': {
        'move_probability': {
          'distribution': 'normal',
          'parameters': list(run.obstacle_move_freq_distr)
        }
      }
    },
    'agent_settings': {
      'random_agents': min(int(run.agents), tiles),
      'spawn_mode': run.agents_spawn_mode
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
      {'map_info': info, 'result': result_data, 'completed': completed,
       'args': args},
      indent=2
    ))

    jobs.task_done()


def run_jobs():
  '''Run jobs in the job queue in parallel.'''

  print('Starting worker threads')

  workers = [threading.Thread(target=worker) for _ in range(threads)]
  for w in workers: w.start()

  jobs.join()

  for _ in range(threads): jobs.put(None)
  for w in workers: w.join()


def do_maps(maps, run, scenarios, dry):
  '''Run the experiments for one implementation and one configuration on all
  available maps.
  '''

  reset_jobs()

  for (map_path, info) in maps:
    scenario_dir = scenarios
    scenario_dir.mkdir(parents=True, exist_ok=True)
    scenario = make_scenario(info, make_relative(map_path, scenario_dir), run)
    if scenario is None: continue

    scenario_path = scenario_dir / (map_path.stem + '.json')
    scenario_path.open(mode='w').write(json.dumps(scenario, indent=2))
    result_path = scenario_dir / (map_path.stem + '.result.json')

    jobs.put((map_path.stem,
              [str(solver_path.resolve()),
               '--scenario', str(scenario_path.resolve())]
              + run.args,
              result_path, info, run.timeout))

  if not dry:
    run_jobs()


def make_out_dirs(set_name, run):
  '''Make output directories including meta.json files. Returns the output
  directory path.'''

  parent = experiments_path / set_name
  for name, pretty_name in run.hierarchy:
    (parent / name).mkdir(parents=True, exist_ok=True)

    with (parent / name / 'meta.json').open(mode='w') as out:
      json.dump({'name': pretty_name}, out)

    parent = parent / name

  return parent


def do_runs(scenario, runs, set_name, dry):
  '''Run the experiments for the given scenario on all available runs.'''

  reset_jobs()

  for run in runs:
    parent = make_out_dirs(set_name, run)

    with scenario.open() as s:
      scenario_data = json.load(s)

    map_path = scenario.parent / scenario_data['map']
    map_info_path = map_path.parent / (map_path.stem + '.json')

    with map_info_path.open() as mi:
      info = json.load(mi)

    result_path = parent / (map_path.stem + '.result.json')
    jobs.put((' / '.join(pretty_name for name, pretty_name in run.hierarchy),
              [str(solver_path.resolve()),
               '--scenario', str(scenario.resolve())]
              + run.args,
              result_path, info, run.timeout))

  if not dry:
    run_jobs()


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('--set', type=str, default='all',
                      help='Set name to run. Defaults to "all"')
  parser.add_argument('--dry', action='store_true')
  args = parser.parse_args()

  all_maps = []
  small_maps = []

  for f in maps_path.glob('*.json'):
    if f.stem == 'arena-choices': continue

    map_path = f.parent / (f.stem + '.txt')
    if not map_path.exists():
      print('No matching map file for {}'.format(f.name))
      continue

    info = json.load(f.open())

    if not info['connected']:
      print('{} not connected; skipping'.format(f.stem))
      continue

    all_maps.append((map_path, info))

    if 1000 <= info['passable_tiles'] < 3000:
      small_maps.append((map_path, info))

  sets = {
    'standard': (small_maps, None),
    'pack': (small_maps, None),
    'rejoin': (small_maps, None),
    'penalty': (small_maps, None),
    'cutoff': (small_maps, None),
    'choices': (None, 'choices.json'),
    'traffic': (None, 'traffic.json')
  }
  all_sets = list(sets.keys())

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

    maps, scenario = sets[set_name]
    assert maps is None or scenario is None

    if maps is not None:
      for run in set_runs[set_name]:
        print('=== {} ==='.format(
          set_name + ' / '
          + ' / '.join(pretty_name for (_, pretty_name) in run.hierarchy)
        ))
        name = '/'.join(name for (name, _) in run.hierarchy)

        parent = make_out_dirs(set_name, run)
        do_maps(maps, run, parent, args.dry)

    else:
      assert scenario is not None
      do_runs(fixed_scenarios / scenario, set_runs[set_name], set_name,
              args.dry)

if __name__ == '__main__':
  main()
