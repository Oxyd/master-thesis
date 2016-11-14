#!/usr/bin/env python3

from pathlib import Path
import argparse
import json
import subprocess
import sys
import threading
import queue

def make_scenario(map_info, map_path):
  '''Create a scenario for the given map.'''

  return {
    'map': str(map_path),
    'obstacles': {
      'mode': 'random',
      'tile_probability': 0.05,
      'obstacle_movement': {
        'move_probability': {
          'distribution': 'normal',
          'parameters': [5, 1]
        }
      }
    },
    'agent_settings': {
      'random_agents': int(map_info['passable_tiles'] / 100)
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
timeout = None

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


def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('maps', help='Path to maps to run experiments on')
  parser.add_argument('scenarios',
                      help='Where to put resulting scenarios')
  parser.add_argument('solver', nargs=argparse.REMAINDER, metavar='solver args',
                      help='Solver invokation command line. {} is replaced by '
                      + 'the scenario path')
  parser.add_argument('--timeout', type=int,
                      help='Time, in seconds, to run the solver for')
  parser.add_argument('--threads', type=int, default=1,
                      help='How many threads to use for running the experiments')

  args = parser.parse_args()

  maps = Path(args.maps)
  scenarios = Path(args.scenarios)
  solver_args = args.solver

  global timeout
  if 'timeout' in args:
    timeout = args.timeout
  else:
    timeout = None

  scenarios.mkdir(parents=True, exist_ok=True)

  for f in maps.glob('*.json'):
    map_path = f.parent / (f.stem + '.map')
    if not map_path.exists():
      print('No matching map file for {}'.format(f.name))
      continue

    info = json.load(f.open())

    if not info['connected']:
      print('{} not connected; skipping'.format(f.stem))
      continue

    scenario_path = scenarios / (f.stem + '.json')
    scenario_path.open(mode='w').write(json.dumps(
      make_scenario(info, make_relative(map_path, scenarios)),
      indent=2
    ))

    result_path = scenarios / (f.stem + '.result.json')

    jobs.put((f.stem,
              substitute_scenario(solver_args, scenario_path.resolve()),
              result_path,
              info))

  print('Starting worker threads')

  threads = [threading.Thread(target=worker) for _ in range(args.threads)]
  for t in threads: t.start()

  jobs.join()

  for _ in range(args.threads): jobs.put(None)
  for t in threads: t.join()

if __name__ == '__main__':
  main()
