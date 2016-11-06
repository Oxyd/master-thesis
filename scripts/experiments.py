#!/usr/bin/env python3

from pathlib import Path
import json
import subprocess
import sys

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
  for i in range(i, len(relative_to)): result /= '..'

  return result.joinpath(*path[i :])

def substitute_scenario(solver_args, scenario_path):
  '''Replace {} in solver_args with scenario_path and return the resulting
  list.
  '''

  def subst(arg):
    if arg == '{}': return str(scenario_path)
    else: return arg

  return list(map(subst, solver_args))

def main():
  if len(sys.argv) < 3:
    print('Usage: {} <maps> <scenarios-to-create> <solver invokation>'.format(
      sys.argv[0]
    ))
    raise SystemExit(1)

  maps = Path(sys.argv[1])
  scenarios = Path(sys.argv[2])
  solver_args = sys.argv[3 :]

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

    print(f.stem)

    scenario_path = scenarios / (f.stem + '.json')
    scenario_path.open(mode='w').write(json.dumps(
      make_scenario(info, make_relative(map_path, scenarios)),
      indent=2
    ))

    result_path = scenarios / (f.stem + '.result.json')
    try:
      result = subprocess.run(substitute_scenario(solver_args,
                                                  scenario_path.resolve()),
                              stdout=subprocess.PIPE,
                              universal_newlines=True,
                              timeout=5)
      result_data = json.loads(result.stdout)
      completed = True
    except subprocess.TimeoutExpired:
      print('{} timed out '.format(f.stem))

      result_data = {}
      completed = False

    result_path.open(mode='w').write(json.dumps(
      {'map_info': info, 'result': result_data, 'completed': completed},
      indent=2
    ))

if __name__ == '__main__':
  main()
