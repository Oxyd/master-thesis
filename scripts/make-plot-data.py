#!/usr/bin/env python3

from pathlib import Path
import json
import re

input_dir = Path('../tmp/')
output_dir = Path('../experiments')

var_map = {
  'map_size': lambda d: d['map_info']['width'] * d['map_info']['height'],
  'ticks': lambda d: d['result']['ticks'],
  'time_ms': lambda d: d['result']['time_ms']
}

def get_var(data, name):
  return var_map[name](data)


def process_scatter(x, y, in_dir_path, out_file_path):
  '''Make scatter plot data for X axis `x', Y axis `y' using input data at
  the directory `in_dir_path' and writing results to the file `out_file_path'.
  '''

  with out_file_path.open(mode='w') as out:
    for f in in_dir_path.glob('*.result.json'):
      data = json.load(f.open())

      if not data['completed'] or not data['result']['success']:
        continue

      out.write('{} {}\n'.format(get_var(data, x), get_var(data, y)))


def scatter(set_dir):
  for run_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    for config_dir in (d for d in (run_dir).iterdir() if d.is_dir()):
      out = output_dir / set_dir.name / config_dir.name / run_dir.name
      out.parent.mkdir(parents=True, exist_ok=True)

      process_scatter('map_size', 'ticks', config_dir,
                      out.parent / (out.name + '-size-ticks.txt'))
      process_scatter('map_size', 'time_ms', config_dir,
                      out.parent / (out.name + '-size-time.txt'))


def natural_key(string_):
  """See http://www.codinghorror.com/blog/archives/001018.html"""
  has_no = string_.startswith('No') or string_.startswith('no')
  return (not has_no, [int(s) if s.isdigit() else s
                       for s in re.split(r'(\d+)', string_)])


def algo_compare(set_dir):
  '''Make histogram plot data for comparing algorithms on a small set of runs.'''

  data = {} # num_obstacles -> num_agents -> run -> avg time
  def add(obstacles, agents, run, avg):
    if obstacles not in data: data[obstacles] = {}
    if agents not in data[obstacles]: data[obstacles][agents] = {}
    data[obstacles][agents][run] = avg

  run_pretty_names = {}

  for run_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    with (run_dir / 'meta.json').open() as meta:
      info = json.load(meta)
      run_pretty_names[run_dir.name] = info['name']

    for config_dir in (d for d in run_dir.iterdir() if d.is_dir()):
      match = re.match(r'''(\d+)-agents-([0-9.]+)-obst''', config_dir.name)
      if match is None:
        raise RuntimeError('Invalid config name: {}'.format(config_dir.name))

      total_time = 0
      scenarios = 0
      for result_path in config_dir.glob('*.result.json'):
        with result_path.open() as f:
          result = json.load(f)
          if not result['completed']: continue

          total_time += float(result['result']['time_ms'])
          scenarios += 1

      if scenarios > 0:
        avg = total_time / scenarios
      else:
        avg = 0
      add(float(match.group(2)), int(match.group(1)), run_dir.name, avg)

  for obstacles in data:
    out_path = output_dir / set_dir.name / (str(obstacles) + '.txt')
    out_path.parent.mkdir(parents=True, exist_ok=True)

    runs = None

    with out_path.open(mode='w') as out:
      for agents in sorted(data[obstacles]):
        if runs is None:
          runs = sorted(data[obstacles][agents], key=natural_key)

        line = '"{}" '.format(agents)
        line += ' '.join(str(data[obstacles][agents][run]) for run in runs)
        out.write(line + '\n')

    out_info_path = output_dir / set_dir.name / (str(obstacles) + '-meta.json')
    with out_info_path.open(mode='w') as out:
      json.dump({'algorithms': list(run_pretty_names[r] for r in runs)}, out)


def heuristic_compare(set_dir):
  '''Make histogram plot data for comparing the effect different heuristics.'''

  data = {} # Algorithm name -> heuristic name -> avg time
  def add(algo, heuristic, avg):
    if algo not in data: data[algo] = {}
    data[algo][heuristic] = avg

  heuristics = []

  for run_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    with (run_dir / 'meta.json').open() as meta:
      info = json.load(meta)
      algo_name = info['name']
      heuristic_name = info['heuristic_name']

      if heuristic_name not in heuristics:
        heuristics.append(heuristic_name)

    configs = [d for d in run_dir.iterdir() if d.is_dir()]
    if len(configs) != 1:
      raise RuntimeError('Expected exactly 1 config for heuristic compare')

    config_dir = configs[0]

    total_time = 0
    scenarios = 0
    for result_path in config_dir.glob('*.result.json'):
      with result_path.open() as f:
        result = json.load(f)
        if not result['completed']: continue

        total_time += float(result['result']['time_ms'])
        scenarios += 1

    if scenarios > 0:
      avg = total_time / scenarios
    else:
      avg = 0

    add(algo_name, heuristic_name, avg)

  heuristics.sort(key=natural_key)
  algo_names = sorted(data.keys(), key=natural_key)

  out_path = output_dir / set_dir.name / (set_dir.name + '.txt')
  out_path.parent.mkdir(parents=True, exist_ok=True)
  with out_path.open(mode='w') as out:
    for algo_name in algo_names:
      line = '"{}" '.format(algo_name)
      line += ' '.join(str(data[algo_name][h]) for h in heuristics)
      out.write(line + '\n')

  out_info_path = output_dir / set_dir.name / (set_dir.name + '-meta.json')
  with out_info_path.open(mode='w') as out:
    json.dump({'heuristics': list(heuristics)}, out)


set_plots = {
  'full': scatter,
  'algos_small': algo_compare,
  'rejoin_small': heuristic_compare
}

for set_dir in (d for d in input_dir.iterdir() if d.is_dir()):
  if set_dir.name not in set_plots:
    print('set {} does not have plot data configuration'.format(set_dir.name))
    continue

  set_plots[set_dir.name](set_dir)
