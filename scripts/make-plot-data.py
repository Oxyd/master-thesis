#!/usr/bin/env python3

from collections import namedtuple
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


SetData = namedtuple(
  'SetData',
  ['runs', # List of ((a1, a2, ..., an), [d]), where ai are attribute names, d
           # is a list of dicts with loaded JSON data for the set of attributes
   'attr_names'] # Pretty name for each attribute
)

def gather_data(set_dir):
  '''Go through the set directory, reading all run information and building a
  flat representation of it.'''

  names = {}

  def do(d, attrs):
    meta_path = d / 'meta.json'
    if meta_path.exists():
      with meta_path.open() as f:
        meta = json.load(f)
        if d.name in names:
          assert names[d.name] == meta['name']
        else:
          names[d.name] = meta['name']

      attrs = attrs + (d.name,)

    subdirs = list(s for s in d.iterdir() if s.is_dir())
    if len(subdirs) > 0:
      # This is an attribute dir and we need to recurse further

      result = []
      for subd in subdirs:
        result.extend(do(subd, attrs))

      return result

    else:
      # This is the final directory, we have the experiment data here.

      data = []
      for experiment in d.glob('*.result.json'):
        with experiment.open() as f:
          data.append(json.load(f))

      return [(attrs, data)]

  return SetData(runs=do(set_dir, ()), attr_names=names)


def find(key, runs):
  '''Return the list of experiment data for the given key.'''

  for k, d in runs:
    if k == key:
      return d

  return None


def algo_compare(data, out_dir):
  '''Make histogram plot data for comparing algorithms on a small set of
  runs.'''

  # Expected hierarchy is (agents, obstacles, algorithm). We'll make a separate
  # output file for each obstacle count, so we want to group by that first. Then
  # we group by agents and compute the average time for the obstacles/agents
  # combination -- that will give a single line in the output file.

  agent_configs = list(set(run[0][0] for run in data.runs))
  obstacle_configs = set(run[0][1] for run in data.runs)
  algorithm_configs = list(set(run[0][2] for run in data.runs))

  agent_configs.sort(key=natural_key)
  algorithm_configs.sort(key=natural_key)

  for obst in obstacle_configs:
    num_obstacles = re.match(r'([0-9.]+)-obst', obst).group(1)
    out_path = out_dir / (num_obstacles + '.txt')

    out_dir.mkdir(parents=True, exist_ok=True)

    out_info_path = out_dir / (num_obstacles + '-meta.json')
    with out_info_path.open(mode='w') as out:
      json.dump({'algorithms': list(data.attr_names[a]
                                    for a in algorithm_configs)},
                out)

    with out_path.open(mode='w') as out:
      for agents in agent_configs:
        num_agents = re.match(r'(\d+)-agents', agents).group(1)

        line = '"{}" '.format(num_agents)

        algorithm_results = []
        for algo in algorithm_configs:
          experiments = find((agents, obst, algo), data.runs)

          total_time = 0
          scenarios = 0
          for e in experiments:
            if not e['completed']: continue

            total_time += float(e['result']['time_ms'])
            scenarios += 1

          if scenarios > 0:
            avg = total_time / scenarios
          else:
            avg = 0

          algorithm_results.append(str(avg))

        line += ' '.join(algorithm_results)
        out.write(line + '\n')


def get_path(d, path):
  '''get_path(d, (a, b, c)) -> d[a][b][c]'''

  x = d
  for e in path:
    x = x[e]

  return x


def heuristic_compare(data, out_path, key):
  '''Make histogram plot data for comparing the effect of different
  heuristics.'''

  # Expected hierarchy is (heuristic, algorithm). We will produce one output
  # file with one line for each algorithm. Each line contains a column for each
  # heuristic with the average of the quantity given by `key'.

  heuristics = list(set(run[0][0] for run in data.runs))
  algorithms = list(set(run[0][1] for run in data.runs))

  heuristics.sort(key=natural_key)
  algorithms.sort(key=natural_key)

  out_path.parent.mkdir(parents=True, exist_ok=True)

  out_meta_path = out_path.parent / (out_path.stem + '-meta.json')
  with out_meta_path.open(mode='w') as f:
    json.dump({'heuristics': list(data.attr_names[h] for h in heuristics)}, f)

  with out_path.open(mode='w') as out:
    for algo in algorithms:
      line = '"{}" '.format(data.attr_names[algo])

      heuristics_results = []
      for heuristic in heuristics:
        experiments = find((heuristic, algo), data.runs)

        num = 0
        total = 0.0
        for e in experiments:
          if not e['completed']: continue

          total += float(get_path(e, key))
          num += 1

        if num > 0:
          avg = total / num
        else:
          avg = 0

        heuristics_results.append(str(avg))

      line += ' '.join(heuristics_results)
      out.write(line + '\n')


def rejoin_small(data, out_dir):
  heuristic_compare(data, out_dir / 'time.txt',
                    ('result', 'time_ms'))
  heuristic_compare(data, out_dir / 'rejoin_success.txt',
                    ('result', 'algorithm_statistics', 'Rejoin success rate'))


def predict(data, out_dir):
  heuristic_compare(data, out_dir / 'time.txt', ('result', 'time_ms'))
  heuristic_compare(data, out_dir / 'ticks.txt', ('result', 'ticks'))


def predict_algos(data, out_path):
  # Expected hierarchy is (predictor, setting, algorithm). We will produce an
  # output for each predictor setting.

  predictors = set(run[0][0] for run in data.runs)
  for p in predictors:
    subdata = SetData(
      runs=[(run[0][1 :], run[1]) for run in data.runs if run[0][0] == p],
      attr_names=data.attr_names
    )
    predict(subdata, out_path / p)

set_plots = {
  'full': scatter,
  'algos_small': algo_compare,
  'rejoin_small': lambda data, path: rejoin_small(data, path),
  'predict_penalty': lambda data, path: predict_algos(data, path),
  'predict_threshold': lambda data, path: predict_algos(data, path)
}

def main():
  for set_dir in (d for d in input_dir.iterdir() if d.is_dir()):
    if set_dir.name not in set_plots:
      print('set {} does not have plot data configuration'.format(set_dir.name))
      continue

    set_plots[set_dir.name](gather_data(set_dir), output_dir / set_dir.name)

if __name__ == '__main__':
  main()
