#!/usr/bin/env python3

from collections import namedtuple
from matplotlib import cm
from matplotlib.font_manager import FontProperties
from pathlib import Path
import json
import matplotlib.pyplot as plt
import numpy as np
import re

input_dir = Path('../tmp/')
plot_dir = Path('../plots')

var_map = {
  'map_size': lambda d: d['map_info']['width'] * d['map_info']['height'],
  'ticks': lambda d: d['result']['ticks'],
  'time_ms': lambda d: d['result']['time_ms']
}

small_font = FontProperties()
small_font.set_size('small')

def get_var(data, name):
  return var_map[name](data)


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
  '''Return the list of experiment data for the given key. Key may contain
  None's in it, which act as a wildcard.
  '''

  result = []

  for k, d in runs:
    assert len(k) == len(key)
    if all(key[i] is None or key[i] == k[i] for i in range(len(key))):
      result.extend(d)

  return result


def algo_compare(data, out_dir):
  '''Make histogram plot data for comparing algorithms on a small set of
  runs.'''

  # Expected hierarchy is (agents, obstacles, seed, algorithm). We'll make a
  # separate output file for each obstacle count, so we want to group by that
  # first. Then we group by agents and compute the average time for the
  # obstacles/agents combination.

  agent_configs = list(set(run[0][0] for run in data.runs))
  obstacle_configs = set(run[0][1] for run in data.runs)
  algorithm_configs = list(set(run[0][3] for run in data.runs))

  agent_configs.sort(key=natural_key)
  algorithm_configs.sort(key=natural_key)

  for obst in obstacle_configs:
    num_obstacles = re.match(r'([0-9.]+)-obst', obst).group(1)
    out_path = out_dir / (num_obstacles + '.png')
    success_out_path = out_dir / (num_obstacles + '-success.png')

    out_dir.mkdir(parents=True, exist_ok=True)

    algo_names = list(data.attr_names[a] for a in algorithm_configs)
    algo_results = {}
    success_results = {}

    agent_nums = []
    for agents in agent_configs:
      num_agents = re.match(r'(\d+)-agents', agents).group(1)
      agent_nums.append(num_agents)

    for algo in algorithm_configs:
      time = []
      success = []

      for agents in agent_configs:
        experiments = find((agents, obst, None, algo), data.runs)

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

        if algo not in algo_results: algo_results[algo] = []
        if algo not in success_results: success_results[algo] = []

        algo_results[algo].append(avg)
        success_results[algo].append(scenarios / len(experiments))

    index = np.arange(len(agent_nums))
    bar_width = 0.8 / len(algo_names)

    plt.clf()

    for i, algo in enumerate(algorithm_configs):
      plt.bar(index + (i - 1) * bar_width, algo_results[algo], bar_width,
              label=data.attr_names[algo],
              color=cm.Set1(i / len(algorithm_configs)))

    plt.xlabel('Number of agents')
    plt.ylabel('Time (ms)')
    plt.yscale('log')
    plt.xticks(index + len(algo_names) * bar_width / 2, agent_nums)
    plt.legend(loc='best')

    plt.tight_layout()
    plt.savefig(str(out_path))

    plt.clf()

    for i, algo in enumerate(algorithm_configs):
      plt.bar(index + (i - 1) * bar_width,
              100 * np.array(success_results[algo]), bar_width,
              label=data.attr_names[algo],
              color=cm.Set1(i / len(algorithm_configs)))

    plt.xlabel('Number of agents')
    plt.ylabel('Success rate (%)')
    plt.xticks(index + len(algo_names) * bar_width / 2, agent_nums)
    plt.legend(loc='lower left')

    plt.tight_layout()
    plt.savefig(str(success_out_path))


def get_path(d, path):
  '''get_path(d, (a, b, c)) -> d[a][b][c]'''

  x = d
  for e in path:
    x = x[e]

  return x


def heuristic_compare(data, out_path, key, has_seed=False, only_completed=True):
  '''Make histogram plot data for comparing the effect of different
  heuristics.'''

  # Expected hierarchy is (heuristic, seed?, algorithm). We will produce one
  # output file.

  heuristics = list(set(run[0][0] for run in data.runs))
  algorithms = list(set(run[0][2 if has_seed else 1] for run in data.runs))

  heuristics.sort(key=natural_key)
  algorithms.sort(key=natural_key)

  out_path.parent.mkdir(parents=True, exist_ok=True)

  algo_names = list(data.attr_names[a] for a in algorithms)
  heuristic_names = list(data.attr_names[h] for h in heuristics)
  heuristic_results = {}
  success_results = {}

  for algo in algorithms:
    time = []
    success = []

    for heuristic in heuristics:
      if has_seed: experiments_key = (heuristic, None, algo)
      else:        experiments_key = (heuristic, algo)
      experiments = find(experiments_key, data.runs)

      num = 0
      total = 0.0
      for e in experiments:
        if only_completed and not e['completed']: continue

        total += float(get_path(e, key))
        num += 1

      if num > 0:
        avg = total / num
      else:
        avg = 0

      if heuristic not in heuristic_results: heuristic_results[heuristic] = []
      if heuristic not in success_results: success_results[heuristic] = []

      heuristic_results[heuristic].append(avg)
      success_results[heuristic].append(num / len(experiments))

  index = np.arange(len(algorithms))
  bar_width = 0.8 / len(heuristics)

  plt.clf()

  for i, heuristic in enumerate(heuristics):
    plt.bar(index + (i - 1) * bar_width, heuristic_results[heuristic],
            bar_width, label=data.attr_names[heuristic],
            color=cm.Set1(i / len(heuristics)))

  plt.xlabel('Algorithm')
  plt.ylabel('Time (ms)')
  plt.xticks(index + len(heuristics) / 2 * bar_width / 2, algo_names,
             rotation=-45)
  plt.legend(loc='upper center', prop=small_font, bbox_to_anchor=(0, 0))

  plt.tight_layout()
  plt.savefig(str(out_path))

  plt.clf()

  for i, heuristic in enumerate(heuristics):
    plt.bar(index + (i - 1) * bar_width,
            100 * np.array(success_results[heuristic]), bar_width,
            label=data.attr_names[heuristic],
            color=cm.Set1(i / len(heuristics)))

  plt.xlabel('Algorithm')
  plt.ylabel('Success rate (%)')
  plt.xticks(index + len(heuristics) / 2 * bar_width / 2, algo_names,
             rotation=-45)
  plt.legend(loc='upper center', prop=small_font, bbox_to_anchor=(0, 0))

  plt.tight_layout()
  plt.savefig(str(out_path.parent / (out_path.stem + '-success.png')))


def rejoin_small(data, out_dir, has_seed=False):
  heuristic_compare(data, out_dir / 'time.png',
                    ('result', 'time_ms'),
                    has_seed)
  heuristic_compare(data, out_dir / 'rejoin_success.png',
                    ('result', 'algorithm_statistics', 'Rejoin success rate'),
                    has_seed)


def predict(data, out_dir, has_seed=False):
  heuristic_compare(data, out_dir / 'time.png', ('result', 'time_ms'), has_seed)
  heuristic_compare(data, out_dir / 'ticks.png', ('result', 'ticks'), has_seed)
  heuristic_compare(data, out_dir / 'success.png', ('completed',),
                    has_seed=True, only_completed=False)


def predict_algos(data, out_path, has_seed=False):
  # Expected hierarchy is (predictor, setting, seed?, algorithm). We will
  # produce an output for each predictor setting.

  predictors = set(run[0][0] for run in data.runs)
  for p in predictors:
    subdata = SetData(
      runs=[(run[0][1 :], run[1]) for run in data.runs if run[0][0] == p],
      attr_names=data.attr_names
    )
    predict(subdata, out_path / p, has_seed)


set_plots = {
  'algos_small': algo_compare,
  'pack_algos': algo_compare,
  'rejoin_small': lambda data, path: rejoin_small(data, path, True),
  'predict_penalty': lambda data, path: predict_algos(data, path, True),
  'predict_threshold': lambda data, path: predict_algos(data, path, True),
  'predict_cutoff': lambda data, path: predict_algos(data, path, True),
  'predict_distrib': lambda data, path: predict_algos(data, path, True),
  'choices': lambda data, path: predict(data, path, True),
  'traffic': lambda data, path: predict(data, path, True)
}

def main():
  for set_dir in (d for d in input_dir.iterdir() if d.is_dir()):
    if set_dir.name not in set_plots:
      print('set {} does not have plot data configuration'.format(set_dir.name))
      continue

    print('> {}'.format(set_dir.name))
    set_plots[set_dir.name](gather_data(set_dir), plot_dir / set_dir.name)

if __name__ == '__main__':
  main()
