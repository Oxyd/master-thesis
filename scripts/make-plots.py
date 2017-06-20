#!/usr/bin/env python3

from matplotlib import cm
from matplotlib.font_manager import FontProperties
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import re

from analysis import *

input_dir = Path('../tmp/')
plot_dir = Path('../plots')

var_map = {
  'map_size': lambda d: d['map_info']['width'] * d['map_info']['height'],
  'ticks': lambda d: d['result']['ticks'],
  'time_ms': lambda d: d['result']['time_ms']
}

small_font = FontProperties()
small_font.set_size('small')

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
    out_path = out_dir / (num_obstacles + '.pdf')
    success_out_path = out_dir / (num_obstacles + '-success.pdf')
    ticks_out_path = out_dir / (num_obstacles + '-ticks.pdf')

    out_dir.mkdir(parents=True, exist_ok=True)

    algo_names = list(data.attr_names[a] for a in algorithm_configs)
    algo_results = {}
    success_results = {}
    ticks_results = {}

    agent_nums = []
    for agents in agent_configs:
      num_agents = re.match(r'(\d+)-agents', agents).group(1)
      agent_nums.append(num_agents)

    for algo in algorithm_configs:
      for agents in agent_configs:
        if algo not in algo_results: algo_results[algo] = []
        if algo not in success_results: success_results[algo] = []
        if algo not in ticks_results: ticks_results[algo] = []

        key = (agents, obst, None, algo)

        algo_results[algo].append(average(key, data.runs,
                                          ('result', 'time_ms')))
        ticks_results[algo].append(average(key, data.runs, ('result', 'ticks')))
        success_results[algo].append(average(key, data.runs,
                                             ('completed',), True))

    index = np.arange(len(agent_nums))
    bar_width = 0.8 / len(algo_names)

    def plot(f, y_label, log, filename):
      plt.clf()

      for i, algo in enumerate(algorithm_configs):
        plt.bar(index + (i - 1) * bar_width, f(algo), bar_width,
                label=data.attr_names[algo],
                color=cm.Set1(i / len(algorithm_configs)))

      plt.xlabel('Number of agents')
      plt.ylabel(y_label)
      if log: plt.yscale('log')
      plt.xticks(index + len(algo_names) / 2 * bar_width / 2, agent_nums)

      lgd = plt.legend(loc='upper left', prop=small_font,
                       bbox_to_anchor=(1.04, 1.00))

      plt.savefig(filename, bbox_extra_artists=[lgd], bbox_inches='tight')

    plot(lambda algo: algo_results[algo], 'Time (ms)', True, str(out_path))
    plot(lambda algo: 100 * np.array(success_results[algo]),
         'Success rate (%)', False, str(success_out_path))
    plot(lambda algo: ticks_results[algo], 'Length', False, str(ticks_out_path))


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
    for heuristic in heuristics:
      if has_seed: experiments_key = (heuristic, None, algo)
      else:        experiments_key = (heuristic, algo)

      if heuristic not in heuristic_results: heuristic_results[heuristic] = []
      if heuristic not in success_results: success_results[heuristic] = []

      heuristic_results[heuristic].append(average(experiments_key, data.runs,
                                                  key))
      success_results[heuristic].append(average(experiments_key, data.runs,
                                                ('completed',)))

  index = np.arange(len(algorithms))
  bar_width = 0.8 / len(heuristics)

  def plot(f, y_label, filename):
    plt.clf()

    for i, heuristic in enumerate(heuristics):
      plt.bar(index + (i - 1) * bar_width, f(heuristic),
              bar_width, label=data.attr_names[heuristic],
              color=cm.Set1(i / len(heuristics)))

    plt.xlabel('Algorithm')
    plt.ylabel(y_label)
    plt.xticks(index + len(heuristics) / 2 * bar_width / 2, algo_names,
               rotation=-45)
    lgd = plt.legend(loc='upper left', prop=small_font,
                     bbox_to_anchor=(1.04, 1.0))
    plt.savefig(filename, bbox_extra_artists=[lgd], bbox_inches='tight')

  plot(lambda heuristic: heuristic_results[heuristic], 'Time (ms)',
       str(out_path))
  plot(lambda heuristic: 100 * np.array(success_results[heuristic]),
       'Success rate (%)', str(out_path.parent / (out_path.stem + '-success.png')))


def rejoin_small(data, out_dir, has_seed=False):
  heuristic_compare(data, out_dir / 'time.png',
                    ('result', 'time_ms'),
                    has_seed)
  heuristic_compare(data, out_dir / 'ticks.png',
                    ('result', 'ticks'),
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
