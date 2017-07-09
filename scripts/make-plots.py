#!/usr/bin/env python3

from matplotlib import cm
from matplotlib.font_manager import FontProperties
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import re

from analysis import *

input_dir = Path('../experiments/')
plot_dir = Path('../plots')

var_map = {
  'map_size': lambda d: d['map_info']['width'] * d['map_info']['height'],
  'ticks': lambda d: d['result']['ticks'],
  'time_ms': lambda d: d['result']['time_ms']
}

small_font = FontProperties()
small_font.set_size('small')

plt.figure(figsize=(8, 4))

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
    num_obstacles = num_obstacles.translate(str.maketrans({'.': '-'}))

    out_path = out_dir / (num_obstacles + '.pdf')
    success_out_path = out_dir / (num_obstacles + '-success.pdf')
    ticks_out_path = out_dir / (num_obstacles + '-ticks.pdf')
    recalcs_out_path = out_dir / (num_obstacles + '-recalcs.pdf')
    nodes_out_path = out_dir / (num_obstacles + '-nodes.pdf')

    out_dir.mkdir(parents=True, exist_ok=True)

    algo_names = list(data.attr_names[a] for a in algorithm_configs)
    algo_results = {}
    success_results = {}
    ticks_results = {}
    recalc_results = {}
    nodes_results = {}

    agent_nums = []
    for agents in agent_configs:
      num_agents = re.match(r'(\d+)-agents', agents).group(1)
      agent_nums.append(num_agents)

    for algo in algorithm_configs:
      for agents in agent_configs:
        if algo not in algo_results: algo_results[algo] = []
        if algo not in success_results: success_results[algo] = []
        if algo not in ticks_results: ticks_results[algo] = []
        if algo not in recalc_results: recalc_results[algo] = []
        if algo not in nodes_results: nodes_results[algo] = []

        key = (agents, obst, None, algo)

        algo_results[algo].append(average(key, data.runs,
                                          ('result', 'time_ms')))
        ticks_results[algo].append(average(key, data.runs, ('result', 'ticks')))
        success_results[algo].append(average(key, data.runs,
                                             ('completed',), True))
        recalc_results[algo].append(average(key, data.runs, recalculations))
        nodes_results[algo].append(average(key, data.runs, nodes_expanded))

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
    plot(lambda algo: ticks_results[algo], 'Makespan', False,
         str(ticks_out_path))
    plot(lambda algo: recalc_results[algo], 'Recalculations', False,
         str(recalcs_out_path))
    plot(lambda algo: nodes_results[algo], 'Nodes', False,
         str(nodes_out_path))


def average_compare(algorithms, heuristics, seeds, key, data):
  '''Compare the averages of each algorithm-heuristic combination.'''

  results = {}

  for algo in algorithms:
    for heuristic in heuristics:
      experiments_key = (heuristic, None, algo)
      if heuristic not in results: results[heuristic] = []
      results[heuristic].append(average(experiments_key, data.runs,
                                        key))

  return results


def percent_compare(algorithms, heuristics, seeds, key, data, base_heuristic):
  '''Compare percent improvement against a base heuristic.'''

  results = {}

  for algo in algorithms:
    for heuristic in heuristics:
      if heuristic == base_heuristic: continue
      if heuristic not in results: results[heuristic] = []

      total = 0

      for seed in seeds:
        base_key = (base_heuristic, seed, algo)
        experiment_key = (heuristic, seed, algo)

        b = average(base_key, data.runs, key)
        e = average(experiment_key, data.runs, key)

        if b > 0:
          total += e / b

      results[heuristic].append(total / len(seeds))

  return results


def heuristic_plot(data, out_path, key, y_label, compare, only_completed=True,
                   rotate=True, exclude_heuristics=[], compare_kwargs={}):
  # Expected hierarchy is (heuristic, seed, algorithm). We will produce one
  # output file.

  heuristics = list(set(run[0][0] for run in data.runs))
  seeds = list(set(run[0][1] for run in data.runs))
  algorithms = list(set(run[0][2] for run in data.runs))

  heuristics.sort(key=natural_key)
  algorithms.sort(key=natural_key)

  out_path.parent.mkdir(parents=True, exist_ok=True)

  algo_names = list(data.attr_names[a] for a in algorithms)
  heuristic_names = list(data.attr_names[h] for h in heuristics)

  heuristic_results = compare(algorithms, heuristics, seeds, key, data,
                              **compare_kwargs)

  index = np.arange(len(algorithms))
  bar_width = 0.8 / len(heuristics)

  plot_heuristics = heuristics.copy()
  for exclude in exclude_heuristics:
    plot_heuristics.remove(exclude)

  def plot(f, filename):
    plt.clf()

    for i, heuristic in enumerate(plot_heuristics):
      plt.bar(index + (i - 1) * bar_width, f(heuristic),
              bar_width, label=data.attr_names[heuristic],
              color=cm.Set1(i / len(plot_heuristics)))

    plt.xlabel('Algorithm')
    plt.ylabel(y_label)
    plt.xticks(index + len(plot_heuristics) / 2 * bar_width / 2, algo_names,
               rotation=-45 if rotate else 0)
    lgd = plt.legend(loc='upper left', prop=small_font,
                     bbox_to_anchor=(1.04, 1.0))
    plt.savefig(filename, bbox_extra_artists=[lgd], bbox_inches='tight')

  plot(lambda heuristic: heuristic_results[heuristic], str(out_path))


def heuristic_compare(data, out_path, key, y_label, only_completed=True,
                      rotate=True):
  '''Make histogram plot for comparing the effect of different
  heuristics.'''

  heuristic_plot(data, out_path, key, y_label, average_compare, only_completed,
                 rotate)


def heuristic_percent_compare(data, out_path, key, y_label, base_heuristic,
                              rotate=True):
  '''Make histogram plot for comparing the relative improvement of various
  heuristics over a base heuristic.
  '''

  heuristic_plot(data, out_path, key, y_label, percent_compare,
                 only_completed=True, rotate=rotate,
                 exclude_heuristics=[base_heuristic],
                 compare_kwargs={'base_heuristic': base_heuristic})


def rejoin_small(data, out_dir):
  heuristic_compare(data, out_dir / 'time.pdf',
                    ('result', 'time_ms'), 'Time (ms)',
                    rotate=False)
  heuristic_compare(data, out_dir / 'ticks.pdf',
                    ('result', 'ticks'), 'Makespan',
                    rotate=False)
  heuristic_compare(data, out_dir / 'rejoin-success-rate.pdf',
                    ('result', 'algorithm_statistics', 'Rejoin success rate'),
                    'Rejoin success rate')
  heuristic_compare(data, out_dir / 'success.pdf',
                    ('completed',), 'Solved %', only_completed=False)
  heuristic_compare(data, out_dir / 'nodes.pdf', nodes_expanded, 'Nodes')


def predict(data, out_dir):
  heuristic_compare(data, out_dir / 'time.pdf', ('result', 'time_ms'),
                    'Time (ms)')
  heuristic_compare(data, out_dir / 'ticks.pdf', ('result', 'ticks'),
                    'Makespan')
  heuristic_compare(data, out_dir / 'success.pdf', ('completed',),
                    'Success rate', only_completed=False)
  heuristic_compare(data, out_dir / 'recalcs.pdf', recalculations,
                    'Recalculations')
  heuristic_compare(data, out_dir / 'invalids.pdf', invalids,
                    'Times plan was invalid')
  heuristic_compare(data, out_dir / 'nodes.pdf', nodes_expanded, 'Nodes')
  heuristic_percent_compare(data, out_dir / 'ticks-percent.pdf',
                            ('result', 'ticks'), 'Makespan %',
                            base_heuristic='none')


def predict_algos(data, out_path):
  # Expected hierarchy is (predictor, setting, algorithm). We will
  # produce an output for each predictor setting.

  predictors = set(run[0][0] for run in data.runs)
  for p in predictors:
    subdata = SetData(
      runs=[(run[0][1 :], run[1]) for run in data.runs if run[0][0] == p],
      attr_names=data.attr_names
    )
    predict(subdata, out_path / p)


set_plots = {
  'algos_small': algo_compare,
  'pack_algos': algo_compare,
  'rejoin_small': lambda data, path: rejoin_small(data, path),
  'predict_penalty': lambda data, path: predict(data, path),
  'predict_cutoff': lambda data, path: predict(data, path),
  'choices': lambda data, path: predict(data, path),
  'traffic': lambda data, path: predict(data, path)
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
