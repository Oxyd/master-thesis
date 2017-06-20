#!/usr/bin/env python3

from pathlib import Path
import sys

from analysis import *

input_dir = Path('../tmp/')

def print_table(table):
  '''Prints a table -- a list of lists -- as a LaTeX table.'''

  for row in table:
    print(' & '.join(str(c) for c in row) + ' \\\\')


def algos(data):
  agent_configs = list(set(run[0][0] for run in data.runs))
  obstacle_configs = set(run[0][1] for run in data.runs)
  algorithm_configs = list(set(run[0][3] for run in data.runs))

  agent_configs.sort(key=natural_key)
  algorithm_configs.sort(key=natural_key)

  for obst in obstacle_configs:
    print()
    print('obst = {}'.format(obst))
    print()

    table = [['\\rot{Agents}']
             + ['\\rot{{{}}}'.format(data.attr_names[a])
                for a in algorithm_configs]]
    for agents in agent_configs:
      table.append([data.attr_names[agents]])

      for algo in algorithm_configs:
        a = average((agents, obst, None, algo), data.runs,
                    ('completed',), True)
        table[-1].append('{}\\%'.format(int(100 * a)))

    print_table(table)


set_configs = {
  'algos_small': algos
}

def main():
  if len(sys.argv) != 2:
    print('Usage: {} <set-name>'.format(sys.argv[0]))
    return 1

  set = sys.argv[1]
  if set not in set_configs:
    print('Set {} does not have a configuration'.format(set))
    return 1

  set_configs[set](gather_data(input_dir / set))


if __name__ == '__main__':
  main()
