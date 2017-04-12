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
  return [int(s) if s.isdigit() else s for s in re.split(r'(\d+)', string_)]


def avg_time(set_dir):
  '''Make histogram plot data for comparing algorithms on a small set of runs.'''

  data = {} # num_agents -> run -> avg time
  def add(agents, run, avg):
    if agents not in data: data[agents] = {}
    data[agents][run] = avg

  for run_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    for config_dir in (d for d in run_dir.iterdir() if d.is_dir()):
      m = re.match(r'''(\d+)-agents-0\.01-obst''', config_dir.name)
      if m is None:
        raise RuntimeError('Invalid config name: {}'.format(config_dir.name))

      total_time = 0
      scenarios = 0
      for result_path in config_dir.glob('*.result.json'):
        with result_path.open() as f:
          result = json.load(f)
          if not result['completed']: continue

          total_time += float(result['result']['time_ms'])
          scenarios += 1

      add(int(m.group(1)), run_dir.name, total_time / scenarios)

  out_path = (output_dir / set_dir.name / 'data.txt')
  out_path.parent.mkdir(parents=True, exist_ok=True)

  runs = None

  with out_path.open(mode='w') as out:
    for agents in sorted(data):
      if runs is None:
        runs = sorted(data[agents], key=natural_key)

      line = '"{}" '.format(agents)
      line += ' '.join(str(data[agents][run]) for run in runs)
      out.write(line + '\n')

  out_info_path = (output_dir / set_dir.name / 'data-meta.json')
  with out_info_path.open(mode='w') as out:
    json.dump({'algorithms': list(runs)}, out)


set_plots = {
  'full': scatter,
  'small': avg_time
}

for set_dir in (d for d in input_dir.iterdir() if d.is_dir()):
  if set_dir.name not in set_plots:
    print('set {} does not have plot data configuration'.format(set_dir.name))
    continue

  set_plots[set_dir.name](set_dir)
