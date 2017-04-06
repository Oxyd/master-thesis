#!/usr/bin/env python3

from pathlib import Path
import json

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

  out_file_path.parent.mkdir(parents=True, exist_ok=True)

  with out_file_path.open(mode='w') as out:
    for f in in_dir_path.glob('*.result.json'):
      data = json.load(f.open())

      if not data['completed'] or not data['result']['success']:
        continue

      out.write('{} {}\n'.format(get_var(data, x), get_var(data, y)))


def scatter(in_, out):
  process_scatter('map_size', 'ticks', in_,
                  out.parent / (out.name + '-size-ticks.txt'))
  process_scatter('map_size', 'time_ms', in_,
                  out.parent / (out.name + '-size-time.txt'))

for set_dir in (d for d in input_dir.iterdir() if d.is_dir()):
  for run_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    for config_dir in (d for d in (run_dir).iterdir() if d.is_dir()):
      scatter(config_dir,
              output_dir / set_dir.name / config_dir.name / run_dir.name)
