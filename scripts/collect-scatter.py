#!/usr/bin/env python3

from pathlib import Path
import argparse
import json

var_map = {
  'map_size': lambda d: d['map_info']['width'] * d['map_info']['height'],
  'ticks': lambda d: d['result']['ticks'],
  'time_ms': lambda d: d['result']['time_ms']
}

def get_var(data, name):
  return var_map[name](data)

def main():
  parser = argparse.ArgumentParser()
  parser.add_argument('x', choices=['map_size'], help='X axis data')
  parser.add_argument('y', choices=['ticks', 'time_ms'], help='Y axis data')
  parser.add_argument('results', help='Directory with experiment results')
  parser.add_argument('out', help='Output file')
  args = parser.parse_args()

  in_path = Path(args.results)

  out_path = Path(args.out)
  out_path.parent.mkdir(parents=True, exist_ok=True)
  out = out_path.open(mode='w')

  result = []

  for f in in_path.glob('*.result.json'):
    data = json.load(f.open())

    if not data['completed'] or not data['result']['success']:
      continue

    out.write('{} {}\n'.format(get_var(data, args.x), get_var(data, args.y)))


if __name__ == '__main__':
  main()
