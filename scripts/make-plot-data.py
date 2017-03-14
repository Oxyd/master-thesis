#!/usr/bin/env python3

from pathlib import Path
import subprocess

input_dir = Path('../tmp/')
output_dir = Path('../experiments')

def scatter(in_, out):
  subprocess.run(['./collect-scatter.py', 'map_size', 'ticks',
                  str(in_), str(out) + '-size-ticks.txt'])
  subprocess.run(['./collect-scatter.py', 'map_size', 'time_ms',
                  str(in_), str(out) + '-size-time.txt'])

for run_dir in (d for d in input_dir.iterdir() if d.is_dir()):
  for config_dir in (d for d in (run_dir).iterdir() if d.is_dir()):
    scatter(config_dir, output_dir / config_dir.name / run_dir.name)
