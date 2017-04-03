#!/usr/bin/env python3

from pathlib import Path
import subprocess

experiments_dir = Path('../experiments')
plots_dir = Path('../plots')

for set_dir in (d for d in experiments_dir.iterdir() if d.is_dir()):
  for config in (d for d in set_dir.iterdir() if d.is_dir()):
    (plots_dir / set_dir.name / config.name).mkdir(parents=True, exist_ok=True)
    subprocess.run(['gnuplot',
                    '-e', "set='{}'".format(set_dir.name),
                    '-e', "config='{}'".format(config.name),
                    './scatter.gpl'])
