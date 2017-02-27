#!/usr/bin/env python3

from pathlib import Path
import subprocess

experiments_dir = Path('../experiments')
plots_dir = Path('../plots')

for config in (d for d in experiments_dir.iterdir() if d.is_dir()):
  (plots_dir / config.name).mkdir(parents=True, exist_ok=True)
  subprocess.run(['gnuplot',
                  '-e', "config='{}'".format(config.name),
                  './scatter.gpl'])
