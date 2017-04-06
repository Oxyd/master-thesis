#!/usr/bin/env python3

from pathlib import Path
import subprocess

experiments_dir = Path('../experiments')
plots_dir = Path('../plots')

set_plots = {
  'all': './scatter.gpl'
}

def main():
  for set_dir in (d for d in experiments_dir.iterdir() if d.is_dir()):
    if set_dir.name not in set_plots:
      print('set {} does not have plot configuration'.format(set_dir.name))
      continue

    for config in (d for d in set_dir.iterdir() if d.is_dir()):
      print('set = {}, config = {}'.format(set_dir.name, config.name))

      (plots_dir / set_dir.name / config.name).mkdir(parents=True,
                                                     exist_ok=True)
      subprocess.run(['gnuplot',
                      '-e', "set='{}'".format(set_dir.name),
                      '-e', "config='{}'".format(config.name),
                      set_plots[set_dir.name]])


if __name__ == '__main__':
  main()
