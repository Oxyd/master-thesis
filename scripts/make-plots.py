#!/usr/bin/env python3

from pathlib import Path
import re
import subprocess

experiments_dir = Path('../experiments')
plots_dir = Path('../plots')
tmp_dir = Path('../gnuplot')

def plot_header(set_dir, config_dir, out_name):
  return '''\
set term png size 600, 400
set colorsequence podo
base_dir = "{}/"
out_dir = "{}/"
set output out_dir."{}"
'''.format(
  Path('../experiments/' + set_dir.name + '/' + config_dir.name).resolve(),
  Path('../plots/' + set_dir.name + '/' + config_dir.name).resolve(),
  out_name
)

def scatter_plots(set_dir, config_dir, y_name, y_label):
  runs = []
  for run_name in (experiments_dir / set_dir.name / config_dir.name).iterdir():
    m = re.match('lra-size-{}'.format(y_name), run_name.name)
    if m is not None:
      runs.append((run_name.name, 'LRA'))
      continue

    m = re.match(r'''whca-(\d+)-size-{}'''.format(y_name), run_name.name)
    if m is not None:
      runs.append((run_name.name, 'WHCA* ({})'.format(m.group(1))))

  common = '''\
set xlabel "Map size (width × height) "
set ylabel "{}"
'''.format(y_label)

  return common + 'plot ' + ', '.join(
    'base_dir."{}" smooth unique lw 2 title "{}"'.format(name, title)
    for (name, title) in runs
  )


def scatter(set_dir, config_dir):
  return (('scatter-size-ticks',
           plot_header(set_dir, config_dir, 'scatter-size-ticks.png')
           + scatter_plots(set_dir, config_dir, 'ticks', 'Average ticks')),
          ('scatter-size-time',
           plot_header(set_dir, config_dir, 'scatter-size-time.png')
           + scatter_plots(set_dir, config_dir, 'time', 'Average time (ms)')))


set_plots = {
  'all': scatter
}

def run(script):
  (name, text) = script
  path = tmp_dir / (name + '.gpl')

  path.parent.mkdir(parents=True, exist_ok=True)
  with path.open(mode='w') as f:
    f.write(text)

  subprocess.run(['gnuplot', str(path)])


def main():
  for set_dir in (d for d in experiments_dir.iterdir() if d.is_dir()):
    if set_dir.name not in set_plots:
      print('set {} does not have plot configuration'.format(set_dir.name))
      continue

    for config in (d for d in set_dir.iterdir() if d.is_dir()):
      print('set = {}, config = {}'.format(set_dir.name, config.name))

      (plots_dir / set_dir.name / config.name).mkdir(parents=True,
                                                     exist_ok=True)
      for script in set_plots[set_dir.name](set_dir, config):
        run(script)


if __name__ == '__main__':
  main()
