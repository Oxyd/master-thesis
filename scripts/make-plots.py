#!/usr/bin/env python3

from pathlib import Path
import json
import re
import subprocess

experiments_dir = Path('../experiments')
plots_dir = Path('../plots')
tmp_dir = Path('../gnuplot')

def plot_header(subdir, out_name):
  return '''\
set term png size 600, 400
set colorsequence podo
base_dir = "{}/"
out_dir = "{}/"
set output out_dir."{}"
'''.format(
  (Path('../experiments') / subdir).resolve(),
  (Path('../plots') / subdir).resolve(),
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


def scatter(set_dir):
  scripts = []

  for config_dir in (d for d in set_dir.iterdir() if d.is_dir()):
    print('  > config = {}'.format(config_dir.name))

    (plots_dir / set_dir.name / config_dir.name).mkdir(parents=True,
                                                       exist_ok=True)

    scripts.append(
      ('scatter-size-ticks',
       plot_header('{}/{}'.format(set_dir.name, config_dir.name),
                   'scatter-size-ticks.png')
       + scatter_plots(set_dir, config_dir, 'ticks', 'Average ticks'))
    )
    scripts.append(
      ('scatter-size-time',
       plot_header('{}/{}'.format(set_dir.name, config_dir.name),
                   'scatter-size-time.png')
       + scatter_plots(set_dir, config_dir, 'time', 'Average time (ms)'))
    )

  return scripts


def avg_time(set_dir):
  (plots_dir / set_dir.name).mkdir(parents=True, exist_ok=True)
  result = []

  for meta_path in set_dir.glob('*-meta.json'):
    with meta_path.open() as meta_in:
      meta = json.load(meta_in)
      algorithms = meta['algorithms']

      match = re.match(r'''([0-9.]+)-meta.json''', meta_path.name)
      if not match:
        raise RuntimeError('Invalid meta file name: {}'.format(meta_path.name))

      name = match.group(1)
      data_path = set_dir / (name + '.txt')

      result.append(
        (name,
         plot_header(set_dir.name, name + '.png')
         + 'set style data histogram\n'
         + 'set auto x\n'
         + 'set style fill solid border -1\n'
         + 'set key left top\n'
         + 'set logscale y\n'
         + 'plot base_dir."{}" using 2:xtic(1) title "{}",\\\n'.format(
           data_path.name, algorithms[0]
         )
         + ',\\\n'.join('  "" using {} title "{}"'.format(i + 2, algorithms[i])
                     for i in range(1, len(algorithms)))
         + '\n'
        )
      )

  return result

set_plots = {
  'full': scatter,
  'small': avg_time
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

    print('set = {}'.format(set_dir.name))
    for script in set_plots[set_dir.name](set_dir):
      run(script)


if __name__ == '__main__':
  main()
