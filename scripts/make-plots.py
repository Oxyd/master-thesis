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


def avg_histogram(set_dir, key, extra=[], hierarchy=None):
  if hierarchy is None: hierarchy = Path(set_dir.name)

  (plots_dir / hierarchy).mkdir(parents=True, exist_ok=True)
  result = []

  meta_paths = list(set_dir.glob('*-meta.json'))
  if len(meta_paths) == 0:
    for d in set_dir.iterdir():
      result.extend(avg_histogram(d, key, extra, hierarchy / d.name))

    return result

  for meta_path in meta_paths:
    with meta_path.open() as meta_in:
      meta = json.load(meta_in)
      columns = meta[key]

      match = re.match(r'''(.+)-meta.json''', meta_path.name)
      if not match:
        raise RuntimeError('Invalid meta file name: {}'.format(meta_path.name))

      name = match.group(1)
      data_path = set_dir / (name + '.txt')

      result.append(
        (name,
         plot_header(hierarchy, name + '.png')
         + 'set style data histogram\n'
         + 'set auto x\n'
         + 'set style fill solid border -1\n'
         + 'set key left top\n'
         + '\n'.join(extra) + '\n'
         + 'plot base_dir."{}" using 2:xtic(1) title "{}",\\\n'.format(
           data_path.name, columns[0]
         )
         + ',\\\n'.join('  "" using {} title "{}"'.format(i + 2, columns[i])
                     for i in range(1, len(columns)))
         + '\n'
        )
      )

  return result

set_plots = {
  'full': scatter,
  'algos_small': lambda d: avg_histogram(d, 'algorithms', ['set logscale y']),
  'rejoin_small': lambda d: avg_histogram(d, 'heuristics',
                                          ['set xtics rotate by -45\n']),
  'predict_penalty': lambda d: avg_histogram(
    d, 'heuristics', ['set xtics rotate by -45\n']
  ),
  'predict_threshold': lambda d: avg_histogram(
    d, 'heuristics', ['set xtics rotate by -45\n']
  ),
  'predict_distrib': lambda d: avg_histogram(
    d, 'heuristics', ['set xtics rotate by -45\n']
  )
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
