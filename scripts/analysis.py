from collections import namedtuple
import json


SetData = namedtuple(
  'SetData',
  ['runs', # List of ((a1, a2, ..., an), [d]), where ai are attribute names, d
           # is a list of dicts with loaded JSON data for the set of attributes
   'attr_names'] # Pretty name for each attribute
)

def gather_data(set_dir):
  '''Go through the set directory, reading all run information and building a
  flat representation of it.'''

  names = {}

  def do(d, attrs):
    meta_path = d / 'meta.json'
    if meta_path.exists():
      with meta_path.open() as f:
        meta = json.load(f)
        if d.name in names:
          assert names[d.name] == meta['name']
        else:
          names[d.name] = meta['name']

      attrs = attrs + (d.name,)

    subdirs = list(s for s in d.iterdir() if s.is_dir())
    if len(subdirs) > 0:
      # This is an attribute dir and we need to recurse further

      result = []
      for subd in subdirs:
        result.extend(do(subd, attrs))

      return result

    else:
      # This is the final directory, we have the experiment data here.

      data = []
      for experiment in d.glob('*.result.json'):
        with experiment.open() as f:
          data.append(json.load(f))

      return [(attrs, data)]

  return SetData(runs=do(set_dir, ()), attr_names=names)


def find(key, runs):
  '''Return the list of experiment data for the given key. Key may contain
  None's in it, which act as a wildcard.
  '''

  result = []

  for k, d in runs:
    assert len(k) == len(key)
    if all(key[i] is None or key[i] == k[i] for i in range(len(key))):
      result.extend(d)

  return result


def get_path(d, path):
  '''get_path(d, (a, b, c)) -> d[a][b][c]'''

  x = d
  for e in path:
    x = x[e]

  return x


def average(key, runs, path, include_failed=False):
  num = 0
  total = 0

  experiments = find(key, runs)
  for e in experiments:
    if not include_failed and not e['completed']: continue

    num += 1
    total += float(get_path(e, path))

  if num > 0:
    return total / num
  else:
    return 0
