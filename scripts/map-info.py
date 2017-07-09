#!/usr/bin/env python3

from pathlib import Path
import json
import re
import sys

def _expect_match(expr, line):
  '''Match a line against a regular expression and return the match object.'''

  m = re.match(expr, line)
  if m is None:
    raise RuntimeError('Unexpected input: {}'.format(line))

  return m

def _tile_passable(tile):
  '''Given a character representing a map tile, decide whether it is passable.
  '''

  return tile in ('.', 'G')

class Map:
  '''A representation of the Dragon Age map format.'''

  def __init__(self, file):
    '''Read file and parse the map contained in it.'''

    _expect_match(r'type octile', file.readline())
    self.height = int(_expect_match(r'height (\d+)', file.readline()).group(1))
    self.width = int(_expect_match(r'width (\d+)', file.readline()).group(1))
    _expect_match(r'map', file.readline())

    self._data = []

    for y in range(self.height):
      line = file.readline()[: -1]

      if len(line) != self.width:
        raise RuntimeError('Unexpected line in map: {}'.format(line))

      self._data.extend(_tile_passable(x) for x in line)

    assert len(self._data) == self.width * self.height

  def in_bounds(self, x, y):
    '''Test whether x, y is in bounds of this map.'''

    return x >= 0 and x < self.width and y >= 0 and y < self.height

  def __getitem__(self, coords):
    '''map[x, y]: Whether the tile at x, y is passable or not.'''

    x, y = coords

    if not self.in_bounds(x, y):
      raise KeyError('{}, {} not in map', format(x, y))

    return self._data[y * self.width + x]

  def __iter__(self):
    '''Return an iterator over all valid positions in the map.'''

    for y in range(0, self.height):
      for x in range(0, self.width):
        yield x, y


def neighbours(map, x, y):
  '''Yield the neighbouring positions of x, y that are in the given map'''

  candidates = ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))

  for x, y in candidates:
    if map.in_bounds(x, y):
      yield x, y


def connected(map):
  '''Check whether a map is a connected graph.'''

  initial = None
  for x, y in map:
    if map[x, y]:
      initial = x, y
      break

  if initial is None:
    return False

  visited = set()
  queue = [initial]

  while len(queue) > 0:
    x, y = queue.pop(0)

    if (x, y) in visited:
      continue

    visited.add((x, y))

    for n_x, n_y in neighbours(map, x, y):
      if map[n_x, n_y]:
        queue.append((n_x, n_y))

  for x, y in map:
    if map[x, y] and (x, y) not in visited:
      return False

  return True

def passable_tiles(m):
  '''Return the number of passable tiles on the given map.'''

  return sum(map(lambda p: m[p[0], p[1]], m))

def main(dir):
  maps_dir = Path(dir)
  for m in (f for f in maps_dir.iterdir() if f.suffix == '.txt'):
    print('{}'.format(m.name))

    map = Map(m.open())
    info = {'width': map.width,
            'height': map.height,
            'connected': connected(map),
            'passable_tiles': passable_tiles(map)}

    print(info)

    info_path = m.parent / (m.stem + '.json')
    with info_path.open('w') as out:
      json.dump(info, out)


if __name__ == '__main__':
  main(sys.argv[1])
