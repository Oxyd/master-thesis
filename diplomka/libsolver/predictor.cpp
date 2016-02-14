#include "predictor.hpp"

predictor::impassable_reserved::impassable_reserved(
  reservation_table_type const& reservations,
  agent const& agent,
  position from
)
  : reservations_(reservations)
  , agent_(agent)
  , from_(from)
{ }

bool
predictor::impassable_reserved::operator () (
  position where, position from, world const& w, unsigned distance
) {
  if (reservations_.count(position_time{where, w.tick() + distance}))
    return false;

  auto vacated = reservations_.find(
    position_time{from, w.tick() + distance}
  );
  if (vacated != reservations_.end() &&
      vacated->second.from &&
      *vacated->second.from == where)
    return false;

  return w.get(where) == tile::free || !neighbours(where, from_);
}

void
predictor::reserve(agent::id_type a_id, path const& path, tick_t from) {
  for (tick_t distance = 0; distance < path.size(); ++distance) {
    position const p = path[path.size() - distance - 1];
    position_time const pt{p, from + distance};

    assert(!agent_reservations_.count(pt));

    if (distance > 0)
      agent_reservations_[pt] = {a_id, path[path.size() - distance]};
    else
      agent_reservations_[pt] = {a_id, boost::none};
  }
}

void
predictor::unreserve(agent::id_type a_id) {
  auto it = agent_reservations_.begin();
  while (it != agent_reservations_.end())
    if (it->second.agent == a_id)
      it = agent_reservations_.erase(it);
    else
      ++it;
}

auto
predictor::impassable_predicate(agent const& a, position from) const
  -> impassable_reserved
{
  return impassable_reserved{agent_reservations_, a, from};
}
