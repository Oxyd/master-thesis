#include "world_scene.hpp"
#include "world.hpp"

#include <QBrush>
#include <QColor>
#include <QGraphicsEllipseItem>
#include <QGraphicsSceneMouseEvent>
#include <QPainterPath>
#include <QPen>
#include <QPoint>
#include <QRect>

constexpr double tile_size = 10;

static QRectF
tile_rect(position::coord_type x, position::coord_type y) {
  return {x * tile_size, y * tile_size, tile_size, tile_size};
}

static QRectF
tile_rect(position p) {
  return tile_rect(p.x, p.y);
}

void
world_scene::attach(world const* w) {
  if (world_) {
    clear();
    agent_items_.clear();
    obstacle_items_.clear();
  }

  world_ = w;

  if (w)
    re_render();
}

static QPen const black_pen{{0, 0, 0}};
static QPen const highlight_pen{QBrush{QColor{0, 0, 0}}, 2};
static QPen const target_pen{QBrush{QColor{127, 127, 127, 127}}, 1};
static QBrush const wall_brush{QColor{0, 0, 0}};
static QBrush const obstacle_brush{QColor{255, 0, 0}};
static QBrush const brush{{0, 255, 0}};

void
world_scene::update() {
  assert(world_);

  for (auto& pos_items : agent_items_)
    for (QGraphicsItem* item : pos_items.second)
      removeItem(item);

  for (QGraphicsItem* item : obstacle_items_)
    removeItem(item);

  agent_items_.clear();
  obstacle_items_.clear();

  for (auto const& pos_agent : world_->agents())
    render_agent(pos_agent.first, pos_agent.second);

  for (auto const& pos_obstacle : world_->obstacles())
    render_obstacle(pos_obstacle.first);
}

void
world_scene::re_render() {
  if (!world_)
    return;

  agent_items_.clear();
  obstacle_items_.clear();
  clear();

  for (auto t : *world_->map())
    if (t.tile == tile::wall)
      addRect(tile_rect(t.x, t.y), black_pen, wall_brush);

  for (auto const& pos_agent : world_->agents()) {
    position const pos = pos_agent.first;
    agent const& agent = pos_agent.second;

    render_agent(pos, agent);
  }

  for (auto const& pos_obstacle : world_->obstacles())
    render_obstacle(pos_obstacle.first);
}

void
world_scene::update_agent(position pos) {
  assert(world_);

  if (agent_items_.count(pos))
    for (QGraphicsItem* item : agent_items_[pos])
      removeItem(item);

  if (world_->get_agent(pos))
    render_agent(pos, *world_->get_agent(pos));
}

void
world_scene::highlight_agent(position p, bool set) {
  if (set)
    highlighted_.insert(p);
  else
    highlighted_.erase(p);
}

void
world_scene::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  int x = event->scenePos().x() / tile_size;
  int y = event->scenePos().y() / tile_size;

  emit mouse_moved(x, y);
}

void
world_scene::mouseReleaseEvent(QGraphicsSceneMouseEvent* event) {
  int x = event->scenePos().x() / tile_size;
  int y = event->scenePos().y() / tile_size;

  emit mouse_clicked(x, y);
}

void
world_scene::render_agent(position pos, agent const& agent) {
  std::vector<QGraphicsItem*> items;

  QPen const& pen = highlighted_.count(pos) ? highlight_pen : black_pen;
  items.push_back(addEllipse(tile_rect(pos), pen, brush));

  position const target = agent.target;
  if (target != pos) {
    QPointF const start{(pos.x + 0.5) * tile_size,
        (pos.y + 0.5) * tile_size};
    QPointF const end{(target.x + 0.5) * tile_size,
        (target.y + 0.5) * tile_size};

    QPainterPath target_path{start};
    target_path.lineTo(end);

    double const slope_angle = std::atan2(start.y() - end.y(),
                                          start.x() - end.x());
    double const arrow_angle = 3.141592 / 8;
    double const arrow_len = 0.8 * tile_size;

    QPointF const a{
      end.x() + arrow_len * std::cos(slope_angle + arrow_angle),
      end.y() + arrow_len * std::sin(slope_angle + arrow_angle)
    };
    QPointF const b{
      end.x() + arrow_len * std::cos(slope_angle - arrow_angle),
      end.y() + arrow_len * std::sin(slope_angle - arrow_angle)
    };

    target_path.lineTo(a);
    target_path.moveTo(end);
    target_path.lineTo(b);

    items.push_back(addPath(target_path, target_pen));
  }

  agent_items_[pos] = std::move(items);
}

void
world_scene::render_obstacle(position pos) {
  QGraphicsItem* item = addRect(tile_rect(pos.x, pos.y), black_pen,
                                obstacle_brush);
  obstacle_items_.push_back(item);
}
