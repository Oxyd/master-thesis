#include "world_scene.hpp"
#include "world.hpp"

#include <QBrush>
#include <QColor>
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
world_scene::render(world const& w) {
  QPen const black_pen{{0, 0, 0}};
  QPen const target_pen{QBrush{QColor{127, 127, 127, 127}}, 1};
  QBrush const wall_brush{QColor{0, 0, 0}};
  QBrush const obstacle_brush{QColor{255, 0, 0}};

  for (auto t : *w.map())
    if (t.tile == tile::wall)
      addRect(tile_rect(t.x, t.y), black_pen, wall_brush);

  QBrush const brush{{0, 255, 0}};

  for (auto const& pos_agent : w.agents()) {
    position const pos = pos_agent.first;
    agent const& agent = pos_agent.second;

    addEllipse(tile_rect(pos), black_pen, brush);

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

      addPath(target_path, target_pen);
    }
  }

  for (auto const& pos_obstacle : w.obstacles()) {
    position const pos = pos_obstacle.first;
    addRect(tile_rect(pos.x, pos.y), black_pen, obstacle_brush);
  }
}

void
world_scene::mouseMoveEvent(QGraphicsSceneMouseEvent* event) {
  int x = event->scenePos().x() / tile_size;
  int y = event->scenePos().y() / tile_size;

  emit mouse_moved(x, y);
}
