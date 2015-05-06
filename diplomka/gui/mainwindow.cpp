#include "mainwindow.hpp"

#include "world.hpp"

#include <QBrush>
#include <QColor>
#include <QFileDialog>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QPainter>
#include <QPixmap>
#include <QMessageBox>
#include <QPen>
#include <QString>

#include <cassert>
#include <cstddef>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>

#include <iostream>

main_window::main_window(QWidget *parent) :
  QMainWindow(parent)
{
  ui_.setupUi(this);
  ui_.world_view->setScene(&world_scene_);
}

void
main_window::open_map() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Map");
  if (filename.isEmpty())
    return;

  try {
    world_ = load_world(filename.toStdString());

    std::ostringstream os;
    os << "Map size: " << world_->map().width()
       << "x" << world_->map().height();
    ui_.size_label->setText(QString::fromStdString(os.str()));

    update_world_view();
  } catch (bad_world_format& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
main_window::change_zoom(int exponent) {
  double const zoom = std::pow(2, exponent);
  ui_.zoom_text->setText(QString::number(zoom));

  ui_.world_view->resetTransform();
  ui_.world_view->scale(zoom, zoom);
}

constexpr double tile_size = 30;

static QRectF
tile_rect(position::coord_type x, position::coord_type y) {
  return {x * tile_size, y * tile_size, tile_size, tile_size};
}

static QRectF
tile_rect(position p) {
  return tile_rect(p.x, p.y);
}

void
main_window::update_world_view() {
  for (QGraphicsItem* item : world_scene_.items())
    world_scene_.removeItem(item);

  if (!world_)
    return;

  QPen const black_pen{{0, 0, 0}};
  QPen const target_pen{QBrush{QColor{127, 127, 127}}, 3};
  QBrush const nontraversable_brush{QColor{0, 0, 0}};

  for (auto t : world_->map()) {
    if (!traversable(t.tile))
      world_scene_.addRect(tile_rect(t.x, t.y), black_pen, nontraversable_brush);
  }

  QBrush const agent_brush[team_count]{
    {QColor{0, 255, 0}},
    {QColor{255, 0, 0}}
  };

  for (auto const& pos_agent : world_->agents()) {
    position const pos = pos_agent.first;
    agent const& agent = pos_agent.second;

    QBrush const& brush = agent_brush[(std::size_t) agent.team()];

    world_scene_.addEllipse(tile_rect(pos), black_pen, brush);

    if (agent.target()) {
      position const target = *agent.target();
      QPointF const start{(pos.x + 0.5) * tile_size, (pos.y + 0.5) * tile_size};
      QPointF const end{(target.x + 0.5) * tile_size, (target.y + 0.5) * tile_size};

      world_scene_.addLine(start.x(), start.y(), end.x(), end.y(), target_pen);

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

      world_scene_.addLine(end.x(), end.y(), a.x(), a.y(), target_pen);
      world_scene_.addLine(end.x(), end.y(), b.x(), b.y(), target_pen);
    }
  }
}
