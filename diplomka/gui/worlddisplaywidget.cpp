#include "worlddisplaywidget.hpp"

#include "world.hpp"

#include <QColor>
#include <QMouseEvent>
#include <QPainter>

static QColor
tile_color(tile t) {
  switch (t) {
  case tile::passable: return {255, 255, 255};
  case tile::out_of_bounds: return {0, 0, 0};
  case tile::tree: return {30, 200, 30};
  case tile::swamp: return {127, 127, 127};
  case tile::water: return {0, 0, 255};
  }
}

static QColor
team_color(team_type t) {
  if (t == 0)
    return {0, 255, 255};
  else
    return {255, 0, 0};
}

world_display_widget::world_display_widget(QWidget* parent)
  : QWidget(parent)
{
  setMinimumSize(1, 1);
  setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
  updateGeometry();
}

void
world_display_widget::attach(world* m) {
  world_ = m;
  update_size();
}

void
world_display_widget::zoom(int z) {
  assert(z >= 0.0);
  assert((unsigned) z < sizeof(zoom_) * CHAR_BIT);

  zoom_ = 1 << z;
  update_size();
}

void
world_display_widget::update() {
  repaint(rect());
}

void
world_display_widget::paintEvent(QPaintEvent*) {
  if (!world_)
    return;

  QPainter painter(this);
  for (auto const& t : world_->map()) {
    QColor color;

    if (auto agent = world_->get_agent({t.x, t.y}))
      color = team_color(agent->team());
    else
      color = tile_color(t.tile);

    painter.fillRect(t.x * zoom_, t.y * zoom_, zoom_, zoom_, color);
  }
}

void
world_display_widget::mousePressEvent(QMouseEvent* ev) {
  emit tile_clicked(ev->x() / zoom_, ev->y() / zoom_);
}

void
world_display_widget::update_size() {
  if (world_) {
    unsigned const w = world_->map().width() * zoom_;
    unsigned const h = world_->map().height() * zoom_;
    setMinimumSize(w, h);
    resize(w, h);
  }
}
