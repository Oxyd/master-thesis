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
    map const m = load(filename.toStdString());
    world_ = world{m};

    std::ostringstream os;
    os << "Map size: " << world_->map().width()
       << "x" << world_->map().height();
    ui_.size_label->setText(QString::fromStdString(os.str()));

    update_world_view();
  } catch (map_format_error& e) {
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

void
main_window::update_world_view() {
  for (QGraphicsItem* item : world_scene_.items())
    world_scene_.removeItem(item);

  if (!world_)
    return;

  QPen const blackPen{{0, 0, 0}};
  QBrush const nontraversableBrush{QColor{0, 0, 0}};

  constexpr double tile_size = 30;

  for (auto t : world_->map()) {
    if (!traversable(t.tile))
      world_scene_.addRect(t.x * tile_size, t.y * tile_size,
                           tile_size, tile_size,
                           blackPen, nontraversableBrush);
  }
}
