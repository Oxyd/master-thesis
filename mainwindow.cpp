#include "mainwindow.hpp"

#include "map.hpp"

#include <QFileDialog>
#include <QPainter>
#include <QPixmap>
#include <QMessageBox>
#include <QString>

#include <cassert>
#include <cstddef>
#include <sstream>

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

main_window::main_window(QWidget *parent) :
  QMainWindow(parent)
{
  ui_.setupUi(this);
}

void
main_window::open_map() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Map");
  if (filename.isEmpty())
    return;

  try {
    map_ = load(filename.toStdString());
    redraw_map();

    std::ostringstream os;
    os << "Map size: " << map_->width() << "x" << map_->height();
    ui_.size_label->setText(QString::fromStdString(os.str()));
  } catch (map_format_error& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
main_window::redraw_map() {
  if (!map_) {
    ui_.map_label->setText("No map loaded");
    return;
  }

  QPixmap pix(map_->width() * zoom_level_, map_->height() * zoom_level_);
  QPainter painter(&pix);
  for (auto const& t : *map_)
    painter.fillRect(t.x * zoom_level_, t.y * zoom_level_,
                     zoom_level_, zoom_level_,
                     tile_color(t.tile));

  ui_.map_label->setPixmap(pix);
}

void
main_window::change_zoom(int z) {
  assert(z >= 0.0);
  assert((unsigned) z < sizeof(zoom_level_) * CHAR_BIT);
  zoom_level_ = 1 << z;
  redraw_map();
}
