#include "mainwindow.hpp"

#include "world.hpp"

#include <QFileDialog>
#include <QPainter>
#include <QPixmap>
#include <QMessageBox>
#include <QString>

#include <cassert>
#include <cstddef>
#include <sstream>

#include <iostream>

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
    ui_.world_display->attach(&*map_);

    std::ostringstream os;
    os << "Map size: " << map_->width() << "x" << map_->height();
    ui_.size_label->setText(QString::fromStdString(os.str()));
  } catch (map_format_error& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
main_window::tile_activate(unsigned x, unsigned y) {
  std::cerr << x << ", " << y << '\n';
}
