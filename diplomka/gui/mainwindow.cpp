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
    map m = load(filename.toStdString());
    world_ = world{m};
    ui_.world_display->attach(&*world_);

    std::ostringstream os;
    os << "Map size: " << world_->map().width()
       << "x" << world_->map().height();
    ui_.size_label->setText(QString::fromStdString(os.str()));
  } catch (map_format_error& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
main_window::tile_activate(unsigned x, unsigned y) {
  if (!world_)
    return;

  if (!traversable(world_->map().get(x, y)))
    return;

  if (world_->get_agent({x, y}))
    world_->remove_agent({x, y});
  else
    world_->put_agent(
      {x, y},
      agent{{x, y}, {}, ui_.attacker_radio->isChecked() ? 0u : 1u}
  );

  ui_.world_display->update();
}
