#include "mainwindow.hpp"

#include "map.hpp"

#include <QFileDialog>
#include <QMessageBox>

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
    load(filename.toStdString());
  } catch (map_format_error& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}
