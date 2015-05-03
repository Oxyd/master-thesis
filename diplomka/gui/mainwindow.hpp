#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QGraphicsScene>
#include <QMainWindow>
#include "ui_mainwindow.h"

#include "world.hpp"

#include <boost/optional.hpp>

namespace Ui {
class MainWindow;
}

class main_window : public QMainWindow
{
  Q_OBJECT

public:
  explicit main_window(QWidget *parent = 0);

private slots:
  void open_map();
  void change_zoom(int);

private:
  Ui::MainWindow ui_;
  boost::optional<world> world_;
  QGraphicsScene world_scene_;

  void update_world_view();
};

#endif // MAINWINDOW_HPP
