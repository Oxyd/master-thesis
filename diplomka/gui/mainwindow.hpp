#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QGraphicsScene>
#include <QMainWindow>
#include <QTimer>
#include "ui_mainwindow.h"

#include "world.hpp"

#include <boost/optional.hpp>

#include <random>
#include <string>

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
  void step();
  void run();
  void change_run_interval(double);
  void reset_world();

private:
  Ui::MainWindow ui_;
  std::string world_file_;
  boost::optional<world> world_;
  QGraphicsScene world_scene_;
  std::default_random_engine rng_;
  QTimer run_timer_;

  void stop();
  void load_world(std::string const&);
  void update_world_view();
};

#endif // MAINWINDOW_HPP
