#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QGraphicsScene>
#include <QMainWindow>
#include <QStandardItemModel>
#include <QTimer>
#include "ui_mainwindow.h"

#include "log_sinks.hpp"
#include "solvers.hpp"
#include "world.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <random>
#include <string>

namespace Ui {
class MainWindow;
}

class mouse_graphics_scene : public QGraphicsScene {
  Q_OBJECT

signals:
  void mouse_moved(QPointF scene_pos);

protected:
  void mouseMoveEvent(QGraphicsSceneMouseEvent*) override;
};

class main_window : public QMainWindow
{
  Q_OBJECT

public:
  explicit main_window(QWidget *parent = 0);

private slots:
  void open_map();
  void scroll_zoom(int);
  void change_zoom(int);
  void step();
  void run();
  void change_run_interval(double);
  void reset_world();
  void update_mouse_pos(QPointF pos);

private:
  class gui_log_sink : public log_sink {
  public:
    QPlainTextEdit* text_field_;

  private:
    void
    do_put(std::string msg) override;
  };

  Ui::MainWindow ui_;
  std::string world_file_;
  boost::optional<world> world_;
  mouse_graphics_scene world_scene_;
  std::default_random_engine rng_;
  QTimer run_timer_;
  std::unique_ptr<solver> solver_;
  gui_log_sink log_sink_;
  QStandardItemModel stats_;

  void stop();
  void load_world(std::string const&);
  void update_world_view();
  void update_stats_headers();
  void update_stats();
};

#endif // MAINWINDOW_HPP
