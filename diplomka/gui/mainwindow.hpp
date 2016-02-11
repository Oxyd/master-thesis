#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QStandardItemModel>
#include <QTimer>
#include "ui_mainwindow.h"

#include "bottom_bar_controller.hpp"
#include "log_sinks.hpp"
#include "solvers.hpp"
#include "world.hpp"
#include "world_scene.hpp"

#include <boost/optional.hpp>

#include <memory>
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
  void step();
  void run();
  void change_run_interval(double);
  void reset_world();
  void make_random_seed();
  void edit();
  void algorithm_changed();
  void window_changed(int);
  void tile_clicked(int, int);

private:
  class gui_log_sink : public log_sink {
  public:
    QPlainTextEdit* text_field_;

    void clear();

  private:
    void do_put(std::string msg) override;
  };

  Ui::MainWindow ui_;
  std::string world_file_;
  boost::optional<world> world_;
  world_scene world_scene_;
  std::default_random_engine rng_;
  QTimer run_timer_;
  std::unique_ptr<solver> solver_;
  gui_log_sink log_sink_;
  QStandardItemModel stats_;
  bottom_bar_controller bottom_bar_controller_;

  void stop();
  void load_world(std::string const&);
  void update_stats_headers();
  void update_stats();
  std::unique_ptr<solver> make_solver();
};

#endif // MAINWINDOW_HPP
