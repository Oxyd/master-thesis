#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include <QMutex>
#include <QStandardItemModel>
#include <QTimer>
#include "ui_mainwindow.h"

#include "bottom_bar_controller.hpp"
#include "gui_log_sink.hpp"
#include "log_sinks.hpp"
#include "predictor.hpp"
#include "solver_runner.hpp"
#include "solvers.hpp"
#include "world.hpp"
#include "world_scene.hpp"

#include <boost/optional.hpp>

#include <memory>
#include <random>
#include <string>
#include <thread>

namespace Ui {
class MainWindow;
}

// Algorithm progress view. Allows setting various algorithm settings, viewing
// algorithm performance statistics and the current world state.
class main_window : public QMainWindow
{
  Q_OBJECT

public:
  explicit main_window(QWidget *parent = 0);

private slots:
  void open_map();
  void step();
  void make_step();
  void run();
  void change_run_interval(double);
  void reset_world();
  void make_random_seed();
  void edit();
  void algorithm_changed();
  void window_changed(int);
  void visualisation_params_changed();
  void show_targets_changed(bool);
  void step_finished();
  void add_log_line(QString);

protected:
  void closeEvent(QCloseEvent*);

private:
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
  std::thread step_thread_;
  std::unique_ptr<solver_runner> runner_;
  QMutex runner_mutex_;

  void stop();
  void load_world(std::string const&);
  void update_stats_headers();
  void update_stats();
  std::unique_ptr<solver> make_solver();
  std::unique_ptr<predictor> make_predictor();
  void highlight_paths();
  void highlight_obstacle_field();
  void interrupt_runner();
};

#endif // MAINWINDOW_HPP
