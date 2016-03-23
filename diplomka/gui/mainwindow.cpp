#include "mainwindow.hpp"

#include "action.hpp"
#include "scenario_edit.hpp"
#include "solvers.hpp"
#include "world.hpp"

#include <QDateTime>
#include <QFileDialog>
#include <QGraphicsView>
#include <QPlainTextEdit>
#include <QMessageBox>
#include <QStandardItem>
#include <QString>

#include <cassert>
#include <cstddef>
#include <cmath>
#include <memory>
#include <sstream>
#include <unordered_map>

#include <iostream>

static QColor agent_path_color{0x33, 0x66, 0xff, 0x80};

main_window::main_window(QWidget *parent)
  : QMainWindow(parent)
{
  ui_.setupUi(this);
  bottom_bar_controller_.attach_to_ui(ui_.zoom_slider,
                                      ui_.zoom_text,
                                      ui_.size_label,
                                      ui_.mouse_coord_label,
                                      ui_.world_view,
                                      world_scene_);

  log_sink_.text_field_ = ui_.log_edit;

  ui_.world_view->setScene(&world_scene_);

  run_timer_.setSingleShot(false);
  run_timer_.setInterval(100);
  connect(&run_timer_, &QTimer::timeout, this, &main_window::step);

  ui_.algorithm_combo->addItem("WHCA*");
  ui_.algorithm_combo->addItem("LRA*");
  ui_.algorithm_combo->addItem("Greedy");

  ui_.stats_view->setModel(&stats_);

  ui_.predictor_method_combo->addItem("Recursive");
  ui_.predictor_method_combo->addItem("Markov");

  showMaximized();
}

void
main_window::open_map() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Scenario");
  if (filename.isEmpty())
    return;

  load_world(filename.toStdString());
}

void
main_window::step() {
  if (!world_)
    return;

  if (solved(*world_)) {
    stop();
    return;
  }

  if (!solver_ ||
      solver_->name() != ui_.algorithm_combo->currentText().toStdString()) {
    solver_ = make_solver();
    update_stats_headers();
  }

  world_->next_tick(rng_);
  joint_action action = solver_->get_action(*world_, rng_);
  *world_ = apply(action, *world_);

  world_scene_.update();
  update_stats();
  visualisation_params_changed();
}

void
main_window::run() {
  if (!run_timer_.isActive()) {
    run_timer_.start();
    ui_.start_stop_button->setText("Stop");
  } else {
    stop();
  }
}

void
main_window::change_run_interval(double d) {
  run_timer_.setInterval(d * 1000);
}

void
main_window::reset_world() {
  if (!world_file_.empty()) {
    load_world(world_file_);
    solver_.reset();

    log_sink_.clear();
    update_stats_headers();
    update_stats();
  }
}

void
main_window::make_random_seed() {
  qint64 seed = QDateTime::currentMSecsSinceEpoch();
  ui_.seed_edit->setText(QString::number(seed));
  rng_.seed(seed);
}

void
main_window::edit() {
  auto edit = new scenario_edit(this);
  edit->setAttribute(Qt::WA_DeleteOnClose);
  edit->show();
}

void
main_window::algorithm_changed() {
  bool enable_window = ui_.algorithm_combo->currentText() == "WHCA*";
  bool enable_rejoin = ui_.algorithm_combo->currentText() == "WHCA*";

  if (enable_window) {
    ui_.window_label->setEnabled(true);
    ui_.window_spin->setEnabled(true);
  } else {
    ui_.window_label->setEnabled(false);
    ui_.window_spin->setEnabled(false);
  }

  if (enable_rejoin) {
    ui_.rejoin_checkbox->setEnabled(true);
    ui_.rejoin_limit_spin->setEnabled(true);
  } else {
    ui_.rejoin_checkbox->setEnabled(false);
    ui_.rejoin_limit_spin->setEnabled(false);
  }
}

void
main_window::gui_log_sink::clear() {
  text_field_->clear();
}

void
main_window::gui_log_sink::do_put(std::string msg) {
  text_field_->moveCursor(QTextCursor::End);
  text_field_->insertPlainText(QString::fromStdString(std::move(msg)));
  text_field_->moveCursor(QTextCursor::End);
}

void
main_window::stop() {
  run_timer_.stop();
  ui_.start_stop_button->setText("Run");
}

static std::default_random_engine::result_type
make_seed(QString const& seed_str) {
  using seed_type = std::default_random_engine::result_type;

  bool ok;
  seed_type seed = seed_str.toULongLong(&ok);

  if (!ok)
    for (QChar c : seed_str)
      seed += c.unicode();

  return seed;
}

void
main_window::load_world(std::string const& filename) {
  try {
    rng_.seed(make_seed(ui_.seed_edit->text()));

    world_ = ::load_world(filename, rng_);
    world_file_ = filename;

    bottom_bar_controller_.set_world(*world_);

    world_scene_.attach(&*world_);
    ui_.world_view->viewport()->update();
    world_scene_.setSceneRect(world_scene_.itemsBoundingRect());

    solver_.reset();
    log_sink_.clear();

    update_stats_headers();
    update_stats();

  } catch (bad_world_format& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
main_window::update_stats_headers() {
  stats_.clear();
  if (!world_)
    return;

  QStringList headers;
  headers << "Tick";

  if (solver_)
    for (auto const& header : solver_->stat_names())
      headers << QString::fromStdString(header);

  stats_.setVerticalHeaderLabels(headers);
}

void
main_window::update_stats() {
  if (!world_)
    return;

  assert(stats_.rowCount() >= 1);
  stats_.setItem(0, new QStandardItem{QString::number(world_->tick())});

  if (!solver_)
  {
    assert(stats_.rowCount() == 1);
    return;
  }

  auto const& stats = solver_->stat_values();
  assert(stats.size() == stats_.rowCount() - 1u);

  for (unsigned i = 0; i < stats.size(); ++i)
    stats_.setItem(i + 1, new QStandardItem{QString::fromStdString(stats[i])});
}

std::unique_ptr<solver>
main_window::make_solver() {
  assert(world_);

  QString algo = ui_.algorithm_combo->currentText();
  if (algo == "WHCA*")
    return make_whca(
      log_sink_,
      ui_.window_spin->value(),
      ui_.rejoin_checkbox->isChecked() ? ui_.rejoin_limit_spin->value() : 0,
      make_predictor(),
      ui_.obstacle_penalty_spin->value(),
      ui_.obstacle_threshold_spin->value()
    );
  else if (algo == "LRA*")
    return make_lra(log_sink_);
  else if (algo == "Greedy")
    return make_greedy();

  assert(!"Won't get here");
  return {};
}

std::unique_ptr<predictor>
main_window::make_predictor() {
  assert(world_);

  if (!ui_.avoid_obstacles_groupbox->isChecked())
    return {};

  int cutoff = ui_.predictor_cutoff_spin->value();
  QString method = ui_.predictor_method_combo->currentText();
  if (method == "Recursive")
    return make_recursive_predictor(*world_->map(), cutoff);
  else if (method == "Markov")
    return make_markov_predictor(*world_->map(), cutoff);

  assert(!"Won't get here");
  return {};
}

void
main_window::highlight_paths() {
  if (!world_ || !solver_)
    return;

  for (auto pos_agent : world_->agents())
    for (position p : solver_->get_path(std::get<1>(pos_agent).id()))
      world_scene_.highlight_tile(p, agent_path_color);
}

void
main_window::highlight_obstacle_field() {
  if (!world_ || !solver_)
    return;

  for (auto pos_time_value : solver_->get_obstacle_field()) {
    position_time pt = std::get<0>(pos_time_value);

    if ((int) pt.time - (int) world_->tick() != ui_.obstacle_field_spin->value())
      continue;

    double value = std::min(1.0, std::get<1>(pos_time_value));
    int saturation = 255 * (1 - value);
    QColor color{255, saturation, saturation};
    world_scene_.highlight_tile({pt.x, pt.y}, color);
  }
}

void
main_window::window_changed(int new_window) {
  if (!solver_)
    return;

  solver_->window(new_window);
}

void
main_window::visualisation_params_changed() {
  world_scene_.remove_all_highlights();

  if (ui_.visualise_paths_check->isChecked())
    highlight_paths();

  if (ui_.obstacle_field_check->isChecked())
    highlight_obstacle_field();
}
