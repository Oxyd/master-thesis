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

  for (uint i = 0; i < solvers.size(); ++i)
    ui_.algorithm_combo->addItem(
      QString::fromStdString(std::get<0>(solvers[i])),
      i
    );

  ui_.stats_view->setModel(&stats_);
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
    uint const solver_index = ui_.algorithm_combo->currentData().toUInt();
    solver_ = std::get<1>(solvers[solver_index])(log_sink_, *world_);
    update_stats_headers();
  }

  world_->next_tick(rng_);
  joint_action action = solver_->get_action(*world_, rng_);
  *world_ = apply(action, *world_);

  world_scene_.update();
  update_stats();
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
