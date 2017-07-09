#include "scenario_edit.hpp"

#include <QCloseEvent>
#include <QColor>
#include <QFileDialog>
#include <QMessageBox>

#include <algorithm>
#include <iterator>

static const QColor obstacle_spawn_color{255, 128, 128};
static const QColor obstacle_goal_color{255, 204, 0};
static const QColor agent_spawn_color{128, 255, 128};
static const QColor agent_goal_color{0, 204, 255};

scenario_edit::scenario_edit(QWidget* parent)
  : QMainWindow(parent)
{
  ui_.setupUi(this);
  bottom_bar_controller_.attach_to_ui(ui_.zoom_slider,
                                      ui_.zoom_text,
                                      ui_.text_label,
                                      ui_.world_view,
                                      scene_);

  ui_.world_view->setScene(&scene_);
  connect(&scene_, &world_scene::mouse_clicked,
          this, &scenario_edit::clicked);
  connect(&scene_, &world_scene::mouse_moved,
          this, &scenario_edit::mouse_moved);

  ui_.mode_combo->addItem("Random");
  ui_.mode_combo->addItem("Spawn-to-Goal");

  ui_.agents_spawn_combo->addItem("Uniform");
  ui_.agents_spawn_combo->addItem("Pack");
}

void
scenario_edit::new_scenario() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Map");
  if (filename.isEmpty())
    return;

  try {
    attach(world(load_map(filename.toStdString())), filename);
  } catch (bad_world_format& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
scenario_edit::open_scenario() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Scenario");
  if (filename.isEmpty())
    return;

  try {
    attach(load_world(filename.toStdString()), filename);
  } catch (bad_world_format& e) {
    QMessageBox::critical(this, "Error", e.what());
  }
}

void
scenario_edit::save_scenario() {
  if (!world_)
    return;

  QString const filename = QFileDialog::getSaveFileName(this, "Save Scenario");
  if (filename.isEmpty())
    return;

  reset_goal();

  obstacle_settings& obstacles = world_->obstacle_settings();

  if (ui_.mode_combo->currentText() == "Random")
    obstacles.mode = obstacle_mode::random;
  else if (ui_.mode_combo->currentText() == "Spawn-to-Goal")
    obstacles.mode = obstacle_mode::spawn_to_goal;

  obstacles.tile_probability = ui_.tile_probability_spin->value();
  obstacles.move_probability.mean = ui_.mean_ticks_spin->value();
  obstacles.move_probability.std_dev = ui_.std_dev_spin->value();
  obstacles.spawn_points = obstacle_spawn_points_;
  obstacles.goal_points = obstacle_goal_points_;

  agent_settings& agents = world_->agent_settings();
  agents.random_agent_number = ui_.random_agents_spin->value();

  if (ui_.agents_spawn_combo->currentText() == "Uniform")
    agents.spawn_mode = agent_settings::random_spawn_mode::uniform;
  else if (ui_.agents_spawn_combo->currentText() == "Pack")
    agents.spawn_mode = agent_settings::random_spawn_mode::pack;

  agents.spawn_points = agent_spawn_points_;
  agents.goal_points = agent_goal_points_;

  save_world(*world_, filename.toStdString());

  dirty_ = false;
}

void
scenario_edit::clicked(int x, int y) {
  if (!world_ || !in_bounds(x, y, *world_->map()))
    return;

  if (ui_.add_agent_button->isChecked() &&
      world_->get({x, y}) == tile::free) {
    world_->put_agent({x, y}, world_->create_agent({x, y}));
    scene_.update_agent({x, y});
    dirty_ = true;
  }

  else if (ui_.remove_agent_button->isChecked() &&
           world_->get({x, y}) == tile::agent) {
    world_->remove_agent({x, y});
    scene_.update_agent({x, y});
    dirty_ = true;
  }

  else if (ui_.set_goal_button->isChecked()) {
    if (!selected_ && world_->get({x, y}) == tile::agent) {
      selected_ = position{x, y};
      selected_original_goal_ = world_->get_agent(*selected_)->target;

      scene_.highlight_agent(*selected_, true);
      scene_.update_agent(*selected_);
    }
    else if (selected_ && world_->get({x, y}) != tile::wall) {
      world_->get_agent(*selected_)->target = {x, y};

      scene_.highlight_agent(*selected_, false);
      scene_.update_agent(*selected_);

      selected_ = boost::none;
      selected_original_goal_ = boost::none;

      dirty_ = true;
    }
  }

  else if (ui_.add_obstacle_spawn_button->isChecked()) {
    obstacle_spawn_points_.insert({x, y});
    scene_.highlight_tile({x, y}, obstacle_spawn_color);
  }

  else if (ui_.remove_obstacle_spawn_button->isChecked()) {
    obstacle_spawn_points_.erase({x, y});
    scene_.dehighlight_tile({x, y});
  }

  else if (ui_.add_obstacle_goal_button->isChecked()) {
    obstacle_goal_points_.insert({x, y});
    scene_.highlight_tile({x, y}, obstacle_goal_color);
  }

  else if (ui_.remove_obstacle_goal_button->isChecked()) {
    obstacle_goal_points_.erase({x, y});
    scene_.dehighlight_tile({x, y});
  }

  else if (ui_.add_agent_spawn_button->isChecked()) {
    agent_spawn_points_.insert({x, y});
    scene_.highlight_tile({x, y}, agent_spawn_color);
  }

  else if (ui_.remove_agent_spawn_button->isChecked()) {
    agent_spawn_points_.erase({x, y});
    scene_.dehighlight_tile({x, y});
  }

  else if (ui_.add_agent_goal_button->isChecked()) {
    agent_goal_points_.insert({x, y});
    scene_.highlight_tile({x, y}, agent_goal_color);
  }

  else if (ui_.remove_agent_goal_button->isChecked()) {
    agent_goal_points_.erase({x, y});
    scene_.dehighlight_tile({x, y});
  }
}

void
scenario_edit::reset_goal() {
  if (!ui_.set_goal_button->isChecked() && selected_original_goal_) {
    assert(selected_);
    assert(world_);

    world_->get_agent(*selected_)->target = *selected_original_goal_;

    selected_ = boost::none;
    selected_original_goal_ = boost::none;
  }
}

void
scenario_edit::mouse_moved(int x, int y) {
  if (!world_ || !in_bounds(x, y, *world_->map()) || !selected_ ||
      last_mouse_pos_ == position{x, y})
    return;

  world_->get_agent(*selected_)->target = {x, y};
  scene_.update_agent(*selected_);

  last_mouse_pos_ = {x, y};
}

void
scenario_edit::closeEvent(QCloseEvent* event) {
  if (!dirty_) {
    event->accept();
    return;
  }

  QMessageBox::StandardButton answer = QMessageBox::question(
    this, "Not saved", "Scenario not saved. Save now?",
    QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel
  );
  if (answer == QMessageBox::Yes)
    save_scenario();

  if (answer != QMessageBox::Cancel)
    event->accept();
  else
    event->ignore();
}

void
scenario_edit::attach(world w, QString const& filename) {
  world_ = std::move(w);

  bottom_bar_controller_.set_world(*world_);

  scene_.attach(&*world_);
  ui_.world_view->viewport()->update();
  scene_.setSceneRect(scene_.itemsBoundingRect());

  obstacle_settings const& obstacles = world_->obstacle_settings();
  ui_.mode_combo->setCurrentIndex(static_cast<int>(obstacles.mode));
  ui_.tile_probability_spin->setValue(obstacles.tile_probability);
  ui_.mean_ticks_spin->setValue(obstacles.move_probability.mean);
  ui_.std_dev_spin->setValue(obstacles.move_probability.std_dev);

  obstacle_spawn_points_.clear();
  for (position p : obstacles.spawn_points) {
    obstacle_spawn_points_.insert(p);
    scene_.highlight_tile(p, obstacle_spawn_color);
  }

  obstacle_goal_points_.clear();
  for (position p : obstacles.goal_points) {
    obstacle_goal_points_.insert(p);
    scene_.highlight_tile(p, obstacle_goal_color);
  }

  agent_settings const& agents = world_->agent_settings();
  ui_.random_agents_spin->setValue(agents.random_agent_number);
  ui_.agents_spawn_combo->setCurrentIndex(static_cast<int>(agents.spawn_mode));

  agent_spawn_points_.clear();
  for (position p : agents.spawn_points) {
    agent_spawn_points_.insert(p);
    scene_.highlight_tile(p, agent_spawn_color);
  }

  agent_goal_points_.clear();
  for (position p : agents.goal_points) {
    agent_goal_points_.insert(p);
    scene_.highlight_tile(p, agent_goal_color);
  }

  setWindowTitle(QString(QString("Edit Scenario: %1").arg(filename)));

  dirty_ = false;
}
