#include "scenario_edit.hpp"

#include <QFileDialog>
#include <QMessageBox>

scenario_edit::scenario_edit(QWidget* parent)
  : QMainWindow(parent)
{
  ui_.setupUi(this);
  bottom_bar_controller_.attach_to_ui(ui_.zoom_slider,
                                      ui_.zoom_text,
                                      ui_.size_label,
                                      ui_.mouse_coord_label,
                                      ui_.world_view,
                                      scene_);

  ui_.world_view->setScene(&scene_);
  connect(&scene_, &world_scene::mouse_clicked,
          this, &scenario_edit::clicked);
  connect(&scene_, &world_scene::mouse_moved,
          this, &scenario_edit::mouse_moved);
}

void
scenario_edit::new_scenario() {
  QString const filename = QFileDialog::getOpenFileName(this, "Open Map");
  if (filename.isEmpty())
    return;

  try {
    attach(world(load_map(filename.toStdString())));
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
    attach(load_world(filename.toStdString()));
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

  obstacle_settings& obstacles = world_->obstacle_settings();
  obstacles.tile_probability = ui_.tile_probability_spin->value();
  obstacles.move_probability.mean = ui_.mean_ticks_spin->value();
  obstacles.move_probability.std_dev = ui_.std_dev_spin->value();

  save_world(*world_, filename.toStdString());
}

void
scenario_edit::clicked(int x, int y) {
  if (!world_ || !in_bounds(x, y, *world_->map()))
    return;

  if (ui_.add_agent_button->isChecked() &&
      world_->get({x, y}) == tile::free) {
    world_->put_agent({x, y}, agent{{x, y}});
    scene_.update_agent({x, y});
  }

  else if (ui_.remove_agent_button->isChecked() &&
           world_->get({x, y}) == tile::agent) {
    world_->remove_agent({x, y});
    scene_.update_agent({x, y});
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
    }
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
scenario_edit::attach(world w) {
  world_ = std::move(w);

  bottom_bar_controller_.set_world(*world_);

  scene_.attach(&*world_);
  ui_.world_view->viewport()->update();
  scene_.setSceneRect(scene_.itemsBoundingRect());

  obstacle_settings const& obstacles = world_->obstacle_settings();
  ui_.tile_probability_spin->setValue(obstacles.tile_probability);
  ui_.mean_ticks_spin->setValue(obstacles.move_probability.mean);
  ui_.std_dev_spin->setValue(obstacles.move_probability.std_dev);
}
