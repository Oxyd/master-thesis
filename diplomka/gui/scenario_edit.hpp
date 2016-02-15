#ifndef SCENARIO_EDIT_HPP
#define SCENARIO_EDIT_HPP

#include <QMainWindow>
#include <QString>
#include "ui_edit.h"

#include <boost/optional.hpp>

#include <unordered_set>

#include "bottom_bar_controller.hpp"
#include "world.hpp"
#include "world_scene.hpp"

namespace Ui { class ScenarioEdit; }

class scenario_edit : public QMainWindow {
  Q_OBJECT

public:
  explicit scenario_edit(QWidget* parent = nullptr);

private slots:
  void new_scenario();
  void open_scenario();
  void save_scenario();
  void clicked(int, int);
  void reset_goal();
  void mouse_moved(int, int);

protected:
  void closeEvent(QCloseEvent*);

private:
  Ui::ScenarioEdit ui_;
  boost::optional<world> world_;
  world_scene scene_;
  boost::optional<position> selected_;
  boost::optional<position> selected_original_goal_;
  position last_mouse_pos_{-1, -1};
  bottom_bar_controller bottom_bar_controller_;
  bool dirty_ = false;
  std::unordered_set<position> obstacle_spawn_points_;
  std::unordered_set<position> obstacle_goal_points_;

  void attach(world, QString const& name);
};

#endif
