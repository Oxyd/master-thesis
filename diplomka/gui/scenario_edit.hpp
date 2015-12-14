#ifndef SCENARIO_EDIT_HPP
#define SCENARIO_EDIT_HPP

#include <QMainWindow>
#include "ui_edit.h"

#include "bottom_bar_controller.hpp"
#include "world.hpp"
#include "world_scene.hpp"

#include <boost/optional.hpp>

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

private:
  Ui::ScenarioEdit ui_;
  boost::optional<world> world_;
  world_scene scene_;
  boost::optional<position> selected_;
  boost::optional<position> selected_original_goal_;
  position last_mouse_pos_{-1, -1};
  bottom_bar_controller bottom_bar_controller_;

  void attach(world);
};

#endif
