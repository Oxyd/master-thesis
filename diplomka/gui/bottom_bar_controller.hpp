#ifndef BOTTOM_BAR_CONTROLLER_HPP
#define BOTTOM_BAR_CONTROLLER_HPP

#include <QObject>

#include <boost/optional.hpp>

#include "solvers.hpp"

class QLabel;
class QSlider;

class world;
class world_scene;
class zoomable_graphics_view;

class bottom_bar_controller : public QObject {
  Q_OBJECT

public:
  void attach_to_ui(QSlider* slider,
                    QLabel* zoom_label,
                    QLabel* text_label,
                    zoomable_graphics_view* view,
                    world_scene& scene);

  void set_world(boost::optional<world const&> world);

  void set_obstacle_field(obstacle_field_type const&,
                          tick_t time);

private slots:
  void scroll_zoom(int);
  void change_zoom(int);
  void update_mouse_pos(int, int);

private:
  QSlider* slider_ = nullptr;
  QLabel* zoom_label_ = nullptr;
  QLabel* text_label_ = nullptr;
  zoomable_graphics_view* view_ = nullptr;
  boost::optional<world const&> world_;
  int mouse_x_ = 0;
  int mouse_y_ = 0;
  obstacle_field_type obstacle_field_;
  tick_t obstacle_field_time_;

  void update_text();
};

#endif
