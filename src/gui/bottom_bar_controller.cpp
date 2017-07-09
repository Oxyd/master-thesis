#include "bottom_bar_controller.hpp"

#include "world_scene.hpp"
#include "zoomable_graphics_view.hpp"

#include <QLabel>
#include <QSlider>

void
bottom_bar_controller::attach_to_ui(QSlider* slider,
                                    QLabel* zoom_label,
                                    QLabel* text_label,
                                    zoomable_graphics_view* view,
                                    world_scene& scene) {
  slider_ = slider;
  zoom_label_ = zoom_label;
  text_label_ = text_label;
  view_ = view;

  connect(slider_, &QSlider::valueChanged,
          this, &bottom_bar_controller::change_zoom);
  connect(view_, &zoomable_graphics_view::zoom_changed,
          this, &bottom_bar_controller::scroll_zoom);
  connect(&scene, &world_scene::mouse_moved,
          this, &bottom_bar_controller::update_mouse_pos);
}

void
bottom_bar_controller::set_world(boost::optional<world const&> world) {
  world_ = world;

  if (world_)
    slider_->setValue(0);

  update_text();
}

void
bottom_bar_controller::set_obstacle_field(obstacle_field_type const& of,
                                          tick_t time) {
  obstacle_field_ = of;
  obstacle_field_time_ = time;

  update_text();
}

void
bottom_bar_controller::scroll_zoom(int change) {
  int new_value = slider_->value() + change;
  if (new_value < slider_->minimum() || new_value > slider_->maximum())
    return;

  slider_->setValue(new_value);
}

void
bottom_bar_controller::change_zoom(int exponent) {
  double const zoom = std::pow(2, exponent);
  zoom_label_->setText(QString::number(zoom));

  view_->resetTransform();
  view_->scale(zoom, zoom);
}

void
bottom_bar_controller::update_mouse_pos(int x, int y) {
  if (!world_ || !in_bounds(x, y, *world_->map()))
    return;

  mouse_x_ = x;
  mouse_y_ = y;

  update_text();
}

void
bottom_bar_controller::update_text() {
  QString text;

  if (world_)
    text += QString("Map size: %1x%2 | ")
      .arg(world_->map()->width())
      .arg(world_->map()->height())
      ;

  text += QString("Mouse position: %1, %2")
    .arg(mouse_x_)
    .arg(mouse_y_)
    ;

  if (!obstacle_field_.empty() && world_) {
    auto prob = obstacle_field_.find({mouse_x_, mouse_y_,
                                      world_->tick() + obstacle_field_time_});

    text += QString(" | Obstacle probability: %1")
      .arg(prob != obstacle_field_.end()
           ? QString::number(prob->second)
           : "?");
  }

  text_label_->setText(text);
}
