#include "bottom_bar_controller.hpp"

#include "world_scene.hpp"
#include "zoomable_graphics_view.hpp"

#include <QLabel>
#include <QSlider>

void
bottom_bar_controller::attach_to_ui(QSlider* slider,
                                    QLabel* zoom_label,
                                    QLabel* size_label,
                                    QLabel* mouse_coord_label,
                                    zoomable_graphics_view* view,
                                    world_scene& scene) {
  slider_ = slider;
  zoom_label_ = zoom_label;
  size_label_ = size_label;
  mouse_coord_label_ = mouse_coord_label;
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
  size_label_->setText("");
  mouse_coord_label_->setText("");

  world_ = world;

  if (world_) {
    size_label_->setText(QString("Map size: %1x%2")
                         .arg(world_->map()->width())
                         .arg(world_->map()->height()));

    change_zoom(0);
  }
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

  mouse_coord_label_->setText(QString("Mouse position: %1, %2")
                              .arg(x).arg(y));
}
