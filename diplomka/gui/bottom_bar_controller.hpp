#ifndef BOTTOM_BAR_CONTROLLER_HPP
#define BOTTOM_BAR_CONTROLLER_HPP

#include <QObject>

#include <boost/optional.hpp>

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
                    QLabel* size_label,
                    QLabel* mouse_coord_label,
                    zoomable_graphics_view* view,
                    world_scene& scene);

  void set_world(boost::optional<world const&> world);

private slots:
  void scroll_zoom(int);
  void change_zoom(int);
  void update_mouse_pos(int, int);

private:
  QSlider* slider_ = nullptr;
  QLabel* zoom_label_ = nullptr;
  QLabel* size_label_ = nullptr;
  QLabel* mouse_coord_label_ = nullptr;
  zoomable_graphics_view* view_ = nullptr;
  boost::optional<world const&> world_;
};

#endif
