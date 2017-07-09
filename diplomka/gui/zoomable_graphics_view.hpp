#ifndef ZOOMABLE_GRAPHICS_VIEW_HPP
#define ZOOMABLE_GRAPHICS_VIEW_HPP

#include <QGraphicsView>

// A QGraphicsView that emits zoom_changed on Ctrl+Scroll.
class zoomable_graphics_view : public QGraphicsView {
  Q_OBJECT

public:
  zoomable_graphics_view(QWidget* parent = nullptr);

signals:
  void zoom_changed(int direction);

protected:
  void wheelEvent(QWheelEvent*) override;
};

#endif
