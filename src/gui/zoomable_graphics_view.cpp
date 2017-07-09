#include "zoomable_graphics_view.hpp"

#include <QWheelEvent>

zoomable_graphics_view::zoomable_graphics_view(QWidget* parent)
  : QGraphicsView(parent) { }

void
zoomable_graphics_view::wheelEvent(QWheelEvent* event) {
  if (!event->modifiers().testFlag(Qt::ControlModifier)) {
    QGraphicsView::wheelEvent(event);
    return;
  }

  if (event->angleDelta().y() > 0)
    emit zoom_changed(1);
  else
    emit zoom_changed(-1);

  event->accept();
}
