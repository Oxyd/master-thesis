#ifndef WORLD_SCENE_HPP
#define WORLD_SCENE_HPP

#include <QGraphicsScene>

class world;

class world_scene : public QGraphicsScene {
  Q_OBJECT

public:
  void render(world const&);

signals:
  void mouse_moved(int x, int y);

protected:
  void mouseMoveEvent(QGraphicsSceneMouseEvent*) override;
};

#endif
