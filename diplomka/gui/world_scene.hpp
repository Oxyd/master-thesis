#ifndef WORLD_SCENE_HPP
#define WORLD_SCENE_HPP

#include <QGraphicsScene>

#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "world.hpp"

class world_scene : public QGraphicsScene {
  Q_OBJECT

public:
  void attach(world const*);
  void update();
  void re_render();
  void update_agent(position);
  void highlight_agent(position, bool set_highlight);
  void highlight_tile(position, QColor);
  void dehighlight_tile(position);
  void remove_all_highlights();

signals:
  void mouse_moved(int x, int y);
  void mouse_clicked(int x, int y);

protected:
  void mouseMoveEvent(QGraphicsSceneMouseEvent*) override;
  void mousePressEvent(QGraphicsSceneMouseEvent*) override;
  void mouseReleaseEvent(QGraphicsSceneMouseEvent*) override;

private:
  world const* world_ = nullptr;
  std::unordered_map<position, std::vector<QGraphicsItem*>> agent_items_;
  std::vector<QGraphicsItem*> obstacle_items_;
  std::unordered_set<position> highlighted_agents_;
  std::unordered_map<position, QGraphicsItem*> highlighted_tiles_;
  boost::optional<std::tuple<int, int>> last_mouse_clicked_;

  void render_agent(position, agent const&);
  void render_obstacle(position);
};

#endif
