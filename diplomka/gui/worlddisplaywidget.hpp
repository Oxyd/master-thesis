#ifndef WORLDDISPLAYWIDGET_HPP
#define WORLDDISPLAYWIDGET_HPP

#include <QWidget>

class world;

class world_display_widget : public QWidget {
  Q_OBJECT

public:
  world_display_widget(QWidget* parent = nullptr);

  void attach(world*);

signals:
  void tile_clicked(unsigned x, unsigned y);

public slots:
  void zoom(int);
  void update();

protected:
  void paintEvent(QPaintEvent* ev) override;
  void mousePressEvent(QMouseEvent* ev) override;

private:
  world* world_ = nullptr;
  unsigned zoom_ = 1 << 2;

  void update_size();
};

#endif // WORLDDISPLAYWIDGET_HPP

