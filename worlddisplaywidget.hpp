#ifndef WORLDDISPLAYWIDGET_HPP
#define WORLDDISPLAYWIDGET_HPP

#include <QWidget>

class map;

class world_display_widget : public QWidget {
  Q_OBJECT

public:
  world_display_widget(QWidget* parent = nullptr);

  void attach(map*);

signals:
  void tile_clicked(unsigned x, unsigned y);

public slots:
  void zoom(int);

protected:
  void paintEvent(QPaintEvent* ev) override;
  void mousePressEvent(QMouseEvent* ev) override;

private:
  map* map_ = nullptr;
  unsigned zoom_ = 1 << 2;

  void update_size();
};

#endif // WORLDDISPLAYWIDGET_HPP

