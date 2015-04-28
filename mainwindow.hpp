#ifndef MAINWINDOW_HPP
#define MAINWINDOW_HPP

#include <QMainWindow>
#include "ui_mainwindow.h"

#include "map.hpp"

#include <boost/optional.hpp>

namespace Ui {
class MainWindow;
}

class main_window : public QMainWindow
{
  Q_OBJECT

public:
  explicit main_window(QWidget *parent = 0);

private slots:
  void open_map();
  void redraw_map();
  void change_zoom(int);

private:
  Ui::MainWindow ui_;
  boost::optional<map> map_;
  unsigned zoom_level_ = 4;
};

#endif // MAINWINDOW_HPP
