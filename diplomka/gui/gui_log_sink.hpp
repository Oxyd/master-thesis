#ifndef GUI_LOG_SINK_HPP
#define GUI_LOG_SINK_HPP

#include <QObject>
#include <QPlainTextEdit>
#include <QString>

#include "log_sinks.hpp"

// Log sink that translates solver messages to the add_line signal.
class gui_log_sink : public QObject, public log_sink {
  Q_OBJECT

public:
  QPlainTextEdit* text_field_;

  void clear();

signals:
  void add_line(QString);

private:
  void do_put(std::string msg) override;
};

#endif
