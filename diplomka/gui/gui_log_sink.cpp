#include "gui_log_sink.hpp"

void
gui_log_sink::clear() {
  text_field_->clear();
}

void
gui_log_sink::do_put(std::string msg) {
  emit add_line(QString::fromStdString(std::move(msg)));
}
