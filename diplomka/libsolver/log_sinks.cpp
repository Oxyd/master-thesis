#include "log_sinks.hpp"

#include <iostream>

null_log_sink_t null_log_sink;

void
stdout_log_sink::do_put(std::string message) {
  std::cout << std::move(message) << '\n';
}
