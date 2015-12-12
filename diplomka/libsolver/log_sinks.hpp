#ifndef LOG_SINKS_HPP
#define LOG_SINKS_HPP

#include <sstream>
#include <string>
#include <utility>

class log_sink {
public:
  virtual
  ~log_sink() { }

  template <typename T>
  void
  put(T&& t) {
    std::ostringstream os;
    os << std::move(t);
    do_put(os.str());
  }

private:
  virtual void
  do_put(std::string msg) = 0;
};

template <typename T>
log_sink&
operator << (log_sink& log, T const& t) {
  log.put(t);
  return log;
}

class stdout_log_sink : public log_sink {
  void
  do_put(std::string msg) override;
};

class null_log_sink_t : public log_sink {
  void
  do_put(std::string) override { };
};

extern null_log_sink_t null_log_sink;

#endif
