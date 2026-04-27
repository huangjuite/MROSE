#pragma once

#include <chrono>
#include <fstream>
#include <iomanip>
#include <string>
#include <spdlog/spdlog.h>

namespace rplrio {

class ScopedTimer {
public:
  /**
    Moving the \c logger here after copying it in saves 2 atomic adds, and apparently helps with thread contention.
    Moving the \c label because we're almost certainly passing rvalue (string literals), and using const & would be 
      wasting a move operation. 
   */
  ScopedTimer(std::shared_ptr<spdlog::logger> logger, std::string label,
              std::ofstream *csv = nullptr)
      : logger_(std::move(logger)), label_(std::move(label)), csv_(csv),
        start_(std::chrono::steady_clock::now()) {}

  ~ScopedTimer() {
    auto elapsed = std::chrono::steady_clock::now() - start_;
    double ms =
        std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count() /
        1000.0;
    logger_->info("[timer] {}: {:.3f} ms", label_, ms);
    if (csv_) {
      auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         start_.time_since_epoch())
                         .count();
      *csv_ << wall_ms << "," << label_ << "," << std::fixed
            << std::setprecision(3) << ms << "\n";
    }
  }

  double elapsed_ms() const {
    auto elapsed = std::chrono::steady_clock::now() - start_;
    return std::chrono::duration_cast<std::chrono::microseconds>(elapsed)
               .count() /
           1000.0;
  }

private:
  std::shared_ptr<spdlog::logger> logger_;
  std::string label_;
  std::ofstream *csv_;
  std::chrono::steady_clock::time_point start_;
};

} // namespace rplrio
