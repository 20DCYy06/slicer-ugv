#pragma once

#include <cmath>

namespace slicer_navigation {

struct HeadingStep {
  double angular = 0.;
  bool ready = false;
  bool failed = false;
};

// A fixed-rate turn with measured heading and stationary checks at its end.
class StartHeading {
 public:
  StartHeading(double rate, double tolerance, double settle, double timeout)
      : rate_(rate), tolerance_(tolerance), settle_(settle), timeout_(timeout) {}

  void start(double now) {
    start_ = now;
    quiet_since_ = -1.;
    direction_ = 0;
    done_ = motion_seen_ = false;
  }

  HeadingStep update(double error, bool stopped, double now) {
    if (done_) return {0., true, false};
    if (now - start_ >= timeout_) return {0., false, true};

    motion_seen_ = motion_seen_ || !stopped || direction_ != 0;
    if (std::abs(error) <= tolerance_) {
      direction_ = 0;
      if (!motion_seen_) done_ = true;
      else if (!stopped) quiet_since_ = -1.;
      else if (quiet_since_ < 0.) quiet_since_ = now;
      else if (now - quiet_since_ >= settle_) done_ = true;
      return {0., done_, false};
    }
    quiet_since_ = -1.;
    const int requested = error > 0. ? 1 : -1;
    if (direction_ != 0 && direction_ != requested) {
      // Decelerate to zero before a correction in the opposite direction.
      direction_ = 0;
      return {};
    }
    if (direction_ == 0 && !stopped) return {};
    direction_ = requested;
    return {rate_ * direction_, false, false};
  }

 private:
  const double rate_, tolerance_, settle_, timeout_;
  double start_ = 0., quiet_since_ = -1.;
  int direction_ = 0;
  bool done_ = false, motion_seen_ = false;
};

}  // namespace slicer_navigation
