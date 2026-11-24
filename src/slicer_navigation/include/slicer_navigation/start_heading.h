#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace slicer_navigation {

struct HeadingStep {
  double angular = 0.;
  bool ready = false;
  bool failed = false;
};

// Direction of the first short segment of a locally transformed path.
template<class Iter, class Position>
double pathStartTangent(Iter begin, Iter end, Position position, double lookahead) {
  if (begin == end) return std::numeric_limits<double>::quiet_NaN();
  const auto origin = position(*begin);
  auto previous = origin;
  auto endpoint = origin;
  double length = 0.;
  for (auto it = ++begin; it != end; ++it) {
    const auto next = position(*it);
    const double dx = next.first-previous.first, dy = next.second-previous.second;
    const double segment = std::hypot(dx, dy);
    if (!std::isfinite(segment)) return std::numeric_limits<double>::quiet_NaN();
    if (segment < 1e-6) continue;
    const double fraction = std::min(1., (lookahead-length)/segment);
    endpoint = {previous.first+fraction*dx, previous.second+fraction*dy};
    length += fraction*segment;
    if (length >= lookahead-1e-6) break;
    previous = next;
  }
  const double dx = endpoint.first-origin.first, dy = endpoint.second-origin.second;
  return std::hypot(dx,dy) >= .05 ? std::atan2(dy,dx) :
      std::numeric_limits<double>::quiet_NaN();
}

// Heading feedback from TF and measured yaw rate controls the bounded turn.
class StartHeading {
 public:
  StartHeading(double rate, double tolerance, double settle, double timeout, double kp, double kd)
      : rate_(rate), tolerance_(tolerance), settle_(settle), timeout_(timeout), kp_(kp), kd_(kd) {}

  void start(double now) {
    start_ = now;
    quiet_since_ = -1.;
    direction_ = 0;
    done_ = motion_seen_ = false;
  }

  HeadingStep update(double error, double yaw_rate, bool stopped, double now) {
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
    const double speed = std::max(0., std::min(rate_, kp_*std::abs(error)-kd_*yaw_rate*direction_));
    return {speed*direction_, false, false};
  }

 private:
  const double rate_, tolerance_, settle_, timeout_, kp_, kd_;
  double start_ = 0., quiet_since_ = -1.;
  int direction_ = 0;
  bool done_ = false, motion_seen_ = false;
};

}  // namespace slicer_navigation
