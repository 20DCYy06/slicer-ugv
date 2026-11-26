#pragma once

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <utility>

namespace slicer_navigation {

struct HeadingStep {
  double angular = 0.;
  bool ready = false;
  bool failed = false;
};

using HeadingPoint = std::pair<double, double>;

constexpr double kHeadingPi = 3.14159265358979323846;

// Start at the closest projection on the path and find its first exit from
// the lookahead circle. A short path uses its last point.
template<class Iter, class Position>
double pathLookaheadChord(Iter begin, Iter end, Position position,
                          HeadingPoint center, double lookahead) {
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  if (begin == end) return invalid;
  const auto relative = [&](const auto& pose) {
    const auto point = position(pose);
    return HeadingPoint{point.first-center.first, point.second-center.second};
  };
  const auto dot = [](HeadingPoint a, HeadingPoint b) {
    return a.first*b.first + a.second*b.second;
  };
  const auto delta = [](HeadingPoint a, HeadingPoint b) {
    return HeadingPoint{a.first-b.first, a.second-b.second};
  };
  const auto advance = [](HeadingPoint a, HeadingPoint d, double fraction) {
    return HeadingPoint{a.first+fraction*d.first, a.second+fraction*d.second};
  };
  auto previous = relative(*begin);
  if (!std::isfinite(dot(previous, previous))) return invalid;
  auto closest = previous;
  auto next = std::next(begin);
  for (auto it = next; it != end; ++it) {
    const auto point = relative(*it);
    if (!std::isfinite(dot(point, point))) return invalid;
    const auto d = delta(point, previous);
    const double length_sq = dot(d, d);
    if (length_sq > 1e-12) {
      const auto projected = advance(previous, d, std::max(0., std::min(1., -dot(previous, d)/length_sq)));
      if (dot(projected, projected) < dot(closest, closest)) {
        closest = projected;
        next = it;
      }
    }
    previous = point;
  }

  const double radius_sq = lookahead*lookahead;
  if (dot(closest, closest) > radius_sq) return invalid;
  auto endpoint = closest;
  for (auto it = next; it != end; ++it) {
    const auto point = relative(*it);
    if (dot(point, point) >= radius_sq) {
      const auto d = delta(point, endpoint);
      const double a = dot(d, d);
      const double b = dot(endpoint, d);
      const double c = dot(endpoint, endpoint)-radius_sq;
      if (a > 1e-12)
        endpoint = advance(endpoint, d, (-b + std::sqrt(std::max(0., b*b-a*c)))/a);
      break;
    }
    endpoint = point;
  }
  return std::hypot(endpoint.first, endpoint.second) >= .05 ?
      std::atan2(endpoint.second, endpoint.first) : invalid;
}

template<class IsBlocked>
bool headingCorridorBlocked(HeadingPoint origin, double yaw, double distance,
                            double sample_step, IsBlocked is_blocked) {
  const int steps = std::max(1, static_cast<int>(std::ceil(distance/sample_step)));
  const double dx = std::cos(yaw)*distance/steps;
  const double dy = std::sin(yaw)*distance/steps;
  for (int i = 1; i <= steps; ++i)
    if (is_blocked(origin.first+i*dx, origin.second+i*dy)) return true;
  return false;
}

class AlignmentHeadingLock {
 public:
  bool active() const { return std::isfinite(locked_align_yaw_); }
  double yaw() const { return locked_align_yaw_; }
  void acquire(double yaw) {
    if (!active()) locked_align_yaw_ = yaw;
  }
  void beginForward(HeadingPoint origin, double odom_yaw) {
    origin_ = origin;
    forward_ = {std::cos(odom_yaw), std::sin(odom_yaw)};
  }
  bool travelledForward(HeadingPoint point, double distance) const {
    return (point.first-origin_.first)*forward_.first +
           (point.second-origin_.second)*forward_.second > distance;
  }
  void clear() { locked_align_yaw_ = std::numeric_limits<double>::quiet_NaN(); }

 private:
  double locked_align_yaw_ = std::numeric_limits<double>::quiet_NaN();
  HeadingPoint origin_{}, forward_{};
};

// Heading feedback from TF and measured yaw rate controls the bounded turn.
class StartHeading {
 public:
  StartHeading(double rate, double tolerance, double settle, double timeout, double kp, double kd)
      : rate_(rate), tolerance_(tolerance), settle_(settle), timeout_(timeout), kp_(kp), kd_(kd) {}

  void start(double now) {
    start_ = now;
    quiet_since_ = -1.;
    direction_ = 0;
    done_ = motion_seen_ = has_error_ = false;
  }

  HeadingStep update(double error, double yaw_rate, bool stopped, double now) {
    if (done_) return {0., true, false};
    if (now - start_ >= timeout_) return {0., false, true};

    // Unwrap successive errors so ±pi does not change the chosen turn direction.
    remaining_ = has_error_ ? remaining_ + std::remainder(error-last_error_, 2.*kHeadingPi) : error;
    last_error_ = error;
    has_error_ = true;
    motion_seen_ = motion_seen_ || !stopped || direction_ != 0;
    if (std::abs(remaining_) <= tolerance_) {
      if (!motion_seen_) done_ = true;
      else if (!stopped) quiet_since_ = -1.;
      else if (quiet_since_ < 0.) quiet_since_ = now;
      else if (now - quiet_since_ >= settle_) done_ = true;
      return {0., done_, false};
    }
    quiet_since_ = -1.;
    if (direction_ == 0) {
      if (!stopped) return {};
      direction_ = remaining_ > 0. ? 1 : -1;
    }
    // After overshoot, stop instead of reversing within the same alignment.
    // A persistent error outside tolerance is handled by the existing timeout.
    if (remaining_*direction_ <= 0.) return {};
    const double speed = std::max(0., std::min(rate_, kp_*std::abs(remaining_)-kd_*yaw_rate*direction_));
    return {speed*direction_, false, false};
  }

 private:
  const double rate_, tolerance_, settle_, timeout_, kp_, kd_;
  double start_ = 0., quiet_since_ = -1.;
  double remaining_ = 0., last_error_ = 0.;
  int direction_ = 0;
  bool done_ = false, motion_seen_ = false, has_error_ = false;
};

}  // namespace slicer_navigation
