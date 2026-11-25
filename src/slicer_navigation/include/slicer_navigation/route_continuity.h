#pragma once

#include <slicer_navigation/bspline_core.h>
#include <limits>

namespace slicer_navigation {

// A lower-cost route may still use unknown cells beyond the reconnecting segment.
// The local planner and Guard independently require observed space for motion.
template<class CostAt>
double routeCost(const std::vector<Point2>& route, double spacing, double connector,
                 CostAt cost_at, int max_cost = 252) {
  if (route.size() < 2) return std::numeric_limits<double>::infinity();
  double distance = 0., cost = 0.;
  for (size_t i = 1; i < route.size(); ++i) {
    const Point2 delta = route[i] - route[i-1];
    const double length = norm(delta);
    const int n = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int j = 0; j <= n; ++j) {
      const double fraction = static_cast<double>(j) / n;
      const int cell = cost_at(route[i-1] + delta * fraction);
      if (cell < 0 || (cell != 255 && cell > max_cost) ||
          (cell == 255 && distance + fraction * length <= connector)) {
        return std::numeric_limits<double>::infinity();
      }
      if (j) cost += (1. + cell / 255.) * length / n;
    }
    distance += length;
  }
  return cost;
}

inline bool preferPreviousRoute(double previous, double alternative, double extra_cost) {
  return std::isfinite(previous) && previous <= alternative + extra_cost;
}


enum class RouteIssue { None, NoPath, Rejoin, Outside, Unknown, Clearance, Collision };

inline const char* issueName(RouteIssue issue) {
  switch (issue) {
    case RouteIssue::None: return "none";
    case RouteIssue::NoPath: return "no path";
    case RouteIssue::Rejoin: return "rejoin";
    case RouteIssue::Outside: return "outside map";
    case RouteIssue::Unknown: return "unknown";
    case RouteIssue::Clearance: return "clearance";
    case RouteIssue::Collision: return "collision";
  }
  return "invalid";
}

struct RouteBlock {
  RouteIssue issue = RouteIssue::None;
  double distance = 0.;
  int cost = 0;
};

struct RouteCheck {
  RouteBlock near;
  RouteBlock far;
  double observed = 0.;
  double cost = 0.;
  bool valid() const { return near.issue == RouteIssue::None && far.issue == RouteIssue::None; }
};

inline double checkedHorizon(double max_speed, double preview, double current_speed,
                             double minimum_time, double reaction_time, double brake) {
  // Match the Guard's constant-speed swept stopping envelope, and cover the
  // local planner's preview even when starting at rest.
  return std::max(max_speed * preview,
                  current_speed * std::max(minimum_time, reaction_time + current_speed / brake));
}

inline RouteIssue cellIssue(int cell, int max_cost) {
  if (cell < 0) return RouteIssue::Outside;
  if (cell == 255) return RouteIssue::Unknown;
  if (cell == 254) return RouteIssue::Collision;
  if (cell > max_cost) return RouteIssue::Clearance;
  return RouteIssue::None;
}

template<class CostAt, class FootprintAt>
RouteCheck inspectRoute(const std::vector<Point2>& route, double spacing, double horizon,
                        int max_cost, bool allow_unknown, CostAt cost_at, FootprintAt footprint_at) {
  RouteCheck check;
  if (route.size() < 2) {
    check.near.issue = RouteIssue::NoPath;
    return check;
  }
  double distance = 0.;
  bool observed = true;
  for (size_t i = 1; i < route.size(); ++i) {
    const Point2 delta = route[i] - route[i - 1];
    const double length = norm(delta);
    const double yaw = std::atan2(delta.y, delta.x);
    const int n = std::max(1, static_cast<int>(std::ceil(length / spacing)));
    for (int j = 0; j <= n; ++j) {
      const double advance = length * j / n;
      const Point2 sample = route[i - 1] + delta * (double(j) / n);
      const int cell = cost_at(sample);
      const double at = distance + advance;
      const bool near = at <= horizon + spacing * .5;
      RouteIssue issue = cellIssue(cell, max_cost);
      if (issue == RouteIssue::None && near && length > 1e-9)
        issue = footprint_at(sample, yaw);
      if (issue != RouteIssue::None) {
        observed = false;
        if (near) {
          check.near = {issue, at, cell};
          return check;
        }
        if (check.far.issue == RouteIssue::None && (issue != RouteIssue::Unknown || !allow_unknown))
          check.far = {issue, at, cell};
      }
      if (observed) check.observed = at;
      if (j) check.cost += (1. + std::max(0, cell) / 255.) * length / n;
    }
    distance += length;
  }
  return check;
}

inline Point2 routePoint(const std::vector<Point2>& route, double distance) {
  for (size_t i = 1; i < route.size(); ++i) {
    const Point2 delta = route[i] - route[i - 1];
    const double length = norm(delta);
    if (length > 0. && distance <= length)
      return route[i - 1] + delta * (distance / length);
    distance -= length;
  }
  return route.back();
}

inline bool sameCorridor(const std::vector<Point2>& a, const std::vector<Point2>& b,
                         double horizon, double deviation) {
  for (double fraction : {.25, .5, 1.}) {
    if (norm(routePoint(a, horizon * fraction) - routePoint(b, horizon * fraction)) > deviation)
      return false;
  }
  return true;
}

}  // namespace slicer_navigation
