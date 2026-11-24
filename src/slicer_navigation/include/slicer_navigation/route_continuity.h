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

}  // namespace slicer_navigation
