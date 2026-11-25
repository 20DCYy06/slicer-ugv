#pragma once

#include <slicer_navigation/bspline_core.h>

#include <cstdint>
#include <limits>

namespace slicer_navigation {

struct ContinuityCostOptions {
  double weight = 2.;
  double decay_length_m = 2.;
  double obstacle_weight = .70;
  double deviation_cap_m = 1.;
  double min_clearance_m = .23;
  double rejoin_distance_m = .30;
};

inline bool validContinuityOptions(const ContinuityCostOptions& options) {
  const auto nonnegative = [](double value) { return std::isfinite(value) && value >= 0.; };
  const double deviation_squared = options.deviation_cap_m * options.deviation_cap_m;
  return nonnegative(options.weight) && nonnegative(options.obstacle_weight) &&
      std::isfinite(options.decay_length_m) && options.decay_length_m > 0. &&
      nonnegative(options.deviation_cap_m) && std::isfinite(deviation_squared * options.weight) &&
      nonnegative(options.min_clearance_m) && nonnegative(options.rejoin_distance_m);
}

namespace continuity_detail {

inline bool finitePoint(const Point2& point) {
  return std::isfinite(point.x) && std::isfinite(point.y);
}

inline Point2 projection(const Point2& point, const Point2& start, const Point2& end) {
  const Point2 segment = end - start;
  const double squared_length = dot(segment, segment);
  if (squared_length == 0.) return start;
  const double along = dot(point - start, segment) / squared_length;
  if (along <= 0.) return start;
  if (along >= 1.) return end;
  return start + segment * along;
}

struct CellBox {
  int x_begin = 0, x_end = 0;
  int y_begin = 0, y_end = 0;
  bool contains(int x, int y) const {
    return x >= x_begin && x < x_end && y >= y_begin && y < y_end;
  }
  std::size_t rows() const { return std::max(0, y_end - y_begin); }
  std::size_t columns() const { return std::max(0, x_end - x_begin); }
  std::size_t index(int x, int y) const { return (y - y_begin) * columns() + x - x_begin; }
};

inline CellBox cellBox(const Point2& lower, const Point2& upper, const Point2& origin,
                       double resolution, int width, int height) {
  const auto clip = [](double grid_index, int size) {
    return static_cast<int>(std::max(0., std::min(static_cast<double>(size), grid_index)));
  };
  return {
    clip(std::ceil((lower.x - origin.x) / resolution - .5), width),
    clip(std::floor((upper.x - origin.x) / resolution - .5) + 1., width),
    clip(std::ceil((lower.y - origin.y) / resolution - .5), height),
    clip(std::floor((upper.y - origin.y) / resolution - .5) + 1., height),
  };
}

inline double center(int index, double origin, double resolution) {
  return origin + (index + .5) * resolution;
}

}  // namespace continuity_detail

inline std::vector<Point2> remainingRoute(const std::vector<Point2>& previous, const Point2& start,
                                          double rejoin_distance) {
  using continuity_detail::finitePoint;
  using continuity_detail::projection;
  if (previous.size() < 2 || !finitePoint(start) ||
      !std::isfinite(rejoin_distance) || rejoin_distance < 0. ||
      !std::all_of(previous.begin(), previous.end(), finitePoint)) return {};

  std::size_t nearest_end = 0;
  double minimum = std::numeric_limits<double>::infinity();
  Point2 nearest;
  for (std::size_t i = 1; i < previous.size(); ++i) {
    const Point2 projected = projection(start, previous[i-1], previous[i]);
    const Point2 difference = projected - start;
    const double squared_distance = dot(difference, difference);
    // Keep the earlier segment when a route crosses or retraces itself.
    if (squared_distance < minimum) {
      minimum = squared_distance;
      nearest = projected;
      nearest_end = i;
    }
  }
  if (!nearest_end || minimum > rejoin_distance * rejoin_distance) return {};
  std::vector<Point2> suffix{nearest};
  suffix.reserve(previous.size() - nearest_end + 1);
  for (std::size_t i = nearest_end; i < previous.size(); ++i) {
    if (previous[i].x != suffix.back().x || previous[i].y != suffix.back().y) {
      suffix.push_back(previous[i]);
    }
  }
  return suffix;
}

inline std::vector<std::uint8_t> hardContinuityMask(const std::uint8_t* measured,
                                                    int width, int height,
                                                    double resolution, double radius) {
  if (!measured || width <= 0 || height <= 0 || !std::isfinite(resolution) || resolution <= 0. ||
      !std::isfinite(radius) || radius < 0.) return {};
  std::vector<std::uint8_t> hard(static_cast<std::size_t>(width) * height, 0);
  const auto row = [&](int y) { return hard.data() + static_cast<std::size_t>(y) * width; };
  const double cell_radius = std::nextafter(radius / resolution, std::numeric_limits<double>::infinity());
  const int border = static_cast<int>(std::min(static_cast<double>(std::max(width, height)),
                                               std::max(1., std::floor(cell_radius + .5))));
  const int vertical_reach = static_cast<int>(std::min(static_cast<double>(height-1), cell_radius));
  std::vector<int> row_reach(vertical_reach+1);
  for (int dy = 0; dy <= vertical_reach; ++dy) {
    const double reach = std::sqrt(std::max(0., cell_radius * cell_radius - double(dy) * dy));
    row_reach[dy] = static_cast<int>(std::min(static_cast<double>(width-1), reach));
  }

  for (int y = 0; y < height; ++y) {
    const bool horizontal_edge = y == 0 || y == height-1;
    const std::uint8_t* source = measured + static_cast<std::size_t>(y) * width;
    for (int x = 0; x < width; ++x) {
      const bool vertical_edge = x == 0 || x == width-1;
      if (x < border || x >= width - border || y < border || y >= height - border ||
          source[x] == 253 || source[x] == 254) {
        row(y)[x] = 1;
      }
      if (source[x] != 254 || !vertical_reach) continue;
      // In a solid block, a cardinal neighbor is closer to any outside cell.
      // Rasterizing the boundary sources is sufficient and avoids duplicate disks.
      if (!horizontal_edge && !vertical_edge && source[x-1] == 254 && source[x+1] == 254 &&
          source[x-width] == 254 && source[x+width] == 254) continue;
      for (int wy = std::max(0, y-vertical_reach); wy <= std::min(height-1, y+vertical_reach); ++wy) {
        const int reach = row_reach[std::abs(wy-y)];
        std::fill(row(wy) + std::max(0, x-reach), row(wy) + std::min(width-1, x+reach) + 1, 1);
      }
    }
  }
  return hard;
}

inline void encodeContinuityCosts(const std::uint8_t* measured, const std::vector<std::uint8_t>& hard,
                                  int width, int height, double resolution, Point2 origin, Point2 start,
                                  const std::vector<Point2>& reference,
                                  const ContinuityCostOptions& options, std::uint8_t* search) {
  using namespace continuity_detail;
  if (!measured || !search || width <= 0 || height <= 0 ||
      hard.size() != static_cast<std::size_t>(width) * height ||
      !std::isfinite(resolution) || resolution <= 0. || !finitePoint(origin) ||
      !validContinuityOptions(options)) return;

  const double cap = options.deviation_cap_m;
  const double radius = 4. * options.decay_length_m;
  const Point2 radial_extent(radius, radius);
  // Omit the exponential tail beyond four decay lengths to bound the
  // rasterized neighborhood independently of the full map size.
  const bool has_bias = options.weight > 0. && cap > 0. && finitePoint(start) &&
      !reference.empty() && std::all_of(reference.begin(), reference.end(), finitePoint);
  const CellBox window = has_bias
      ? cellBox(start - radial_extent, start + radial_extent, origin, resolution, width, height)
      : CellBox{};
  std::vector<double> lateral_squared(window.rows() * window.columns(), cap * cap);
  const std::size_t first_end = reference.size() == 1 ? 0 : 1;
  for (std::size_t i = first_end; !lateral_squared.empty() && i < reference.size(); ++i) {
    const Point2 a = reference[i == 0 ? 0 : i-1], b = reference[i];
    const CellBox box = cellBox({std::min(a.x,b.x)-cap, std::min(a.y,b.y)-cap},
                                {std::max(a.x,b.x)+cap, std::max(a.y,b.y)+cap},
                                origin, resolution, width, height);
    for (int y = std::max(box.y_begin, window.y_begin); y < std::min(box.y_end, window.y_end); ++y) {
      const double wy = center(y, origin.y, resolution);
      for (int x = std::max(box.x_begin, window.x_begin); x < std::min(box.x_end, window.x_end); ++x) {
        const Point2 point(center(x, origin.x, resolution), wy);
        const Point2 difference = point - projection(point, a, b);
        double& nearest = lateral_squared[window.index(x,y)];
        nearest = std::min(nearest, dot(difference, difference));
      }
    }
  }

  for (int y = 0; y < height; ++y) {
    const double dy = center(y, origin.y, resolution) - start.y;
    for (int x = 0; x < width; ++x) {
      const auto i = static_cast<std::size_t>(y) * width + x;
      const std::uint8_t raw = measured[i];
      if (raw == 253 || raw == 254) {
        search[i] = raw;
      } else if (hard[i]) {
        search[i] = 254;
      } else if (raw == 255) {
        search[i] = raw;
      } else {
        double preference = 0.;
        if (window.contains(x, y)) {
          const double dx = center(x, origin.x, resolution) - start.x;
          const double squared_distance = dx * dx + dy * dy;
          if (squared_distance <= radius * radius) {
            preference = options.weight * std::exp(-std::sqrt(squared_distance) / options.decay_length_m)
                       * lateral_squared[window.index(x,y)];
          }
        }
        const double scaled_cost = (50. / .8) * (options.obstacle_weight * (raw / 252.) + preference);
        search[i] = static_cast<std::uint8_t>(std::round(std::min(252., scaled_cost)));
      }
    }
  }
}

}  // namespace slicer_navigation
