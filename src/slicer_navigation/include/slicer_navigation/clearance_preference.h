#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace slicer_navigation {

// Extend a graded preference outside the hard inflation band on a planner copy.
// Unknown, lethal and inscribed cells keep their original meanings.
inline void preferClearance(std::uint8_t* costs, int width, int height, double resolution,
                            double radius, double inscribed, double decay, int cost_cap) {
  const int cells = static_cast<int>(std::ceil(radius / resolution));
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (costs[y * width + x] != 254) continue;
      for (int ny = std::max(0, y-cells); ny <= std::min(height-1, y+cells); ++ny) {
        for (int nx = std::max(0, x-cells); nx <= std::min(width-1, x+cells); ++nx) {
          auto& cost = costs[ny * width + nx];
          if (cost >= 253) continue;
          const double distance = resolution * std::hypot(nx-x, ny-y);
          if (distance > radius) continue;
          const int preference = static_cast<int>(252. * std::exp(-decay * (distance-inscribed)));
          cost = std::max(cost, static_cast<std::uint8_t>(std::min(cost_cap, preference)));
        }
      }
    }
  }
}

}  // namespace slicer_navigation
