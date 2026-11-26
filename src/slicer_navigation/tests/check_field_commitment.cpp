#include <slicer_navigation/field_commitment.h>
#include <cmath>
#include <cstdlib>
#include <iostream>

using namespace slicer_navigation;
#define CHECK(expression) do { if (!(expression)) { std::cerr << #expression << '\n'; return 1; } } while (false)

int main() {
  const std::vector<Point2> straight{{0.,0.},{1.,0.},{4.,0.}};
  const auto known = routeSamples(straight, .025, 4.);
  const auto anchor = commitmentAnchor(known, [](Point2 p) { return p.x < 3.; }, .5);
  CHECK(std::abs(anchor.x - 2.5) <= .026);
  CHECK(norm(anchor - Point2{1.6,0.}) >= .8);
  CHECK(norm(anchor - Point2{1.8,0.}) < .8);
  const auto empty = commitmentAnchor(known, [](Point2) { return false; }, .5);
  CHECK(norm(empty) == 0.);
  const std::vector<Point2> corner{{0.,0.},{1.,0.},{1.,3.}};
  const auto clipped = routeSamples(corner, .05, 1.2);
  CHECK(std::abs(clipped.back().x - 1.) < 1e-9 && std::abs(clipped.back().y - .2) < 1e-9);

  SideHysteresis side;
  side.reset({0.,0.}, {6.,0.});
  const std::vector<Point2> left{{0.,0.},{2.,1.2},{4.,1.2},{6.,0.}};
  const std::vector<Point2> right{{0.,0.},{2.,-1.2},{4.,-1.2},{6.,0.}};
  side.accept(left);
  CHECK(side.side() == 1);
  for (int i=0; i<10; ++i) CHECK(side.keep(right, routeLength(left)));
  CHECK(!side.keep(right, 20.));
  side.accept(right);
  CHECK(side.side() == -1 && side.keep(left, routeLength(right)));
  side.reset({0.,0.}, {6.,0.});
  CHECK(!side.keep(left, routeLength(right)));
  std::cout << "Known prefix, margin, anchor radius and symmetric side hysteresis passed\n";
}
