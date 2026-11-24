#include <cassert>
#include <utility>
#include <vector>
#include <slicer_navigation/start_heading.h>

using slicer_navigation::StartHeading;

int main() {
  StartHeading turn(0.45, 0.12, 0.25, 16., 1.6, .25);
  turn.start(0.);
  assert(turn.update(.9, 0., false, .1).angular == 0.);
  assert(turn.update(.9, 0., true, .2).angular == .45);
  assert(turn.update(.4, 0., false, 1.1).angular == .45);
  assert(turn.update(.2, .3, false, 1.2).angular <
         turn.update(.2, 0., false, 1.2).angular);
  assert(turn.update(.2, 0., false, 1.2).angular < .45);
  assert(turn.update(.10, 0., false, 1.8).angular == 0.);
  assert(!turn.update(.10, 0., true, 2.).ready);
  assert(turn.update(.10, 0., true, 2.3).ready);
  assert(turn.update(.3, 0., false, 2.4).ready);

  turn.start(5.);
  assert(turn.update(-.8, 0., true, 5.1).angular == -.45);
  assert(turn.update(.2, 0., false, 6.).angular == 0.);
  assert(turn.update(.2, 0., true, 6.2).angular > 0.);
  assert(turn.update(.2, 0., false, 21.).failed);
  assert(turn.update(0., 0., true, 21.5).failed);

  turn.start(22.);
  assert(!turn.update(.02, 0., false, 22.).ready);
  assert(!turn.update(.02, 0., true, 22.2).ready);
  assert(turn.update(.02, 0., true, 22.5).ready);

  turn.start(23.);
  assert(turn.update(.02, 0., true, 23.).ready);
  const std::vector<std::pair<double, double>> plan{{0.,0.},{0.,0.},{0.,.35},{3.,.35}};
  const auto point = [](const auto& p) { return p; };
  const double bearing = slicer_navigation::pathStartTangent(plan.begin(), plan.end(), point, .20);
  assert(std::abs(bearing - std::atan2(1., 0.)) < 1e-9);
}
