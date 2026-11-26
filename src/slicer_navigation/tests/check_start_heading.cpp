#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>
#include <slicer_navigation/start_heading.h>

using slicer_navigation::AlignmentHeadingLock;
using slicer_navigation::HeadingPoint;
using slicer_navigation::StartHeading;
using slicer_navigation::headingCorridorBlocked;
using slicer_navigation::kHeadingPi;
using slicer_navigation::pathLookaheadChord;

constexpr double radians = kHeadingPi / 180.;

void require(bool pass, const char* description) {
  if (!pass) {
    std::cerr << description << '\n';
    std::exit(1);
  }
}

#define CHECK(expression) require((expression), #expression)

void symmetricObstacleReplansKeepOneTurn() {
  const HeadingPoint center{0., 0.};
  const double target_yaw = -161.*radians;
  const auto polar = [](double distance, double yaw) {
    return HeadingPoint{distance*std::cos(yaw), distance*std::sin(yaw)};
  };
  const auto goal = polar(3., target_yaw);
  // Two candidate paths at the same symmetric rectangular obstacle.
  // The short initial segments point to opposite sides; both join the
  // same rear goal. The first path is selected by the simulated planner.
  const std::vector<HeadingPoint> first{
      center, {.03, .18}, polar(.65, target_yaw), polar(2.1, target_yaw), goal};
  const std::vector<HeadingPoint> other{
      center, {.03, -.18}, polar(.65, -target_yaw), polar(2.1, -target_yaw), goal};
  const auto chord = [&](const auto& path) {
    return pathLookaheadChord(path.begin(), path.end(), [](const auto& p) { return p; }, center, 1.35);
  };
  const auto rectangle = [](double x, double y) { return .35<=x && x<=.85 && std::abs(y)<=.30; };
  CHECK(headingCorridorBlocked(center, 0., 1.5, .025, rectangle));
  CHECK(!headingCorridorBlocked(center, chord(first), 1.5, .025, rectangle));
  CHECK(!headingCorridorBlocked(center, chord(other), 1.5, .025, rectangle));
  CHECK(std::abs(chord(first) - target_yaw) < 1e-9);
  CHECK(std::abs(chord(other) + target_yaw) < 1e-9);

  AlignmentHeadingLock lock;
  lock.acquire(chord(first));
  StartHeading turn(.45, .12, .25, 16., 1.6, .25);
  turn.start(0.);
  constexpr double dt = .02;
  double yaw = 0., measured_rate = 0.;
  unsigned replan_count = 0, commanded = 0;
  double settled_at = 0.;
  for (unsigned i = 0; i < 800; ++i) {
    const double now = i*dt;
    if (i % 50 == 0) {
      lock.acquire(chord(replan_count++ % 2 == 0 ? other : first));
      CHECK(lock.yaw() == chord(first));
    }
    const double remaining = std::remainder(lock.yaw()-yaw, 2.*kHeadingPi);
    const auto output = turn.update(remaining, measured_rate, std::abs(measured_rate)<=.06, now);
    CHECK(!output.failed);
    CHECK(output.angular <= 0.);
    if (output.angular < 0.) ++commanded;
    measured_rate += .35*(output.angular-measured_rate);
    yaw += measured_rate*dt;
    if (output.ready) {
      CHECK(std::abs(remaining) < 10.*radians);
      settled_at = now;
      break;
    }
  }
  CHECK(replan_count >= 5 && commanded > 0 && settled_at > 0.);
  CHECK(lock.active());
  lock.beginForward(center, yaw);
  lock.acquire(chord(other));
  CHECK(lock.yaw() == chord(first));
  CHECK(!lock.travelledForward(polar(.49, yaw), .5));
  CHECK(!lock.travelledForward(polar(.70, yaw+kHeadingPi/2.), .5));
  CHECK(!lock.travelledForward(polar(.70, yaw+kHeadingPi), .5));
  CHECK(lock.travelledForward(polar(.51, yaw), .5));

  const auto obstacle = polar(1.46, lock.yaw());
  const auto new_obstacle = [&](double x, double y) {
    return std::max(std::abs(x-obstacle.first), std::abs(y-obstacle.second)) <= .025;
  };
  CHECK(!headingCorridorBlocked(center, lock.yaw(), 1.2, .025, new_obstacle));
  CHECK(headingCorridorBlocked(center, lock.yaw(), 1.5, .025, new_obstacle));
  lock.clear();
  lock.acquire(chord(other));
  CHECK(lock.yaw() == chord(other));
  std::cout << "161-degree symmetric-obstacle simulation passed: " << replan_count
            << " replans, settled at " << settled_at << " s, no angular sign reversal\n";
}

int main() {
  StartHeading turn(0.45, 0.12, 0.25, 16., 1.6, .25);
  turn.start(0.);
  CHECK(turn.update(.9, 0., false, .1).angular == 0.);
  CHECK(turn.update(.9, 0., true, .2).angular == .45);
  CHECK(turn.update(.4, 0., false, 1.1).angular == .45);
  CHECK(turn.update(.2, .3, false, 1.2).angular <
         turn.update(.2, 0., false, 1.2).angular);
  CHECK(turn.update(.2, 0., false, 1.2).angular < .45);
  CHECK(turn.update(.10, 0., false, 1.8).angular == 0.);
  CHECK(!turn.update(.10, 0., true, 2.).ready);
  CHECK(turn.update(.10, 0., true, 2.3).ready);
  CHECK(turn.update(.3, 0., false, 2.4).ready);

  turn.start(3.);
  CHECK(turn.update(-161. * radians, 0., true, 3.1).angular < 0.);
  // An opposite yaw estimate crosses the ±pi boundary while the locked
  // direction is unchanged. Even a stopped sample must not reverse the turn.
  CHECK(turn.update(179. * radians, 0., true, 3.2).angular < 0.);

  turn.start(5.);
  CHECK(turn.update(-.8, 0., true, 5.1).angular == -.45);
  CHECK(turn.update(.2, 0., false, 6.).angular == 0.);
  CHECK(turn.update(.2, 0., true, 6.2).angular == 0.);
  CHECK(turn.update(.2, 0., false, 21.).failed);
  CHECK(turn.update(0., 0., true, 21.5).failed);

  turn.start(22.);
  CHECK(!turn.update(.02, 0., false, 22.).ready);
  CHECK(!turn.update(.02, 0., true, 22.2).ready);
  CHECK(turn.update(.02, 0., true, 22.5).ready);

  turn.start(23.);
  CHECK(turn.update(.02, 0., true, 23.).ready);
  const std::vector<HeadingPoint> plan{{-.4,0.},{0.,0.},{0.,0.},{3.,0.}};
  const auto point = [](const auto& p) { return p; };
  CHECK(pathLookaheadChord(plan.begin(), plan.end(), point, {.5,.1}, 1.35) < 0.);
  const std::vector<HeadingPoint> short_goal{{0.,0.},{.4,.1}};
  CHECK(std::abs(pathLookaheadChord(short_goal.begin(), short_goal.end(), point, {0.,0.}, 1.35) -
                 std::atan2(.1, .4)) < 1e-9);
  CHECK(!std::isfinite(pathLookaheadChord(plan.begin(), plan.end(), point, {0.,3.}, 1.35)));
  symmetricObstacleReplansKeepOneTurn();
}
