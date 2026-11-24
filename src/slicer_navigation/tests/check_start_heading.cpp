#include <cassert>
#include <slicer_navigation/start_heading.h>

using slicer_navigation::StartHeading;

int main() {
  StartHeading turn(0.45, 0.12, 0.25, 16.);
  turn.start(0.);
  assert(turn.update(.9, false, .1).angular == 0.);
  assert(turn.update(.9, true, .2).angular == .45);
  assert(turn.update(.4, false, 1.1).angular == .45);
  assert(turn.update(.10, false, 1.8).angular == 0.);
  assert(!turn.update(.10, true, 2.).ready);
  assert(turn.update(.10, true, 2.3).ready);
  assert(turn.update(.3, false, 2.4).ready);

  turn.start(5.);
  assert(turn.update(-.8, true, 5.1).angular == -.45);
  assert(turn.update(.2, false, 6.).angular == 0.);
  assert(turn.update(.2, true, 6.2).angular == .45);
  assert(turn.update(.2, false, 21.).failed);
  assert(turn.update(0., true, 21.5).failed);

  turn.start(22.);
  assert(!turn.update(.02, false, 22.).ready);
  assert(!turn.update(.02, true, 22.2).ready);
  assert(turn.update(.02, true, 22.5).ready);

  turn.start(23.);
  assert(turn.update(.02, true, 23.).ready);
}
