# Validation scope

Run on the Jetson host:

```bash
cd /home/potato/Slicer_Workspace
NAV=./catkin_ws/src/slicer_navigation/scripts/run_navigation.sh
"$NAV" build
"$NAV" check
```

`check` uses a `--network none` Noetic Docker container without serial/LiDAR device
access. It parses Python/YAML/launch/shell files, checks runtime dependencies,
exercises the existing WASD panel/map storage/self-filter, and runs two isolated
ROS tests:

1. A fake move_base server deliberately stalls. Patrol cancels, clears and retries.
   The retry reaches XY with a deliberately wrong yaw, but never reports action
   success. Patrol must latch XY, cancel the action, then issue pure yaw commands.
   During each yaw phase a 25 cm localization position shift is injected; no
   recenter action may be sent. Both waypoint yaw phases must complete and exit 0.
2. Real AMCL, GlobalPlanner and TEB load a synthetic map with a lethal obstacle
   between robot and goal. The robot starts more than 120 degrees away from the
   goal bearing. The test requires a global detour avoiding lethal cells, a
   nonempty TEB path and nonzero raw **and smoothed** velocity. It also checks
   pure-spin passthrough, goal cancellation and command timeout stopping.

These tests do not prove tire traction, open-loop PWM speed calibration, physical
braking distance or the accuracy of a particular saved field map. An actual motor
start is a separate field test. A planner can legitimately return NO PATH when
the true footprint cannot fit or the target is occupied; the smoother must not
invent motion to bypass that decision.

The patrol test also delays cancellation acknowledgement by 3.2 seconds, beyond
the original 2-second abort threshold. It must report `CANCEL_PENDING`, retain a
zero-speed hold, and complete both full-pose goals after acknowledgement.

Field GridPath reproduction (2026-10-21): `scripts/snapshot_planner_grid.py` captures
the existing global OccupancyGrid, TF pose and first waypoint without issuing any
goal. `tests/probe_global_grid.cpp` loads the capture in an isolated ROS master and
exercises the installed GlobalPlanner in four configurations. OccupancyGrid cost
translation is lossy; all four runs use the same reconstructed costs.

| Integer start (`old_navfn_behavior`) | Quadratic potential | Result |
| --- | --- | --- |
| false | true | NO PATH, 0 poses, 0.144 s |
| true | true | 94 poses, 0.009 s |
| false | false | NO PATH, 0 poses, 0.137 s |
| true | false | 94 poses, 0.007 s |

The deployed combination keeps quadratic potentials and switches integer start
initialization on. Captured data and the probe binary are under `logs/navigation/`.
