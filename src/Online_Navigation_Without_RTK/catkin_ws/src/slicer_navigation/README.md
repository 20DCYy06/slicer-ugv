# Slicer navigation — ROS Noetic Docker

## Control flow

`GlobalPlanner/Dijkstra -> B-spline plugin -> TEB -> /cmd_vel/nav_raw -> slicer_nav_arc_guard -> /cmd_vel/nav -> velocity_mux -> /cmd_vel -> C30D`

B-spline global-path postprocessing, collision fallback, configuration and validation:
[docs/BSPLINE.md](docs/BSPLINE.md).

The `slicer_nav_arc_guard` name and topics are retained for compatibility. Its
implementation is a velocity smoother with forward-only turn-start compensation.
Ordinary TEB commands are saturated and ramped without a second path veto. Only
reverse rejection or low-speed turn boosting changes the planned path; those
modified paths are checked against the local grid and fresh TF before use.
There is no radius restriction or heading-alignment veto. Pure spin is allowed.
Explicit zero, invalid input, and
commands older than 0.5 seconds produce zero output. A stop is immediate.

Obstacle collision decisions belong to move_base/TEB using the measured polygon,
full 360-degree `/scan`, and global/local costmaps. Removing the duplicate velocity
veto does not remove planner collision checks. `/scan_front` remains the 160-degree
inspection topic. AMCL, RViz, recording, self-filtering, and C30D firmware are retained.

Patrol faces the target before sending each waypoint to move_base, including the
first target of a newly started sequence. **The first measured XY entry within
0.15 m is latched immediately**, without waiting for action success, final yaw or
remaining via-points. Patrol publishes zero through the mission mux, cancels the
position goal and waits for its acknowledgement. It then publishes **vx=0 only**
on `/cmd_vel/patrol_spin_raw` through the existing navigation outlet, rotating to
the YAML yaw. Error below **0.30 rad** stops the wheels and finishes the waypoint.
After the existing dwell time, patrol aligns the measured heading toward the next
waypoint before sending its navigation goal. Departure uses **0.15 rad** tolerance,
separate from the unchanged **0.30 rad** final yaw tolerance. After recovery or a
navigation pause, departure alignment is repeated before re-sending. Position is never checked
again for that waypoint after the XY latch, so a localization shift during spin
does not create another recenter goal. Final yaw uses the existing 300-permille
turn floor, 350-permille upper bound and acceleration ramp. No extra displacement
limit or separate spin deadline is added. Existing chassis/localization pauses,
collision handling remain unchanged. The 120-second navigation budget no longer
cuts short either heading-alignment phase: only measured heading error ends it.
Heading comes from the existing map/base TF (IMU/LIO + EKF + map localization),
not an unreferenced raw IMU quaternion or a fixed-duration rotation.
YAML coordinates remain in `frame_id`. A waypoint without yaw requires no final
orientation adjustment. Per-point tolerance overrides remain supported.

At least 2 cm translation **or** 0.035 rad rotation counts as progress. Three seconds
without measured progress (six seconds within 0.35 m of the goal) triggers
recovery, even if TEB keeps publishing a fresh nonempty local plan. A nonempty
band does not prove that the chassis can execute it. A failed action or
navigation timeout also triggers recovery. A departure turn with no measured
motion uses that same stall window and backup budget, rather than spinning
indefinitely against an obstacle. No additional collision filter is introduced.
After cancellation is acknowledged, patrol commands straight reverse at
**-0.20 m/s for 2.2 s**, then zero for 0.5 s, asynchronously calls
`/move_base/clear_costmaps`, waits at least 0.5 s for fresh scan/grid data and
re-sends the same goal. The commanded reverse distance is about 0.44 m; the
open-loop PWM driver does not guarantee that physical distance.
Patrol holds zero through the mission mux while cancelling or clearing. A 2-second
cancellation delay prints `CANCEL_PENDING` with diagnostics; it waits up to 15
seconds for acknowledgement before failing, and never re-sends an unacknowledged
goal. `max_backup_attempts: 2` limits recovery per waypoint. A further planning
failure skips that point after cancellation acknowledgement and continues the
sequence; the log records `SKIPPED`, not `REACHED`. `COMPLETED` means the sequence
finished, and a warning explicitly lists skipped points if any. Each re-issued
navigation attempt has its existing 120-second budget. Loss of fresh
TF/localization/chassis pauses a goal; a single pause
may last at most 15 seconds. Ctrl+C cancels the active action. Normal completion
exits 0; `auto_exit:=false` leaves the node resident. No required-node death warning.

Final yaw tolerance is 0.30 rad in both patrol and TEB. The launch loads
`config/patrol_params.yaml` directly; it does not override these stall settings.

Recovery reverse uses the existing `/cmd_vel/mission` mux input (keyboard still
has priority), not the forward-only ordinary navigation smoother. Driver PWM
limits, slew and MCU watchdogs remain in place. No new rear-obstacle check is
introduced by this change: keep at least the rear 0.5 m clear during field acceptance.
An interrupted backup stops rather than restarting its full timed action.

## Parameter ownership

| Source | Owns |
| --- | --- |
| `Slicer_Workspace/config/slicer_config.yaml`, section `navigation` | Vehicle envelope, padding, speeds, accelerations, radius, inflation and forward preference |
| `config/navigation_params.yaml` | GlobalPlanner algorithm and move_base native recovery |
| `config/teb_local_planner_params.yaml` | TEB optimization, feasibility and horizon settings |
| `config/arc_guard.yaml` | Smoother topics, rate and command watchdog |
| `config/patrol_params.yaml` | Mission waiting, stall and recovery settings |

`scripts/prepare_workspace.py` generates `.generated/navigation.yaml` from vehicle
settings. Algorithm parameters are loaded afterward from this package. Physical
limits are not duplicated in the override YAMLs. `navigation.max_forward` is the
shared planner/smoother forward command limit, not a calibrated physical-speed
guarantee for an open-loop motor driver.

Defaults: radius **0 m**, forward 0.125 m/s, ordinary planner body reverse **disabled** (timed recovery exception above), angular 1.2362573 rad/s (350‰ reference),
angular acceleration 1.2 rad/s², forward weight 900, inflation radius 0.192 m,
scaling 4.5, obstacle distance 0.144 m and obstacle weight 60. TEB inflation distance
is 0.25 m; this is separate from costmap inflation radius. Padding is 0.03 m,
applied **once** when generating the shared polygon. Measured left-to-right
width is 0.22 m (not a 0.22 m radius), so physical half-width is 0.11 m and
padded half-width is 0.14 m. Front/rear dimensions are unchanged; costmap `footprint_padding`
is consequently 0, while TEB sees the same already-padded vertices. Distances are
soft planning targets, not proof of guaranteed real-world clearance. The actual
wheel-end count remains provisional 60000 counts/rev; EKF uses wheel `/wheel/odom`
vx only and retains LIO yaw. The real footprint and encoder chain are not replaced.

`use_grid_path: true` selects discrete grid path extraction. It is paired with
`old_navfn_behavior: true` for integer-cell start initialization: the installed
planner reproduced NO PATH on a captured field grid with the default precise
initialization, while integer initialization returned 94 poses on that same grid.
Dijkstra and unknown
global cells are enabled. The local obstacle layer continues to require observed
free space. The navigation outlet clips negative body vx to zero and retains wz.
Feasibility checks cover three poses; actual lethal cells remain
blocked.

TEB optimizes a single band with at most 60 samples, 3 inner/3 outer iterations and
1.5 m lookahead. This bounds controller work after field warnings reported 3–14 s
control cycles and up to 21 s map-update cycles. It preserves collision checks;
the global planner supplies obstacle detours. The 0.5 s velocity watchdog is not
extended to replay old velocity commands across those stalls.

At `STALL`, `RECOVERY_TRIGGER` and `CANCEL_PENDING`, `DIAG` prints actual received
message counts, last values, age/gaps, publisher connections, action status age,
scan/grid age and recent move_base warnings. `MOVE_BASE` lines relay planner
warnings to the same terminal, including path-extraction and missed-cycle errors.

## Compatible commands

All ROS execution/build/check operations use the configured Noetic Docker image.
Run the wrapper on the Jetson host; do not source Humble for these commands.

```bash
cd /home/potato/Slicer_Workspace
NAV=./catkin_ws/src/slicer_navigation/scripts/run_navigation.sh
"$NAV" build
"$NAV" check

"$NAV" mapping --background start_control:=true gui:=true
"$NAV" wasd
"$NAV" save-map my_room
"$NAV" stop

"$NAV" nav --background my_room.yaml start_control:=true gui:=true
"$NAV" record waypoints.yaml
"$NAV" patrol
"$NAV" stop
```

`record` stays in the foreground and prints recorded names and coordinates.
Same-name maps are replaced with backup. Mapping/navigation run separately and
reuse the existing WASD driver/mux. `patrol file:=... cycles:=2 auto_exit:=false`
remains supported. Parameter changes take effect after restarting navigation.

Read-only diagnosis (use coordinates from your current waypoints):

```bash
docker logs --tail 100 slicer-navigation
docker exec -it slicer-navigation bash -lc 'source /opt/ros/noetic/setup.bash; rostopic echo /cmd_vel/nav_raw'
docker exec -it slicer-navigation bash -lc 'source /opt/ros/noetic/setup.bash; rostopic echo /cmd_vel/nav'
docker exec -it slicer-navigation bash -lc 'source /opt/ros/noetic/setup.bash; rostopic hz /move_base/TebLocalPlannerROS/local_plan'
docker exec -it slicer-navigation bash -lc 'source /opt/ros/noetic/setup.bash; rostopic hz /wheel/odom'
```

See `VALIDATION.md` for what the isolated checks prove and their limits.

Forward-turn tuning keeps radius zero, initializes TEB without a backwards band,
and uses a strong (900) forward preference. The hard body-reverse prohibition is
`navigation.max_reverse: 0.0` at the output, rather than relying on this soft weight.
`turning_radius_opt_threshold` is not implemented by the installed TEB and is not
set. The installed TEB has no `min_vel_theta`. Its internal `max_vel_x_backwards`
stays 0.02 because zero is not a valid hard reverse prohibition in TEB; the nav
outlet's zero limit represents permitted body motion.
The smoother ramps linear/angular components independently; a tiny linear sign
change no longer resets angular velocity. Explicit stops remain immediate.

`navigation.turn_start_angular: 1.0596491228` retains the previous angular
floor directly in rad/s. Near-stationary deliberate turns are boosted to that
value, below the unchanged 1.2362573 rad/s cap, with a 1.2 rad/s² ramp. Eligibility
is |vx| <= 0.03 and |wz| >= 0.02; zero/noise are never boosted. If the increased
swept footprint is blocked, a safe unboosted turn can be retained; forbidden
reverse is never restored. `adjustment` and console logs explain the decision.
The C30D driver converts the resulting Twist to encoder-feedback wheel-speed targets;
the navigation planner, obstacle checks and velocity topics are unchanged.
STALL requires both small measured translation and small wrapped yaw change;
`NO_PROGRESS` prints both values rather than trusting the velocity command.

Patrol now prints `REACHED point_001 elapsed=...s`, including final orientation.
For a read-only command/encoder recording, run in another terminal before patrol:

```bash
docker exec -it slicer-navigation bash -lc 'source /opt/ros/noetic/setup.bash; python3 /slicer/catkin_ws/src/slicer_navigation/scripts/measure_patrol.py --output /slicer/logs/navigation/turn_tuning.jsonl --duration 180'
```

The `.summary.json` reports per-point elapsed time and command sample fractions.
Reverse/turning samples describe commands, not measured physical displacement.
`wheel` traces are encoder feedback. The original provided successful run took
60.34 s to point_001 and 33.44 s to point_002; field starts/maps must be comparable
before claiming an improvement. Keeping a 0.125 m/s forward limit also sets a lower
bound on physically achievable route time; raising turn limits does not guarantee
that every route can finish in 15–25 s.
