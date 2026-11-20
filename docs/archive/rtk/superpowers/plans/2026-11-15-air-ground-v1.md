# Air–ground V1 implementation plan

**Goal:** One ROS task source schedules ordered WGS84 position-pass via points and one geographically reviewed parking goal.
**Spec:** `docs/CODEX_AIR_GROUND_V1_20261115.md` (user approved implementation; no further design approval).
**Baseline:** `76367a7`, branch `workspace2/air-ground-v1-20261115`; earlier dirty work saved in the outer `.tmp/air-ground-20261115/preexisting.patch` and preserved in place.
**Architecture:** A pure task core owns command ordering and route revisions. One worker owns the existing move_base Action; a separate bounded ROS publisher publishes the latest state. Conversion and final arrival review reuse geographic.py/geographic_goal_core.py. No velocity publishing, localization/firmware/planner/safety changes.

## Scope and decisions

- Formal nonlatched `/air_ground/command` (`slicer_air_ground_msgs/Command`); latched, bounded `/air_ground/state` (`std_msgs/String`, documented JSON).
- Receiver boot ID and globally increasing sequence identify ordering, not control leases. A sender reads current state; request replay is idempotent. One serialized logical producer.
- Latitude/longitude are WGS84 degrees for horizontal base_link. Missing height uses the current task-origin ellipsoid height (2D Up=0); never antenna compensation a second time.
- Via radius defaults 0.25m and is checked against fresh map pose before move_base's existing 0.10m final position tolerance; cancel/terminal acknowledgment precedes the next segment. No via yaw alignment or geographic parking review.
- Existing externally-held Mux is never silently released. Existing readiness/release remains the field startup step. PAUSE/CANCEL acquire existing hold; only the identical instance/generation acquired by this service may be released on explicit RESUME or a subsequent runnable task.
- Active old Action is canceled and awaited; no new goal after cancellation timeout or LOST. Revision/token protects late results and final-review completion.

## Implementation and validation

- [x] Task 1: `air_ground_core.py`, pure regression tests: five operations, whole-batch validation, paused replacements, duplicate/conflicting/old requests, terminal task IDs and stale callbacks. Observe red before green.
- [x] Task 2: message package, `air_ground.py`/wire helper and real rospy example; file/schema, sequencing and official-topic client tests.
- [x] Task 3: `air_ground_executor.py`/`air_ground_node.py`, existing geographic conversion/review and control.py adapter; fake navigation backend tests plus isolated ROS Action/Mux fixtures. Prove cancel acknowledgment, position-only via, final three-way review, external-hold preservation, no subscriber dependency, restart behavior.
- [x] Task 4: build/install/launch/run.sh entry, minimal interface config, legacy communication conflict checks; Noetic build and software tests with network none/no devices. Fresh focused review before completion.
- [x] Task 5: only if online, snapshot own deployment, incremental deploy/ARM build/install/read-only version check. No assistant-started real driver/release/goal; physical motion requires user at vehicle. Offline remains useful when vehicle unavailable.
- [x] Task 6: `docs/air_ground_interface_v1.md` is the sole teammate entry. Verify examples/fields against code, package the minimal interface, record exact verification/deployment gaps and rollback.

## Progress

Local feature implemented and committed through 4a53b66; 429 Catkin checks pass serially with network none/no devices. Final 4a53b66 ARM64 build/install and isolated ROS lifecycle/restart fixture passed. LAN status/rejection round trip passed and all six running module hashes match local. Physical tests await the user; only Jetson has been powered. Existing dirty work preserved. Main document records exact gaps; no real driver, goal or release was sent by the assistant.
