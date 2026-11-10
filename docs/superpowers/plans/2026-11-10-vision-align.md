# Workspace2 visual alignment implementation plan

> **For agentic workers:** Implement the bounded tasks inline or in parallel with distinct files. The user has approved starting local work; no further design approval is needed.

**Goal:** Select a visible point or edge midpoint and align it with adjustable image xy through the existing Guard raw interface.

**Architecture:** One OpenCV tracker/controller module and one small ROS/image viewer. Observe by default. A measured local 2x2 image Jacobian is required for control; do not guess camera geometry or derive distance from the temporary block. Preserve existing chassis and safety control. Separate the planner raw output during vision experiments while retaining move_base costmaps.

**Tech Stack:** Python 3, NumPy, OpenCV LK, ROS1 Image/cv_bridge/Twist, unittest.

**Spec:** `docs/reviews/20261110-workspace2/REVIEW_AND_PLAN.md` in the retained old workspace; user's follow-up approves implementation in the copied new workspace.

## Constraints

- Local development only. Remote teammate sources and archived projects remain unchanged.
- No chassis port, firmware, driver, release, goals or actual motion.
- No fixed object color, class or 10x7x5cm geometry. Servos do not participate.
- No controller output to `/cmd_vel`; only existing `/cmd_vel/nav_raw`.
- Existing speed bounds come from deployed control configuration; no hidden extra speed cap.

## Review focus

- A point leaving the image or failed LK identity must become invalid, not switch to background.
- Target changes must affect the displayed error and control without resetting the selected feature.
- Invalid/stale images, stop and forbidden reverse must yield zero, not silently clipped success.
- Image Jacobian must have documented pixel/metre and pixel/radian units and be nonsingular.
- Default launch behavior must remain unchanged; vision remapping must keep planner output off Guard raw.

## Task 1: Track and compute commands

Files: `Test/Vision_adjust/vision_core.py`, `Test/Vision_adjust/tests/test_vision_core.py`.

- [x] Write/run failing tests: translated synthetic feature, edge midpoint, lost feature, invalid selection, target update; control sign/coupling, singular Jacobian, reverse rejection, uniform speed scaling, reached zero.
- [x] Implement `PointTracker.select(frame, points)`, `PointTracker.update(frame)` and `servo_command(feature, target, jacobian, gain, max_linear, max_angular, tolerance_px)`; expose valid/feature/reason and vx/wz/status without ROS.
- [x] Run targeted tests without hardware; retain actual results.

## Task 2: Viewer, ROS adapter, minimal launch remap

Files: `vision_align.py`, `config.yaml`, `README.md`, one launch argument and targeted tests.

- [x] Implement image/video observation and ROS Image input, mouse-selected point or two edge endpoints, xy sliders, visible tracking status, opt-in control and explicit stop.
- [x] Require measured Jacobian, fresh frame and isolated planner output for control. Publish zero on loss/end; never release Mux or open serial.
- [x] Add optional planner topic remap preserving old defaults; document exact sensor/control entry and existing configuration bounds.
- [x] Run offline core/app tests, syntax and launch checks. If Docker unavailable, report that ROS integration was not run.

## Task 3: Handoff

- [x] Verify source import identity and archive restore checks; add current-workspace pointer and short field steps.
- [x] Commit only local candidate changes. Report software results separately from camera/vehicle validation.
