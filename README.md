# my_coverage

`my_coverage` is a reusable ROS 2 package for cleaning-robot coverage using the same `opennav_coverage` server pattern as `cleanbot`, but split into smaller pieces so it can be reused on another robot more easily.

This package does not implement its own zigzag planner. Coverage planning is still done by `opennav_coverage` and executed through Nav2.

## What This Package Provides

- `launch/localization.launch.py`
  Starts `map_server` and `amcl` using the standard Nav2 localization bringup.

- `launch/coverage_nav.launch.py`
  Starts the coverage-following navigation stack in a composable container:
  `planner_server`, `controller_server`, `smoother_server`, `behavior_server`,
  `backported_bt_navigator`, `velocity_smoother`, `coverage_server`, and the
  Nav2 lifecycle manager.

- `launch/coverage_gui.launch.py`
  Starts a lightweight UI backend plus a standalone map window that loads
  `map.yaml` and `map.pgm` so you can click the cleaning polygon directly on
  the map.

- `launch/coverage_client.launch.py`
  Starts a simple reusable client that can send a polygon through parameters.

- `my_coverage/coverage_executor.py`
  Command-line client for sending a polygon to the
  `navigate_complete_coverage` action.

- `behavior_trees/coverage_bt.xml`
  Coverage behavior tree used by the package.

## Typical Use Cases

- Reuse `coverage_server` on another robot without copying the entire
  `cleanbot` package.
- Run localization separately from coverage navigation.
- Launch a simple coverage UI without depending on a big all-in-one launch.
- Trigger coverage either from a GUI or from a command line action client.

## Prerequisites

Before using `my_coverage`, your robot should already provide:

- TF tree for `map`, `odom`, and your base frame
- odometry
- a valid scan topic for obstacle avoidance
- motor control through `cmd_vel`

If another package already provides localization, you can skip
`localization.launch.py` and only use the coverage navigation and UI parts.

## Build

```bash
cd ~/cleanbot_ws
colcon build --packages-select my_coverage
source install/setup.bash
```

## Launch Flow

### 1. Localization

Use this if `my_coverage` should own `map_server` and `amcl`:

```bash
ros2 launch my_coverage localization.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

### 2. Coverage Navigation

This starts Nav2 coverage-following components and `coverage_server`:

```bash
ros2 launch my_coverage coverage_nav.launch.py use_sim_time:=True
```

### 3. GUI Area Selection

This starts the standalone map-based selection UI:

```bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

### 4. Set Initial Pose

If AMCL is running, set the robot initial pose in RViz before starting a
coverage task.

## GUI Workflow

The GUI window loads the map image from the `map.yaml` file and lets you define
the cleaning area directly on the map.

Use it like this:

1. Launch `localization.launch.py` if needed.
2. Launch `coverage_nav.launch.py`.
3. Launch `coverage_gui.launch.py` with the same `map:=...` path.
4. In the map window, click `Select Area`.
5. Click the cleaning-area corners on the map in order.
6. Click `Send Polygon`.

Available controls:

- `Select Area`
  Enables corner selection mode.

- `Undo Corner`
  Removes the most recently selected corner.

- `Clear`
  Clears the current polygon.

- `Send Polygon`
  Closes the polygon automatically and sends it to `coverage_server`.

## Command-Line Coverage Client

You can also send a polygon without the GUI:

```bash
ros2 run my_coverage coverage_executor --ros-args \
  -p frame_id:=map \
  -p polygon:="[0.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 3.0]"
```

Notes:

- The polygon format is a flat list:
  `[x1, y1, x2, y2, x3, y3, ...]`
- Coordinates are in the `map` frame, in meters.
- The executor automatically closes the polygon by repeating the first point.

## Package Layout

```text
my_coverage/
├── behavior_trees/
│   └── coverage_bt.xml
├── launch/
│   ├── coverage_client.launch.py
│   ├── coverage_gui.launch.py
│   ├── coverage_nav.launch.py
│   └── localization.launch.py
├── my_coverage/
│   ├── coverage_executor.py
│   ├── gui_coverage.py
│   └── polygon_drawer.py
├── params/
│   ├── coverage_nav2_params.yaml
│   └── executor_params.yaml
└── README.md
```

## Reuse On Another Robot

To adapt this package to a different robot, update:

- `params/coverage_nav2_params.yaml`

The most important items to check are:

- `robot_base_frame`
- `odom_frame`
- `global_frame`
- scan topic
- footprint or robot radius
- controller tuning
- velocity limits
- `robot_width`, `operation_width`, and turning radius for coverage

If your robot publishes different topics or uses different frames, change them
in the params file instead of modifying the coverage code.

## Important Behavior Notes

- This package follows the same coverage-server approach as `cleanbot`.
- Obstacle avoidance during execution is handled by Nav2 costmaps and the
  controller, not by a custom planner in this package.
- The selected polygon should be clicked in boundary order.
- For complex concave rooms such as `L`-shaped areas, it is often better to
  send multiple simpler polygons instead of one complex shape.

## Example Session

```bash
cd ~/cleanbot_ws
source install/setup.bash

ros2 launch my_coverage localization.launch.py \
  map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml \
  use_sim_time:=True
```

Open a second terminal:

```bash
cd ~/cleanbot_ws
source install/setup.bash

ros2 launch my_coverage coverage_nav.launch.py use_sim_time:=True
```

Open a third terminal:

```bash
cd ~/cleanbot_ws
source install/setup.bash

ros2 launch my_coverage coverage_gui.launch.py \
  map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml \
  use_sim_time:=True
```

Then:

1. Set the robot initial pose in RViz.
2. Click `Select Area` in the GUI.
3. Click the map corners.
4. Click `Send Polygon`.

## Current Scope

`my_coverage` is intentionally focused on reusable coverage execution. It does
not try to replace the rest of your robot bringup, sensors, controller manager,
or visualization stack.
