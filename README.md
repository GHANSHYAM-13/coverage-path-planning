# my_coverage

`my_coverage` is a reusable ROS 2 package for cleaning-robot coverage using the
same `opennav_coverage` server pattern as `cleanbot`, split into smaller pieces
so it can be reused on another robot more easily.

Coverage planning is still done by `opennav_coverage` and executed through Nav2.
This package does not implement its own zigzag planner.

---

## What This Package Provides

- **`launch/localization.launch.py`**
  Starts `map_server` and `amcl` using the standard Nav2 localization bringup.

- **`launch/coverage_nav.launch.py`**
  Starts the coverage-following navigation stack in a composable container:
  `planner_server`, `controller_server`, `smoother_server`, `behavior_server`,
  `backported_bt_navigator`, `velocity_smoother`, `coverage_server`, and the
  Nav2 lifecycle manager.

- **`launch/coverage_gui.launch.py`**
  Starts a lightweight UI backend plus a standalone map window that loads
  `map.yaml` and the corresponding PGM so you can click the cleaning polygon
  directly on the map.

- **`launch/coverage_client.launch.py`**
  Starts a simple reusable client that can send a polygon through parameters.

- **`my_coverage/coverage_executor.py`**
  Command-line client for sending a polygon to the
  `navigate_complete_coverage` action.

- **`behavior_trees/coverage_bt.xml`**
  Coverage behavior tree used by the package.

---

## GUI Overview

The GUI (`polygon_drawer.py` + `gui_coverage.py`) provides an industrial
SCADA-style interface for selecting a cleaning area on the map and monitoring
the coverage task.

### Features

| Feature | Detail |
|---|---|
| **Map auto-fit** | Map image scales to fill the canvas (PIL/Lanczos if Pillow is installed, integer scale otherwise) |
| **Arbitrary polygons** | Draw 3 or more corners in any shape — convexity is not enforced |
| **Select / Undo / Clear** | Corner-by-corner polygon editing |
| **Send Polygon** | Publishes the closed polygon to `/user_selected_field` and starts coverage |
| **Map / floor selector** | Registers map YAML files and switches the GUI between saved floor maps |
| **Permanent saved sections** | Stores named sections per map under `~/.ros/my_coverage/maps/<map-id>/sections.yaml` |
| **Queued saved sections** | Select one or more saved sections and run them in order |
| **Cancel Cleaning** | Sends a cancel request to the active coverage action goal |
| **Robot indicator** | Live orange circle + heading arrow on the map, updated from `/amcl_pose` |
| **Light / Dark theme** | One-click toggle; all widgets re-styled at runtime |
| **Status indicator** | Colour-coded dot + text (Idle / Running / Succeeded / Cancelled / Failed) |

### Controls

| Button | Action |
|---|---|
| `Select Area` | Enables corner-placement mode — click the map to add corners |
| `Undo Corner` | Removes the most recently placed corner |
| `Clear Selection` | Removes all corners and clears the preview |
| `Send Polygon` | Closes and sends the polygon; coverage starts immediately |
| `Save Section` | Stores the selected polygon permanently for the currently selected map |
| `Clean Selected` | Sends selected saved section(s) to the coverage queue |
| `Delete` | Removes a saved section from the current map |
| `Cancel Cleaning` | Cancels the active coverage task (publishes to `/cancel_coverage`) |
| `☀ Light Mode` / `🌙 Dark Mode` | Toggles between the two UI themes |

### Map / Floor Section Storage

The GUI now treats each map YAML as a separate floor/map record. Saved sections
are permanent and scoped to that map, so a house with multiple floors can keep
different section names and polygons for each floor.

```text
~/.ros/my_coverage/
├── maps.yaml
└── maps/
    └── <map-id>/
        └── sections.yaml
```

- The map selector in the top bar lists registered map YAML files.
- `Add Map` registers and opens another map YAML, such as a second floor.
- When a map is selected, only the sections saved for that map are shown.
- If the old `~/.ros/coverage_sections.yaml` file exists, it is imported once
  into the first selected map that does not already have a new section file.

### ROS Topics Used by the GUI

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/user_selected_field` | `geometry_msgs/Polygon` | pub | Polygon drawn by the user |
| `/coverage_task_status` | `std_msgs/String` | sub | Status updates from backend |
| `/cancel_coverage` | `std_msgs/Empty` | pub | Cancel signal to backend |
| `/user_selected_field_marker` | `visualization_msgs/Marker` | pub | Preview overlay in RViz |
| `/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | sub | Live robot position on map |

### Optional dependency — Pillow

Install Pillow for smooth (Lanczos) map scaling:

```bash
pip install Pillow
```

Without Pillow the map still loads using integer zoom/subsample but quality
may be lower for maps that require a non-integer scale factor.

---

## Prerequisites

Before using `my_coverage`, your robot should already provide:

- TF tree for `map`, `odom`, and your base frame
- Odometry
- A valid scan topic for obstacle avoidance
- Motor control through `cmd_vel`

If another package already provides localization, skip
`localization.launch.py` and only launch the coverage navigation and GUI parts.

---

## Build

```bash
cd ~/cleanbot_ws
colcon build --packages-select my_coverage
source install/setup.bash
```

---

## Launch Flow

### 1. Localization

Use this if `my_coverage` should own `map_server` and `amcl`:

```bash
ros2 launch my_coverage localization.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

### 2. Coverage Navigation

Starts Nav2 coverage-following components and `coverage_server`:

```bash
ros2 launch my_coverage coverage_nav.launch.py use_sim_time:=True
```

### 3. GUI Area Selection

```bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

### 4. Set Initial Pose

If AMCL is running, set the robot's initial pose in RViz **before** starting a
coverage task. Once localized, the robot indicator will appear on the map
automatically.

---

## GUI Workflow

1. Launch `localization.launch.py` (if needed).
2. Launch `coverage_nav.launch.py`.
3. Launch `coverage_gui.launch.py` with the same `map:=...` path.
4. Set the robot pose in RViz — the orange robot indicator will appear on the map.
5. Click **Select Area** in the GUI.
6. Click the cleaning-area corners on the map in order (3 or more).
7. Click **Send Polygon** — coverage starts.
8. To stop early, click **Cancel Cleaning**.

---

## Command-Line Coverage Client

Send a polygon without the GUI:

```bash
ros2 run my_coverage coverage_executor --ros-args \
  -p frame_id:=map \
  -p polygon:="[0.0, 0.0, 4.0, 0.0, 4.0, 3.0, 0.0, 3.0]"
```

- Polygon format: flat list `[x1, y1, x2, y2, x3, y3, …]`
- Coordinates are in the `map` frame, in metres.
- The executor automatically closes the polygon by repeating the first point.

---

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
│   ├── gui_coverage.py       ← coverage action backend
│   └── polygon_drawer.py     ← GUI frontend
├── params/
│   ├── coverage_nav2_params.yaml
│   └── executor_params.yaml
└── README.md
```

---

## Reuse On Another Robot

To adapt this package to a different robot, update
`params/coverage_nav2_params.yaml`.  Key parameters to review:

- `robot_base_frame`
- `odom_frame`
- `global_frame`
- scan topic
- footprint or robot radius
- controller and velocity-limit tuning
- `robot_width`, `operation_width`, and turning radius for coverage planning

---

## Important Behavior Notes

- **Arbitrary polygons accepted** — convexity is not enforced. `opennav_coverage`
  handles the planning for any simple polygon shape.
- For complex concave rooms (e.g. L-shaped) it can be better to send multiple
  simpler polygons sequentially rather than one complex shape.
- Obstacle avoidance during execution is handled by Nav2 costmaps and the
  controller, not by a custom planner in this package.
- The robot indicator on the map is driven by `/amcl_pose`.  It will not appear
  until AMCL has localized the robot.

---

## Example Session

```bash
# Terminal 1 — localization
cd ~/cleanbot_ws && source install/setup.bash
ros2 launch my_coverage localization.launch.py \
  map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml \
  use_sim_time:=True
```

```bash
# Terminal 2 — coverage navigation
cd ~/cleanbot_ws && source install/setup.bash
ros2 launch my_coverage coverage_nav.launch.py use_sim_time:=True
```

```bash
# Terminal 3 — GUI
cd ~/cleanbot_ws && source install/setup.bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/home/jd/dev_ws/src/my_bot/map/Dock.yaml \
  use_sim_time:=True
```

Then in the GUI:

1. Set the robot initial pose in RViz.
2. Wait for the orange robot indicator to appear on the map.
3. Click **Select Area** and click the map corners.
4. Click **Send Polygon**.
5. Monitor status in the GUI; click **Cancel Cleaning** to stop early.

---

## Current Scope

`my_coverage` is intentionally focused on reusable coverage execution. It does
not try to replace the rest of your robot bringup, sensors, controller manager,
or visualisation stack.
