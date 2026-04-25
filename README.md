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
  Use this for a standalone coverage setup on a single robot.

- **`launch/coverage_nav.launch.py`**
  Starts the coverage-following navigation stack in a composable container:
  `planner_server`, `controller_server`, `smoother_server`, `behavior_server`,
  `backported_bt_navigator`, `velocity_smoother`, `coverage_server`, and the
  Nav2 lifecycle manager. Use this with `localization.launch.py` for the full stack.

- **`launch/coverage_bringup.launch.py`** *(Alternative)*
  Includes the standard `nav2_bringup bringup_launch.py` and adds only the
  coverage-specific components (`coverage_server`, `backported_bt_navigator`).
  **Use this if your robot is already using `nav2_bringup` successfully.**

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

## Installation & Dependencies

> **⚠️ Cross-System Note:** Different machines and ROS 2 distributions may have varying dependency availability. If `rosdep install` reports unresolvable packages like `nav2_ros_common`, this is normal. See the [Troubleshooting Installation](#troubleshooting-installation) section below for solutions. The recommended approach is to **build everything from source** using `colcon build`.

### System Requirements

- **ROS 2** (Humble or later recommended)
- **Python 3.8+**
- **colcon-core** build system

### Required ROS 2 Dependencies

When you clone this repository into your ROS 2 workspace, the following dependencies must be installed:

#### Core ROS 2 Packages
```bash
sudo apt-get install -y \
  ros-<distro>-nav2-bringup \
  ros-<distro>-nav2-map-server \
  ros-<distro>-nav2-amcl \
  ros-<distro>-nav2-controller \
  ros-<distro>-nav2-planner \
  ros-<distro>-nav2-behaviors \
  ros-<distro>-nav2-smoother \
  ros-<distro>-nav2-velocity-smoother \
  ros-<distro>-nav2-lifecycle-manager
```

#### OpenNav Coverage Stack (Required)

This is the **most critical dependency**. Clone these packages into your workspace:

```bash
cd ~/cleanbot_ws/src

# Clone the OpenNav coverage packages (Humble branch)
git clone --branch humble https://github.com/open-navigation/opennav_coverage.git

# This will provide:
# - opennav_coverage
# - opennav_row_coverage
# - opennav_coverage_msgs
# - opennav_coverage_bt
# - opennav_coverage_navigator
# - backported_bt_navigator
```

> **Note:** Branch specification (`--branch humble`) ensures compatibility with ROS 2 Humble. For other ROS 2 distributions, use the appropriate branch name (e.g., `iron`, `jazzy`, `rolling`).

Alternative (if using a workspace with opennav already):
```bash
sudo apt-get install -y ros-<distro>-opennav-coverage
```

> **Note:** The `opennav_coverage` package is essential for coverage path planning. Without it, the coverage navigation will not work.

#### Fields2Cover (Optional but Recommended)

For advanced coverage planning and field decomposition:

```bash
cd ~/cleanbot_ws/src
git clone https://github.com/Fields2Cover/Fields2Cover.git
```

Fields2Cover provides:
- Field boundary decomposition algorithms
- Optimal swath generation
- Headland management for field coverage
- Integration with opennav_coverage for custom planning strategies

> **Note:** Fields2Cover is optional for basic coverage tasks but highly recommended for agricultural and large-scale coverage applications.

#### Standard Message/Service Types
```bash
sudo apt-get install -y \
  ros-<distro>-geometry-msgs \
  ros-<distro>-action-msgs \
  ros-<distro>-lifecycle-msgs \
  ros-<distro>-nav2-msgs \
  ros-<distro>-std-msgs \
  ros-<distro>-visualization-msgs
```

### Optional Python Dependencies

To enable smooth map scaling in the GUI, install Pillow:

```bash
pip install Pillow
```

Without Pillow, the map still loads but uses integer-scale interpolation which may have lower quality for some map sizes.

If you want to use the GUI with better performance:

```bash
pip install Pillow PyQt5
```

### Complete Installation Procedure

1. **Clone the workspace (if not already done):**
   ```bash
   mkdir -p ~/cleanbot_ws/src
   cd ~/cleanbot_ws/src
   git clone <this-repo-url> my_coverage
   ```

2. **Clone OpenNav coverage packages (Humble branch):**
   ```bash
   cd ~/cleanbot_ws/src
   git clone --branch humble https://github.com/open-navigation/opennav_coverage.git
   ```

3. **Clone Fields2Cover (optional):**
   ```bash
   cd ~/cleanbot_ws/src
   git clone https://github.com/Fields2Cover/Fields2Cover.git
   ```
   > Skip this if you only need basic coverage. Include it for advanced planning features.

4. **Install system dependencies:**
   ```bash
   cd ~/cleanbot_ws
   rosdep install --from-paths src --ignore-src -r -y --continue-on-error
   ```
   > **Note:** You may see errors about `nav2_ros_common` or similar packages — these are expected and safe to ignore. The `--continue-on-error` flag allows rosdep to skip unresolvable dependencies. The packages will be built from source instead.

5. **Install Python dependencies:**
   ```bash
   pip install Pillow
   ```

6. **Build the packages:**
   ```bash
   cd ~/cleanbot_ws
   # Build opennav_coverage and my_coverage (optional: add Fields2Cover if cloned)
   colcon build --packages-select my_coverage opennav_coverage
   source install/setup.bash
   ```

   To include Fields2Cover in the build:
   ```bash
   colcon build --packages-select my_coverage opennav_coverage Fields2Cover
   source install/setup.bash
   ```

7. **Verify installation:**
   ```bash
   ros2 pkg list | grep my_coverage
   ros2 pkg list | grep opennav_coverage
   ros2 pkg list | grep Fields2Cover  # Optional, only if cloned
   ```

### Dependency Summary Table

| Package | Type | Source | Purpose |
|---------|------|--------|---------|
| **opennav_coverage** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Coverage path planning core engine |
| **opennav_row_coverage** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Row-based coverage planning |
| **opennav_coverage_msgs** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Message definitions for coverage |
| **opennav_coverage_bt** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Behavior tree definitions |
| **opennav_coverage_navigator** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Coverage action server |
| **backported_bt_navigator** | ROS 2 | GitHub (open-navigation/opennav_coverage) | Behavior tree navigation support |
| **nav2_bringup** | ROS 2 | apt | Nav2 launch utilities |
| **nav2_map_server** | ROS 2 | apt | Map file server |
| **nav2_amcl** | ROS 2 | apt | Adaptive Monte Carlo Localization |
| **nav2_controller** | ROS 2 | apt | Base controller |
| **nav2_planner** | ROS 2 | apt | Path planner |
| **nav2_behaviors** | ROS 2 | apt | Nav2 behavior plugins |
| **nav2_smoother** | ROS 2 | apt | Path smoothing |
| **nav2_velocity_smoother** | ROS 2 | apt | Velocity profile smoothing |
| **nav2_lifecycle_manager** | ROS 2 | apt | Lifecycle node management |
| **Fields2Cover** | C++ Library | GitHub (Fields2Cover/Fields2Cover) | Optional: Advanced field decomposition and swath planning |
| **Pillow** | Python | pip | Optional: image processing for GUI |

### Troubleshooting Installation

**Issue:** rosdep returns errors like `Cannot locate rosdep definition for [nav2_ros_common]` or `[nav2_minimal_tb3_sim]`

This is normal and expected on many systems. These packages don't have universal rosdep definitions.

```bash
# Solution: Use the --continue-on-error flag to skip unresolvable dependencies
rosdep install --from-paths src --ignore-src -r -y --continue-on-error

# Or if you're building from source (recommended for opennav_coverage packages):
# The build system will handle these dependencies automatically
cd ~/cleanbot_ws
colcon build --packages-select my_coverage
```

The error message "Continuing to install resolvable dependencies..." is normal — the rosdep install will succeed with the packages that *can* be resolved.

**Issue:** Package not found error for opennav_coverage
```bash
# Solution: Ensure you cloned the opennav_coverage repository
cd ~/cleanbot_ws/src
ls opennav_coverage/
# Should show subdirectories like opennav_coverage, opennav_row_coverage, opennav_coverage_navigator, etc.
```

**Issue:** colcon build fails with `Could not find a package configuration file` for nav2_ros_common

```bash
# Solution: This package doesn't have a pre-built rosdep. Build from source instead.
# It's likely already in your opennav_coverage clone, so just build:
cd ~/cleanbot_ws
colcon build --packages-select opennav_coverage my_coverage
source install/setup.bash
```

If you still get errors, try building with more verbose output:
```bash
colcon build --packages-select opennav_coverage my_coverage --event-handlers console_cohesion+
```

**Issue:** GUI loads but map scaling is poor quality
```bash
# Solution: Install Pillow for better image interpolation
pip install Pillow
```

**Issue:** `Could not find a package configuration file` for backported_bt_navigator

```bash
# Solution: This is provided by opennav_coverage repository. Ensure you cloned it:
cd ~/cleanbot_ws/src
git clone https://github.com/open-navigation/opennav_coverage.git
cd ~/cleanbot_ws
colcon build
```

**Issue:** ROS 2 package resolution fails across systems

Different ROS 2 distributions (Humble, Iron, Jazzy) may have varying dependency availability. If your system is notably different:

```bash
# Check your ROS 2 distribution
echo $ROS_DISTRO

# Clone opennav_coverage with the correct branch for your distribution:
cd ~/cleanbot_ws/src
git clone --branch $ROS_DISTRO https://github.com/open-navigation/opennav_coverage.git

# Available branches: humble, iron, jazzy, rolling, master
# For older/newer distributions, you may need to build from source
# or use Docker for consistency. See the opennav_coverage repository for version compatibility.
```

**Issue:** `fatal: Remote branch <distro> not found` when cloning opennav_coverage

```bash
# Solution: The branch you specified doesn't exist. Check available branches:
cd ~/cleanbot_ws/src
git ls-remote --heads https://github.com/open-navigation/opennav_coverage.git | grep -oE 'refs/heads/\K[^ ]+'

# Clone with the appropriate branch (e.g., humble, iron, jazzy):
git clone --branch humble https://github.com/open-navigation/opennav_coverage.git
```

**Issue:** Fields2Cover build fails with CMake errors

```bash
# Solution: Fields2Cover requires additional system dependencies (GDAL, boost, etc.)
# Install all build dependencies and rebuild:
sudo apt-get install -y gdal-bin libgdal-dev libboost-all-dev cmake build-essential
cd ~/cleanbot_ws
colcon build --packages-select Fields2Cover
```

If you don't need Fields2Cover's advanced features, you can skip it entirely for basic coverage tasks.

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

Before using `my_coverage`, ensure:

1. **Installation complete:** Follow the [Installation & Dependencies](#installation--dependencies) section above
2. **ROS 2 environment sourced:** `source ~/cleanbot_ws/install/setup.bash`
3. **Robot hardware provides:**
   - TF tree for `map`, `odom`, and your base frame
   - Odometry topic (typically `/odom`)
   - A valid scan topic for obstacle avoidance (typically `/scan`)
   - Motor control through `cmd_vel` topic

If another package already provides localization (e.g., from your robot's main bringup), you can skip `localization.launch.py` and only launch the coverage navigation and GUI components.

---

## Build

Once all dependencies are installed (see [Installation & Dependencies](#installation--dependencies)), build the package:

```bash
cd ~/cleanbot_ws
colcon build --packages-select my_coverage
source install/setup.bash
```

To rebuild with fresh configuration:

```bash
cd ~/cleanbot_ws
colcon build --packages-select my_coverage --cmake-clean-cache
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

## Alternative Launch Flow — Using Standard Nav2 Bringup

If your robot is already using `nav2_bringup bringup_launch.py` successfully,
use this simpler approach that adds only the coverage components:

### 1. Launch Coverage with Bringup

This single command includes the standard Nav2 bringup **plus** coverage-specific nodes:

```bash
ros2 launch my_coverage coverage_bringup.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

This includes:
- Map server
- AMCL localization
- Nav2 navigation stack (planner, controller, smoother)
- Coverage server
- Behavior tree navigator for coverage
- GUI (optional, launch separately if needed)

### 2. GUI Area Selection (Optional)

```bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

### 3. Alternatively, Use Your Robot's Existing Launch

If your robot already has a custom launch file that works well, you can just run:

```bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/absolute/path/to/map.yaml \
  use_sim_time:=True
```

This assumes your robot's navigation stack is already running (map server, localization, navigation).

---

## GUI Workflow

### Workflow A: Using my_coverage's full stack (with localization)

1. Launch `localization.launch.py` (if needed).
2. Launch `coverage_nav.launch.py`.
3. Launch `coverage_gui.launch.py` with the same `map:=...` path.
4. Set the robot pose in RViz — the orange robot indicator will appear on the map.
5. Click **Select Area** in the GUI.
6. Click the cleaning-area corners on the map in order (3 or more).
7. Click **Send Polygon** — coverage starts.
8. To stop early, click **Cancel Cleaning**.

### Workflow B: Using standard Nav2 bringup (recommended for robots with existing nav2_bringup)

1. Launch `coverage_bringup.launch.py` (includes map server, localization, and coverage).
2. Launch `coverage_gui.launch.py` with the same `map:=...` path (or use your robot's RViz).
3. Set the robot pose in RViz — the orange robot indicator will appear on the map.
4. Click **Select Area** in the GUI.
5. Click the cleaning-area corners on the map in order (3+ corners).
6. Click **Send Polygon** — coverage starts automatically.
7. To stop early, click **Cancel Cleaning**.

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

### Approach A: Using my_coverage Full Stack

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

### Approach B: Using Standard Nav2 Bringup (Recommended for robots with existing nav2_bringup)

```bash
# Terminal 1 — coverage with bringup (all-in-one)
cd ~/cleanbot_ws && source install/setup.bash
ros2 launch my_coverage coverage_bringup.launch.py \
  map:=/home/svr/ROBOMUSE_WS/testing.yaml \
  use_sim_time:=True
```

```bash
# Terminal 2 — GUI (optional)
cd ~/cleanbot_ws && source install/setup.bash
ros2 launch my_coverage coverage_gui.launch.py \
  map:=/home/svr/ROBOMUSE_WS/testing.yaml \
  use_sim_time:=True
```

Then in the GUI or RViz:

1. Set the robot initial pose in RViz.
2. Wait for the orange robot indicator to appear on the map.
3. Click **Select Area** and click the map corners.
4. Click **Send Polygon** to start coverage.
5. Monitor in the GUI; click **Cancel Cleaning** to stop early.

**Advantages of Approach B:**
- Single launch command instead of three separate ones
- Reuses your robot's existing Nav2 configuration
- Simpler for robots where `nav2_bringup` is already working

---

## Current Scope

`my_coverage` is intentionally focused on reusable coverage execution. It does
not try to replace the rest of your robot bringup, sensors, controller manager,
or visualisation stack.
