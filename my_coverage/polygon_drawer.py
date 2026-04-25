"""
polygon_drawer.py  –  Coverage Area Selector GUI
my_coverage | ROS 2

Features
--------
* Auto-fits map image to available canvas area (PIL-based or integer scaling)
* Cancel Cleaning button — publishes Empty to /cancel_coverage
* Light / Dark theme toggle (industrial SCADA palette for both)
* Full theme system: every widget re-stylable at runtime

Themes
------
Dark  : charcoal #2B2B2B panels, #1A1A1A canvas, #0078D4 accent
Light : chrome  #ECECEC panels, #FFFFFF canvas, #0078D4 accent
"""

import math
import os
import queue
import threading
import tkinter as tk
from tkinter import font as tkfont
from tkinter import messagebox, simpledialog

# Optional PIL for smooth image scaling (pip install Pillow)
try:
    from PIL import Image as _PILImage, ImageTk as _ImageTk
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

from geometry_msgs.msg import Point, Point32, Polygon, PoseWithCovarianceStamped
from nav_msgs.msg import Path
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Empty, String, Float32
from visualization_msgs.msg import Marker
import yaml


# ══════════════════════════════════════════════════════════════════════════════
#  Colour themes
# ══════════════════════════════════════════════════════════════════════════════

THEMES = {
    "dark": {
        # structural
        "bg":           "#2B2B2B",   # window background
        "panel":        "#383838",   # sidebar / toolbars
        "panel_lt":     "#464646",   # slightly lighter panel (buttons)
        "canvas_bg":    "#1A1A1A",   # map canvas
        "border":       "#555555",   # separator lines
        "border_lt":    "#686868",   # canvas border / highlight
        # accent (Microsoft / Siemens industrial blue)
        "accent":       "#0078D4",
        "accent_hov":   "#106EBE",
        "accent_dim":   "#1B4F80",
        # semantic status
        "success":      "#107C10",
        "warning":      "#CA5100",
        "error":        "#C42B1C",
        "cancel_bg":    "#6E1515",   # cancel button bg
        "cancel_hov":   "#8A1C1C",
        "cancel_fg":    "#FFCCCC",
        # text
        "text":         "#F3F3F3",
        "text2":        "#ABABAB",
        "text3":        "#767676",
        # drawing
        "vertex":       "#E8A000",
        "poly":         "#0078D4",
        "poly_fill":    "#264F78",
        "poly_stipple": "gray25",
        # listbox
        "lb_bg":        "#1E1E1E",
        "lb_alt":       "#252525",
        "lb_sel_bg":    "#264F78",
        "lb_sel_fg":    "#FFFFFF",
        # ghost / danger buttons
        "ghost_bg":     "#464646",
        "ghost_abg":    "#525252",
        "ghost_fg":     "#ABABAB",
        "danger_bg":    "#383838",
        "danger_abg":   "#502020",
        "danger_fg":    "#E04040",
        # theme toggle label
        "theme_icon":   "☀  Light Mode",
        # robot indicator
        "robot_body":   "#E05A00",   # burnt orange — stands out on dark canvas
        "robot_outline": "#FF8C40",
        "robot_arrow":  "#FFAA60",
        "robot_label":  "#FFD0A0",
    },
    "light": {
        "bg":           "#ECECEC",
        "panel":        "#D9D9D9",
        "panel_lt":     "#E8E8E8",
        "canvas_bg":    "#FFFFFF",
        "border":       "#BBBBBB",
        "border_lt":    "#999999",
        "accent":       "#0078D4",
        "accent_hov":   "#106EBE",
        "accent_dim":   "#CCE4F7",
        "success":      "#107C10",
        "warning":      "#CA5100",
        "error":        "#C42B1C",
        "cancel_bg":    "#F0DCDC",
        "cancel_hov":   "#E8C8C8",
        "cancel_fg":    "#8B0000",
        "text":         "#1A1A1A",
        "text2":        "#444444",
        "text3":        "#666666",
        "vertex":       "#B06000",
        "poly":         "#0078D4",
        "poly_fill":    "#BDD7EE",
        "poly_stipple": "gray50",
        "lb_bg":        "#FFFFFF",
        "lb_alt":       "#F0F4F8",
        "lb_sel_bg":    "#0078D4",
        "lb_sel_fg":    "#FFFFFF",
        "ghost_bg":     "#CCCCCC",
        "ghost_abg":    "#BBBBBB",
        "ghost_fg":     "#444444",
        "danger_bg":    "#F0E8E8",
        "danger_abg":   "#E0D0D0",
        "danger_fg":    "#C42B1C",
        "theme_icon":   "🌙  Dark Mode",
        # robot indicator
        "robot_body":   "#0050A0",   # dark blue — readable on white canvas
        "robot_outline": "#0078D4",
        "robot_arrow":  "#0063B1",
        "robot_label":  "#003D7A",
    },
}


# ══════════════════════════════════════════════════════════════════════════════
#  ROS 2 node
# ══════════════════════════════════════════════════════════════════════════════

class PolygonDrawer(Node):

    def __init__(self):
        super().__init__("polygon_drawer")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("map", "")

        self.frame_id = self.get_parameter("frame_id").value
        self.map_yaml_path = self.get_parameter("map").value
        if not self.map_yaml_path:
            raise RuntimeError('Parameter "map" must point to a map yaml file')

        self.map_image_path, self.resolution, self.origin = \
            self._load_map_metadata(self.map_yaml_path)

        # Set by main() after the image is loaded/scaled
        self.image_width  = 0   # raw pixel width
        self.image_height = 0   # raw pixel height
        self.display_scale = 1.0   # display_px / raw_px

        self.lock = threading.Lock()
        self.ui_queue = queue.Queue()
        self.canvas_points = []   # display-space (scaled) pixel coords
        self.selection_mode = False
        self.preview_shape_id = None
        self.preview_vertex_ids = []

        # Robot pose state (set from /amcl_pose)
        self.robot_pose = None          # (map_x, map_y, yaw_rad) or None
        self.robot_canvas_ids = []      # canvas item IDs for the robot indicator
        
        # Coverage tracking state
        self.coverage_percent = 0.0     # Current coverage percentage (0-100)
        self.coverage_feedback = ""     # Detailed feedback info from coverage server
        self.task_status = "Idle"       # Current task status
        
        # Swath visualization state
        self.swaths = []                # List of swaths lines (each: [(x1,y1), (x2,y2)])
        self.robot_position = None      # Robot's current position (x, y)
        self.swath_canvas_ids = []      # Canvas item IDs for rendered swaths

        # ROS subscribers / publishers
        self.status_sub = self.create_subscription(
            String, "/coverage_task_status", self._status_cb, 10
        )
        # Coverage percentage from gui_coverage backend
        self.coverage_percent_sub = self.create_subscription(
            Float32, "/coverage_percent", self._coverage_percent_cb, 10
        )
        # Detailed coverage feedback
        self.coverage_feedback_sub = self.create_subscription(
            String, "/coverage_feedback_info", self._coverage_feedback_cb, 10
        )
        # Swaths from coverage server
        self.swaths_sub = self.create_subscription(
            Marker, "/coverage_server/swaths", self._swaths_cb, 10
        )
        # Robot pose — AMCL publishes here after localisation
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._pose_cb, 10
        )
        self.polygon_pub = self.create_publisher(
            Polygon, "/user_selected_field", 10
        )
        self.cancel_pub = self.create_publisher(
            Empty, "/cancel_coverage", 10
        )
        self.marker_pub = self.create_publisher(
            Marker, "/user_selected_field_marker", 10
        )
        
        # Section management
        self.sections = {}  # dict: {section_name: [[x1, y1], [x2, y2], ...]}
        self.sections_file = os.path.expanduser("~/.ros/coverage_sections.yaml")
        self.load_sections()

    # ── section management ─────────────────────────────────────────────────── #
    
    def get_sections_file(self):
        """Get path to sections storage file."""
        os.makedirs(os.path.dirname(self.sections_file), exist_ok=True)
        return self.sections_file
    
    def load_sections(self):
        """Load saved sections from YAML file."""
        sections_file = self.get_sections_file()
        if os.path.exists(sections_file):
            try:
                with open(sections_file, 'r') as f:
                    data = yaml.safe_load(f)
                    self.sections = data.get('sections', {}) if data else {}
                    self.get_logger().info(f"Loaded {len(self.sections)} saved sections")
            except Exception as e:
                self.get_logger().error(f"Failed to load sections: {e}")
                self.sections = {}
        else:
            self.sections = {}
    
    def save_section(self, section_name, field):
        """Save a polygon as a named section."""
        # Ensure polygon is closed
        if len(field) >= 2 and field[0] != field[-1]:
            field = list(field) + [field[0]]
        
        self.sections[section_name] = field
        self._persist_sections()
        self.get_logger().info(f"Saved section: {section_name}")
        return section_name
    
    def delete_section(self, section_name):
        """Delete a saved section."""
        if section_name in self.sections:
            del self.sections[section_name]
            self._persist_sections()
            self.get_logger().info(f"Deleted section: {section_name}")
            return True
        return False
    
    def get_section(self, section_name):
        """Get a saved section by name."""
        return self.sections.get(section_name)
    
    def _persist_sections(self):
        """Write sections to YAML file."""
        try:
            sections_file = self.get_sections_file()
            data = {'sections': self.sections}
            with open(sections_file, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
        except Exception as e:
            self.get_logger().error(f"Failed to save sections: {e}")

    # ── map YAML parser ───────────────────────────────────────────────────── #

    def _load_map_metadata(self, yaml_path):
        img = res = ori = None
        with open(yaml_path, "r", encoding="utf-8") as fh:
            for raw in fh:
                line = raw.split("#", 1)[0].strip()
                if not line or ":" not in line:
                    continue
                k, v = line.split(":", 1)
                k, v = k.strip(), v.strip()
                if k == "image":
                    img = v.strip().strip('"').strip("'")
                elif k == "resolution":
                    res = float(v)
                elif k == "origin":
                    ori = [float(s.strip()) for s in v.strip()[1:-1].split(",")]

        if None in (img, res, ori):
            raise RuntimeError(f"Cannot parse map metadata from {yaml_path}")
        if not os.path.isabs(img):
            img = os.path.join(os.path.dirname(yaml_path), img)
        if not os.path.exists(img):
            raise RuntimeError(f"Map image not found: {img}")

        return img, res, ori

    # ── ROS callbacks ─────────────────────────────────────────────────────── #

    def _status_cb(self, msg):
        self.task_status = msg.data
        self.ui_queue.put(("ros_status", msg.data))
    
    def _coverage_percent_cb(self, msg):
        """Update coverage percentage from gui_coverage backend."""
        self.coverage_percent = msg.data
        self.ui_queue.put(("coverage_percent", msg.data))
    
    def _coverage_feedback_cb(self, msg):
        """Update coverage feedback info from gui_coverage backend."""
        self.coverage_feedback = msg.data
        self.ui_queue.put(("coverage_feedback", msg.data))
    
    def _swaths_cb(self, msg):
        """Process swaths marker from coverage server."""
        if msg.action == Marker.DELETE:
            self.swaths = []
            self.ui_queue.put(("clear_swaths", None))
            return
        
        if msg.type != Marker.LINE_LIST:
            return
        
        # Convert marker points to swath lines
        # LINE_LIST has pairs of points (from, to)
        swaths = []
        for i in range(0, len(msg.points) - 1, 2):
            p1 = msg.points[i]
            p2 = msg.points[i + 1]
            swaths.append([(p1.x, p1.y), (p2.x, p2.y)])
        
        self.swaths = swaths
        self.ui_queue.put(("swaths_update", len(swaths)))

    # ── thread-safe point store ───────────────────────────────────────────── #

    def get_pts(self):
        with self.lock:
            return list(self.canvas_points)

    def set_pts(self, pts):
        with self.lock:
            self.canvas_points = list(pts)

    # ── UI actions ────────────────────────────────────────────────────────── #

    def clear_points(self):
        self.set_pts([])
        self._pub_preview_marker(clear=True)
        self.ui_queue.put(("refresh", None))
        self.ui_queue.put(("local_status", ("idle", "Selection cleared")))

    def undo_last(self):
        pts = self.get_pts()
        if not pts:
            return
        pts.pop()
        self.set_pts(pts)
        self._pub_preview_marker(clear=not pts)
        self.ui_queue.put(("refresh", None))
        n = len(pts)
        self.ui_queue.put((
            "local_status",
            ("info", f"Corner removed — {n} corner{'s' if n != 1 else ''} remaining"),
        ))

    def start_selection_mode(self):
        self.selection_mode = True
        self.ui_queue.put((
            "local_status",
            ("info", "Selection mode — click map corners in order"),
        ))

    def add_corner(self, event):
        if not self.selection_mode:
            return
        x = event.widget.canvasx(event.x)
        y = event.widget.canvasy(event.y)
        pts = self.get_pts()
        pts.append((x, y))
        self.set_pts(pts)
        self._pub_preview_marker()
        self.ui_queue.put(("refresh", None))
        self.ui_queue.put((
            "local_status",
            ("info", f"Corner {len(pts)} placed — add more or Send Polygon"),
        ))

    def send_polygon(self):
        pts = self.get_pts()
        if len(pts) < 3:
            self.ui_queue.put((
                "popup",
                "Select at least 3 corners on the map before sending.",
            ))
            return

        map_pts = [self._c2m(p) for p in pts]
        closed = list(map_pts)
        if closed[0] != closed[-1]:
            closed.append(closed[0])

        msg = Polygon()
        for x, y in closed:
            pt = Point32()
            pt.x, pt.y = float(x), float(y)
            msg.points.append(pt)

        self.polygon_pub.publish(msg)
        self._pub_preview_marker()
        self.ui_queue.put((
            "local_status",
            ("success", f"Polygon ({len(pts)} corners) sent — coverage starting"),
        ))

    def cancel_coverage(self):
        self.cancel_pub.publish(Empty())
        self.ui_queue.put((
            "local_status",
            ("warning", "Cancel request sent to coverage server"),
        ))

    def save_current_area(self, section_name):
        """Save current drawn area as a named section."""
        if not section_name or not section_name.strip():
            self.ui_queue.put((
                "popup",
                "Enter a valid section name (e.g., 'Kitchen', 'Living Room')",
            ))
            return False
        
        pts = self.get_pts()
        if len(pts) < 3:
            self.ui_queue.put((
                "popup",
                "Select at least 3 corners before saving.",
            ))
            return False
        
        map_pts = [self._c2m(p) for p in pts]
        self.save_section(section_name, map_pts)
        self.ui_queue.put((
            "local_status",
            ("success", f"Section '{section_name}' saved successfully"),
        ))
        self.ui_queue.put(("sections_updated", None))
        return True

    # ── coordinate conversion ─────────────────────────────────────────────── #

    def _c2m(self, point):
        """Convert display-canvas pixel → ROS map frame (metres)."""
        cx, cy = point
        # Step 1: undo display scaling to get raw image pixel
        rx = float(cx) / self.display_scale
        ry = float(cy) / self.display_scale
        # Step 2: image pixel → metric (y-axis is flipped)
        mx = self.origin[0] + rx * self.resolution
        my = self.origin[1] + (self.image_height - ry) * self.resolution
        return (mx, my)

    # kept as public alias used by the refresh / listbox code
    def canvas_to_map(self, point):
        return self._c2m(point)

    def map_to_canvas(self, mx, my):
        """Convert ROS map frame (metres) → display-canvas pixel.

        Inverse of _c2m().  Used to position the robot indicator.
        """
        # Step 1: metric → raw image pixel (y-axis flipped: ROS +y = image top)
        rx = (mx - self.origin[0]) / self.resolution
        ry = self.image_height - (my - self.origin[1]) / self.resolution
        # Step 2: apply display scaling
        cx = rx * self.display_scale
        cy = ry * self.display_scale
        return (cx, cy)

    def _pose_cb(self, msg):
        """AMCL pose callback — extract position + yaw and queue a canvas update."""
        p = msg.pose.pose
        x = p.position.x
        y = p.position.y
        # Quaternion → yaw (rotation around Z)
        q = p.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        self.robot_pose = (x, y, yaw)
        self.robot_position = (x, y)  # Store position for swath coverage calculation
        self.ui_queue.put(("robot_pose", (x, y, yaw)))

    # ── RViz preview marker ───────────────────────────────────────────────── #

    def _pub_preview_marker(self, clear=False):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id = "selected_polygon", 1
        m.type = Marker.LINE_STRIP
        m.action = Marker.DELETE if clear else Marker.ADD
        m.scale.x = 0.05
        m.color.r, m.color.g, m.color.b = 0.0, 0.47, 0.84
        m.color.a = 1.0
        if not clear:
            raw = self.get_pts()
            if raw:
                mp = [self._c2m(p) for p in raw]
                if mp[0] != mp[-1]:
                    mp.append(mp[0])
                for x, y in mp:
                    pt = Point()
                    pt.x, pt.y, pt.z = float(x), float(y), 0.0
                    m.points.append(pt)
        self.marker_pub.publish(m)


# ══════════════════════════════════════════════════════════════════════════════
#  Theming helpers
# ══════════════════════════════════════════════════════════════════════════════

class ThemeManager:
    """
    Lightweight runtime theming for Tkinter.

    Usage:
        tm = ThemeManager("dark")
        # Register a widget with which theme keys map to which tk options:
        tm.reg(my_label, bg="panel", fg="text")
        # Later:
        tm.switch("light")   # reconfigures all registered widgets
    """

    def __init__(self, initial: str):
        self._name = initial
        self._hooks: list = []   # list of callables

    @property
    def T(self) -> dict:
        return THEMES[self._name]

    @property
    def name(self) -> str:
        return self._name

    def reg(self, widget, **theme_map):
        """
        Register widget.  theme_map = {tk_option: theme_key}.
        Returns the widget unchanged (useful for chaining).
        """
        def _apply(T):
            try:
                widget.config(**{k: T[v] for k, v in theme_map.items()})
            except Exception:
                pass
        self._hooks.append(_apply)
        return widget

    def reg_canvas_item(self, canvas, item_id, **theme_map):
        """Register a canvas item (oval, rectangle, …)."""
        def _apply(T):
            try:
                canvas.itemconfig(item_id, **{k: T[v] for k, v in theme_map.items()})
            except Exception:
                pass
        self._hooks.append(_apply)

    def hook(self, fn):
        """Register an arbitrary zero-arg callable."""
        self._hooks.append(lambda T, _fn=fn: _fn(T))

    def switch(self, name: str):
        self._name = name
        T = self.T
        for fn in self._hooks:
            fn(T)


# ══════════════════════════════════════════════════════════════════════════════
#  main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    rclpy.init()
    node = PolygonDrawer()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # ── window ────────────────────────────────────────────────────────────── #
    root = tk.Tk()
    root.title("Coverage Area Selector  —  my_coverage")
    root.geometry("1200x760")
    root.minsize(920, 640)

    # ── fonts ─────────────────────────────────────────────────────────────── #
    installed = tkfont.families()

    def pick(*choices):
        for f in choices:
            if f in installed:
                return f
        return "TkDefaultFont"

    sans = pick("Segoe UI", "Ubuntu", "DejaVu Sans", "Helvetica")
    mono = pick("Consolas", "Ubuntu Mono", "DejaVu Sans Mono", "Courier")

    F_TITLE  = tkfont.Font(family=sans, size=11, weight="bold")
    F_HDR    = tkfont.Font(family=sans, size=8,  weight="bold")
    F_BODY   = tkfont.Font(family=sans, size=9)
    F_BODY_B = tkfont.Font(family=sans, size=9,  weight="bold")
    F_MONO   = tkfont.Font(family=mono, size=9)
    F_SMALL  = tkfont.Font(family=sans, size=8)

    # ── theme manager ─────────────────────────────────────────────────────── #
    tm = ThemeManager("dark")
    T = tm.T   # shorthand — theme dict (always current via tm.T)

    # ── load + scale map image ────────────────────────────────────────────── #
    #
    # Strategy: fit the map to (MAX_W × MAX_H) preserving aspect ratio.
    # If PIL is available we get a clean lanczos-resampled result;
    # otherwise we fall back to integer zoom/subsample via tk.PhotoImage.
    #
    MAP_MAX_W = 900
    MAP_MAX_H = 640

    if _HAS_PIL:
        pil_raw = _PILImage.open(node.map_image_path)
        raw_w, raw_h = pil_raw.size
        node.image_width  = raw_w
        node.image_height = raw_h
        node.display_scale = min(MAP_MAX_W / raw_w, MAP_MAX_H / raw_h)
        disp_w = max(1, int(raw_w * node.display_scale))
        disp_h = max(1, int(raw_h * node.display_scale))
        node._display_w = disp_w
        node._display_h = disp_h
        scaled_pil = pil_raw.resize((disp_w, disp_h), _PILImage.LANCZOS)
        node.map_photo = _ImageTk.PhotoImage(scaled_pil)
    else:
        # No PIL: load raw, then integer-scale to fit
        _raw_photo = tk.PhotoImage(master=root, file=node.map_image_path)
        raw_w = _raw_photo.width()
        raw_h = _raw_photo.height()
        node.image_width  = raw_w
        node.image_height = raw_h
        # Integer zoom / subsample
        zoom_x = max(1, MAP_MAX_W // raw_w)
        zoom_y = max(1, MAP_MAX_H // raw_h)
        zoom = min(zoom_x, zoom_y)
        sub_x = max(1, raw_w * zoom // MAP_MAX_W)
        sub_y = max(1, raw_h * zoom // MAP_MAX_H)
        sub = max(sub_x, sub_y)
        if zoom > 1:
            node.map_photo = _raw_photo.zoom(zoom, zoom)
            node.display_scale = float(zoom)
        elif sub > 1:
            node.map_photo = _raw_photo.subsample(sub, sub)
            node.display_scale = 1.0 / sub
        else:
            node.map_photo = _raw_photo
            node.display_scale = 1.0
        node._display_w = int(raw_w * node.display_scale)
        node._display_h = int(raw_h * node.display_scale)

    # ══════════════════════════════════════════════════════════════════════ #
    #  Layout: title_bar | (body: map_col | right_panel) | status_bar        #
    # ══════════════════════════════════════════════════════════════════════ #

    # ── title bar ─────────────────────────────────────────────────────────── #
    title_bar = tm.reg(
        tk.Frame(root, bg=tm.T["panel"], height=48),
        bg="panel",
    )
    title_bar.pack(fill="x")
    title_bar.pack_propagate(False)

    title_lbl = tm.reg(
        tk.Label(title_bar, text="Coverage Area Selector",
                 font=F_TITLE, padx=14, anchor="w"),
        bg="panel", fg="text",
    )
    title_lbl.pack(side="left", pady=8)

    pipe_lbl = tm.reg(
        tk.Label(title_bar, text="|"),
        bg="panel", fg="border_lt",
    )
    pipe_lbl.pack(side="left")

    map_name_lbl = tm.reg(
        tk.Label(title_bar,
                 text=f"  {os.path.basename(node.map_yaml_path)}",
                 font=F_BODY),
        bg="panel", fg="text2",
    )
    map_name_lbl.pack(side="left")

    # theme toggle (right side, before ROS badge)
    theme_btn = tm.reg(
        tk.Button(
            title_bar,
            text=tm.T["theme_icon"],
            font=F_SMALL,
            relief="flat", bd=0,
            padx=10, pady=4,
            cursor="hand2",
        ),
        bg="panel_lt", fg="text2",
        activebackground="border",
        activeforeground="text",
    )
    theme_btn.pack(side="right", padx=(6, 14), pady=8)

    # ROS badge
    ros_frame = tm.reg(tk.Frame(title_bar), bg="panel")
    ros_frame.pack(side="right", padx=6)

    ros_dot_cv = tk.Canvas(ros_frame, width=8, height=8,
                            highlightthickness=0)
    tm.reg(ros_dot_cv, bg="panel")
    ros_dot_cv.pack(side="left", padx=(0, 5), pady=10)
    ros_dot_item = ros_dot_cv.create_oval(0, 0, 8, 8,
                                          fill=tm.T["success"], outline="")
    tm.reg_canvas_item(ros_dot_cv, ros_dot_item, fill="success")

    ros_str = tm.reg(
        tk.Label(ros_frame, text="ROS 2", font=F_SMALL),
        bg="panel", fg="text3",
    )
    ros_str.pack(side="left")

    title_sep = tm.reg(tk.Frame(root, height=1), bg="border")
    title_sep.pack(fill="x")

    # ── body ──────────────────────────────────────────────────────────────── #
    body = tm.reg(tk.Frame(root), bg="bg")
    body.pack(fill="both", expand=True)

    # ── map column ────────────────────────────────────────────────────────── #
    map_col = tm.reg(tk.Frame(body), bg="bg")
    map_col.pack(side="left", fill="both", expand=True)

    map_toolbar = tm.reg(
        tk.Frame(map_col, height=30),
        bg="panel",
    )
    map_toolbar.pack(fill="x")
    map_toolbar.pack_propagate(False)

    tm.reg(
        tk.Label(map_toolbar, text="  FLOOR MAP", font=F_HDR, anchor="w"),
        bg="panel", fg="text3",
    ).pack(side="left", pady=5)

    tm.reg(
        tk.Label(
            map_toolbar,
            text=f"  {node.resolution:.4f} m/px   frame: {node.frame_id}",
            font=F_SMALL,
        ),
        bg="panel", fg="text3",
    ).pack(side="left")

    # PIL availability label (far right of toolbar)
    pil_tag = "(PIL)" if _HAS_PIL else "(no PIL — install Pillow for better scaling)"
    tm.reg(
        tk.Label(map_toolbar, text=pil_tag, font=F_SMALL, padx=8),
        bg="panel", fg="text3",
    ).pack(side="right")

    map_sep = tm.reg(tk.Frame(map_col, height=1), bg="border")
    map_sep.pack(fill="x")

    # Outer pad frame
    cv_outer = tm.reg(tk.Frame(map_col, padx=6, pady=6), bg="bg")
    cv_outer.pack(fill="both", expand=True)

    # 1-px border
    cv_border_frame = tm.reg(
        tk.Frame(cv_outer, padx=1, pady=1),
        bg="border_lt",
    )
    cv_border_frame.pack(fill="both", expand=True)

    cv_inner = tm.reg(tk.Frame(cv_border_frame), bg="canvas_bg")
    cv_inner.pack(fill="both", expand=True)

    h_sb = tk.Scrollbar(cv_inner, orient="horizontal")
    tm.reg(h_sb, bg="panel", troughcolor="bg", activebackground="border_lt")
    h_sb.pack(side="bottom", fill="x")

    v_sb = tk.Scrollbar(cv_inner, orient="vertical")
    tm.reg(v_sb, bg="panel", troughcolor="bg", activebackground="border_lt")
    v_sb.pack(side="right", fill="y")

    canvas = tk.Canvas(
        cv_inner,
        bg=tm.T["canvas_bg"],
        xscrollcommand=h_sb.set,
        yscrollcommand=v_sb.set,
        cursor="crosshair",
        highlightthickness=0,
    )
    canvas.pack(side="left", fill="both", expand=True)
    h_sb.config(command=canvas.xview)
    v_sb.config(command=canvas.yview)

    canvas.create_image(0, 0, image=node.map_photo, anchor="nw")
    canvas.config(scrollregion=(0, 0, node._display_w, node._display_h))
    tm.hook(lambda T: canvas.config(bg=T["canvas_bg"]))

    # ── right panel ───────────────────────────────────────────────────────── #
    rp_div = tm.reg(tk.Frame(body, width=1), bg="border")
    rp_div.pack(side="right", fill="y")

    right = tm.reg(tk.Frame(body, width=278), bg="panel")
    right.pack(side="right", fill="y")
    right.pack_propagate(False)

    # Helper: section label + separator
    def section_hdr(parent, text):
        f = tm.reg(tk.Frame(parent), bg="panel")
        f.pack(fill="x", padx=0, pady=(10, 0))
        tm.reg(tk.Frame(f, height=1), bg="border").pack(fill="x")
        lbl = tm.reg(
            tk.Label(f, text=text.upper(), font=F_HDR,
                     anchor="w", padx=10, pady=4),
            bg="panel", fg="text3",
        )
        lbl.pack(fill="x")

    # Helper: key-value row
    def info_row(parent, label_text, var):
        row = tm.reg(tk.Frame(parent), bg="panel")
        row.pack(fill="x", padx=10, pady=1)
        tm.reg(
            tk.Label(row, text=label_text, font=F_SMALL,
                     anchor="w", width=12),
            bg="panel", fg="text2",
        ).pack(side="left")
        tm.reg(
            tk.Label(row, textvariable=var, font=F_SMALL, anchor="w"),
            bg="panel", fg="text",
        ).pack(side="left", fill="x", expand=True)

    # ── STATUS section ────────────────────────────────────────────────────── #
    section_hdr(right, "Status")

    status_row = tm.reg(tk.Frame(right), bg="panel")
    status_row.pack(fill="x", padx=10, pady=(6, 8))

    st_dot_cv = tk.Canvas(status_row, width=10, height=10, highlightthickness=0)
    tm.reg(st_dot_cv, bg="panel")
    st_dot_cv.pack(side="left", padx=(0, 7))
    st_dot = st_dot_cv.create_oval(1, 1, 9, 9, fill=tm.T["text3"], outline="")

    status_var = tk.StringVar(value="Idle")
    status_lbl = tm.reg(
        tk.Label(status_row, textvariable=status_var, font=F_BODY, anchor="w"),
        bg="panel", fg="text2",
    )
    status_lbl.pack(side="left", fill="x", expand=True)

    # ── CONTROLS section ──────────────────────────────────────────────────── #
    section_hdr(right, "Controls")

    ctrl = tm.reg(tk.Frame(right), bg="panel")
    ctrl.pack(fill="x", padx=10, pady=(4, 0))

    def make_btn(parent, text, command, *, style="primary"):
        """Build a themed tk.Button. style: primary|ghost|danger|cancel."""
        T = tm.T
        cfgs = {
            "primary": dict(bg=T["accent"],     fg="#FFFFFF",
                            abg=T["accent_hov"], afg="#FFFFFF"),
            "ghost":   dict(bg=T["ghost_bg"],   fg=T["ghost_fg"],
                            abg=T["ghost_abg"],  afg=T["ghost_fg"]),
            "danger":  dict(bg=T["danger_bg"],  fg=T["danger_fg"],
                            abg=T["danger_abg"], afg=T["danger_fg"]),
            "cancel":  dict(bg=T["cancel_bg"],  fg=T["cancel_fg"],
                            abg=T["cancel_hov"], afg=T["cancel_fg"]),
        }
        c = cfgs[style]
        btn = tk.Button(
            parent, text=text, command=command,
            bg=c["bg"], fg=c["fg"],
            activebackground=c["abg"], activeforeground=c["afg"],
            font=F_BODY_B if style in ("primary", "cancel") else F_BODY,
            relief="flat", bd=0,
            padx=12, pady=7,
            cursor="hand2",
            highlightthickness=1,
            highlightbackground=T["border"],
            highlightcolor=T["accent"],
        )
        # Register for theming
        cfg_keys = cfgs  # closure
        def _update(T_new, _style=style, _btn=btn):
            c_n = {
                "primary": dict(bg=T_new["accent"],    fg="#FFFFFF",
                                abg=T_new["accent_hov"], afg="#FFFFFF"),
                "ghost":   dict(bg=T_new["ghost_bg"],  fg=T_new["ghost_fg"],
                                abg=T_new["ghost_abg"], afg=T_new["ghost_fg"]),
                "danger":  dict(bg=T_new["danger_bg"], fg=T_new["danger_fg"],
                                abg=T_new["danger_abg"], afg=T_new["danger_fg"]),
                "cancel":  dict(bg=T_new["cancel_bg"], fg=T_new["cancel_fg"],
                                abg=T_new["cancel_hov"], afg=T_new["cancel_fg"]),
            }[_style]
            try:
                _btn.config(
                    bg=c_n["bg"], fg=c_n["fg"],
                    activebackground=c_n["abg"],
                    activeforeground=c_n["afg"],
                    highlightbackground=T_new["border"],
                    highlightcolor=T_new["accent"],
                )
            except Exception:
                pass
        tm.hook(_update)
        return btn

    btn_select = make_btn(ctrl, "Select Area",       node.start_selection_mode, style="primary")
    btn_select.pack(fill="x", pady=3)

    btn_undo   = make_btn(ctrl, "Undo Corner",       node.undo_last,            style="ghost")
    btn_undo.pack(fill="x", pady=3)

    btn_clear  = make_btn(ctrl, "Clear Selection",   node.clear_points,         style="danger")
    btn_clear.pack(fill="x", pady=3)

    # separator
    tm.reg(tk.Frame(ctrl, height=1), bg="border").pack(fill="x", pady=8)

    btn_send   = make_btn(ctrl, "Send Polygon",       node.send_polygon,         style="primary")
    btn_send.pack(fill="x", pady=3)

    btn_cancel = make_btn(ctrl, "Cancel Cleaning",   node.cancel_coverage,      style="cancel")
    btn_cancel.pack(fill="x", pady=3)

    # ── SECTION MANAGEMENT section ────────────────────────────────────────── #
    section_hdr(right, "Section Management")

    # Save section row with input
    save_sec_row = tm.reg(tk.Frame(right), bg="panel")
    save_sec_row.pack(fill="x", padx=10, pady=(4, 2))
    
    section_name_var = tk.StringVar()
    section_name_entry = tm.reg(
        tk.Entry(
            save_sec_row,
            textvariable=section_name_var,
            font=F_SMALL,
            width=15,
        ),
        bg="lb_bg", fg="text",
    )
    section_name_entry.pack(side="left", fill="x", expand=True, padx=(0, 6))
    
    def save_section_action():
        name = section_name_var.get().strip()
        if node.save_current_area(name):
            section_name_var.set("")
            section_name_entry.delete(0, tk.END)
            refresh_saved_sections()
    
    btn_save_sec = make_btn(save_sec_row, "Save Section", save_section_action, style="primary")
    btn_save_sec.pack(side="right")

    # ── SAVED SECTIONS list ──────────────────────────────────────────────── #
    section_hdr(right, "Saved Sections")

    # Listbox with saved sections
    list_wrap_sec = tm.reg(tk.Frame(right), bg="panel")
    list_wrap_sec.pack(fill="both", expand=True, padx=10, pady=(4, 8))

    list_sb_sec = tk.Scrollbar(list_wrap_sec)
    tm.reg(list_sb_sec, bg="panel", troughcolor="bg", activebackground="border_lt")
    list_sb_sec.pack(side="right", fill="y")

    saved_sections_listbox = tk.Listbox(
        list_wrap_sec,
        font=F_SMALL,
        activestyle="none",
        bd=0, relief="flat",
        highlightthickness=0,
        yscrollcommand=list_sb_sec.set,
        selectmode=tk.MULTIPLE,  # Allow multiple selection
    )
    tm.reg(saved_sections_listbox,
           bg="lb_bg", fg="text2",
           selectbackground="lb_sel_bg",
           selectforeground="lb_sel_fg")
    saved_sections_listbox.pack(side="left", fill="both", expand=True)
    list_sb_sec.config(command=saved_sections_listbox.yview)

    def refresh_saved_sections():
        """Refresh the saved sections listbox."""
        saved_sections_listbox.delete(0, tk.END)
        T = tm.T
        for idx, section_name in enumerate(sorted(node.sections.keys())):
            saved_sections_listbox.insert(tk.END, f"  {section_name}")
            if idx % 2 == 0:
                saved_sections_listbox.itemconfig(idx, bg=T["lb_alt"])

    # Buttons for section actions
    sec_btn_row = tm.reg(tk.Frame(right), bg="panel")
    sec_btn_row.pack(fill="x", padx=10, pady=(2, 8))

    def clean_selected_sections():
        """Clean selected sections."""
        selection = saved_sections_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select at least one section")
            return
        
        sections_list = sorted(node.sections.keys())
        selected_names = [sections_list[i] for i in selection]
        
        # For now, clean the first selected section
        # In future, could add support for multiple sequential sections
        first_section = selected_names[0]
        section_polygon = node.get_section(first_section)
        
        if section_polygon:
            node.set_pts([node.map_to_canvas(x, y) for x, y in section_polygon])
            node.ui_queue.put(("refresh", None))
            node.send_polygon()
            set_status("info", f"Cleaning section: {first_section}")

    def delete_selected_section():
        """Delete selected section."""
        selection = saved_sections_listbox.curselection()
        if not selection:
            messagebox.showwarning("No Selection", "Please select a section to delete")
            return
        
        sections_list = sorted(node.sections.keys())
        section_name = sections_list[selection[0]]
        
        if messagebox.askyesno("Confirm Delete", f"Delete section '{section_name}'?"):
            node.delete_section(section_name)
            refresh_saved_sections()
            set_status("info", f"Deleted section: {section_name}")

    btn_clean_sec = make_btn(sec_btn_row, "Clean Selected", clean_selected_sections, style="primary")
    btn_clean_sec.pack(side="left", fill="x", expand=True, padx=(0, 3))

    btn_del_sec = make_btn(sec_btn_row, "Delete", delete_selected_section, style="danger")
    btn_del_sec.pack(side="left", fill="x", expand=True)

    # Initial load of saved sections
    refresh_saved_sections()

    # ── MAP INFO section ──────────────────────────────────────────────────── #
    section_hdr(right, "Map Info")

    v_file    = tk.StringVar(value=os.path.basename(node.map_yaml_path))
    v_frame   = tk.StringVar(value=node.frame_id)
    v_res     = tk.StringVar(value=f"{node.resolution:.4f} m/px")
    v_dim     = tk.StringVar(value=f"{node.image_width} × {node.image_height} px")
    v_corners = tk.StringVar(value="0")

    for lbl, var in [
        ("Map file",   v_file),
        ("Frame",      v_frame),
        ("Resolution", v_res),
        ("Dimensions", v_dim),
        ("Corners",    v_corners),
    ]:
        info_row(right, lbl, var)

    # ── COVERAGE PROGRESS section ─────────────────────────────────────────── #
    section_hdr(right, "Coverage Progress")

    v_coverage_percent = tk.StringVar(value="0.0%")
    v_coverage_feedback = tk.StringVar(value="Ready to start coverage task")
    
    cov_pct_row = tm.reg(tk.Frame(right), bg="panel")
    cov_pct_row.pack(fill="x", padx=10, pady=(6, 2))
    tm.reg(
        tk.Label(cov_pct_row, text="Covered ", font=F_SMALL, anchor="w", width=12),
        bg="panel", fg="text2",
    ).pack(side="left")
    tm.reg(
        tk.Label(cov_pct_row, textvariable=v_coverage_percent, font=F_BODY_B, anchor="w"),
        bg="panel", fg="success",
    ).pack(side="left", fill="x", expand=True)
    
    # Progress bar background
    progress_bg_frame = tm.reg(tk.Frame(right), bg="panel")
    progress_bg_frame.pack(fill="x", padx=10, pady=(2, 6))
    
    progress_canvas = tk.Canvas(
        progress_bg_frame,
        width=200, height=20,
        bg=tm.T["lb_bg"],
        highlightthickness=1,
        highlightbackground=tm.T["border"],
    )
    tm.reg(progress_canvas, bg="lb_bg")
    progress_canvas.pack(fill="x", expand=True)
    
    # Create progress bar fill rectangle
    progress_fill = progress_canvas.create_rectangle(
        0, 0, 0, 20,
        fill=tm.T["success"],
        outline="",
    )
    tm.reg_canvas_item(progress_canvas, progress_fill, fill="success")
    
    # Feedback text
    feedback_row = tm.reg(tk.Frame(right), bg="panel")
    feedback_row.pack(fill="x", padx=10, pady=1)
    tm.reg(
        tk.Label(
            feedback_row,
            text="Status: ",
            font=F_SMALL,
            anchor="w",
        ),
        bg="panel", fg="text2",
    ).pack(side="left")
    tm.reg(
        tk.Label(
            feedback_row,
            textvariable=v_coverage_feedback,
            font=F_SMALL,
            anchor="w",
            wraplength=200,
            justify="left",
        ),
        bg="panel", fg="text",
    ).pack(side="left", fill="x", expand=True)

    def update_progress_bar(percent):
        """Update the progress bar fill based on coverage percentage."""
        # Get canvas width
        width = progress_canvas.winfo_width()
        if width <= 1:
            width = 200
        fill_width = max(0, min(width, (percent / 100.0) * width))
        progress_canvas.coords(progress_fill, 0, 0, fill_width, 20)

    # ── SELECTED CORNERS section ──────────────────────────────────────────── #
    section_hdr(right, "Selected Corners")

    list_wrap = tm.reg(tk.Frame(right), bg="panel")
    list_wrap.pack(fill="both", expand=True, padx=10, pady=(4, 8))

    list_sb = tk.Scrollbar(list_wrap)
    tm.reg(list_sb, bg="panel", troughcolor="bg", activebackground="border_lt")
    list_sb.pack(side="right", fill="y")

    listbox = tk.Listbox(
        list_wrap,
        font=F_MONO,
        activestyle="none",
        bd=0, relief="flat",
        highlightthickness=0,
        yscrollcommand=list_sb.set,
    )
    tm.reg(listbox,
           bg="lb_bg", fg="text2",
           selectbackground="lb_sel_bg",
           selectforeground="lb_sel_fg")
    listbox.pack(side="left", fill="both", expand=True)
    list_sb.config(command=listbox.yview)

    # ── status bar ────────────────────────────────────────────────────────── #
    sbar_sep = tm.reg(tk.Frame(root, height=1), bg="border")
    sbar_sep.pack(fill="x", side="bottom")

    status_bar = tm.reg(tk.Frame(root, height=24), bg="panel")
    status_bar.pack(fill="x", side="bottom")
    status_bar.pack_propagate(False)

    instr_var = tk.StringVar(
        value="Click  Select Area  →  click map corners  →  Send Polygon"
    )
    tm.reg(
        tk.Label(status_bar, textvariable=instr_var,
                 font=F_SMALL, anchor="w", padx=10),
        bg="panel", fg="text3",
    ).pack(side="left", fill="both", expand=True)

    tm.reg(
        tk.Label(status_bar, text="my_coverage v1.0",
                 font=F_SMALL, padx=10),
        bg="panel", fg="text3",
    ).pack(side="right")

    # Apply initial theme to root window
    root.config(bg=tm.T["bg"])
    tm.hook(lambda T: root.config(bg=T["bg"]))
    
    # Theme hook for progress canvas
    tm.hook(lambda T: progress_canvas.config(
        bg=T["lb_bg"],
        highlightbackground=T["border"],
    ))
    tm.hook(lambda T: progress_canvas.itemconfig(progress_fill, fill=T["success"]))

    # ══════════════════════════════════════════════════════════════════════ #
    #  Drawing (polygon preview on canvas)                                   #
    # ══════════════════════════════════════════════════════════════════════ #

    def redraw():
        if node.preview_shape_id is not None:
            canvas.delete(node.preview_shape_id)
            node.preview_shape_id = None
        for vid in node.preview_vertex_ids:
            canvas.delete(vid)
        node.preview_vertex_ids.clear()

        T = tm.T
        pts = node.get_pts()
        n = len(pts)

        if n >= 3:
            flat = [c for p in pts for c in p]
            node.preview_shape_id = canvas.create_polygon(
                flat,
                outline=T["poly"],
                fill=T["poly_fill"],
                stipple=T["poly_stipple"],
                width=2,
            )
        elif n == 2:
            node.preview_shape_id = canvas.create_line(
                *pts[0], *pts[1],
                fill=T["poly"], width=2, dash=(6, 3),
            )

        for idx, (x, y) in enumerate(pts, start=1):
            r = 4
            vid = canvas.create_oval(
                x - r, y - r, x + r, y + r,
                fill=T["vertex"], outline=T["bg"], width=1,
            )
            node.preview_vertex_ids.append(vid)
            lid = canvas.create_text(
                x + 8, y - 8,
                text=str(idx),
                fill=T["text"],
                font=F_SMALL,
                anchor="sw",
            )
            node.preview_vertex_ids.append(lid)

        # Always redraw robot on top of polygon
        draw_robot()

    def draw_robot():
        """Draw (or erase) the robot position indicator on the canvas."""
        for item_id in node.robot_canvas_ids:
            canvas.delete(item_id)
        node.robot_canvas_ids.clear()

        if node.robot_pose is None:
            return

        T = tm.T
        mx, my, yaw = node.robot_pose
        cx, cy = node.map_to_canvas(mx, my)

        # Clamp to scroll region so the indicator is always reachable
        cx = max(0.0, min(cx, float(node._display_w)))
        cy = max(0.0, min(cy, float(node._display_h)))

        # ── body circle ──────────────────────────────────────────────────── #
        R = 9   # radius in display pixels
        body = canvas.create_oval(
            cx - R, cy - R, cx + R, cy + R,
            fill=T["robot_body"],
            outline=T["robot_outline"],
            width=2,
        )
        node.robot_canvas_ids.append(body)

        # ── heading arrow ─────────────────────────────────────────────────── #
        # Canvas y is flipped vs ROS y, so canvas_dy = -sin(yaw)
        arrow_len = R + 10
        ax = cx + arrow_len * math.cos(yaw)
        ay = cy - arrow_len * math.sin(yaw)   # note: minus for y-flip
        arrow = canvas.create_line(
            cx, cy, ax, ay,
            fill=T["robot_arrow"],
            width=3,
            arrow=tk.LAST,
            arrowshape=(10, 12, 4),
        )
        node.robot_canvas_ids.append(arrow)

        # ── "Robot" label ─────────────────────────────────────────────────── #
        lbl = canvas.create_text(
            cx + R + 4, cy - R - 2,
            text="Robot",
            fill=T["robot_label"],
            font=F_SMALL,
            anchor="sw",
        )
        node.robot_canvas_ids.append(lbl)

    tm.hook(lambda _T: redraw())

    def refresh():
        pts = node.get_pts()
        listbox.delete(0, tk.END)
        T = tm.T
        for idx, pt in enumerate(pts, start=1):
            mx, my = node.canvas_to_map(pt)
            listbox.insert(tk.END, f" {idx:>2}   x={mx:>8.3f}   y={my:>8.3f}")
            if idx % 2 == 0:
                listbox.itemconfig(idx - 1, bg=T["lb_alt"])
        v_corners.set(str(len(pts)))
        redraw()

    def draw_swaths():
        """Render zigzag coverage swaths on the canvas."""
        T = tm.T
        
        # Clear old swath renderings
        for item_id in node.swath_canvas_ids:
            try:
                canvas.delete(item_id)
            except Exception:
                pass
        node.swath_canvas_ids.clear()
        
        if not node.swaths:
            return
        
        # Determine coverage color based on distance from robot
        covered_color = T["success"]       # Green for covered swaths
        uncovered_color = T["accent_dim"]  # Faded blue for uncovered swaths
        
        # Coverage distance threshold (within this distance from robot = covered)
        COVERED_THRESHOLD = 0.5  # meters
        
        for swath in node.swaths:
            if len(swath) < 2:
                continue
            
            # Convert map coordinates to canvas coordinates
            p1_map, p2_map = swath[0], swath[1]
            p1_canvas = node.map_to_canvas(p1_map[0], p1_map[1])
            p2_canvas = node.map_to_canvas(p2_map[0], p2_map[1])
            
            # Determine if this swath has been covered
            is_covered = False
            if node.robot_position:
                # Check if robot is close to any part of this swath
                robot_x, robot_y = node.robot_position
                dist_to_p1 = math.sqrt((p1_map[0] - robot_x)**2 + (p1_map[1] - robot_y)**2)
                dist_to_p2 = math.sqrt((p2_map[0] - robot_x)**2 + (p2_map[1] - robot_y)**2)
                dist_to_line = min(dist_to_p1, dist_to_p2)
                is_covered = dist_to_line < COVERED_THRESHOLD
            
            # Draw swath line
            color = covered_color if is_covered else uncovered_color
            line_width = 3 if is_covered else 2
            opacity_hex = "FF" if is_covered else "80"  # Full or 50% opacity
            
            # For Tk, we can't do actual transparency, so we'll use color fading
            dash_pattern = () if is_covered else (4, 4)  # Solid for covered, dashed for uncovered
            
            try:
                item_id = canvas.create_line(
                    p1_canvas[0], p1_canvas[1], p2_canvas[0], p2_canvas[1],
                    fill=color,
                    width=line_width,
                    dash=dash_pattern,
                    capstyle=tk.ROUND,
                    joinstyle=tk.ROUND,
                )
                node.swath_canvas_ids.append(item_id)
            except Exception as e:
                node.get_logger().warn(f"Failed to draw swath: {e}")

    # ══════════════════════════════════════════════════════════════════════ #
    #  Status helpers                                                         #
    # ══════════════════════════════════════════════════════════════════════ #

    def set_status(kind, text):
        T = tm.T
        mapping = {
            "idle":    (T["text3"],   T["text3"]),
            "info":    (T["text"],    T["accent"]),
            "success": (T["success"], T["success"]),
            "warning": (T["warning"], T["warning"]),
            "error":   (T["error"],   T["error"]),
        }
        fg, dot_c = mapping.get(kind, mapping["info"])
        status_var.set(text)
        status_lbl.config(fg=fg)
        st_dot_cv.itemconfig(st_dot, fill=dot_c)

    def decode_ros(text):
        t = text.upper()
        if "SUCCEEDED" in t:
            return "success", "Task completed successfully"
        if "FAILED" in t or "ERROR" in t:
            return "error", text
        if "RUNNING" in t or "COVERAGE" in t:
            return "info", "Coverage task running…"
        if "CANCELLING" in t:
            return "warning", "Cancelling coverage task…"
        if "CANCELED" in t or "CANCELLED" in t:
            return "warning", "Coverage task cancelled"
        if "NO ACTIVE" in t:
            return "idle", "No active task to cancel"
        if "IDLE" in t:
            return "idle", "Idle"
        return "info", text

    # ══════════════════════════════════════════════════════════════════════ #
    #  Theme toggle                                                           #
    # ══════════════════════════════════════════════════════════════════════ #

    def toggle_theme():
        new = "light" if tm.name == "dark" else "dark"
        tm.switch(new)
        theme_btn.config(text=tm.T["theme_icon"])
        # listbox alternating rows need manual refresh
        refresh()
        draw_swaths()  # Redraw swaths with new theme colors

    theme_btn.config(command=toggle_theme)

    # ══════════════════════════════════════════════════════════════════════ #
    #  UI queue pump (100 ms)                                                 #
    # ══════════════════════════════════════════════════════════════════════ #

    def pump():
        while True:
            try:
                evt, data = node.ui_queue.get_nowait()
            except queue.Empty:
                break
            if evt == "refresh":
                refresh()
            elif evt == "ros_status":
                set_status(*decode_ros(data))
            elif evt == "local_status":
                set_status(*data)
            elif evt == "robot_pose":
                # Just redraw to move the indicator; no full refresh needed
                draw_robot()
            elif evt == "coverage_percent":
                # Update coverage progress bar
                v_coverage_percent.set(f"{data:.1f}%")
                update_progress_bar(data)
                draw_swaths()  # Redraw swaths to show coverage progress
            elif evt == "coverage_feedback":
                # Update detailed feedback text
                v_coverage_feedback.set(data)
            elif evt == "swaths_update":
                # Swaths have been updated from coverage server
                draw_swaths()
            elif evt == "clear_swaths":
                # Clear swaths visualization
                for item_id in node.swath_canvas_ids:
                    canvas.delete(item_id)
                node.swath_canvas_ids.clear()
            elif evt == "sections_updated":
                # Refresh saved sections listbox
                refresh_saved_sections()
            elif evt == "popup":
                messagebox.showwarning("Coverage Area Selector", data)
        root.after(100, pump)

    # ══════════════════════════════════════════════════════════════════════ #
    #  Bindings & startup                                                     #
    # ══════════════════════════════════════════════════════════════════════ #

    canvas.bind("<Button-1>", node.add_corner)

    def on_close():
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", on_close)

    # Apply initial theme fully
    tm.switch("dark")

    refresh()
    pump()
    root.mainloop()


if __name__ == "__main__":
    main()
