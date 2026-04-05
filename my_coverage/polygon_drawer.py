import os
import queue
import threading
import tkinter as tk
from tkinter import messagebox

from geometry_msgs.msg import Point, Point32, Polygon
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class PolygonDrawer(Node):
    def __init__(self):
        super().__init__('polygon_drawer')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('map', '')

        self.frame_id = self.get_parameter('frame_id').value
        self.map_yaml_path = self.get_parameter('map').value
        if not self.map_yaml_path:
            raise RuntimeError('Parameter "map" must point to a map yaml file')

        self.map_image_path, self.resolution, self.origin = self._load_map_metadata(
            self.map_yaml_path
        )

        self.lock = threading.Lock()
        self.ui_queue = queue.Queue()
        self.canvas_points = []
        self.selection_mode = False
        self.preview_shape_id = None
        self.preview_vertex_ids = []

        self.status_sub = self.create_subscription(
            String, '/coverage_task_status', self.status_callback, 10
        )
        self.polygon_pub = self.create_publisher(
            Polygon, '/user_selected_field', 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/user_selected_field_marker', 10
        )

    def _load_map_metadata(self, map_yaml_path):
        image_path = None
        resolution = None
        origin = None

        with open(map_yaml_path, 'r', encoding='utf-8') as yaml_file:
            for raw_line in yaml_file:
                line = raw_line.split('#', 1)[0].strip()
                if not line or ':' not in line:
                    continue

                key, value = line.split(':', 1)
                key = key.strip()
                value = value.strip()

                if key == 'image':
                    image_path = value.strip().strip('"').strip("'")
                elif key == 'resolution':
                    resolution = float(value)
                elif key == 'origin':
                    origin_values = value.strip()[1:-1].split(',')
                    origin = [float(item.strip()) for item in origin_values]

        if image_path is None or resolution is None or origin is None:
            raise RuntimeError(f'Unable to parse map metadata from {map_yaml_path}')

        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

        if not os.path.exists(image_path):
            raise RuntimeError(f'Map image not found: {image_path}')

        return image_path, resolution, origin

    def status_callback(self, msg):
        self.ui_queue.put(('status', msg.data))

    def get_canvas_points(self):
        with self.lock:
            return list(self.canvas_points)

    def set_canvas_points(self, points):
        with self.lock:
            self.canvas_points = list(points)

    def clear_points(self):
        self.set_canvas_points([])
        self.publish_preview_marker(clear=True)
        self.ui_queue.put(('refresh', None))
        self.ui_queue.put(('status', 'Selection cleared'))

    def undo_last(self):
        points = self.get_canvas_points()
        if not points:
            return
        points.pop()
        self.set_canvas_points(points)
        self.publish_preview_marker(clear=not points)
        self.ui_queue.put(('refresh', None))
        self.ui_queue.put(('status', 'Removed last corner'))

    def start_selection_mode(self):
        self.selection_mode = True
        self.ui_queue.put(
            ('status', 'Click the map corners one by one, then press Send Polygon')
        )

    def add_corner(self, event):
        if not self.selection_mode:
            return

        points = self.get_canvas_points()
        points.append((event.x, event.y))
        self.set_canvas_points(points)
        self.publish_preview_marker()
        self.ui_queue.put(('refresh', None))
        self.ui_queue.put(
            ('status', f'Added corner {len(points)}. Add more corners or press Send Polygon')
        )

    def publish_polygon(self):
        canvas_points = self.get_canvas_points()
        if len(canvas_points) < 3:
            self.ui_queue.put(('popup', 'Select an area on the map first'))
            return

        polygon_msg = Polygon()
        map_points = [self.canvas_to_map(point) for point in canvas_points]
        closed_map_points = list(map_points)
        if closed_map_points[0] != closed_map_points[-1]:
            closed_map_points.append(closed_map_points[0])

        for x_value, y_value in closed_map_points:
            point = Point32()
            point.x = float(x_value)
            point.y = float(y_value)
            polygon_msg.points.append(point)

        self.polygon_pub.publish(polygon_msg)
        self.publish_preview_marker()
        self.ui_queue.put(('status', 'Polygon sent'))

    def canvas_to_map(self, point):
        x_value, y_value = point
        map_x = self.origin[0] + (float(x_value) * self.resolution)
        map_y = self.origin[1] + ((self.image_height - float(y_value)) * self.resolution)
        return (map_x, map_y)

    def publish_preview_marker(self, clear=False):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_polygon'
        marker.id = 1
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.DELETE if clear else Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        if not clear:
            map_points = [self.canvas_to_map(point) for point in self.get_canvas_points()]
            if map_points:
                closed_map_points = list(map_points)
                if closed_map_points[0] != closed_map_points[-1]:
                    closed_map_points.append(closed_map_points[0])
                for x_value, y_value in closed_map_points:
                    point = Point()
                    point.x = float(x_value)
                    point.y = float(y_value)
                    point.z = 0.0
                    marker.points.append(point)

        self.marker_pub.publish(marker)

def main():
    rclpy.init()
    node = PolygonDrawer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    root = tk.Tk()
    root.title('Coverage Area Selector')
    root.geometry('920x700')

    node.map_image = tk.PhotoImage(master=root, file=node.map_image_path)
    node.image_width = node.map_image.width()
    node.image_height = node.map_image.height()

    toolbar = tk.Frame(root)
    toolbar.pack(fill='x', padx=10, pady=10)

    status_var = tk.StringVar(value='Status: Idle')
    info_var = tk.StringVar(
        value=(
            f'Map: {os.path.basename(node.map_yaml_path)}  '
            f'Resolution: {node.resolution:.3f} m/pixel'
        )
    )

    canvas_frame = tk.Frame(root)
    canvas_frame.pack(fill='both', expand=True, padx=10, pady=(0, 10))

    h_scroll = tk.Scrollbar(canvas_frame, orient='horizontal')
    h_scroll.pack(side='bottom', fill='x')
    v_scroll = tk.Scrollbar(canvas_frame, orient='vertical')
    v_scroll.pack(side='right', fill='y')

    canvas = tk.Canvas(
        canvas_frame,
        bg='black',
        xscrollcommand=h_scroll.set,
        yscrollcommand=v_scroll.set,
        width=min(node.image_width + 4, 860),
        height=min(node.image_height + 4, 560),
        cursor='crosshair',
    )
    canvas.pack(side='left', fill='both', expand=True)
    h_scroll.config(command=canvas.xview)
    v_scroll.config(command=canvas.yview)

    canvas.create_image(0, 0, image=node.map_image, anchor='nw')
    canvas.config(scrollregion=(0, 0, node.image_width, node.image_height))

    selection_button = tk.Button(
        toolbar,
        text='Select Area',
        command=node.start_selection_mode,
    )
    selection_button.pack(side='left', padx=5)

    tk.Button(toolbar, text='Undo Corner', command=node.undo_last).pack(side='left', padx=5)
    tk.Button(toolbar, text='Clear', command=node.clear_points).pack(side='left', padx=5)
    tk.Button(toolbar, text='Send Polygon', command=node.publish_polygon).pack(
        side='right', padx=5
    )

    tk.Label(root, textvariable=info_var, anchor='w').pack(fill='x', padx=10)
    tk.Label(
        root,
        text=(
            'Press Select Area, then click the map corners in order. '
            'After selecting the area, press Send Polygon.'
        ),
        wraplength=860,
        justify='left',
    ).pack(fill='x', padx=10, pady=(6, 4))
    tk.Label(root, textvariable=status_var, anchor='w').pack(fill='x', padx=10, pady=(0, 8))

    points_label = tk.Label(root, text='Selected Polygon Corners')
    points_label.pack(anchor='w', padx=10)

    listbox = tk.Listbox(root, height=8)
    listbox.pack(fill='x', padx=10, pady=(4, 10))

    def redraw_polygon():
        if node.preview_shape_id is not None:
            canvas.delete(node.preview_shape_id)
            node.preview_shape_id = None

        for vertex_id in node.preview_vertex_ids:
            canvas.delete(vertex_id)
        node.preview_vertex_ids = []

        points = node.get_canvas_points()
        if len(points) >= 2:
            flat_points = [coordinate for point in points for coordinate in point]
            node.preview_shape_id = canvas.create_polygon(
                flat_points,
                outline='#00ff66',
                fill='',
                width=2,
            )

        for x_value, y_value in points:
            radius = 4
            vertex_id = canvas.create_oval(
                x_value - radius,
                y_value - radius,
                x_value + radius,
                y_value + radius,
                fill='#ffcc00',
                outline='',
            )
            node.preview_vertex_ids.append(vertex_id)

    def refresh_points():
        listbox.delete(0, tk.END)
        for index, point in enumerate(node.get_canvas_points(), start=1):
            map_x, map_y = node.canvas_to_map(point)
            listbox.insert(tk.END, f'{index}. x={map_x:.3f}, y={map_y:.3f}')
        redraw_polygon()

    def process_ui_queue():
        while True:
            try:
                item_type, payload = node.ui_queue.get_nowait()
            except queue.Empty:
                break

            if item_type == 'refresh':
                refresh_points()
            elif item_type == 'status':
                status_var.set(f'Status: {payload}')
            elif item_type == 'popup':
                messagebox.showwarning('Coverage Area Selector', payload)

        root.after(100, process_ui_queue)

    def on_close():
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        root.destroy()

    canvas.bind('<Button-1>', node.add_corner)

    root.protocol('WM_DELETE_WINDOW', on_close)
    refresh_points()
    process_ui_queue()
    root.mainloop()


if __name__ == '__main__':
    main()
