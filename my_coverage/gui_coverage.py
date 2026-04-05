import math
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Point32, Polygon
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class GuiCoverage(Node):
    def __init__(self):
        super().__init__('gui_coverage')
        if not self.has_parameter('frame_id'):
            self.declare_parameter('frame_id', 'map')

        self.pending_fields = []
        self.current_field = None
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.is_processing = False

        self.coverage_client = ActionClient(
            self, NavigateCompleteCoverage, 'navigate_complete_coverage'
        )
        self.field_sub = self.create_subscription(
            Polygon, '/user_selected_field', self.field_callback, 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/user_selected_field_marker', 10
        )
        self.status_pub = self.create_publisher(
            String, '/coverage_task_status', 10
        )

        self.get_logger().info('GUI coverage backend ready')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def field_callback(self, msg):
        field = [[pt.x, pt.y] for pt in msg.points]
        if len(field) < 4:
            self._publish_status('FAILED: polygon needs at least 3 corners')
            self.get_logger().error('Polygon needs at least 3 corners')
            return

        if field[0] != field[-1]:
            field.append(field[0])

        if not self.is_convex(field[:-1]):
            self._publish_status('FAILED: polygon must be convex')
            self.get_logger().error('Polygon is not convex')
            return

        self.pending_fields.append(field)
        self.get_logger().info(
            f'Queued polygon with {len(field) - 1} corners, queue size: {len(self.pending_fields)}'
        )
        self.publish_polygon_marker(field)
        if not self.is_processing:
            self.process_next_field()

    def process_next_field(self):
        if not self.pending_fields:
            self.current_field = None
            self.is_processing = False
            self._publish_status('Idle')
            return

        self.current_field = self.pending_fields.pop(0)
        self.is_processing = True
        self.send_coverage_goal(self.current_field)

    def send_coverage_goal(self, field):
        if not self.coverage_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('navigate_complete_coverage action server unavailable')
            self._publish_status('FAILED: coverage server unavailable')
            self.is_processing = False
            self.process_next_field()
            return

        goal_msg = NavigateCompleteCoverage.Goal()
        goal_msg.frame_id = self.get_parameter('frame_id').value
        goal_msg.polygons.clear()
        goal_msg.polygons.append(self.to_polygon(field))

        self._publish_status('Running coverage')
        self.get_logger().info(
            f'Sending coverage goal with {len(field) - 1} corners in frame '
            f'{goal_msg.frame_id}'
        )
        send_goal_future = self.coverage_client.send_goal_async(
            goal_msg, self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        try:
            self.goal_handle = future.result()
        except Exception as error:
            self.get_logger().error(f'Coverage goal send failed: {error}')
            self._publish_status(f'FAILED: {error}')
            self.is_processing = False
            self.process_next_field()
            return

        if not self.goal_handle.accepted:
            self.get_logger().error('Coverage goal rejected')
            self._publish_status('FAILED: goal rejected')
            self.is_processing = False
            self.process_next_field()
            return

        self.get_logger().info('Coverage goal accepted')
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        try:
            result = future.result()
        except Exception as error:
            self.get_logger().error(f'Coverage result failed: {error}')
            self._publish_status(f'FAILED: {error}')
            self.reset_state()
            self.process_next_field()
            return

        self.status = result.status
        task_result = self.get_result()
        if task_result == TaskResult.SUCCEEDED:
            self.get_logger().info('Coverage goal succeeded')
            self._publish_status('SUCCEEDED')
        elif task_result == TaskResult.CANCELED:
            self.get_logger().warn('Coverage goal canceled')
            self._publish_status('CANCELED')
        else:
            self.get_logger().error('Coverage goal failed')
            self._publish_status('FAILED')

        self.reset_state()
        self.process_next_field()

    def reset_state(self):
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.is_processing = False

    def feedback_callback(self, message):
        self.feedback = message.feedback

    def to_polygon(self, field):
        polygon = Polygon()
        for x_value, y_value in field:
            point = Point32()
            point.x = float(x_value)
            point.y = float(y_value)
            polygon.points.append(point)
        return polygon

    def publish_polygon_marker(self, field):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'selected_polygon'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        for x_value, y_value in field:
            point = Point()
            point.x = float(x_value)
            point.y = float(y_value)
            point.z = 0.0
            marker.points.append(point)
        self.marker_pub.publish(marker)

    def _publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def get_result(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        if self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        if self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        return TaskResult.UNKNOWN

    def is_convex(self, field):
        if len(field) < 3:
            return False

        def cross_product(p1, p2, p3):
            return (
                (p2[0] - p1[0]) * (p3[1] - p1[1])
                - (p2[1] - p1[1]) * (p3[0] - p1[0])
            )

        sign = 0
        count = len(field)
        for index in range(count):
            p1 = field[index]
            p2 = field[(index + 1) % count]
            p3 = field[(index + 2) % count]
            cross = cross_product(p1, p2, p3)
            if math.isclose(cross, 0.0):
                continue
            current_sign = 1 if cross > 0 else -1
            if sign == 0:
                sign = current_sign
            elif sign != current_sign:
                return False
        return True


def main():
    rclpy.init()
    node = GuiCoverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down gui_coverage')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
