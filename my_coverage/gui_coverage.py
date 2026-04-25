"""
gui_coverage.py  –  Coverage action backend
my_coverage | ROS 2

Subscribes to /user_selected_field (Polygon) and sends it to the
navigate_complete_coverage action server.
Also subscribes to /cancel_coverage (Empty) to cancel the active goal.
Publishes status on /coverage_task_status (String).
"""

from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, Point32, Polygon
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Empty, String, Float32
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
        
        # Track total distance for coverage percentage calculation
        self.total_distance = 0.0
        self.initial_distance_remaining = 0.0

        self.coverage_client = ActionClient(
            self, NavigateCompleteCoverage, 'navigate_complete_coverage'
        )
        self.field_sub = self.create_subscription(
            Polygon, '/user_selected_field', self.field_callback, 10
        )
        # Cancel subscription — the GUI drawer publishes here when the
        # "Cancel Cleaning" button is pressed.
        self.cancel_sub = self.create_subscription(
            Empty, '/cancel_coverage', self._cancel_callback, 10
        )
        self.marker_pub = self.create_publisher(
            Marker, '/user_selected_field_marker', 10
        )
        self.status_pub = self.create_publisher(
            String, '/coverage_task_status', 10
        )
        # New publisher for coverage percentage
        self.coverage_percent_pub = self.create_publisher(
            Float32, '/coverage_percent', 10
        )
        # New publisher for coverage feedback info  
        self.coverage_feedback_pub = self.create_publisher(
            String, '/coverage_feedback_info', 10
        )

        self.get_logger().info('GUI coverage backend ready')

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    # ── polygon reception ─────────────────────────────────────────────────── #

    def field_callback(self, msg):
        """
        Accept a closed polygon with ≥ 3 unique corners.
        The polygon sent by polygon_drawer is already closed (first point
        repeated at the end), so a 3-corner room arrives as 4 points.
        Convexity is NOT enforced — opennav_coverage handles arbitrary shapes.
        """
        field = [[pt.x, pt.y] for pt in msg.points]

        # Ensure the polygon is closed
        if len(field) >= 2 and field[0] != field[-1]:
            field.append(field[0])

        unique_corners = len(field) - 1
        if unique_corners < 3:
            self._publish_status(
                f'FAILED: polygon needs at least 3 corners (got {unique_corners})'
            )
            self.get_logger().error(
                f'Polygon rejected: need >= 3 corners, received {unique_corners}'
            )
            return

        self.get_logger().info(
            f'Accepted polygon with {unique_corners} corners; '
            f'queue size: {len(self.pending_fields)}'
        )

        self.pending_fields.append(field)
        self.publish_polygon_marker(field)
        if not self.is_processing:
            self.process_next_field()

    # ── goal processing ───────────────────────────────────────────────────── #

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
            f'Sending coverage goal: {len(field) - 1} corners in frame {goal_msg.frame_id}'
        )
        send_future = self.coverage_client.send_goal_async(
            goal_msg, self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

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
            self.get_logger().error('Coverage goal rejected by action server')
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

    # ── cancel ────────────────────────────────────────────────────────────── #

    def _cancel_callback(self, _msg):
        """Called when /cancel_coverage (Empty) is received from the GUI."""
        if self.goal_handle is None:
            self.get_logger().warn('Cancel requested but no active coverage goal')
            self._publish_status('No active task to cancel')
            return

        self.get_logger().info('Cancel request received — cancelling coverage goal')
        self._publish_status('Cancelling...')

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_done_callback)

    def _cancel_done_callback(self, future):
        try:
            response = future.result()
            # return_code 0 = ERROR_NONE (accepted)
            if response.return_code == 0:
                self.get_logger().info('Cancel request accepted by action server')
            else:
                self.get_logger().warn(
                    f'Cancel request returned code {response.return_code}'
                )
        except Exception as error:
            self.get_logger().error(f'Cancel request failed: {error}')

    # ── helpers ───────────────────────────────────────────────────────────── #

    def reset_state(self):
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None
        self.is_processing = False
        self.total_distance = 0.0
        self.initial_distance_remaining = 0.0
        
        # Reset coverage percentage
        percent_msg = Float32()
        percent_msg.data = 0.0
        self.coverage_percent_pub.publish(percent_msg)

    def feedback_callback(self, message):
        self.feedback = message.feedback
        
        # Capture initial distance on first feedback
        if self.initial_distance_remaining == 0.0 and self.feedback.distance_remaining > 0:
            self.initial_distance_remaining = self.feedback.distance_remaining
            self.total_distance = self.feedback.distance_remaining
        
        # Calculate and publish coverage percentage
        if self.total_distance > 0:
            distance_covered = self.total_distance - self.feedback.distance_remaining
            coverage_percent = (distance_covered / self.total_distance) * 100.0
            coverage_percent = max(0.0, min(100.0, coverage_percent))  # Clamp to 0-100
            
            percent_msg = Float32()
            percent_msg.data = coverage_percent
            self.coverage_percent_pub.publish(percent_msg)
            
            # Also publish detailed feedback info
            feedback_info = f"Coverage: {coverage_percent:.1f}% | Remaining: {self.feedback.distance_remaining:.2f}m | Time: {self.feedback.navigation_time.sec}s"
            info_msg = String()
            info_msg.data = feedback_info
            self.coverage_feedback_pub.publish(info_msg)

    def to_polygon(self, field):
        polygon = Polygon()
        for x_value, y_value in field:
            pt = Point32()
            pt.x = float(x_value)
            pt.y = float(y_value)
            polygon.points.append(pt)
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
        marker.color.g = 0.47
        marker.color.b = 0.84
        marker.color.a = 1.0
        for x_value, y_value in field:
            pt = Point()
            pt.x = float(x_value)
            pt.y = float(y_value)
            pt.z = 0.0
            marker.points.append(pt)
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
