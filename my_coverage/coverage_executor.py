import time
from enum import Enum

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point32, Polygon
from lifecycle_msgs.srv import GetState
from opennav_coverage_msgs.action import NavigateCompleteCoverage
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node


class TaskResult(Enum):
    UNKNOWN = 0
    SUCCEEDED = 1
    CANCELED = 2
    FAILED = 3


class CoverageExecutor(Node):
    def __init__(self):
        super().__init__('coverage_executor')
        self.goal_handle = None
        self.result_future = None
        self.status = None
        self.feedback = None

        self.declare_parameter('bt_navigator_name', 'bt_navigator')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('polygon', rclpy.Parameter.Type.DOUBLE_ARRAY)

        self.coverage_client = ActionClient(
            self,
            NavigateCompleteCoverage,
            'navigate_complete_coverage',
        )

    def destroy_node(self):
        self.coverage_client.destroy()
        super().destroy_node()

    def _to_polygon(self, coordinates):
        polygon = Polygon()
        closed_coordinates = list(coordinates)
        if closed_coordinates and closed_coordinates[0] != closed_coordinates[-1]:
            closed_coordinates.append(closed_coordinates[0])
            self.get_logger().info('Closing polygon automatically by repeating the first point')

        for x_value, y_value in closed_coordinates:
            point = Point32()
            point.x = float(x_value)
            point.y = float(y_value)
            polygon.points.append(point)
        return polygon

    def _polygon_from_parameter(self):
        values = list(self.get_parameter('polygon').value)
        if len(values) < 6 or len(values) % 2 != 0:
            return []
        return [(values[index], values[index + 1]) for index in range(0, len(values), 2)]

    def wait_for_bt_navigator(self):
        navigator_name = self.get_parameter('bt_navigator_name').value
        service_name = f'{navigator_name}/get_state'
        self.get_logger().info(f'Waiting for {navigator_name} to become active...')
        state_client = self.create_client(GetState, service_name)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{service_name} not available yet...')

        request = GetState.Request()
        state = 'unknown'
        while state != 'active':
            future = state_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info(f'{navigator_name} state: {state}')
            time.sleep(1.0)

    def send_goal(self, field):
        self.get_logger().info('Waiting for navigate_complete_coverage action server...')
        while not self.coverage_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('navigate_complete_coverage not available yet...')

        goal = NavigateCompleteCoverage.Goal()
        goal.frame_id = self.get_parameter('frame_id').value
        goal.polygons.clear()
        goal.polygons.append(self._to_polygon(field))

        self.get_logger().info(f'Sending coverage goal with {len(field)} polygon points')
        send_goal_future = self.coverage_client.send_goal_async(goal, self._feedback_callback)
        rclpy.spin_until_future_complete(self, send_goal_future)
        self.goal_handle = send_goal_future.result()

        if not self.goal_handle.accepted:
            self.get_logger().error('Coverage goal was rejected')
            return False

        self.result_future = self.goal_handle.get_result_async()
        return True

    def is_complete(self):
        if not self.result_future:
            return True

        rclpy.spin_until_future_complete(self, self.result_future, timeout_sec=0.1)
        if not self.result_future.result():
            return False

        self.status = self.result_future.result().status
        return True

    def result(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return TaskResult.SUCCEEDED
        if self.status == GoalStatus.STATUS_ABORTED:
            return TaskResult.FAILED
        if self.status == GoalStatus.STATUS_CANCELED:
            return TaskResult.CANCELED
        return TaskResult.UNKNOWN

    def _feedback_callback(self, message):
        self.feedback = message.feedback


def main():
    rclpy.init()
    executor = CoverageExecutor()
    field = executor._polygon_from_parameter()

    if not field:
        executor.get_logger().error(
            'Set the polygon parameter as a flat list: '
            '[x1, y1, x2, y2, x3, y3, ...]'
        )
        executor.destroy_node()
        rclpy.shutdown()
        return

    executor.wait_for_bt_navigator()
    if not executor.send_goal(field):
        executor.destroy_node()
        rclpy.shutdown()
        return

    counter = 0
    while not executor.is_complete():
        counter += 1
        feedback = executor.feedback
        if feedback and counter % 5 == 0:
            seconds = Duration.from_msg(
                feedback.estimated_time_remaining
            ).nanoseconds / 1e9
            executor.get_logger().info(
                f'Estimated time remaining: {seconds:.0f} seconds'
            )
        time.sleep(1.0)

    result = executor.result()
    if result == TaskResult.SUCCEEDED:
        executor.get_logger().info('Coverage goal succeeded')
    elif result == TaskResult.CANCELED:
        executor.get_logger().warn('Coverage goal canceled')
    elif result == TaskResult.FAILED:
        executor.get_logger().error('Coverage goal failed')
    else:
        executor.get_logger().warn('Coverage goal finished with unknown status')

    executor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
