#!/usr/bin/env python3

import rclpy
import time

from std_msgs.msg import Empty
from rclpy.node import Node
from moveit_controller.goal_builder import build_goal_msg
from moveit_controller.socket_utils import setup_socket, socket_listener
from moveit_controller.error_codes import error_code_dict
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient

class MoveitController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

         # Socket setup
        self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('port', 5000)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f'IP Address: {self.ip_address}')
        self.get_logger().info(f'Port: {self.port}')

        self.conn, self.addr = setup_socket(self.ip_address, self.port, self.get_logger())

        # Publisher
        self.update_start_state_publisher = self.create_publisher(
            Empty, '/rviz/moveit/update_start_state', 10)
  
        # Action clients
        self.move_group_action_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

        self.create_timer(0.1, self.socket_listener_wrapper)

    def socket_listener_wrapper(self):
        socket_listener(self.conn, self.get_logger(), self.plan_and_execute)

    def plan_and_execute(self):
        self.update_start_state()

        self.get_logger().info('Sending planning request...')
        self.send_planning_request()

    def update_start_state(self):
        self.update_start_state_publisher.publish(Empty())
        self.get_logger().info('Updated start state to current state.')

    def send_planning_request(self):
        if not self.move_group_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            return

        goal_msg = build_goal_msg()

        self._send_goal_future = self.move_group_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.planning_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.planning_response_callback)

    def planning_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Planning request rejected!')
            return

        self.get_logger().info('Planning request accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.planning_result_callback)

    def planning_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Planning feedback: {feedback}')

    def planning_result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val
        error_description = error_code_dict.get(error_code, f"Unknown error code: {error_code}")
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Planning successful.')
         
            self.send_execution_request(result.planned_trajectory)
        else:
            self.get_logger().error(f'Planning failed: {error_description}')

    def send_execution_request(self, trajectory):
        if not self.execute_trajectory_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('ExecuteTrajectory action server not available!')
            return

        goal_msg = ExecuteTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self._execute_send_goal_future = self.execute_trajectory_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.execution_feedback_callback
        )
        self._execute_send_goal_future.add_done_callback(self.execution_result_callback)

    def execution_result_callback(self, future):
        goal_handle = future.result()
  
        self.wait(0.5)
        result = goal_handle.get_result().result
        error_code = result.error_code.val
        error_description = self.error_code_dict.get(error_code, f"Unknown error code: {error_code}")
        if error_code == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('Execution completed successfully.')
        else:
            self.get_logger().error(f'Execution failed: {error_description}')

    def execution_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Execution feedback: {feedback}')

    def wait(self, duration):
        start_time = time.time()
        end_time = start_time + duration
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    moveit_controller = MoveitController()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(moveit_controller)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        moveit_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()