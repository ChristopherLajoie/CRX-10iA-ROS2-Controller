#!/usr/bin/env python3

import rclpy
import time
import math
import threading
import queue

from std_msgs.msg import Empty
from rclpy.node import Node
from moveit_controller.goal_builder import build_goal_msg
from moveit_controller.socket_utils import setup_socket, socket_server
from moveit_controller.error_codes import error_code_dict
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient

class MoveitController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        self.declare_parameter('joint_goals', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_goals_deg = self.get_parameter('joint_goals').value
        self.joint_goals_rad = [math.radians(angle) for angle in joint_goals_deg]

        # Set up parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Variables to track execution result and timeout
        self.execution_result_future = None
        self.execution_result_start_time = None
        self.execution_result_timeout = 15
        self.busy = False

        # Socket setup
        self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('port', 5000)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f'IP Address: {self.ip_address}')
        self.get_logger().info(f'Port: {self.port}')

        # Initialize command queue
        self.command_queue = queue.Queue()

        # Initialize socket server
        self.sock = setup_socket(self.ip_address, self.port, self.get_logger())
        if self.sock is None:
            self.get_logger().error('Failed to set up socket server.')
            return

        self.socket_thread = threading.Thread(target=socket_server, args=(self.sock, self.get_logger(), self.command_queue))
        self.socket_thread.daemon = True
        self.socket_thread.start()

        self.create_timer(0.1, self.process_commands)
        self.create_timer(1.0, self.check_execution_result_timeout)

        # Publisher
        self.update_start_state_publisher = self.create_publisher(
            Empty, '/rviz/moveit/update_start_state', 10)
  
        # Action clients
        self.move_group_action_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def process_commands(self):
        if self.busy:
            # Node is busy, skip processing new commands
            return
        while not self.command_queue.empty():
            command = self.command_queue.get()
            if command == 'home':
                self.get_logger().info(f"Processing command: {command}")
                self.busy = True
                self.plan_and_execute()
            else:
                self.get_logger().info(f"Received unrecognized command: {command}")


    def parameter_callback(self, params):
        from rcl_interfaces.msg import SetParametersResult
        successful = True

        for param in params:
            if param.name == 'joint_goals':
                try:
                    joint_goals_deg = param.value
                    joint_goals_deg = list(joint_goals_deg)

                    self.get_logger().info(f"homing to: {joint_goals_deg}")
                    joint_goals_deg = [float(angle) for angle in joint_goals_deg]

                    # Convert degrees to radians
                    self.joint_goals_rad = [math.radians(angle) for angle in joint_goals_deg]
               
                except Exception as e:
                    successful = False
                    self.get_logger().error(f"Failed to update joint goals: {e}")
            elif param.name == 'ip_address':
                try:
                    ip_address = param.value
                    if not isinstance(ip_address, str):
                        raise ValueError("ip_address must be a string")
                    self.ip_address = ip_address
                    self.get_logger().info(f"Updated ip_address to {self.ip_address}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update ip_address: {e}")
                    successful = False
            elif param.name == 'port':
                try:
                    port = param.value
                    if not isinstance(port, int):
                        raise ValueError("port must be an integer")
                    self.port = port
                    self.get_logger().info(f"Updated port to {self.port}")
                except Exception as e:
                    self.get_logger().error(f"Failed to update port: {e}")
                    successful = False
            else:
                self.get_logger().warn(f"Unknown parameter: {param.name}")

        return SetParametersResult(successful=successful)

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

        goal_msg = build_goal_msg(self.joint_goals_rad)

        self._send_goal_future = self.move_group_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.planning_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.planning_response_callback)

    def planning_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Planning request rejected!')
                return

            self.get_logger().info('Planning request accepted.')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.planning_result_callback)

        except Exception as e:
            self.get_logger().error(f'Exception in planning_response_callback: {e}')
            self.busy = False  # Reset busy flag

    def planning_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Planning feedback: {feedback}')

    def planning_result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
            error_description = error_code_dict.get(error_code, f"Unknown error code: {error_code}")
            if error_code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Planning successful.')
                self.send_execution_request(result.planned_trajectory)
            else:
                self.get_logger().error(f'Planning failed: {error_description}')
                self.busy = False  # Reset busy flag
        except Exception as e:
            self.get_logger().error(f'Exception in planning_result_callback: {e}')
            self.busy = False  # Reset busy flag


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
        self._execute_send_goal_future.add_done_callback(self.execution_response_callback)  # Updated callback


    def execution_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Execution request rejected!')
                return

            self.get_logger().info('Execution request accepted.')
            self.execution_result_future = goal_handle.get_result_async()
            self.execution_result_start_time = time.time()
            self.execution_result_future.add_done_callback(self.execution_result_callback)
        except Exception as e:
            self.get_logger().error(f'Exception in execution_response_callback: {e}')


    def check_execution_result_timeout(self):
        if self.execution_result_future and not self.execution_result_future.done():
            elapsed_time = time.time() - self.execution_result_start_time
            if elapsed_time > self.execution_result_timeout:
                self.get_logger().error('Execution result timeout reached. Cancelling goal.')
                # Cancel the goal if possible
                self.execution_result_future.cancel()
                self.execution_result_future = None
                # Optionally, set a flag to indicate the node is ready for new commands
                self.busy = False
        elif self.execution_result_future and self.execution_result_future.done():
            # Execution result received, no need to check further
            self.execution_result_future = None

    def execution_result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
            error_description = error_code_dict.get(error_code, f"Unknown error code: {error_code}")
            if error_code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Execution completed successfully.')
            else:
                self.get_logger().error(f'Execution failed: {error_description}')
        except Exception as e:
            self.get_logger().error(f'Exception in execution_result_callback: {e}')
        finally:
            # Reset the future and busy flag
            self.execution_result_future = None
            self.busy = False


    def execution_feedback_callback(self, feedback_msg):
        # Log the feedback message for debugging
        self.get_logger().info(f'Execution feedback: {feedback_msg.state}')
        # Check if 'failed' is in the state string
        if 'failed' in feedback_msg.state.lower():
            self.get_logger().error('Execution failed according to feedback.')
            # Optionally cancel the goal
            if self.execution_result_future:
                self.execution_result_future.cancel()
                self.execution_result_future = None
            # Reset busy flag to allow new commands
        self.busy = False



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