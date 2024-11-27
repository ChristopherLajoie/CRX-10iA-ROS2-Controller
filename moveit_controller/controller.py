#!/usr/bin/env python3

import rclpy
import math
import threading
import queue

from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Empty
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from moveit_controller.goal_builder import build_goal_msg
from moveit_controller.socket_utils import setup_socket, socket_server
from moveit_controller.error_codes import error_code_dict, status_code_dict
from moveit_controller.part_manager import GripperPartManager
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient

class MoveitController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        # Define planning parameters
        self.group_name = 'arm'
        self.planner_id = 'BITstarConfigDefault'
        self.num_planning_attempts = 10
        self.allowed_planning_time = 15.0
        self.max_velocity_scaling_factor = 1.0
        self.max_acceleration_scaling_factor = 1.0
        self.plan_only = True

        self.declare_parameter('group_name', self.group_name)
        self.declare_parameter('planner_id', self.planner_id)
        self.declare_parameter('num_planning_attempts', self.num_planning_attempts)
        self.declare_parameter('allowed_planning_time', self.allowed_planning_time)
        self.declare_parameter('max_velocity_scaling_factor', self.max_velocity_scaling_factor)
        self.declare_parameter('max_acceleration_scaling_factor', self.max_acceleration_scaling_factor)
        self.declare_parameter('plan_only', self.plan_only)

        self.group_name = self.get_parameter('group_name').value
        self.planner_id = self.get_parameter('planner_id').value
        self.num_planning_attempts = self.get_parameter('num_planning_attempts').value
        self.allowed_planning_time = self.get_parameter('allowed_planning_time').value
        self.max_velocity_scaling_factor = self.get_parameter('max_velocity_scaling_factor').value
        self.max_acceleration_scaling_factor = self.get_parameter('max_acceleration_scaling_factor').value
        self.plan_only = self.get_parameter('plan_only').value

        self.declare_parameter('joint_goals', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_goals_deg = self.get_parameter('joint_goals').value
        self.joint_goals_rad = [math.radians(angle) for angle in joint_goals_deg]

        self.execution_goal_handle = None
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Socket setup
        self.declare_parameter('ip_address', '127.0.0.1')
        self.declare_parameter('port', 5000)

        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value

        self.get_logger().info(f'IP Address: {self.ip_address}')
        self.get_logger().info(f'Port: {self.port}')

        self.command_queue = queue.Queue()
        self.part_manager_command_pub = self.create_publisher(String, 'gripper_part_command', 10)

        # Initialize socket server
        self.sock = setup_socket(self.ip_address, self.port, self.get_logger())
        if self.sock is None:
            self.get_logger().error('Failed to set up socket server.')
            return

        # Initialize command and response queues
        self.command_queue = queue.Queue()
        self.response_queue = queue.Queue()

        self.socket_thread = threading.Thread(target=socket_server, args=(self.sock, self.get_logger(), self.command_queue, self.response_queue))
        self.socket_thread.daemon = True
        self.socket_thread.start()

        self.create_timer(0.1, self.process_commands)

        # Publisher
        self.update_start_state_publisher = self.create_publisher(
            Empty, '/rviz/moveit/update_start_state', 10)
  
        # Action clients
        self.move_group_action_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def process_commands(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            if command == 'home':
                self.get_logger().info(f"Processing command: {command}")
                self.busy = True
                self.plan_and_execute()
            elif command in ['attach', 'detach']:
                self.part_manager_command_pub.publish(String(data=command))
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

            elif param.name == 'group_name':
                self.group_name = param.value
            elif param.name == 'planner_id':
                self.planner_id = param.value
            elif param.name == 'num_planning_attempts':
                self.num_planning_attempts = param.value
            elif param.name == 'allowed_planning_time':
                self.allowed_planning_time = param.value
            elif param.name == 'max_velocity_scaling_factor':
                self.max_velocity_scaling_factor = param.value
            elif param.name == 'max_acceleration_scaling_factor':
                self.max_acceleration_scaling_factor = param.value
            elif param.name == 'plan_only':
                self.plan_only = param.value
            else:
                self.get_logger().warn(f"Unknown parameter: {param.name}")
                successful = False
        
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

        goal_msg = build_goal_msg(
            joint_positions=self.joint_goals_rad,
            group_name=self.group_name,
            planner_id=self.planner_id,
            num_planning_attempts=self.num_planning_attempts,
            allowed_planning_time=self.allowed_planning_time,
            max_velocity_scaling_factor=self.max_velocity_scaling_factor,
            max_acceleration_scaling_factor=self.max_acceleration_scaling_factor,
            plan_only=self.plan_only
        )

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
            self.busy = False  

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
                self.response_queue.put("failed")
        except Exception as e:
            self.get_logger().error(f'Exception in planning_result_callback: {e}')
            self.response_queue.put("failed")

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
        self._execute_send_goal_future.add_done_callback(self.execution_response_callback) 


    def execution_response_callback(self, future):
        try:
            self.execution_goal_handle = future.result()
            if not self.execution_goal_handle.accepted:
                self.get_logger().error('Execution request rejected!')
                self.busy = False
                return

            self.get_logger().info('Execution request accepted.')
            self.execution_result_future = self.execution_goal_handle.get_result_async()
            self.execution_result_future.add_done_callback(self.execution_result_callback)
        except Exception as e:
            self.get_logger().error(f'Exception in execution_response_callback: {e}')

    def execution_result_callback(self, future):
        try:
            goal_result = future.result()
            if goal_result is None:
                self.get_logger().error('Goal result is None.')
                return

            status = goal_result.status
            status_name = status_code_dict.get(status, f'Unknown status code: {status}')

            result = goal_result.result

            self.get_logger().info(f'Execution result status: {status_name}')
            self.get_logger().info(f'Execution result error code: {result.error_code.val}')

            if status == GoalStatus.STATUS_SUCCEEDED:
                error_code = result.error_code.val
                error_description = error_code_dict.get(
                    error_code, f"Unknown error code: {error_code}")
                if error_code == MoveItErrorCodes.SUCCESS:
                    self.get_logger().info('Execution completed successfully.')
                    # Send success message to client
                    self.response_queue.put("succeeded")
                else:
                    self.get_logger().error(f'Execution failed: {error_description}')
                    # Send failure message to client
                    self.response_queue.put("failed")
            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f'Execution was aborted by the action server. Status: {status_name}')
                # Send aborted message to client
                self.response_queue.put("aborted")
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f'Execution goal was canceled. Status: {status_name}')
                # Send canceled message to client
                self.response_queue.put("canceled")
            else:
                self.get_logger().error(f'Execution failed with status: {status_name}')
                # Send generic failure message to client
                self.response_queue.put("failed")
        except Exception as e:
            self.get_logger().error(f'Exception in execution_result_callback: {e}')
        finally:
            self.execution_result_future = None
            self.execution_goal_handle = None

    def execution_feedback_callback(self, feedback_msg):
        state = feedback_msg.feedback.state
        self.get_logger().info(f'Execution feedback: {state}')
        if 'failed' in state.lower():
            self.get_logger().error('Execution failed according to feedback.')

    def cancel_done_callback(self, future):
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                self.get_logger().info('Goal successfully canceled.')
            else:
                self.get_logger().info('Goal failed to cancel.')
        except Exception as e:
            self.get_logger().error(f'Exception in cancel_done_callback: {e}')

def main(args=None):
    rclpy.init(args=args)
    moveit_controller = MoveitController()
    part_manager = GripperPartManager()
    
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(moveit_controller)
    executor.add_node(part_manager)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        moveit_controller.destroy_node()
        part_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()