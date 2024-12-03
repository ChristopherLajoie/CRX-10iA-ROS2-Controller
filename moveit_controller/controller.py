#!/usr/bin/env python3

import rclpy, math, threading, queue, time

from std_msgs.msg import String, Empty
from action_msgs.msg import GoalStatus
from rclpy.node import Node
from moveit_controller.goal_builder import build_goal_msg
from moveit_controller.comm import setup_socket, socket_server, eip_comm
from moveit_controller.error_codes import error_code_dict, status_code_dict
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from rclpy.action import ActionClient
from rcl_interfaces.msg import SetParametersResult

## SELECT COMMUNICATION METHOD ##
SOCKET_MSG = False

class MoveitController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        self.declare_parameter('joint_goals', [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        joint_goals_deg = self.get_parameter('joint_goals').value
        self.joint_goals_rad = [math.radians(angle) for angle in joint_goals_deg]

        self.execution_goal_handle = None
        self.retry_timer = None
        self.previous_part_cmd = None
        self.previous_home_cmd = None

        self.max_retries = 3
        self.plan_retries = 0
        self.exec_retries = 0

        self.declare_parameter('ip_address', '127.0.0.1')
        self.ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self.get_logger().info(f'ip address: {self.ip_address}')

        self.response_queue = queue.Queue()
        self.command_queue = queue.Queue()

        self.create_timer(0.1, self.process_commands)

        if SOCKET_MSG:
            self.declare_parameter('port', 5000)
            self.port = self.get_parameter('port').get_parameter_value().integer_value
            self.get_logger().info(f'Port: {self.port}')

            self.sock = setup_socket(self.ip_address, self.port, self.get_logger())

            self.socket_thread = threading.Thread(target=socket_server, args=(self.sock, self.get_logger(), self.command_queue, self.response_queue))
            self.socket_thread.daemon = True
            self.socket_thread.start()

            if self.sock is None:
                self.get_logger().error('Failed to set up socket server.')
                return
        else:
            self.eip_thread = threading.Thread(target=eip_comm, args=(self.get_logger(), self.ip_address, self.command_queue, self.response_queue))
            self.eip_thread.daemon = True
            self.eip_thread.start()

        self.add_on_set_parameters_callback(self.parameter_callback)

        # Publisher
        self.update_start_state_publisher = self.create_publisher(
            Empty, '/rviz/moveit/update_start_state', 10)
        self.part_manager_command_pub = self.create_publisher(String, 'gripper_part_command', 10)
  
        # Action clients
        self.move_group_action_client = ActionClient(self, MoveGroup, 'move_action')
        self.execute_trajectory_action_client = ActionClient(self, ExecuteTrajectory, 'execute_trajectory')

    def process_commands(self):
        while not self.command_queue.empty():
            command = self.command_queue.get()
            reg_num = command['RegNum']
            value = command['Value']

            if not SOCKET_MSG:
                home_cmd = self.previous_home_cmd != 4 and value == 4 
                attach_cmd = self.previous_part_cmd != 1 and value == 1 
                detach_cmd = self.previous_part_cmd != 2 and value == 2 
                
                if attach_cmd:
                    command = 'attach'
                elif detach_cmd:
                    command = 'detach'

            if home_cmd or (SOCKET_MSG and command == 'home'):
                self.get_logger().info(f"Processing command: {command}")
                self.busy = True
                self.plan_and_execute()

            elif command in ['attach', 'detach']:
                self.part_manager_command_pub.publish(String(data=command))

            elif reg_num == 0:
                  
                if isinstance(value, list) and len(value) == 6 and all(isinstance(v, float) for v in value):
                    self.set_parameters([rclpy.parameter.Parameter('joint_goals', rclpy.Parameter.Type.DOUBLE_ARRAY, value)])
                else:
                    self.get_logger().warn(f"Invalid value for joint_goals: {value}")

            elif SOCKET_MSG:
                self.get_logger().info(f"Received unrecognized command: {command}")

            if reg_num == 5:
                self.previous_home_cmd = value
            elif reg_num == 7:
                self.previous_part_cmd = value
            

    def parameter_callback(self, params):
        successful = True

        for param in params:
            if param.name == 'joint_goals':
                try:
                    joint_goals_deg = param.value
                    joint_goals_deg = list(joint_goals_deg)

                    self.get_logger().info(f"homing to: {joint_goals_deg}")
                    joint_goals_deg = [float(angle) for angle in joint_goals_deg]

                    self.joint_goals_rad = [math.radians(angle) for angle in joint_goals_deg]

                    self.plan_retries = 0
                    self.exex_retries = 0
                except Exception as e:
                    successful = False
                    self.get_logger().error(f"Failed to update joint goals: {e}")
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
        self.get_logger().info('Updated start state to current state')

    def send_planning_request(self):
        if not self.move_group_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('MoveGroup action server not available!')
            self.response_queue.put(2)
            return

        goal_msg = build_goal_msg(
            joint_positions=self.joint_goals_rad
        )

        self._send_goal_future = self.move_group_action_client.send_goal_async(
            goal_msg
        )
        self._send_goal_future.add_done_callback(self.planning_response_callback)

    def planning_response_callback(self, future):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().error('Planning request rejected!')
                return

            self.get_logger().info('Planning request accepted')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.planning_result_callback)

        except Exception as e:
            self.get_logger().error(f'Exception in planning_response_callback: {e}')
            self.busy = False  

    def planning_result_callback(self, future):
        try:
            result = future.result().result
            error_code = result.error_code.val
            error_description = error_code_dict.get(error_code, f"Unknown error code: {error_code}")
            if error_code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Planning successful')
                self.send_execution_request(result.planned_trajectory)
            else:
                self.get_logger().error(f'Planning failed: {error_description}')

                self.plan_retries += 1
                if self.plan_retries < self.max_retries:
                    self.get_logger().info(f'Retrying.. attempt {self.plan_retries}')
                    retry_delay = 4.0 
                    if self.retry_timer is None:
                        self.retry_timer = self.create_timer(
                            retry_delay,
                            self.retry_planning
                        )
                else:
                    self.plan_retries = 0
                    self.get_logger().error(f'Failed after {self.max_retries} retries.')
                    if SOCKET_MSG:
                        self.response_queue.put("failed")
                    else:
                        self.response_queue.put(2)

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
                self.get_logger().error('Goal result is None')
                return

            status = goal_result.status
            status_name = status_code_dict.get(status, f'Unknown status code: {status}')

            result = goal_result.result

            self.get_logger().info(f'Execution result status: {status_name}')
            self.get_logger().info(f'Execution result error code: {result.error_code.val}')

            if status == GoalStatus.STATUS_SUCCEEDED:
                error_code = result.error_code.val
                error_description = error_code_dict.get(error_code, f"Unknown error code: {error_code}")
                if error_code == MoveItErrorCodes.SUCCESS:
                    self.get_logger().info('Execution completed successfully')
                    if SOCKET_MSG:
                        self.response_queue.put("succeeded")
                    else:
                        self.response_queue.put(1)
                else:
                    self.get_logger().error(f'Execution failed: {error_description}')
                        
                    self.exec_retries += 1
                    if self.exec_retries < self.max_retries:
                        self.get_logger().info(f'Retrying execution. Attempt {self.exec_retries}.')
                        self.plan_and_execute()
                    else:
                        self.exec_retries = 0
                        if SOCKET_MSG:
                            self.response_queue.put("failed")
                        else:
                            self.response_queue.put(2)

            elif status == GoalStatus.STATUS_ABORTED:
                self.get_logger().error(f'Execution was aborted by the action server. Status: {status_name}')
                if SOCKET_MSG:
                        self.response_queue.put("aborted")
                else:
                    self.response_queue.put(3)
            elif status == GoalStatus.STATUS_CANCELED:
                self.get_logger().info(f'Execution goal was canceled. Status: {status_name}')
                if SOCKET_MSG:
                        self.response_queue.put("canceled")
                else:
                    self.response_queue.put(4)
            else:
                self.get_logger().error(f'Execution failed with status: {status_name}')
                if SOCKET_MSG:
                        self.response_queue.put("failed")
                else:
                    self.response_queue.put(2)
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

    def retry_planning(self):
        self.retry_timer.cancel()
        self.retry_timer = None

        self.plan_and_execute()

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