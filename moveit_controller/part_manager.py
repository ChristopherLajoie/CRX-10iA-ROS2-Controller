#!/usr/bin/env python3

import rclpy
import os
import trimesh
import math

from std_msgs.msg import String
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject, AttachedCollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point
from transformations import quaternion_from_euler


class GripperPartManager(Node):
    def __init__(self):
        super().__init__('part_manager')
        self.scene_publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.command_subscriber = self.create_subscription(
            String,
            'gripper_part_command',
            self.command_callback,
            10
        )

        self.get_logger().info('PartManager node started.')

    def command_callback(self, msg):
        command = msg.data
        if command == 'attach':
            self.get_logger().info(f"Processing command: {command}")
            self.attach_part()
        elif command == 'detach':
            self.get_logger().info(f"Processing command: {command}")
            self.detach_part()
        else:
            self.get_logger().warn(f"Received unrecognized command: {command}")

    def attach_part(self):
        collision_object = CollisionObject()
        collision_object.id = "part"
        collision_object.header.frame_id = "gripper_link"

        mesh_path = os.path.join(os.path.dirname(__file__), 'meshes', '/home/chris/ros2_ws/src/fanuc_ros2_driver/crx_description/share/crx_description/meshes/crx10ia_l/visual/cardboard_box.stl')
        mesh = self.load_mesh(mesh_path)

        pose = Pose()
        pose.position.x = 0.15  
        pose.position.y = 0.025
        pose.position.z = -0.015

        angle_rad = math.radians(45)
        q = quaternion_from_euler(0, 0, angle_rad)  

        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]

        collision_object.meshes = [mesh]
        collision_object.mesh_poses = [pose]
        collision_object.operation = CollisionObject.ADD

        attached_object = AttachedCollisionObject()
        attached_object.object = collision_object
        attached_object.link_name = "gripper_link"
        attached_object.touch_links = ["gripper_link", "part"]  

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.robot_state.is_diff = True

        self.scene_publisher.publish(planning_scene)
        self.get_logger().info('Part attached to gripper.')

    def load_mesh(self, file_path):
        mesh = trimesh.load(file_path)
        mesh_msg = Mesh()

        for face in mesh.faces:
            triangle = MeshTriangle(vertex_indices=[face[0], face[1], face[2]])
            mesh_msg.triangles.append(triangle)

        for vertex in mesh.vertices:
            point = Point(x=vertex[0], y=vertex[1], z=vertex[2])
            mesh_msg.vertices.append(point)

        return mesh_msg

    def detach_part(self):
        attached_object = AttachedCollisionObject()
        attached_object.object.id = "part"
        attached_object.object.operation = CollisionObject.REMOVE
        attached_object.link_name = "gripper_link"

        collision_object = CollisionObject()
        collision_object.id = "part"
        collision_object.operation = CollisionObject.REMOVE

        planning_scene = PlanningScene()
        planning_scene.is_diff = True
        planning_scene.robot_state.is_diff = True
        planning_scene.robot_state.attached_collision_objects.append(attached_object)
        planning_scene.world.collision_objects.append(collision_object)

        self.scene_publisher.publish(planning_scene)
        self.get_logger().info('Part detached from gripper.')

    def apply_planning_scene(self, planning_scene):
        request = ApplyPlanningScene.Request()
        request.scene = planning_scene
        future = self.apply_scene_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() and future.result().success:
            self.get_logger().info('Planning scene applied successfully.')
        else:
            self.get_logger().warn('Failed to apply planning scene.')

def main(args=None):
    rclpy.init(args=args)
    node = GripperPartManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()