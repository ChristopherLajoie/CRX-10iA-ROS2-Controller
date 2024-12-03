import rclpy, trimesh, os, sys, glob

from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle
from geometry_msgs.msg import Point, Pose


def main(args=None):
    rclpy.init(args=args)
    node = Node('add_mesh_obstacle_node')

    if len(sys.argv) < 2:
        node.get_logger().error("Please provide a directory path containing STL files.")
        rclpy.shutdown()
        return
    
    folder_path = sys.argv[1]

    if not os.path.isdir(folder_path):
        node.get_logger().error(f"{folder_path} is not a valid directory.")
        rclpy.shutdown()
        return

    scale_factor = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0  

    node.get_logger().info(f"Using initial scale factor for the first file: {scale_factor}")

    client = node.create_client(ApplyPlanningScene, 'apply_planning_scene')

    node.get_logger().info("Checking if apply_planning_scene service is available...")
    if client.wait_for_service(timeout_sec=10.0):
        node.get_logger().info("Service is available!")
    else:
        node.get_logger().error("Service not available after 10 seconds, exiting.")
        rclpy.shutdown()
        return

    planning_scene = PlanningScene()
    planning_scene.is_diff = True

    stl_files = glob.glob(os.path.join(folder_path, "*.stl"))

    if not stl_files:
        node.get_logger().error(f"No STL files found in {folder_path}.")
        rclpy.shutdown()
        return

    for idx, stl_file in enumerate(stl_files):
        node.get_logger().info(f"Loading mesh from {stl_file}")

        if idx > 0:
            try:
                scale_input = input(f"Enter scale factor for {os.path.basename(stl_file)} (default: 1.0): ")
                scale_factor = float(scale_input) if scale_input.strip() else 1.0
            except ValueError:
                node.get_logger().warn(f"Invalid input for scale factor, using default: 1.0")
                scale_factor = 1.0

        node.get_logger().info(f"Using scale factor: {scale_factor}")

        collision_object = CollisionObject()
        collision_object.id = os.path.basename(stl_file).replace('.stl', '_obstacle')
        collision_object.header.frame_id = "map"  

        mesh_pose = Pose()
        mesh_pose.position.x = 0.0
        mesh_pose.position.y = 0.0
        mesh_pose.position.z = 0.0  
        mesh_pose.orientation.w = 1.0

        collision_object.mesh_poses.append(mesh_pose)

        mesh = load_mesh(stl_file, node, scale_factor)

        collision_object.meshes.append(mesh)
        collision_object.operation = CollisionObject.ADD

        planning_scene.world.collision_objects.append(collision_object)

    request = ApplyPlanningScene.Request()
    request.scene = planning_scene
    future = client.call_async(request)

    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('All mesh obstacles added to the planning scene')
    else:
        node.get_logger().error('Failed to add mesh obstacles')

    rclpy.shutdown()

def load_mesh(mesh_path, node, scale_factor):

    mesh_trimesh = trimesh.load(mesh_path)

    node.get_logger().info(f'Mesh loaded with {len(mesh_trimesh.vertices)} vertices and {len(mesh_trimesh.faces)} faces')

    mesh = Mesh()

    node.get_logger().info(f'Applying scale factor: {scale_factor}')

    for vertex in mesh_trimesh.vertices:
        point = Point(x=vertex[0] * scale_factor, y=vertex[1] * scale_factor, z=vertex[2] * scale_factor)
        mesh.vertices.append(point)

    for face in mesh_trimesh.faces:
        triangle = MeshTriangle()
        triangle.vertex_indices[0] = int(face[0])
        triangle.vertex_indices[1] = int(face[1])
        triangle.vertex_indices[2] = int(face[2])
        mesh.triangles.append(triangle)

    return mesh

if __name__ == '__main__':
    main()
