import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped
from moveit_ros_planning_interface.move_group_interface import MoveGroupInterface
from moveit_configs_utils import MoveItConfigsBuilder
from assistive_handover.msg import Handover
from assistive_handover.action import RequestHandover
import time

class HandoverManager(Node):
    def __init__(self):
        super().__init__('handover_manager')
        self.declare_parameter('robot_model', 'vx300s')
        robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        
        self.get_logger().info(f"Loading MoveIt for '{robot_model}'...")
        
        moveit_config = (
            MoveItConfigsBuilder(robot_model, package_name=f"{robot_model}_moveit_config")
            .robot_description(file_path=f"config/{robot_model}.urdf.xacro")
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(pipelines=["ompl"])
            .to_moveit_configs()
        )

        self.move_group = MoveGroupInterface(self, "interbotix_arm", "base_link", moveit_config)
        self.eef_link = self.move_group.get_end_effector_link()
        
        self.target_object_pose = None
        self.target_hand_pose = None
        
        self.object_sub = self.create_subscription(
            Handover, '/handover/object_pose', self.object_pose_callback, 10)
        self.hand_sub = self.create_subscription(
            Handover, '/handover/hand_pose', self.hand_pose_callback, 10)
            
        self.action_server = ActionServer(
            self, RequestHandover, 'request_handover', self.execute_handover)
            
        self.get_logger().info('Handover Manager is ready.')

    def object_pose_callback(self, msg):
        self.target_object_pose = msg.pose
        self.get_logger().info(f"Received object '{msg.object_id}' pose.", throttle_duration_sec=5.0)

    def hand_pose_callback(self, msg):
        self.target_hand_pose = msg.pose
        self.get_logger().info(f"Received '{msg.object_id}' pose.", throttle_duration_sec=5.0)

    def execute_handover(self, goal_handle):
        self.get_logger().info(f"Handover requested for: {goal_handle.request.object_name}")
        feedback = RequestHandover.Feedback()
        
        # 1. Check if we have a pose for the requested object
        if self.target_object_pose is None or self.target_object_pose.header.frame_id != goal_handle.request.object_name:
            self.get_logger().error(f"No pose for '{goal_handle.request.object_name}'. Aborting.")
            goal_handle.abort()
            return RequestHandover.Result(success=False, message="Object not found.")
            
        object_pose = self.target_object_pose

        # 2. Go to a pre-grasp position (above the object)
        feedback.status = "Moving to pre-grasp position..."
        goal_handle.publish_feedback(feedback)
        
        pre_grasp_pose = self.clone_pose(object_pose, z_offset=0.1)
        if not self.move_to_pose(pre_grasp_pose):
            goal_handle.abort()
            return RequestHandover.Result(success=False, message="Pre-grasp move failed.")

        # 3. Open gripper
        feedback.status = "Opening gripper..."
        goal_handle.publish_feedback(feedback)
        self.move_gripper(1.0) # 1.0 = fully open
        time.sleep(1.0)

        # 4. Move to grasp
        feedback.status = "Moving to grasp..."
        goal_handle.publish_feedback(feedback)
        
        grasp_pose = self.clone_pose(object_pose, z_offset=0.01) # Small offset
        if not self.move_to_pose(grasp_pose):
            goal_handle.abort()
            return RequestHandover.Result(success=False, message="Grasp move failed.")

        # 5. Close gripper
        feedback.status = "Grasping object..."
        goal_handle.publish_feedback(feedback)
        self.move_gripper(0.0) # 0.0 = fully closed
        time.sleep(1.0)
        
        # 6. Lift object
        feedback.status = "Lifting object..."
        goal_handle.publish_feedback(feedback)
        lift_pose = self.clone_pose(pre_grasp_pose)
        if not self.move_to_pose(lift_pose):
            goal_handle.abort()
            return RequestHandover.Result(success=False, message="Lift move failed.")
            
        # 7. Wait for hand
        feedback.status = "Waiting for user's hand..."
        goal_handle.publish_feedback(feedback)
        while self.target_hand_pose is None:
            if not goal_handle.is_active:
                return RequestHandover.Result() # Canceled
            self.get_logger().info("Waiting for hand...", throttle_duration_sec=2.0)
            time.sleep(0.5)
            
        # 8. Move to handover pose (near the hand)
        feedback.status = "Moving to hand..."
        goal_handle.publish_feedback(feedback)
        
        handover_pose = self.clone_pose(self.target_hand_pose, z_offset=0.1)
        if not self.move_to_pose(handover_pose):
            goal_handle.abort()
            return RequestHandover.Result(success=False, message="Move to hand failed.")

        # 9. Open gripper
        feedback.status = "Releasing object..."
        goal_handle.publish_feedback(feedback)
        self.move_gripper(1.0)
        time.sleep(1.0)
        
        # 10. Retreat
        feedback.status = "Retreating..."
        goal_handle.publish_feedback(feedback)
        retreat_pose = self.clone_pose(handover_pose, x_offset=-0.1)
        self.move_to_pose(retreat_pose)

        # 11. Go home
        self.move_to_named_pose("Home")
        
        goal_handle.succeed()
        return RequestHandover.Result(success=True, message="Handover complete.")

    def move_to_pose(self, pose):
        self.move_group.set_pose_target(pose, self.eef_link)
        plan = self.move_group.plan()
        if not plan.joint_trajectory.points:
            self.get_logger().error("Failed to plan path.")
            return False
        return self.move_group.execute(plan)

    def move_to_named_pose(self, pose_name):
        self.move_group.set_named_target(pose_name)
        plan = self.move_group.plan()
        if not plan.joint_trajectory.points:
            self.get_logger().error(f"Failed to plan path to '{pose_name}'.")
            return False
        return self.move_group.execute(plan)

    def move_gripper(self, position):
        gripper_move_group = MoveGroupInterface(self, "interbotix_gripper", "base_link")
        gripper_move_group.set_joint_value_target([position, position])
        plan = gripper_move_group.plan()
        if not plan.joint_trajectory.points:
            self.get_logger().error("Failed to plan gripper path.")
            return False
        return gripper_move_group.execute(plan)
        
    def clone_pose(self, pose, x_offset=0.0, y_offset=0.0, z_offset=0.0):
        new_pose = PoseStamped()
        new_pose.header = pose.header
        new_pose.pose = pose.pose
        new_pose.pose.position.x += x_offset
        new_pose.pose.position.y += y_offset
        new_pose.pose.position.z += z_offset
        return new_pose

def main(args=None):
    rclpy.init(args=args)
    node = HandoverManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()