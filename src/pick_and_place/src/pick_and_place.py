#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander.robot_trajectory import RobotTrajectory
from moveit_commander.robot_state import RobotState

class PickAndPlace(Node):
    def __init__(self):
        super().__init__('pick_and_place_node')
        self.move_group = MoveGroupCommander("manipulator")
        self.move_group.set_max_velocity_scaling_factor(1.0)

    def pick(self, pick_pose: Pose):
        self.move_group.set_pose_target(pick_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

    def place(self, place_pose: Pose):
        self.move_group.set_pose_target(place_pose)
        success = self.move_group.go(wait=True)
        self.move_group.stop()
        self.move_group.clear_pose_targets()
        return success

def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlace()

    pick_pose = Pose()
    pick_pose.position.x = 0.4
    pick_pose.position.y = 0.0
    pick_pose.position.z = 0.4
    pick_pose.orientation.w = 1.0

    place_pose = Pose()
    place_pose.position.x = 0.6
    place_pose.position.y = 0.0
    place_pose.position.z = 0.4
    place_pose.orientation.w = 1.0

    if node.pick(pick_pose):
        node.get_logger().info("Pick operation successful")
    else:
        node.get_logger().error("Pick operation failed")

    if node.place(place_pose):
        node.get_logger().info("Place operation successful")
    else:
        node.get_logger().error("Place operation failed")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
