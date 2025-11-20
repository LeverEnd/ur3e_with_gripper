#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
from pymoveit2 import MoveIt2

def main():
    time.sleep(10)
    rclpy.init()
    node = Node("robot_commander_node")

    # MoveIt2 inicializálása
    moveit2 = MoveIt2(
        node=node,
        joint_names=[
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        ],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur_manipulator",
        use_move_group_action=False
    )

    # 1. Mozgás a kezdő pozíció fölé
    start_pose = Pose()
    start_pose.position.x = 0.3
    start_pose.position.y = 0.1
    start_pose.position.z = 0.5
    start_pose.orientation.w = 1.0

    node.get_logger().info("Mozgás a kezdőpozíció fölé...")
    moveit2.move_to_pose(position=[start_pose.position.x,
                                   start_pose.position.y,
                                   start_pose.position.z],
                         quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                         frame_id="base_link")
    moveit2.wait_until_executed()

    # 2. Gripper nyitása
    gripper_pub = node.create_publisher(Bool, '/gripper/command', 10)
    msg = Bool()
    msg.data = True
    gripper_pub.publish(msg)
    node.get_logger().info("Gripper nyitva.")
    rclpy.spin_once(node, timeout_sec=1.0)

    # 3. Leereszkedés a tárgyhoz
    pick_pose = Pose()
    pick_pose.position.x = 0.3
    pick_pose.position.y = 0.1
    pick_pose.position.z = 0.25
    pick_pose.orientation.w = 1.0

    node.get_logger().info("Leereszkedés a tárgyhoz...")
    moveit2.move_to_pose(position=[pick_pose.position.x,
                                   pick_pose.position.y,
                                   pick_pose.position.z],
                         quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                         frame_id="base_link")
    moveit2.wait_until_executed()

    # 4. Gripper zárása
    msg.data = False
    gripper_pub.publish(msg)
    node.get_logger().info("Gripper zárva.")
    rclpy.spin_once(node, timeout_sec=1.0)

    # 5. Tárgy felemelése
    node.get_logger().info("Tárgy felemelése...")
    moveit2.move_to_pose(position=[start_pose.position.x,
                                   start_pose.position.y,
                                   start_pose.position.z],
                         quat_xyzw=[0.0, 0.0, 0.0, 1.0],
                         frame_id="base_link")
    moveit2.wait_until_executed()

    node.get_logger().info("Mozdulatsor sikeresen végrehajtva!")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
