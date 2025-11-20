#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import time
from geometry_msgs.msg import PoseStamped
from pymoveit2 import MoveIt2
import math

def main():
    rclpy.init()
    node = Node("ur3e_commander_node")
    logger = get_logger("ur3e_commander_node")
    logger.info("Várakozás a MoveIt indítására...")
    time.sleep(5.0)

    # --- MoveIt2 objektumok inicializálása ---
    arm = MoveIt2(
        node=node,
        joint_names=[ # Ezek a nevek az URDF/SRDF alapján kellenek
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"],
        base_link_name="base_link",
        end_effector_name="tool0",
        group_name="ur3e", # Ez a KAR tervezési csoportja
    )

    gripper = MoveIt2( # Külön objektum a gripperhez
        node=node,
        joint_names=["robotiq_85_left_knuckle_joint"], # Csak a vezérelt joint kell ide
        base_link_name="base_link", # Lehet a wrist_3_link is, de base_link általában jó
        end_effector_name="tool0", # Vagy a gripper egy releváns linkje
        group_name="robotiq_gripper", # Ez a GRIPPER tervezési csoportja
    )

    CUBE_1_NAME = "kocka_1" # Ennek meg kell egyeznie az SDF <model name="..."> nevével
    cube_1_size = [0.07, 0.07, 0.07]
    cube_1_pos = Pose()
    cube_1_pos.position.x = 0.58  # X az SDF-ből
    cube_1_pos.position.y = 0.38  # Y az SDF-ből
    cube_1_pos.position.z = 0.035 # Kocka közepének Z-je
    cube_1_pos.orientation.z = math.sin(0.5)
    cube_1_pos.orientation.w = math.cos(0.5)

    cube1_1 = [-2.44, -3.14, 0.26, -3.14, -1.74, 0.0]
    arm.move_to_configuration(cube1_1)
    arm.wait_until_executed()

    cube1_2 = [-2.44, -3.14, 0.0, -3.14, -1.74, 0.0]
    arm.move_to_configuration(cube1_2)
    arm.wait_until_executed()

    #close = [0.4]
    #gripper.move_to_configuration(close_1)
    #gripper.wait_until_executed()

    cube1_3 = [-2.44, -3.14, 0.26, -3.14, -1.74, 0.0]
    arm.move_to_configuration(cube1_3)
    arm.wait_until_executed()

    base1_1 = [-4.71, -2.35, -1.57, -2.35, -1.57, 0.0]
    arm.move_to_configuration(base1_1)
    arm.wait_until_executed()

    #open = [0.8]
    #gripper.move_to_configuration(open)
    #gripper.wait_until_executed()

    base1_2 = [-4.71, -2.35, -1.83, -1.57, -1.57, 0.0]
    arm.move_to_configuration(base1_2)
    arm.wait_until_executed()

    cube2_1 = [-1.6, -2.35, -1.57, -2.35, -1.74, 0.0]
    arm.move_to_configuration(cube2_1)
    arm.wait_until_executed()

    cube2_2 = [-1.6, -2.55, -1.37, -2.35, -1.74, 0.0]
    arm.move_to_configuration(cube2_2)
    arm.wait_until_executed()

    #gripper.move_to_configuration(close)
    #gripper.wait_until_executed()

    cube2_3 = [-1.6, -2.55, -1.37, -2.09, -1.74, 0.0]
    arm.move_to_configuration(cube2_3)
    arm.wait_until_executed()

    base2_1 = [-4.71, -2.15, -1.77, -2.35, -1.57, 0.0]
    arm.move_to_configuration(base2_1)
    arm.wait_until_executed()

    #gripper.move_to_configuration(open)
    #gripper.wait_until_executed()

    base2_2 = [-4.71, -2.15, -2.03, -1.57, -1.57, 0.0]
    arm.move_to_configuration(base2_2)
    arm.wait_until_executed()

    cube3_1 = [1.75, -1.57, 1.83, -2.09, -1.57, 2.27]
    arm.move_to_configuration(cube3_1)
    arm.wait_until_executed()

    cube3_2 = [1.75, -1.3, 1.83, -2.09, -1.57, 2.27]
    arm.move_to_configuration(cube3_2)
    arm.wait_until_executed()

    #gripper.move_to_configuration(close)
    #gripper.wait_until_executed()

    cube3_3 = [1.75, -1.3, 1.57, -2.09, -1.57, 2.27]
    arm.move_to_configuration(cube3_3)
    arm.wait_until_executed()

    base3_1 = [1.57, -1.83, -1.48, -0.52, 1.57, 0.0]
    arm.move_to_configuration(base3_1)
    arm.wait_until_executed()

    #gripper.move_to_configuration(open)
    #gripper.wait_until_executed()

    base3_2 = [1.57, -1.83, -1.04, -0.52, 1.57, 0.0]
    arm.move_to_configuration(base3_2)
    arm.wait_until_executed()

    stand = [0.0, -1.57, 0.0, 0.0, 0.0, 1.57]
    arm.move_to_configuration(stand)
    arm.wait_until_executed()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
