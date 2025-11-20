#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
import time
from geometry_msgs.msg import PoseStamped # Fontos: PoseStamped kell!

# Ellenőrizd, hogy a pymoveit2 telepítve van-e forrásból vagy apt-ból
# Ha forrásból:
# from pymoveit2 import MoveIt2, MoveIt2Gripper
# Ha apt-ból, és régebbi pymoveit2 van:
from pymoveit2 import MoveIt2

def main():
    rclpy.init()
    node = Node("ur3e_commander_node")
    logger = get_logger("ur3e_commander_node")

    # Kis várakozás indításkor, hogy a MoveIt biztosan felálljon
    logger.info("Várakozás a MoveIt indítására...")
    time.sleep(10.0)

    # --- MoveIt2 objektumok inicializálása ---
    # Figyelj a group_name nevekre! Ezeknek egyezniük kell az SRDF-fel.
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
    logger.info("MoveIt2 objektumok létrehozva.")

    # --- TESZT: Mozgás egy NEVESÍTETT célba (pl. "home") ---
    # Cseréld le a "home" nevet egy létező, SRDF-ben definiált póz nevére!
    logger.info("Mozgás a 'home' pozícióba...")
    #arm.move_to_configuration("test")
    test_joint_goal = [1.56, -2.34, 0.0, -1.56, -1.56, 0.0]
    arm.move_to_configuration(test_joint_goal)
    arm.wait_until_executed() # Várunk, amíg befejezi

    # --- 1. LÉPÉS: A kar mozgatása egy kiinduló pozíció fölé ---
    logger.info("1. Mozgás a kezdőpozíció fölé...")
    start_pose = PoseStamped() # PoseStamped kell!
    start_pose.header.frame_id = "base_link"
    start_pose.header.stamp = node.get_clock().now().to_msg()
    start_pose.pose.orientation.w = 1.0
    start_pose.pose.position.x = 0.3
    start_pose.pose.position.y = 0.1
    start_pose.pose.position.z = 0.5
    arm.move_to_pose(pose=start_pose) # A metódus neve is más lehet
    arm.wait_until_executed()

    # --- 2. LÉPÉS: A gripper kinyitása (MoveIt-tal) ---
    logger.info("2. Gripper nyitása...")
    #gripper.move_to_configuration("open") # Használjuk a nevesített állapotot!
    open_gripper = [0.0]
    gripper.move_to_configuration(open_gripper)
    gripper.wait_until_executed()
    time.sleep(1.0)

    # --- 3. LÉPÉS: Leereszkedés a tárgyhoz ---
    logger.info("3. Leereszkedés a tárgyhoz...")
    pick_pose_stamped = PoseStamped() # PoseStamped kell!
    pick_pose_stamped.header.frame_id = "base_link"
    pick_pose_stamped.header.stamp = node.get_clock().now().to_msg()
    pick_pose_stamped.pose.orientation.w = 1.0
    pick_pose_stamped.pose.position.x = 0.3
    pick_pose_stamped.pose.position.y = 0.1
    pick_pose_stamped.pose.position.z = 0.25
    arm.move_to_pose(pose=pick_pose_stamped)
    arm.wait_until_executed()

    # --- 4. LÉPÉS: A gripper bezárása (MoveIt-tal) ---
    logger.info("4. Gripper zárása...")
    #gripper.move_to_configuration("close") # Használjuk a nevesített állapotot!
    close_gripper = [0.8]
    gripper.move_to_configuration(close_gripper)
    gripper.wait_until_executed()
    time.sleep(1.0)

    # --- 5. LÉPÉS: A tárgy felemelése ---
    logger.info("5. Tárgy felemelése...")
    arm.move_to_pose(pose=start_pose)
    arm.wait_until_executed()

    logger.info("Mozdulatsor sikeresen végrehajtva!")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
