#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, roscpp_initialize
from geometry_msgs.msg import Pose
import sys

def main():
    # ROS 2 és MoveIt Commander inicializálása
    rclpy.init(args=sys.argv)
    roscpp_initialize(sys.argv)
    
    # Egy egyszerű Node létrehozása
    node = Node("robot_commander_node")

    # --- MoveGroupCommander inicializálása a KARHOZ és a GRIPPERHEZ ---
    # A neveknek ("ur_manipulator", "hand") meg kell egyezniük a MoveIt konfigurációdban lévőkkel!
    arm_group = MoveGroupCommander(node, "ur_manipulator")
    gripper_group = MoveGroupCommander(node, "hand")

    # --- 1. LÉPÉS: A kar mozgatása egy kiinduló pozíció fölé ---
    print("1. Mozgás a kezdőpozíció fölé...")
    start_pose = Pose()
    start_pose.orientation.w = 1.0
    start_pose.position.x = 0.3
    start_pose.position.y = 0.1
    start_pose.position.z = 0.5  # Magasan a tárgy felett
    arm_group.set_pose_target(start_pose)
    
    if not arm_group.go(wait=True):
        print("A kezdőpozíció elérése sikertelen!")
        rclpy.shutdown()
        return
    arm_group.stop()

    # --- 2. LÉPÉS: A gripper kinyitása ---
    print("2. Gripper nyitása...")
    gripper_group.set_named_target("open")
    if not gripper_group.go(wait=True):
        print("A gripper nyitása sikertelen!")
        rclpy.shutdown()
        return
    gripper_group.stop()
    time.sleep(1) # Kis szünet

    # --- 3. LÉPÉS: Leereszkedés a tárgyhoz ---
    print("3. Leereszkedés a tárgyhoz...")
    pick_pose = Pose()
    pick_pose.orientation.w = 1.0
    pick_pose.position.x = 0.3
    pick_pose.position.y = 0.1
    pick_pose.position.z = 0.25 # Alacsonyabban, a "tárgy" szintjén
    arm_group.set_pose_target(pick_pose)

    if not arm_group.go(wait=True):
        print("A tárgyhoz ereszkedés sikertelen!")
        rclpy.shutdown()
        return
    arm_group.stop()

    # --- 4. LÉPÉS: A gripper bezárása (tárgy megfogása) ---
    print("4. Gripper zárása...")
    gripper_group.set_named_target("close")
    if not gripper_group.go(wait=True):
        print("A gripper zárása sikertelen!")
        rclpy.shutdown()
        return
    gripper_group.stop()
    time.sleep(1) # Kis szünet, hogy a fogás stabil legyen

    # --- 5. LÉPÉS: A tárgy felemelése ---
    print("5. Tárgy felemelése...")
    arm_group.set_pose_target(start_pose) # Vissza a magasabb pozícióba
    if not arm_group.go(wait=True):
        print("A tárgy felemelése sikertelen!")
        rclpy.shutdown()
        return
    arm_group.stop()

    print("Mozdulatsor sikeresen végrehajtva!")

    # ROS 2 leállítása
    roscpp_shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
