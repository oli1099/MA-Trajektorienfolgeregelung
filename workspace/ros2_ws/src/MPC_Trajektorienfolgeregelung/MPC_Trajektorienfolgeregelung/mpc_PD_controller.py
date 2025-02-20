#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from controller.mecanum import MecanumChassis

class MPCTrajectoryController(Node):
    def __init__(self):
        super().__init__('mpc_trajectory_controller')

        # Trajektorie festlegen
        self.trajectory = [
            (0.0000, 0.0000, 0.0000),
            (0.7384, 0.4495, 2.8184),
            (1.6148, 0.6226, 2.9602),
            (2.0703, 1.0683, 2.7809),
            (2.5771, 1.5752, 2.5775),
            (3.0276, 2.0257, 2.3967),
            (3.5362, 2.5309, 2.1942),
            (3.9740, 3.9740, 2.0287),
            (4.0024, 4.0024, 1.7488),
            (4.0024, 4.0024, 1.7488),  
            (4.5836, 4.5836, 1.1525),
            (5.4704, 5.4704, 0.9325),
            (6.4533, 6.4533, 0.7011),
            (7.1901, 7.1901, 0.5210),
            (7.7014, 7.7014, 0.3176),
            (8.1519, 8.1519, 0.1367),
            (8.6587, 8.6587, 0.0013),
            (9.0000, 8.0000, 0.0000)]


        #Startposition und aktuelle Position initzialisieren
        self.start_position = None
        self.current_position = None # enthält x,y
        self.current_orientation = 0

        #Index für die Stützpunkte
        self.waypoints_index = 0
        
        #Toleranz für Wegpunkt erreicht
        self.tolerence = 0.05

        #Regelparameter

        self.k = 1
        self.k_ang = 1

        #Mecanum-Chassis Objekt erstellen
        self.mecanum_chassis = MecanumChassis()

        #ROS2 Publisher (Winkelgeschwindikeiten der vier Räder) und subscriber(Position)
        self.motor_pub = self.create_publisher(MotorsState,'ros_robot_controller/set_motor',10)
        self.get_position = self.create_subscription(Odometry,'odom',self.odom_callback,10)

        self.get_logger().info("Trajektorienfolgeregelung Startet")

    def odom_callback(self,msg: Odometry):
        self.current_position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.current_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if self.start_position is None:
            self.start_position = self.current_position

        self.get_logger().info(f"Roboterposition: x = {self.current_position[0]:.4f}, y = {self.current_position[1]:.4f}, z = {self.current_position[2]:.4f}")

    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        if not self.trajectory:
            self.stop_robot()
            return
        if self.waypoints_index >= len(self.trajectory):
            self.stop_robot()
            return
        
        # Fehlerberechnung

        error_x = self.trajectory[self.waypoints_index][0]-self.current_position[0]
        error_y = self.trajectory[self.waypoints_index][1]-self.current_position[1]
        distance_error = np.sqrt(np.square(error_x) + np.square(error_y))

        error_orientation = self.trajectory[self.waypoints_index][2]-self.current_orientation
        #Normalisieren auf [-pi,pi]
        error_orientation = np.arctan2(np.sin(error_orientation),np.cos(error_orientation))

        #P-Regler
        v = self.k*distance_error
        v_x = v*np.cos(error_orientation)
        v_y = v*np.sin(error_orientation)
        theta = self.k_ang*error_orientation

        #Geschwindigkeit Begrenzung
        v_x = min(v_x,0.25)
        v_y = min(v_y,0.25)
        theta = max(min(theta,1),-1)

        #Geschwindigkeit an Motor übergeben
        motor_v=self.mecanum_chassis.set_velocity(v_x,v_y,theta)
        self.motor_pub.publish(motor_v)

        self.get_logger().info(f"Target: ({self.trajectory[self.waypoints_index][0]:.2f}, {self.trajectory[self.waypoints_index][1]:.2f}), "
                               f"Current: {self.current_position}, "
                               f"Distance Error: {distance_error:.2f}, "
                               f"Angular Error: {error_orientation:.2f}")
        
        #Prüfe ob zielpunkt erreicht?

        if distance_error < self.tolerence:
            self.get_logger().info(f"Waypoint {self.waypoints_index} erreicht.")
            self.waypoints_index += 1
            self.stop_robot()

    def stop_robot(self):
        motor_v=self.mecanum_chassis.set_velocity(0,0,0)
        self.motor_pub.publish(motor_v)

def main(args=None):
    rclpy.init(args=args)
    node = MPCTrajectoryController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    


