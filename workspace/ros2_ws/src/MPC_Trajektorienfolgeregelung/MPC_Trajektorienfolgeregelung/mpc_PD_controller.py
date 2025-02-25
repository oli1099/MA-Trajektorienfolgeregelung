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
        """[(0.0,0,0),(0.5,0,0),(1,0,0),(1,0.5,0.0),(1,1,0)]"""
        # Trajektorie festlegen
        self.trajectory = [(0.0,0,0),(0.5,0.5,0),(1,1,0),(1.5,1,0),(2,1,0),(2,0.5,0),(1,0.5,0),(0,0,0)]
        """[
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
            (9.0000, 8.0000, 0.0000)]"""


        #Startposition und aktuelle Position initzialisieren
        self.start_position = None
        self.current_position = None # enthält x,y
        self.current_orientation = 0

        #Feedforward konstante geschwindigkeit

        self.v_ff= 0.35

        # Liste für aktuelle path
        self.actual_path = []

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
        self.stop_pub = self.create_publisher(Twist,'cmd_vel',10)

        self.timer = self.create_timer(0.1, self.control_loop)
        self.plot_timer = self.create_timer(1, self.plot_callback)
        #self.shutdowntimer = self.create_timer(2,self.stop_robot)

        plt.ion()
        plt.show()
        self.fig ,self.ax = plt.subplots()

        self.get_logger().info("Trajektorienfolgeregelung Startet")

    def odom_callback(self,msg):
        self.current_position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.current_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if self.current_position is not None:
            self.actual_path.append(self.current_position)

        if self.start_position is None:
            self.start_position = self.current_position
            shift_x = self.start_position[0]-self.trajectory[0][0]
            shift_y = self.start_position[1]-self.trajectory[0][1]
            self.trajectory = [(x +shift_x,y +shift_y,theta) for (x,y,theta) in self.trajectory]
            self.get_logger().info(f"Trajectory:{self.trajectory}")

        #self.get_logger().info(f"Roboterposition: x = {self.current_position[0]:.4f}, y = {self.current_position[1]:.4f}, z = {self.current_orientation:.4f}")

    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        if self.current_position is None:
            return
        if self.waypoints_index >= len(self.trajectory):
            self.stop_robot()
            return
        
        # Fehlerberechnung

        error_x = self.trajectory[self.waypoints_index][0]-self.current_position[0]
        error_y = self.trajectory[self.waypoints_index][1]-self.current_position[1]
        distance_error = np.sqrt(np.square(error_x) + np.square(error_y))

        desired_angle = math.atan2(error_y, error_x)

        error_orientation = self.trajectory[self.waypoints_index][2]-self.current_orientation
        #Normalisieren auf [-pi,pi]
        error_orientation = np.arctan2(np.sin(error_orientation),np.cos(error_orientation))

        #FF-Komponente
        v_ff_x = self.v_ff*np.cos(desired_angle)
        v_ff_y = self.v_ff*np.sin(desired_angle)

        
        
        #P-Regler      
        v = self.k*distance_error
        v_fb_x = v*np.cos(desired_angle)
        v_fb_y = v*np.sin(desired_angle)
        theta = self.k_ang*error_orientation

        v_x = v_ff_x +v_fb_x
        v_y = v_ff_y +v_fb_y

        #Geschwindigkeit Begrenzung
        v_x = min(v_x,0.5)
        v_y = min(v_y,0.5)
        theta = max(min(theta,1),-1)

        self.get_logger().info(f"V_x={v_x}, V_y ={v_y}")

        #Geschwindigkeit an Motor übergeben
        motor_v=self.mecanum_chassis.set_velocity(v_x,v_y,theta)
        self.motor_pub.publish(motor_v)

        twist = Twist()
        twist.linear.x = float(v_x)
        twist.linear.y = float(v_y)
        twist.angular.z = float(theta)
        self.stop_pub.publish(twist)

        self.get_logger().info(f"Target: ({self.trajectory[self.waypoints_index][0]:.2f}, {self.trajectory[self.waypoints_index][1]:.2f}), "
                              f"Current: {self.current_position}, "
                           f"Distance Error: {distance_error:.2f}, "
                            f"Angular Error: {error_orientation:.2f}")
        #self.get_logger().info(f"Timer:{self.shutdowntimer}")
        
        #Prüfe ob zielpunkt erreicht?

        if distance_error < self.tolerence:
            self.get_logger().info(f"Waypoint {self.waypoints_index} erreicht.")
            self.waypoints_index += 1
            #self.stop_robot()

    def stop_robot(self):
        motor_stopp  = Twist()
        motor_stopp.linear.x = 0.0 
        motor_stopp.linear.y = 0.0
        motor_stopp.angular.z = 0.0
        self.stop_pub.publish(motor_stopp)
        motor_v=self.mecanum_chassis.set_velocity(0,0,0)
        self.motor_pub.publish(motor_v)
        self.fig.savefig("trajectory_plot1.png")
        #self.shutdowntimer.cancel()

    def plot_callback(self):
        if not self.actual_path:
            return
        
        self.ax.cla()

        # Solltrajektorie extrahieren
        traj_x = [pt[0] for pt in self.trajectory]
        traj_y = [pt[1] for pt in self.trajectory]
        self.ax.plot(traj_x, traj_y, 'r--', label='Solltrajektorie')

        # Tatsächlichen Pfad extrahieren
        path_x = [pt[0] for pt in self.actual_path]
        path_y = [pt[1] for pt in self.actual_path]
        self.ax.plot(path_x, path_y, 'b-', label='Tatsächlicher Weg')

        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.legend()
        self.ax.set_title("Trajektorie vs. tatsächlicher Weg")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = MPCTrajectoryController()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()
    


