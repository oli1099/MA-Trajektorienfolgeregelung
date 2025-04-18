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
from MPC.SystemModel import DynamicModel

class TrajectoryPController(Node):
    def __init__(self):
        super().__init__('trajectory_pcontroller')
        """[(0.0,0,0),(0.5,0,0),(1,0,0),(1,0.5,0.0),(1,1,0)]"""
        # Trajektorie festlegen
        self.trajectory = [(0,0,0),(0.5,0,0),(1,0.75,0),(1.5,1,0),(2,1,0),(2.5,1,0),(3,0.75,0),(3.5,0,0),(4,0,0)]
        
        #Zeitliste erstellen
        self.total_time = 30
        self.num_waypoints = len(self.trajectory)
        self.times = [i*(self.total_time/(self.num_waypoints -1)) for i in range(self.num_waypoints)]

        self.start_timer = None


        #Startposition und aktuelle Position initzialisieren
        self.start_position = None
        self.current_position = None # enthält x,y
        self.current_orientation = 0

        #Feedforward konstante geschwindigkeit

        self.v_ff= 0

        # Liste für aktuelle path
        self.mpc_model = DynamicModel()
        self.actual_path = []
        self.actual_u = []


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
        self.fig_u, self.ax_u = plt.subplots()
        
    def odom_callback(self,msg):
        self.current_position = (msg.pose.pose.position.x,msg.pose.pose.position.y)
        self.current_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)

        if self.current_position is not None:
            self.actual_path.append(self.current_position)

        if self.start_timer is None:
            self.start_timer = self.get_clock().now()

        #self.get_logger().info(f"Roboterposition: x = {self.current_position[0]:.4f}, y = {self.current_position[1]:.4f}, z = {self.current_orientation:.4f}")

    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def control_loop(self):
        if self.current_position is None or self.start_timer is None:
            return
        
        #Aktuelle Zeit bekommen
        current_time = (self.get_clock().now() - self.start_timer).nanoseconds * 1e-9

        if self.waypoints_index >= len(self.trajectory) or current_time >= self.total_time:
            self.stop_robot()
            return
        # Zeitsegment bestimmen
        while(self.waypoints_index < len(self.times) -1 and 
              current_time > self.times[self.waypoints_index+1]):
            self.waypoints_index += 1

        # Falls am Ende letzte Punkt als Sollpunkt
        if self.waypoints_index >= len(self.trajectory):
            desired_state = self.trajectory[len(self.trajectory)]
        else:
            #Interpolation zwischen den wegpunkten
            t_i = self.times[self.waypoints_index]
            t_i1 =self.times[self.waypoints_index +1]
            ratio =  (current_time - t_i) / (t_i1 - t_i) if (t_i1 - t_i) != 0 else 0.0  

            x_i, y_i, theta_i = self.trajectory[self.waypoints_index]
            x_i1, y_i1, theta_i1 = self.trajectory[self.waypoints_index + 1]
            
            #Lineare Interpolation
            x_des = x_i + ratio*(x_i1 - x_i)
            y_des = y_i + ratio * (y_i1 - y_i)
            theta_des = theta_i + ratio * (theta_i1 - theta_i)
            desired_state = (x_des, y_des, theta_des)
              
        

        # Fehlerberechnung

        error_x = desired_state[0] - self.current_position[0]
        error_y = desired_state[1] - self.current_position[1]
        distance_error = np.sqrt(np.square(error_x) + np.square(error_y))

        desired_angle = math.atan2(error_y, error_x)

        error_orientation = desired_state[2]-self.current_orientation
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
        v_x = max(min(v_x, 0.2298), -0.22989)
        v_y = max(min(v_y, 0.2298), -0.2298)
       

        theta = max(min(theta,1),-1)

        self.get_logger().info(f"V_x={v_x}, V_y ={v_y}")

        omega_vec = self.mpc_model.get_omega(v_x, v_y, theta)
        self.get_logger().info(f"Omega_vec={omega_vec}")
        self.actual_u.append(omega_vec)

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

        #if distance_error < self.tolerence:
         #   self.get_logger().info(f"Waypoint {self.waypoints_index} erreicht.")
          #  self.waypoints_index += 1
            #self.stop_robot()

    def stop_robot(self):
        motor_stopp  = Twist()
        motor_stopp.linear.x = 0.0 
        motor_stopp.linear.y = 0.0
        motor_stopp.angular.z = 0.0
        self.stop_pub.publish(motor_stopp)
        motor_v=self.mecanum_chassis.set_velocity(0,0,0)
        self.motor_pub.publish(motor_v)
        self.fig.savefig("trajectorytime_plot1.png")
        self.fig_u.savefig("u_plot2.png")
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

        self.ax.set_aspect('equal', adjustable='datalim')
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.legend()
        self.ax.set_title("Trajektorie vs. tatsächlicher Weg")

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

        self.ax_u.cla()  # Zweiten Plot zurücksetzen
        if self.actual_u:
            # Wandeln der Liste in einen NumPy-Array (jede Zeile entspricht einem Regelzyklus)
            u_arr = np.array(self.actual_u)  # Shape: (Anzahl Zeitschritte, 4)
            t = np.arange(u_arr.shape[0])  # Zeit bzw. Iterationsindex
            # Plot für jedes der 4 Räder
            self.ax_u.plot(t, u_arr[:, 0], label='Rad 1')
            self.ax_u.plot(t, u_arr[:, 1], label='Rad 2')
            self.ax_u.plot(t, u_arr[:, 2], label='Rad 3')
            self.ax_u.plot(t, u_arr[:, 3], label='Rad 4')
            
            self.ax_u.set_title("Stellgröße u – Winkelgeschwindigkeiten der Räder")
            self.ax_u.set_xlabel("Zeit (Iterationsschritte)")
            self.ax_u.set_ylabel("Winkelgeschwindigkeit [rad/s]")
            self.ax_u.legend()
            self.ax_u.grid(True)

            self.fig_u.canvas.draw()
            self.fig_u.canvas.flush_events()
    




def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPController()
    try: 
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        

if __name__ == '__main__':
    main()

    time = TrajectoryPController()
    
    


