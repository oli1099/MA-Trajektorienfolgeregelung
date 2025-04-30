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
from MPC.SaveData import SaveData

class TrajectoryPController(Node):
    def __init__(self):
        super().__init__('trajectory_pcontroller')
        """[(0.0,0,0),(0.5,0,0),(1,0,0),(1,0.5,0.0),(1,1,0)]"""
        # Trajektorie festlegen
       # self.trajectory = [(0,0,0),(0.5,0,0),(1,0.75,0),(1.5,1,0),(2,1,0),(2.5,1,0),(3,0.75,0),(3.5,0,0),(4,0,0)]
        self.trajectory = [
    (0.00, 0.00, 0),
    (0.06, 0.00, 0),
    (0.14, 0.00, 0),
    (0.26, 0.00, 0),
    (0.34, 0.00, 0),
    (0.42, 0.00, 0),
    (0.51, 0.06, 0),
    (0.56, 0.12, 0),
    (0.63, 0.18, 0),
    (0.69, 0.23, 0),
    (0.76, 0.29, 0),
    (0.85, 0.34, 0),
    (0.93, 0.36, 0),
    (1.05, 0.38, 0),
    (1.13, 0.38, 0),
    (1.23, 0.39, 0),
    (1.33, 0.40, 0),
    (1.43, 0.40, 0),
    (1.53, 0.40, 0),
    (1.63, 0.40, 0),
    (1.73, 0.40, 0),
    (1.83, 0.40, 0),
    (1.93, 0.39, 0),
    (2.07, 0.37, 0),
    (2.16, 0.32, 0),
    (2.21, 0.28, 0),
    (2.28, 0.22, 0),
    (2.34, 0.16, 0),
    (2.40, 0.09, 0),
    (2.46, 0.04, 0),    
    (2.52, 0.00, 0),
    (2.58, 0.00, 0),
    (2.64, 0.00, 0),
    (2.70, 0.00, 0),
    (2.76, 0.00, 0),
    (2.82, 0.00, 0),
    (2.88, 0.00, 0),
    (2.94, 0.00, 0),
    (3.00, 0.00, 0)
]

        
        self.v_des = 0.2          # gewünschte Vorwärts­geschwindigkeit [m/s]
        self.lookahead = 0.15     # (m) Abstand, um einen Punkt auf der Pfad­gerade vorauszuwählen
        self.k_lat = 3.0          # Querfehler-Gain
        self.k_psi = 2.0          # Heading-Gain
        self.k_s = 1           # Längsfehler-Gain

        #Zeitliste erstellen
        self.total_time = 20
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
        self.predictions_list = []
        self.actual_theta = []
        self.predicted_theta_list = [] 
        self.solve_times = []         


        #Index für die Stützpunkte
        self.waypoints_index = 0
        
        #Toleranz für Wegpunkt erreicht
        self.tolerence = 0.05

        #Regelparameter

        self.k = 0.5
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

        #if self.current_position is not None:
           

        if self.start_timer is None:
            self.start_timer = self.get_clock().now()

        #self.get_logger().info(f"Roboterposition: x = {self.current_position[0]:.4f}, y = {self.current_position[1]:.4f}, z = {self.current_orientation:.4f}")

    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    
    def compute_reference(self):
        # Aktueller und nächster WP
        i = self.waypoints_index
        p0 = np.array(self.trajectory[i][0:2])
        if i >= len(self.trajectory)-1:
            return p0, 0.0, 0.0   # Ende
        p1 = np.array(self.trajectory[i+1][0:2])

        # Richtungseinheitsvektor
        d = p1 - p0
        L = np.linalg.norm(d)
        if L == 0:
            return p1, 0.0, 0.0
        t_hat = d / L

        # Orthogonaler Abstand & Projektionslänge
        p = np.array(self.current_position)
        proj_len = np.dot((p - p0), t_hat)
        e_y = np.cross(np.append(t_hat,0), np.append(p - p0,0))[2]  # signed distance

        # Fortschritt: wenn genug nähergekommen, nächster WP
        if proj_len > L:
            self.waypoints_index += 1
            return self.compute_reference()

        # Look-ahead-Punkt s = proj_len + lookahead
        s_la = min(proj_len + self.lookahead, L)
        ref_pos = p0 + s_la * t_hat
        ref_heading = math.atan2(d[1], d[0])

        return ref_pos, ref_heading, e_y

    
    
    def control_loop(self):
        if self.current_position is None:
            return
        
        

        # ---- Referenz & Fehler ----
        ref_pos, ref_heading, e_y = self.compute_reference()
        e_psi = math.atan2(math.sin(ref_heading - self.current_orientation),
                        math.cos(ref_heading - self.current_orientation))

        # ---- Regler ----
        # nach compute_reference() …


        # Längsfehler
        delta = np.array(ref_pos) - np.array(self.current_position)
        e_s = np.linalg.norm(delta)

        # P-Anteil auf v
        v = self.k_s * e_s
        v_x = v* math.cos(ref_heading)
        v_y = v * math.sin(ref_heading)

    

        theta =  self.k_lat * e_y + self.k_psi * e_psi

        omega_vec = self.mpc_model.get_omega(v_x, v_y, theta)

        if self.waypoints_index >= len(self.trajectory)-1 and abs(e_y) < self.tolerence:
            self.stop_robot()
       

        #Geschwindigkeit Begrenzung
        #v_x = max(min(v_x, 0.2298), -0.22989)
        #v_y = max(min(v_y, 0.2298), -0.2298)

        

        omega_vec = np.clip(omega_vec, -5.0, 5.0)  # Begrenzung der Stellgrößen
        self.actual_u.append(omega_vec)
        v_robot =self.mpc_model.get_velocity(omega_vec)
        #Geschwindigkeit an Motor übergeben
        motor_v=self.mecanum_chassis.set_velocity(v_x,v_y,theta)
        self.motor_pub.publish(motor_v)
        self.actual_u.append(omega_vec)

        twist = Twist()
        twist.linear.x = float(v_robot[0])
        twist.linear.y = float(v_robot[1])
        twist.angular.z = float(v_robot[2])
        self.stop_pub.publish(twist)

        self.get_logger().info(f"Target: ({self.trajectory[self.waypoints_index][0]:.2f}, {self.trajectory[self.waypoints_index][1]:.2f}), "
                              f"Current: {self.current_position}, "
                              f"Ref: {ref_pos}, "
                              f"Heading: {ref_heading:.2f}, "
                              )
        
                      
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
        
        self.saveData = SaveData(self.predictions_list, self.actual_path, self.actual_u,self.actual_theta, self.predicted_theta_list, self.solve_times)
        self.saveData.save_all("TrajectoryPController")
        self.timer.cancel()

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
    
    


