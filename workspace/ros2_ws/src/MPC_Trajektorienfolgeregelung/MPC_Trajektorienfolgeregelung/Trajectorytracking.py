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
        self.k_lat = 1.0          # Querfehler-Gain
        self.k_psi = 1.0          # Heading-Gain
        self.k_s = 1           # Längsfehler-Gain

        self.Ua_max    = 0.2    # maximaler Approach-Speed
        self.Lp        = 0.1  # fester Look-ahead-Abstand

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
        # 1) nächster Pfad-Index
        p = np.array(self.current_position)
        dists = [np.linalg.norm(p - np.array(pt[:2])) for pt in self.trajectory]
        i_min = int(np.argmin(dists))
        self.waypoints_index = i_min
        # 2) segmentweises Fortzählen bis Lp erreicht
        dist_acc = 0.0
        idx   = i_min
        while idx < len(self.trajectory)-1 and dist_acc < self.Lp:
            p0 = np.array(self.trajectory[idx][:2])
            p1 = np.array(self.trajectory[idx+1][:2])
            dist_acc += np.linalg.norm(p1-p0)
            idx += 1
        # 3) Zielpunkt
        x_LA, y_LA, _ = self.trajectory[idx]
        # 4) LOS-Vektor
        ex = self.current_position[0] - x_LA
        ey = self.current_position[1] - y_LA
        return ex, ey
        

    
    
    def control_loop(self):
        if self.current_position is None:
            return
        
        
        self.actual_path.append(self.current_position)
        ex, ey = self.compute_reference()

        denom = math.sqrt(ex*ex + ey*ey + self.Lp*self.Lp)
        v_x = -self.Ua_max * ex/denom
        v_y = -self.Ua_max * ey/denom

        #v_x = -self.k_lat * ex
        #v_y = -self.k_lat * ey
       
        
        phi_d = math.atan2(ey, ex)
        # 4) Gierrate aus Heading-Fehler
        err_phi = math.atan2(math.sin(phi_d - self.current_orientation),
                             math.cos(phi_d - self.current_orientation))
        theta = self.k_psi * err_phi      # ggf. eigenes Gain self.k_ang

        omega_vec = self.mpc_model.get_omega(v_x, v_y, 0)
        self.actual_u.append(omega_vec)

        last_idx = len(self.trajectory)-1
        if self.waypoints_index == last_idx and math.hypot(ex, ey) < self.tolerence:
            self.stop_robot()
            return

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
                                f"v: {v_robot}, "
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
    
    


