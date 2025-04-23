

import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from ros_robot_controller_msgs.msg import MotorsState, MotorState
import numpy as np
import math
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from controller.mecanum import MecanumChassis
from .mpc_OL_dynObs import QP
from MPC.SystemModel import DynamicModel
from MPC.SaveData import SaveData


import sys

class MPCClosedLoop(Node):
    def __init__(self):
        super().__init__('mpc_closed_loop')

        #Dynamic Modell initialisieren

        self.mpc_model = DynamicModel()
        self.Ad = self.mpc_model.A_d
        self.Bd = self.mpc_model.B_d
        self.nx = self.mpc_model.nx
        self.nu = self.mpc_model.nu

        # Gewichtsmatrizen festlegen
        self.Q = np.diag([100,100,50,1,1,1]) #Höhere Bestrafung auf der Position
        self.R = 0.01*np.eye(self.nu)
        self.QN = self.Q
        self.Safezone = 0.2

        self.Ts = 0.1 #Diskretisierungszeit
        self.Np = 25 #Prediction Horizon
        self.Nc = 5  #Control Horizon

     # Beispiel-Hindernisdaten (Rear-Right Safe Point des Hindernisses)
        self.obstacle = {
            'obsXrl': 2,  # x-Koordinate
            'obsYrl': 0.5,   # y-Koordinate
            'obslength': 1.0 # Breite des Hindernisses
        }
        self.road_width = 4.0  # Breite der Straße (Beispielwert)


        #Mecanum-Chassis Objekt erstellen
        self.mecanum_chassis = MecanumChassis()

        #Liste für aktuellen Pfad
        self.actual_path = []
        self.predictions_list = []
        self.actual_u = []
        plt.ion()
        plt.show()
        self.fig ,self.ax = plt.subplots()
        self.fig_u ,self.ax_u = plt.subplots()
        self.x_pred = None


        #ROS2 Publisher (Winkelgeschwindikeiten der vier Räder) und subscriber(Position)
        self.motor_pub = self.create_publisher(MotorsState,'ros_robot_controller/set_motor',10)
        self.get_position = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.control_pub = self.create_publisher(Twist,'cmd_vel',10)

        self.timer = self.create_timer(0.2, self.mpc_closedloop)
        self.plot_timer = self.create_timer(1, self.plot_callback)
        
        #Anfangszustand festlegen

        self.xmeasure = None    #Aktuelle gemessene Position des Roboters
        self.xmeasure_received = None 
        self.x_ref = [4,0,0,0,0,0]
        self.x0 = [0,0,0,0,0,0]
        self.u0 = [0.5,0.5,0.5,0.5]

        #Speicher für geschlossenen Trajektorie
        self.x_cl = []
        self.u_cl = []

        #Wir legen einen guess Wert für die 1. Iteration fest
        self.x_guess =np.zeros((self.nx,self.Np+1))
        self.x_guess[:,0]=self.x0
        self.u_guess = np.zeros((self.nu,self.Nc))

        for i in range(self.Np):
            idx_u = min(i, self.Nc - 1) 
            self.x_guess[:,i+1]= self.Ad @ self.x_guess[:,i] + self.Bd @ self.u_guess[:,idx_u]
        for i in range (self.Nc-1):
             self.u_guess[:,i+1] = self.u_guess[:,i]

        self.z0 = np.concatenate((self.x_guess.flatten(),self.u_guess.flatten()))
        

        #QP initzialisieren
        self.QP = QP(self.Ad, self.Bd, self.Q, self.R, self.QN,self.Safezone, 
                                              self.Nc, self.Np, self.nx, self.nu, self.Ts)
        
        
    def odom_callback(self,msg):
        self.xmeasure = np.array([msg.pose.pose.position.x, #x
                                  msg.pose.pose.position.y, #y
                                  self.quaternion_to_yaw(msg.pose.pose.orientation), #theta
                                  msg.twist.twist.linear.x, #vx
                                  msg.twist.twist.linear.y, #vy
                                  msg.twist.twist.angular.z]) #omega
        self.xmeasure_received = True
        
        #self.get_logger().info(f'Received state update: x={self.xmeasure[0]:.2f}, y={self.xmeasure[1]:.2f}, theta={self.xmeasure[2]:.2f}')
          
    def compute_obstacle_constraints(self,x_current):
        carX = x_current[0] 
        carY = x_current[1]
        obsYrl = self.obstacle['obsYrl'] + self.Safezone
        obsXrl = self.obstacle['obsXrl'] - self.Safezone
        obslength = self.obstacle['obslength']

        xmin = carX
        xmax = 1e6

        adjence_lanecenter = self.road_width/2

        # Schwellenwert wann das Auto in der linken spur ist 
        threshold = 0.2
        epsilon = 0.01

        if obsXrl - carX > 1: # Erst ab 1 meter zum Hinderniss soll reagiert werden
            return 0, -self.road_width/2, xmin, xmax

        if carX <= obsXrl :
            if  abs(carY - adjence_lanecenter) <= threshold:
                cS = 0
                cI = obsYrl
            else:
                if abs(obsXrl - carX) < epsilon:
                # Fallback: Wenn die Differenz zu klein ist, setze cS auf 0 und cI auf obsYrl
                    cS = 0.0
                    cI = obsYrl 
                else:
                    cS = np.tan(np.arctan2((obsYrl - carY), (obsXrl - carX)))
                    cI = obsYrl - cS * obsXrl
        else:
            if carX >= obsXrl + obslength + 2*self.Safezone:
                cS = 0
                cI = -self.road_width/2
            else:
                cS = 0
                cI = obsYrl -0.1 #Hier kommt der Schlenker hinzu, wenn nicht, dann infeasable
        return cS, cI, xmin, xmax
    
    '''def save_data_to_csv(self,filename= 'MPC_dynObs_Data' ):
            # Erstelle den Header: Zeitstempel, aktuelle x, aktuelle y, dann für jeden Vorhersageschritt pred_x_i und pred_y_i.
        header = ['timestamp', 'actual_x', 'actual_y']
        for i in range(self.Np + 1):
            header.append(f'pred_x_{i}')
            header.append(f'pred_y_{i}')
        
        # Öffne die CSV-Datei zum Schreiben
        with open(filename, mode='w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(header)
            
            # Gehe über alle gespeicherten Messungen und Vorhersagen
            # Es wird hier angenommen, dass self.actual_path und self.predictions_list synchron gefüllt werden.
            for act, pred in zip(self.actual_path, self.predictions_list):
                # Zeitstempel (hier aktueller Zeitwert)
                row = [time.time(), act[0], act[1]]
                # pred ist ein NumPy-Array der Form (nx, Np+1)
                # Wir extrahieren hier die x- und y-Koordinaten, also die ersten beiden Zeilen
                pred_x = pred[0, :]
                pred_y = pred[1, :]
                # Füge für jeden Zeitschritt in der prädizierten Trajektorie die Werte hinzu
                for x_val, y_val in zip(pred_x, pred_y):
                    row.append(x_val)
                    row.append(y_val)
                writer.writerow(row)'''
        
    
    def mpc_closedloop(self):
        if self.xmeasure_received is None:
            self.get_logger().warn("Keine gültige Zustandsmessung erhalten")
            return

        error = np.linalg.norm(np.array(self.xmeasure[0:2])-np.array(self.x_ref[0:2]))
        if error < 0.05:
            motor_stopp  = Twist()
            motor_stopp.linear.x = 0.0 
            motor_stopp.linear.y = 0.0
            motor_stopp.angular.z = 0.0
            self.control_pub.publish(motor_stopp)
            self.fig.savefig("MPC_Nc_CL_plot")
            self.fig_u.savefig("MPC_Nc_CL_u_plot")
            
            self.saveData = SaveData(self.predictions_list, self.actual_path, self.actual_u)
            self.saveData.save_all("MPC_CL_dynObs")

            self.timer.cancel()
            #self.save_data_to_csv()
            return

        cS, cI, xmin, xmax = self.compute_obstacle_constraints(self.xmeasure)

        self.actual_path.append((self.xmeasure[0], self.xmeasure[1]))
        #x_current muss der gemessene aktuelle Zustand sein, wir müssen noch die geschwindigkeit bekommen, wie bekomme ich die aktuelle Geschwinfigkeit
        x_opt, u_opt = self.QP.solveMPC(self.xmeasure, self.x_ref,self.z0,cS, cI, self.road_width, xmax, xmin)
        u_cl = u_opt[:,0]
        x_cl = x_opt[:,0]
        self.get_logger().info(f'Received state update: x={x_cl}, y={u_cl}')
        self.x_pred =x_opt
        self.predictions_list.append(x_opt.copy())
        self.actual_u.append(u_cl.copy())
        

        z0_new = np.concatenate((x_opt.flatten(),u_opt.flatten()))#,slack_opt.flatten()))

        # x_opt hat die Form (nx, N+1) und u_opt die Form (nu, N)
        # Verschieben der Zustände: Entferne das erste Element und hänge den letzten Zustand an
        x_warm = np.hstack((x_opt[:, 1:], x_opt[:, -1:]))
        # Verschieben der Eingänge: Entferne das erste Eingangselement und hänge den letzten Eingang an
        u_warm = np.hstack((u_opt[:, 1:], u_opt[:, -1:]))

        #slacks_warm = np.hstack((slack_opt[1:], slack_opt[-1:]))

        # Neu zusammensetzen des Warmstart-Vektors, indem zuerst x_warm und dann u_warm (beide flach gemacht) konkateniert werden
        z0_new = np.concatenate((x_warm.flatten(), u_warm.flatten()))#, slacks_warm.flatten()))
        #z0_new[0:self.nx] = self.xmeasure
        self.z0 = z0_new
        self.get_logger().info(f'z0: {self.z0}')

        # uopt auf den Roboter publishen
        # Winkelgeschwindikeiten werden mithilfe der Kinematik umgeechnet in Geschwindigkeit des Roboter in x y und theta Richtung
        v_robot = self.mpc_model.get_velocity(u_cl)  

        #Geschwindigkeit des Roboters wird gepublisht
        twist = Twist()
        twist.linear.x = float(v_robot[0])
        twist.linear.y = float(v_robot[1])
        twist.angular.z = float(v_robot[2])
        self.control_pub.publish(twist)

    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def plot_callback(self):
        #self.ax.clear()

        # Falls eine Vorhersage-Trajektorie vom MPC vorliegt, diese plotten
        for i, pred in enumerate(self.predictions_list):
                self.ax.plot(pred[0, :], pred[1, :], 'r--', alpha=0.5)
            
        # Plot des tatsächlichen Pfads, falls vorhanden
        if self.actual_path:
            actual_path_arr = np.array(self.actual_path)
            self.ax.plot(actual_path_arr[:, 0], actual_path_arr[:, 1], 'b-', linewidth=2)

        obs_x = self.obstacle['obsXrl']
        obs_y = 0
        obs_width = 1.0
        obs_height = self.obstacle['obsYrl']

        safe_obs_x = obs_x - self.Safezone
        safe_obs_y = obs_y - self.Safezone
        safe_obs_width = obs_width + 2 * self.Safezone
        safe_obs_height = obs_height + 2 * self.Safezone

        # Erstellen des Hindernis-Patches (gefüllter grauer Bereich)
        obstacle_patch = Rectangle((obs_x, obs_y), obs_width, obs_height,
                                    color='gray', alpha=0.5)
        # Erstellen des Safe Zone-Patches (Umriss, gestrichelt)
        safezone_patch = Rectangle((safe_obs_x, safe_obs_y), safe_obs_width, safe_obs_height,
                                fill=False, linestyle='--', edgecolor='red')

        # Hinzufügen der Patches zum Plot
        self.ax.add_patch(obstacle_patch)
        self.ax.add_patch(safezone_patch)


        self.ax.legend()
        self.ax.set_title("MPC Obstacle Vorhersage & Tatsächlicher Pfad")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.ax.grid(True)

        # Zeichnen des Plots (mit kurzer Pause, um die Aktualisierung zu ermöglichen)
        self.ax.set_xlim([0, 5])  # x-Achse von 0 bis 5
        self.ax.set_ylim([-0.3, 3])

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
    


def main(args=None):
    rclpy.init(args=args)
    node = MPCClosedLoop()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
        




        