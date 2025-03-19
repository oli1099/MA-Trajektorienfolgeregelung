

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
from .adaptiveMPC_OL import QP
from MPC.SystemModel import DynamicModel

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
        self.Q = np.diag([1,1,0.5,1,1,1]) #Höhere Bestrafung auf der Position
        self.R = 0.01*np.eye(self.nu)
        self.QN = self.Q
        self.Penalty = 1e6*np.eye(self.N+1)

        self.Ts = 0.1 #Diskretisierungszeit
        self.N = 25   #Prediktionshorizont

        #Mecanum-Chassis Objekt erstellen
        self.mecanum_chassis = MecanumChassis()

        #Liste für aktuellen Pfad
        self.actual_path = []
        self.predictions_list = []
        plt.ion()
        plt.show()
        self.fig ,self.ax = plt.subplots()
        self.x_pred = None


        #ROS2 Publisher (Winkelgeschwindikeiten der vier Räder) und subscriber(Position)
        self.motor_pub = self.create_publisher(MotorsState,'ros_robot_controller/set_motor',10)
        self.get_position = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.control_pub = self.create_publisher(Twist,'cmd_vel',10)

        self.timer = self.create_timer(0.1, self.mpc_closedloop)
        self.plot_timer = self.create_timer(1, self.plot_callback)
        
        #Anfangszustand festlegen

        self.xmeasure = None    #Aktuelle gemessene Position des Roboters
        self.xmeasure_received = None 
        self.x_ref = [4,0.2,0,0,0,0]
        self.x0 = [0,0,0,0,0,0]
        self.u0 = [0.2,0.2,0.2,0.2]

        #Speicher für geschlossenen Trajektorie
        self.x_cl = []
        self.u_cl = []

        #Wir legen einen guess Wert für die 1. Iteration fest
        self.x_guess =np.zeros((self.nx,self.N+1))
        self.x_guess[:,0]=self.x0
        self.u_guess = np.zeros((self.nu,self.N))

        for i in range(self.N):
             self.x_guess[:,i+1]= self.Ad @ self.x_guess[:,i] + self.Bd @ self.u_guess[:,i]
             
        for i in range (self.N-1):
             self.u_guess[:,i+1] = self.u_guess[:,i]
        
        self.z0 = np.concatenate((self.x_guess.flatten(),self.u_guess.flatten()))
        

        #QP initzialisieren
        self.QP = QP(self.Ad, self.Bd, self.Q, self.R, self.QN,self.Penalty, 
                                              self.N, self.nx, self.nu, self.Ts)
        
        
    def odom_callback(self,msg):
        self.xmeasure = np.array([msg.pose.pose.position.x, #x
                                  msg.pose.pose.position.y, #y
                                  self.quaternion_to_yaw(msg.pose.pose.orientation), #theta
                                  msg.twist.twist.linear.x, #vx
                                  msg.twist.twist.linear.y, #vy
                                  msg.twist.twist.angular.z]) #omega
        self.xmeasure_received = True
        self.actual_path.append((self.xmeasure[0], self.xmeasure[1]))
        #self.get_logger().info(f'Received state update: x={self.xmeasure[0]:.2f}, y={self.xmeasure[1]:.2f}, theta={self.xmeasure[2]:.2f}')
          
    def mpc_closedloop(self):
        if self.xmeasure_received is None:
            self.get_logger().warn("Keine gültige Zustandsmessung erhalten")
            return

        error = np.linalg.norm(np.array(self.xmeasure[0:2])-np.array(self.x_ref[0:2]))
        if error < 0.1:
            motor_stopp  = Twist()
            motor_stopp.linear.x = 0.0 
            motor_stopp.linear.y = 0.0
            motor_stopp.angular.z = 0.0
            self.control_pub.publish(motor_stopp)
            self.fig.savefig("MPC_CL_plot")
            self.timer.cancel()
            return

        #x_current muss der gemessene aktuelle Zustand sein, wir müssen noch die geschwindigkeit bekommen, wie bekomme ich die aktuelle Geschwinfigkeit
        x_opt, u_opt, slack_opt = self.QP.solveMPC(self.xmeasure, self.x_ref,self.z0)
        u_cl = u_opt[:,0]
        x_cl = x_opt[:,0]
        self.get_logger().info(f'Received state update: x={x_cl}, y={u_cl}')
        self.x_pred =x_opt
        self.predictions_list.append(x_opt.copy())
        

        z0_new = np.concatenate((x_opt.flatten(),u_opt.flatten(),slack_opt.flatten()))

        # x_opt hat die Form (nx, N+1) und u_opt die Form (nu, N)
        # Verschieben der Zustände: Entferne das erste Element und hänge den letzten Zustand an
        x_warm = np.hstack((x_opt[:, 1:], x_opt[:, -1:]))
        # Verschieben der Eingänge: Entferne das erste Eingangselement und hänge den letzten Eingang an
        u_warm = np.hstack((u_opt[:, 1:], u_opt[:, -1:]))

        slacks_warm = np.hstack((slack_opt[1:], slack_opt[-1:]))

        # Neu zusammensetzen des Warmstart-Vektors, indem zuerst x_warm und dann u_warm (beide flach gemacht) konkateniert werden
        z0_new = np.concatenate((x_warm.flatten(), u_warm.flatten(), slacks_warm.flatten()))
        self.z0 = z0_new
        self.get_logger().info(f'z0: {self.z0}')


        '''#Neuen Warmstart initialisieren Dabei wird die Lösung in x0 <- x1 x1 <- x2 usw x_n-1 <- x_n und x_n <- xn geshiftet und am ende der gleiche Zustand nochmal drangehängt
        for k in range(self.N):
            z0_new[k*self.nx : (k+1)*self.nx] = x_opt[:, k+1]
        #Letzter Zustand wird nochmal drangehängt 
        z0_new[self.N*self.nx : (self.N+1)*self.nx] = x_opt[:, self.N]
        
        #Analog für U
        for k in range(self.N-1):
            z0_new[(self.N+1)*self.nx + k*self.nu:(self.N+1)*self.nx + (k+1)*self.nu]=u_opt[:,k+1]
        
        # U nochmal dranhängen
        z0_new[(self.N+1)*self.nx + (self.N-1)*self.nu:(self.N+1)*self.nx + self.N*self.nu]= u_opt[:,self.N-1]

        self.z0 = z0_new'''

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
            
        '''if self.x_pred is not None:
            # x_pred[0,:] = x-Koordinaten, x_pred[1,:] = y-Koordinaten
            self.ax.plot(self.x_pred[0, :], self.x_pred[1, :], 'r--', linewidth=2, label='Vorhersage (N Schritte)')'''

        # Plot des tatsächlichen Pfads, falls vorhanden
        if self.actual_path:
            actual_path_arr = np.array(self.actual_path)
            self.ax.plot(actual_path_arr[:, 0], actual_path_arr[:, 1], 'b-', linewidth=2)

        self.ax.legend()
        self.ax.set_title("MPC Vorhersage & Tatsächlicher Pfad")
        self.ax.set_xlabel("y [m]")
        self.ax.set_ylabel("x [m]")
        self.ax.grid(True)

        # Zeichnen des Plots (mit kurzer Pause, um die Aktualisierung zu ermöglichen)



def main(args=None):
    rclpy.init(args=args)
    node = MPCClosedLoop()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()      
        




        