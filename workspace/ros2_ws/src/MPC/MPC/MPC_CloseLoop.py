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
from MPC_OpenLoop import QP
from SystemModel import DynamicModel

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
        self.Q = np.diag([10,10,5,1,1,1]) #Höhere Bestrafung auf der Position
        self.R = np.eye(self.nu)
        self.QN = self.Q

        self.Ts = 0.1 #Diskretisierungszeit
        self.N = 20   #Prediktionshorizont

        #Mecanum-Chassis Objekt erstellen
        self.mecanum_chassis = MecanumChassis()

        #ROS2 Publisher (Winkelgeschwindikeiten der vier Räder) und subscriber(Position)
        self.motor_pub = self.create_publisher(MotorsState,'ros_robot_controller/set_motor',10)
        self.get_position = self.create_subscription(Odometry,'odom',self.odom_callback,10)
        self.stop_pub = self.create_publisher(Twist,'cmd_vel',10)

        self.timer = self.create_timer(0.1, self.mpc_closedloop)
        self.plot_timer = self.create_timer(1, self.plot_callback)
        
        #Anfangszustand festlegen

        self.xmeasure = None    #Aktuelle gemessene Position des Roboters
        self.x_ref = [4,3,0,0,0,0]
        self.x0 = [0,0,0,0,0,0]
        self.u0 = [0.5,0.5,0.5,0.5]

        #Speicher für geschlossenen Trajektorie
        self.x_cl = []
        self.u_cl = []

        #Wir legen einen guess Wert für die 1. Iteration fest#
        self.x_guess =[np.zeros(self.nx,self.N+1)]
        self.x_guess[:,0]=self.x0
        self.u_guess = [np.zeros(self.nu,self.N)]

        for i in range(self.N):
             self.x_guess[:,i+1]= self.Ad*self.x_guess[:,i].self.Bd*self.u_guess[i]
             
        for i in range (self.N-1):
             self.u_guess[:,i+1] = self.u_guess[i]
        
        self.z0 = self.z0 = np.concatenate((self.x_guess.flatten(),self.u_guess.flatten()))
        

        #QP initzialisieren
        self.QP = QP(self.Ad, self.Bd, self.Q, self.R, self.QN, 
                                              self.N, self.nx, self.nu, self.Ts,self.z0)
        
        
    def odom_callback(self,msg):
        self.xmeasure = (msg.pose.pose.position.x,msg.pose.pose.position.y,self.quaternion_to_yaw(msg.pose.pose.orientation))
          
    def mpc_closedloop(self):
        #x_current muss der gemessene aktuelle Zustand sein, wir müssen noch die geschwindigkeit bekommen)
        x_opt, u_opt = self.QP.solveMPC(x_current, self.x_ref,self.z0)
        u_cl = u_opt[:,0]
        x_cl = x_opt[:,0]

        z0_new = np.concatenate((x_opt.flatten(),u_opt.flatten()))

        #Neuen Warmstart initialisieren Dabei wird die Lösung in i+1 geshiftet und am ende der gleiche Zustand nochmal drangehängt
        # Schiebe x(1..N) -> x(0..N-1)
        for k in range(self.N):
            z0_new[k*self.nx : (k+1)*self.nx] = x_opt[:, k+1]
        #Letzter Zustand wird nochmal drangehängt 
        z0_new[self.N*self.nx : (self.N+1)*self.nx] = x_opt[:, self.N]
        
        #Analog für U
        for k in range(self.N-1):
            z0_new[(self.N+1)*self.nx + k*self.nu:(self.N+1)*self.nx + (k+1)*self.nu]=u_opt[:,k+1]
        
        # U nochmal dranhängen
        z0_new[(self.N+1)*self.nx + (self.N-1)*self.nu:(self.N+1)*self.nx + self.N*self.nu]= u_opt[:,self.N-1]

        self.z0 = z0_new

        # uopt auf den Roboter publishen


    def quaternion_to_yaw(self,q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
        




        