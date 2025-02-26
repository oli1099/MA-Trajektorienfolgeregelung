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
from MPC_OpenLoop import MPCController

class MPCClosedLoop(Node):
    def __init__(self):
        super().__init__('mpc_closed_loop')

        