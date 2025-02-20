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

class TrajectoryController(Node):
    def __init__(self):
        super().__init__('trajectory_controller')

        # -----------------------------------
        # 1) Trajektorie laden/definieren:
        #    6 Zeilen => x, y, theta, vx, vy, omega
        #    161 Spalten => Zeitpunkte von t=0...180s
        # -----------------------------------
         # Definiere die Trajektorie: Jede Zeile muss dieselbe Anzahl an Elementen haben.
        x_values = [0.0000, 0.0506, 0.1080, 0.1702, 0.2788, 0.3937, 0.5086, 0.6235, 0.7384, 0.8533,
                    0.9682, 1.0831, 1.1970, 1.3066, 1.4063, 1.4856, 1.5530, 1.6148, 1.6737, 1.7313,
                    1.7882, 1.8448, 1.9013, 1.9576, 2.0140, 2.0703, 2.1266, 2.1829, 2.2393, 2.2956,
                    2.3519, 2.4082, 2.4645, 2.5208, 2.5771, 2.6334, 2.6897, 2.7461, 2.8024, 2.8587,
                    2.9150, 2.9713, 3.0276, 3.0839, 3.1402, 3.1965, 3.2529, 3.3092, 3.3655, 3.4218,
                    3.4781, 3.5362, 3.5966, 3.6581, 3.7203, 3.7827, 3.8391, 3.8903, 3.9384, 3.9740,
                    3.9897, 3.9966, 4.0018, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024,
                    4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024,
                    4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0024, 4.0606, 4.1608, 4.2651, 4.3701,
                    4.4763, 4.5836, 4.6909, 4.7981, 4.9055, 5.0157, 5.1276, 5.2408, 5.3555, 5.4704,
                    5.5815, 5.6902, 5.7990, 5.9080, 6.0171, 6.1261, 6.2351, 6.3442, 6.4533, 6.5624,
                    6.6715, 6.7806, 6.8901, 6.9859, 7.0627, 7.1289, 7.1901, 7.2487, 7.3062, 7.3630,
                    7.4196, 7.4760, 7.5324, 7.5887, 7.6451, 7.7014, 7.7577, 7.8140, 7.8703, 7.9266,
                    7.9830, 8.0393, 8.0956, 8.1519, 8.2082, 8.2645, 8.3208, 8.3771, 8.4334, 8.4897,
                    8.5461, 8.6024, 8.6587, 8.7150, 8.7713, 8.8275, 8.8839, 8.9403, 8.9711, 8.9861,
                    8.9933, 8.9968, 8.9984, 8.9992, 8.9996, 8.9998, 8.9999, 9.0000, 9.0000, 9.0000,
                    9.0000]
        
        y_values = [0.0000, 0.0864, 0.2208, 0.3565, 0.4132, 0.4288, 0.4368, 0.4434, 0.4495, 0.4557,
                    0.4618, 0.4679, 0.4743, 0.4812, 0.4933, 0.5258, 0.5714, 0.6226, 0.6764, 0.7315,
                    0.7872, 0.8432, 0.8994, 0.9557, 1.0120, 1.0683, 1.1246, 1.1809, 1.2372, 1.2935,
                    1.3499, 1.4062, 1.4625, 1.5188, 1.5752, 1.6315, 1.6878, 1.7441, 1.8005, 1.8568,
                    1.9131, 1.9694, 2.0257, 2.0821, 2.1384, 2.1947, 2.2510, 2.3074, 2.3637, 2.4200,
                    2.4763, 2.5309, 2.5834, 2.6348, 2.6855, 2.7361, 2.7926, 2.8542, 2.9188, 2.9958,
                    3.0892, 3.1884, 3.2899, 3.3997, 3.5137, 3.6284, 3.7432, 3.8581, 3.9730, 4.0879,
                    4.2029, 4.3178, 4.4327, 4.5476, 4.6625, 4.7774, 4.8923, 5.0072, 5.1221, 5.2370,
                    5.3519, 5.4668, 5.5817, 5.6966, 5.8115, 5.9264, 5.9740, 5.9752, 5.9762, 5.9773,
                    5.9784, 5.9798, 5.9817, 5.9837, 5.9856, 5.9869, 6.0485, 6.0643, 6.0995, 6.1463,
                    6.1980, 6.2521, 6.3073, 6.3631, 6.4192, 6.4754, 6.5316, 6.5879, 6.6442, 6.7006,
                    6.7569, 6.8132, 6.8695, 6.9258, 6.9822, 7.0385, 7.0948, 7.1511, 7.2075, 7.2638,
                    7.3201, 7.3764, 7.4328, 7.4891, 7.5454, 7.6017, 7.6580, 7.7144, 7.7707, 7.8270,
                    7.8833, 7.9395, 7.9704, 7.9855, 7.9929, 7.9965, 7.9983, 7.9992, 7.9996, 7.9998,
                    7.9999, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000,
                    8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000, 8.0000 , 8.0000, 8.0000, 8.0000, 8.0000]
        
        theta_values = [0.0000, 0.4351, 1.2227, 2.0447, 2.4793, 2.6165, 2.6930, 2.7570, 2.8184, 2.8793,
                        2.9400, 3.0008, 3.0530, 3.0620, 3.0267, 2.9988, 2.9808, 2.9602, 2.9384, 2.9161,
                        2.8937, 2.8712, 2.8486, 2.8261, 2.8035, 2.7809, 2.7583, 2.7357, 2.7131, 2.6905,
                        2.6679, 2.6453, 2.6227, 2.6001, 2.5775, 2.5549, 2.5323, 2.5097, 2.4871, 2.4645,
                        2.4419, 2.4193, 2.3967, 2.3741, 2.3515, 2.3289, 2.3063, 2.2837, 2.2611, 2.2385,
                        2.2159, 2.1942, 2.1739, 2.1541, 2.1345, 2.1150, 2.0953, 2.0747, 2.0520, 2.0287,
                        1.9638, 1.8762, 1.7964, 1.7585, 1.7508, 1.7492, 1.7488, 1.7488, 1.7488, 1.7488,
                        1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488,
                        1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.7488, 1.6371, 1.5107, 1.4171, 1.3299,
                        1.2563, 1.1958, 1.1391, 1.0822, 1.0266, 0.9964, 0.9979, 1.0215, 1.0672, 1.1249,
                        1.1525, 1.1377, 1.1076, 1.0776, 1.0482, 1.0189, 0.9896, 0.9607, 0.9325, 0.9044,
                        0.8764, 0.8484, 0.8243, 0.7890, 0.7624, 0.7439, 0.7231, 0.7011, 0.6789, 0.6564,
                        0.6339, 0.6114, 0.5888, 0.5662, 0.5436, 0.5210, 0.4984, 0.4758, 0.4532, 0.4306,
                        0.4080, 0.3854, 0.3628, 0.3402, 0.3176, 0.2950, 0.2724, 0.2498, 0.2272, 0.2046,
                        0.1820, 0.1594, 0.1367, 0.1140, 0.0913, 0.0686, 0.0454, 0.0240, 0.0114, 0.0055,
                        0.0027, 0.0013, 0.0006, 0.0003, 0.0002, 0.0001, 0.0000, 0.0000, 0.0000, 0.0000,0]
        self.simX = []
        self.simU = []

        # Optional: Überprüfe, ob alle Listen die gleiche Länge haben.
        print(len(theta_values))
        if len(x_values) == len(y_values) == len(theta_values):
            self.trajectory = np.array([x_values, y_values, theta_values])
        else:
            raise ValueError("Die Trajektorien-Daten haben inkonsistente Längen!")
        print(len(self.trajectory[1]))
        # Transponieren auf shape=(161, 6), sodass trajectory[i] = [x, y, theta, vx, vy, omega]
        self.trajectory = self.trajectory.T

        # Maximale Zeit in Sekunden
        self.total_time = 100.0  
        self.start_time = None

        # P-Regler für x,y,theta
        self.kp_xy = 0.5
        self.kp_theta = 0.5

        # Subscriber für Odometrie
        self.odom_sub = self.create_subscription(Odometry, 'odom_raw', self.odom_callback, 10)

        # Publisher für Motorbefehle
        self.motor_pub = self.create_publisher(MotorsState, 'ros_robot_controller/set_motor', 10)

        # Mecanum-Klasse zur Umrechnung (v_x, v_y, omega) -> einzelne Rad-Drehzahlen
        self.mecanum_chassis = MecanumChassis()

        # Timer fürs Regel-/Send-Intervall (z.B. alle 50 ms)
        self.control_timer = self.create_timer(0.05, self.control_loop)

        # Variablen für aktuelle Pose
        self.current_position = None   # (x, y)
        self.current_orientation = 0.0 # theta

        self.get_logger().info("Trajectory Controller gestartet.")

    def odom_callback(self, msg):
        """Odometrie-Callback: aktuelle Roboterpose speichern."""
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        )
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_orientation = self.quaternion_to_euler(
            orientation_q.x, orientation_q.y,
            orientation_q.z, orientation_q.w
        )
        

        if self.start_time is None:
            # Startzeit für die Trajektorie setzen, sobald erste Odometrie-Daten kommen
            self.start_time = self.get_clock().now().nanoseconds * 1e-9

    def control_loop(self):
        """Kontrollschleife, um die vorgegebene Trajektorie (x,y,theta) feedback-basiert abzufahren."""
        if self.start_time is None or self.current_position is None:
            # Noch keine Odometrie-Daten
            return

        # Aktuelle Zeit seit Beginn in s
        now = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self.start_time

        # Wenn die 180 s vorbei sind -> Roboter stoppen
        if elapsed >= self.total_time:
            self.stop_robot()
            return

        # 2) Trajektorie nach Index bestimmen
        #    Wir verteilen 161 Punkte auf 0..180s
        #    => dt pro Schritt ~ 1.125s
        num_points = len(self.trajectory)
        dt = self.total_time / (num_points - 1)
        # Index i in [0, 160]
        i = int(elapsed // dt)
        if i >= num_points - 1:
            i = num_points - 1

        # Aus der Trajektorie: x_des, y_des, theta_des
        x_des    = self.trajectory[i, 0]
        y_des    = self.trajectory[i, 1]
        theta_des= self.trajectory[i, 2]
        # vx_des  = self.trajectory[i, 3]  # ungeachtet
        # vy_des  = self.trajectory[i, 4]  # ungeachtet
        # omg_des = self.trajectory[i, 5]  # ungeachtet

        # 3) Fehlerberechnung in Weltkoordinaten
        x_act = self.current_position[0]
        y_act = self.current_position[1]
        theta_act= self.current_orientation

        

        ex = x_des - x_act
        ey = y_des - y_act
        etheta = theta_des - theta_act
        # Winkel sauber auf [-pi, +pi] bringen
        etheta = math.atan2(math.sin(etheta), math.cos(etheta))

        # 4) P-Regler: gewünscht in x-/y-Richtung des *Welt-Rahmens*
        #    Wir möchten dem Roboter lineare Geschw. in *Roboter*-Koordinaten geben.
        #    => Koordinatentransformation: 
        #       [ v_x ]   [  cos(th)   sin(th) ] [ ex * kp_xy ]
        #       [ v_y ] = [ -sin(th)   cos(th) ] [ ey * kp_xy ]
        #
        #    So bewegt er sich in Richtung (ex, ey) in Weltkoordinaten.
        u_x_world = self.kp_xy * ex
        u_y_world = self.kp_xy * ey

        self.simX.append((x_act,y_act))
        

        # Transformation nach Roboterkoordinaten:
        cosT = math.cos(theta_act)
        sinT = math.sin(theta_act)
        # Inverse Rotation
        v_x =  cosT * u_x_world + sinT * u_y_world
        v_y = -sinT * u_x_world + cosT * u_y_world

        # Winkelgeschwindigkeit
        omega_cmd = self.kp_theta * etheta

        # 5) Begrenzen (optional)
        max_lin = 0.1  # [m/s], Beispielwert
        max_ang = 1.0  # [rad/s]
        # Norm in der Ebene
        lin_norm = math.hypot(v_x, v_y)
        if lin_norm > max_lin:
            scale = max_lin / lin_norm
            v_x *= scale
            v_y *= scale

        if abs(omega_cmd) > max_ang:
            omega_cmd = max_ang * math.copysign(1.0, omega_cmd)

        # 6) Motoren ansteuern
        motor_command = self.mecanum_chassis.set_velocity(v_x, v_y, omega_cmd)
        self.motor_pub.publish(motor_command)

        self.get_logger().info(f"[{elapsed:5.2f}s] i={i:3d}, "
                               f"ex={ex:.3f}, ey={ey:.3f}, etheta={etheta:.3f} | "
                               f"v_x={v_x:.3f}, v_y={v_y:.3f}, w={omega_cmd:.3f}")

    def stop_robot(self):
        """Stoppt den Roboter (alle Motoren auf 0)."""
        stop_msg = MotorsState()
        stop_msg.data = []
        for i in range(4):
            m = MotorState()
            m.id = i+1
            m.rps = 0.0
            stop_msg.data.append(m)
        self.motor_pub.publish(stop_msg)
        self.get_logger().info("Roboter gestoppt.")

    @staticmethod
    def quaternion_to_euler(x, y, z, w):
        """Hilfsfunktion zur Umwandlung Quaternion -> (roll, pitch, yaw)."""
        t0 = +2.0 * (w*x + y*z)
        t1 = +1.0 - 2.0 * (x*x + y*y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w*y - z*x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w*z + x*y)
        t4 = +1.0 - 2.0 * (y*y + z*z)
        yaw = math.atan2(t3, t4)

        return (roll, pitch, yaw)


class MecanumChassis:
    """Implementierung der Mecanum-Chassis-Kinematik."""
    def __init__(self, wheelbase=0.1368, track_width=0.1410, wheel_diameter=0.065):
        # Roboter-spezifische Maße
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed_m_s):
        """Umrechnung [m/s] -> [U/s] für das jeweilige Rad."""
        # Umfang = pi * Raddurchmesser
        return speed_m_s / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        lineare Geschwindigkeit in x-Richtung: linear_x  (m/s)
        lineare Geschwindigkeit in y-Richtung: linear_y   (m/s)
        Drehgeschwindigkeit um z-Achse:        angular_z  (rad/s)

        Liefert MotorsState(), wobei
          motor1 ~ vorne links,
          motor2 ~ vorne rechts,
          motor3 ~ hinten rechts,
          motor4 ~ hinten links
        (oder eine ähnliche Zuordnung; ggf. anpassen.)
        """
        # Standardformeln Mecanum (Varianten sind möglich)
        L = self.wheelbase
        W = self.track_width
        R = (L + W) / 2.0

        # Für Standard-Mecanum-Lage:
        # v1 = x - y - w*R
        # v2 = x + y - w*R
        # v3 = x + y + w*R
        # v4 = x - y + w*R
        motor1 = linear_x - linear_y - angular_z * R
        motor2 = linear_x + linear_y - angular_z * R
        motor3 = linear_x + linear_y + angular_z * R
        motor4 = linear_x - linear_y + angular_z * R

        # Drehraten in [U/s]
        v_s = [self.speed_covert(v) for v in [-motor1, -motor2, motor3, motor4]]

        # Zusammenbauen
        msg = MotorsState()
        msg.data = []
        for i, vs in enumerate(v_s):
            m = MotorState()
            m.id = i + 1
            m.rps = float(vs)  # Runden / casten
            m.rps = float(vs)
            msg.data.append(m)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Strg+C erkannt. Stoppe den Roboter...")
        node.stop_robot()
        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    if node.simX:
                positions = np.array(node.simX)  # Shape: (N, 2)
                plt.figure(figsize=(8, 6))
                plt.plot(positions[:, 0], positions[:, 1], 'bo-', label="Roboterpfad")
                plt.xlabel("x [m]")
                plt.ylabel("y [m]")
                plt.title("Zurückgelegter Pfad des Roboters")
                plt.legend()
                plt.grid(True)
                plt.show()
    else:
                print("Keine Positionsdaten geloggt.")  

       

if __name__ == '__main__':
    main()


    