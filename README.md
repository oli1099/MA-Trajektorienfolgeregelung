# MA-Trajektorienfolgeregelung

Dieses Rep dient zur Entwicklung auf dem HiWonder Mentor PI der mit einem rasberry betrieben wird.

Der HiWonder Pi wurde mit folgender Anleitung aufgesetzt: https://github.com/Matzefritz/HiWonder_MentorPi

Stopp den roboter: 
    ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"



Radumdrehungen an den Roboter geben:
    ros2 topic pub -1 /ros_robot_controller/set_motor ros_robot_controller_msgs/MotorsState "{data: [{id: 1, rps: -0.1}, {id: 2, rps: -0.1}, {id: 3, rps: 0.10}, {id: 4, rps: 0.10}]}" (Vorwärts)

Nodes starten
    ros2 launch controller controller.launch.py

Kreis Trajektorie

    ros2 run Trajektorienfolgeregelung circle_trajectory_p_regler  

Trajektorienfolgereglung
    
    ros2 run MPC_Trajektorienfolgeregelung mpc_PD_controller  

MPC Algorithmus starten

    ros2 run MPC MPC_ClosedLoop   
    mpc_PD_controller = MPC_Trajektorienfolgeregelung.mpc_PD_controller:main',

Zeitbasierte Trajektorienfolgereglung

    ros2 run MPC_Trajektorienfolgeregelung trajectoryPControllerTime  

Package Erstellen in ROS2:
    ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>

Starten des Roboters:
    1. SSH Verbindung zum Roboter : sh prinzessinleia@192.168.1.32 (im Lab)
    2. Zum folgenden ordner navigieren: cd PrinzessinLeia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/ros2_ws/
    3. Controller Launch vom roboter starten: ros2 launch controller controller.launch.py

ROS2 Befehle:

    ros2 topic list -t will return the same list of topics, this time with the topic type appended in brackets

    ros2 topic echo <topic_name> Data being published

    ros2 topic info <topic_name>

    ros2 interface show <Type> gibt genauere Informationen über den Type des Topic

    Orientation vom ROboter ist positiv gegen den Uhrzeigersinn

SSH Verbidung herstellen:

    IP Adresse vom ROboter herausfinden: nmap -sn 192.168.1.0/24

    Verbinden: ssh prinzessinleia@<IP-Adresse>
    Verbinden mit gui: ssh prinzessinleia@<IP-Adresse> -X

    IP Adresse Zuhause: 192.168.2.215
    IP Adresse LAB: 192.168.1.32
Plots vom Roboter speichern
     scp prinzessinleia@192.168.1.32:~/PrinzessinLeia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/trajectory_plot1.png /home/oli/Desktop/Oliver/Uni/MA/Plots

     ---Virtual Environment---
Create venv with:
    python3-m venv <Namevenv>
Activate venv with:
    source <Namevenv>/bin/activate
Deactivate venv with:
    deactivate

---Requirements.txt---\
Requirements.txt erstellen laseen:\
    pip freeze > requirements.txt
Installieren lassen mit:\
    pip install -r requirements.txt

#!/home/prinzessinleia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/venv/bin/python3

#!/usr/bin/env python3

Funktion für MotorsState aktualisiert die Odometry

def motors_state_callback(self,msg: MotorsState):
        wheel_radius = self.mecanum.wheel_diameter/2
        lx = self.mecanum.wheelbase
        ly = self.mecanum.track_width

        omega1 = msg.data[0].rps
        omega2 = msg.data[1].rps
        omega3 = msg.data[2].rps
        omega4 = msg.data[3].rps

        vx = (4 / (wheel_radius**2)) * (omega1 + omega2 + omega3 + omega4)
        vy = (4 / (wheel_radius**2)) * (omega1 - omega2 + omega3 - omega4)
        angular_z = (4 * wheel_radius / ((lx + ly)**2)) * ((omega4 - omega3) - (omega2 - omega1))
        
        now = time.time()
        if self.last_time is None:
            dt = 0.0
        else:
            dt = now - self.last_time
        self.last_time = now

        # Integriere die neuen Werte, um die Pose zu aktualisieren:
        self.x += vx * dt * math.cos(self.pose_yaw) - vy * dt * math.sin(self.pose_yaw)
        self.y += vx * dt * math.sin(self.pose_yaw) + vy * dt * math.cos(self.pose_yaw)
        self.pose_yaw += angular_z * dt

        # Aktualisiere und publiziere die Odometry-Nachricht:
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.orientation = rpy2qua(0, 0, self.pose_yaw)
        self.odom.twist.twist.linear.x = vx
        self.odom.twist.twist.linear.y = vy
        self.odom.twist.twist.angular.z = angular_z

        self.odom_pub.publish(self.odom)

    



