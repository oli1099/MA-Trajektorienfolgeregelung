# MA-Trajektorienfolgeregelung

Dieses Repository dient zur Entwicklung auf dem HiWonder Mentor Pi (Raspberry Pi)
und enthält Implementierungen verschiedener Regelalgorithmen für mobile Roboter.

## Inhaltsverzeichnis

1. [Projektstruktur](#projektstruktur)
2. [Vorraussetzungen](#vorraussetzungen)
3. [Installation](#installation)
4. [Roboter starten](#roboter-starten)
5. [Regelalgorithmen starten](#regelalgorithmen-starten)
6. [Allgemeine ROS 2 Befehle](#allgemeine-ros-2-befehle)

---

## Projektstruktur

```
MA-Trajektorienfolgeregelung/
workspace/ros2_ws/src/MPC/MPC
├── Trajektorienfolgeregelung/   # Pure Pursuit Implementierung
├── MPC/                         # Model Predictive Control Implementierung
└── mpc_obstacle/                # Hindernisvermeidung (statische Hindernisse)

Weitere Ordner:
- controller/                    # ROS2 Package für Controller-Launch
- ldlidar_node/                 # ROS2 Package für Lidar-Integration
- orchestrator_launch/          # ROS2 Package für SLAM-Toolbox
- workspace/ros2_ws/            # ROS2 Workspace mit Quellcode
```

## Vorraussetzungen

* HiWonder Mentor Pi mit installiertem Raspberry Pi OS
* Hier eine Anleitung zum Aufsetzen des HiWonder https://github.com/Matzefritz/HiWonder_MentorPi
* ROS 2 (Foxy, Galactic oder Humble)
* Abhängigkeiten in `requirements.txt` (numpy, scipy, matplotlib, rclpy, ...)

## Installation

1. **SSH-Verbindung zum Roboter**

   ```bash
   ssh prinzessinleia@192.168.1.32
   ```
2. **Ros2 Workspace vorbereiten**

   ```bash
   cd ~/PrinzessinLeia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/ros2_ws
   git pull origin main
   colcon build
   source install/setup.bash
   ```

## Roboter starten

Öffne ein Terminal auf dem Roboter und starte grundlegende Nodes:

```bash
ros2 launch controller controller.launch.py
ros2 launch ldlidar_node ldlidar.launch.py
ros2 launch orchestrator_launch slam_toolbox.launch.py
```

## Regelalgorithmen starten

1. Neues Terminal öffnen auf dem Roboter
2. Ins Workspace-Verzeichnis wechseln:

   ```bash
   cd ~/PrinzessinLeia/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/ros2_ws
   source install/setup.bash
   ```
3. Gewünschten Algorithmus ausführen:

   ```bash
   ros2 run <package_name> <executable_name>
   ```

   * **Pure Pursuit**:  `ros2 run Trajektorienfolgeregelung Trajectorytracking`
   * **MPC**:           `ros2 run MPC MPC_ClosedLoopTrajectory`
   * **Hindernisvermeidung**:  `ros2 run mpc_obstacle mpc_CL_dynObs`

## Allgemeine ROS 2 Befehle

* **Topics auflisten**

  ```bash
  ros2 topic list -t
  ```
* **Topic-Daten anzeigen**

  ```bash
  ros2 topic echo /topic_name
  ```
* **Topic-Informationen**

  ```bash
  ros2 topic info /topic_name
  ```
* **Message-Type untersuchen**

  ```bash
  ros2 interface show geometry_msgs/msg/Twist
  ```

## Beispiel: Manuelle Robotersteuerung

* **Bewegung über `/controller/cmd_vel`**

  ```bash
  ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
  ```
* **Ansteuerung der Radmotoren**

  ```bash
  ros2 topic pub -1 /ros_robot_controller/set_motor \
    ros_robot_controller_msgs/MotorsState \
    "{data: [{id: 1, rps: -0.1}, {id: 2, rps: -0.1}, {id: 3, rps: 0.1}, {id: 4, rps: 0.1}]}"
  ```

---

*Hinweis:* Passe Pfade und Paket-/Executable-Namen an deine lokale Struktur an.
