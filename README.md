# MA-Trajektorienfolgeregelung

Dieses Rep dient zur Entwicklung auf dem HiWonder Mentor PI der mit einem rasberry betrieben wird.

Der HiWonder Pi wurde mit folgender Anleitung aufgesetzt: https://github.com/Matzefritz/HiWonder_MentorPi

Stopp the Roboter: ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"

ros2 launch controller controller.launch.py

ros2 run Trajektorienfolgeregelung circle_trajectory_p_regler

Package Erstellen in ROS2:
    ros2 pkg create --build-type ament_python --license Apache-2.0 <package_name>

Starten des Roboters:
    1. SSH Verbindung zum Roboter : sh prinzessinleia@192.168.1.32 (im Lab)
    2. Zum folgenden ordner navigieren: cd /..
    3. Controller Launch vom roboter starten: ros2 launch controller controller.launch.py


