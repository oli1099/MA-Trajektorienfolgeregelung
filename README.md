# MA-Trajektorienfolgeregelung

Dieses Rep dient zur Entwicklung auf dem HiWonder Mentor PI der mit einem rasberry betrieben wird.

Der HiWonder Pi wurde mit folgender Anleitung aufgesetzt: https://github.com/Matzefritz/HiWonder_MentorPi

Stopp the Roboter: ros2 topic pub --once /controller/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0}}"

ros2 launch controller controller.launch.py

ros2 run Trajektorienfolgeregelung circle_trajectory_p_regler
