import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/oli/Desktop/Oliver/Uni/MA/RepoTrajektorienfolgeregelung/MA-Trajektorienfolgeregelung/workspace/ros2_ws/install/calibration'
