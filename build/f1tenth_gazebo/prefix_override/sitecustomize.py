import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kuy02/gazebo_ws/src/f1tenth_gazebo_sim/install/f1tenth_gazebo'
