import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/simi/franka_ros2_ws/install/realtime_plot_of_data'
