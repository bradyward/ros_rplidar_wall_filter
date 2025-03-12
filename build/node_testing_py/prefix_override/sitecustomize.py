import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/brady/ros2_iron/src/node_testing_py/install/node_testing_py'
