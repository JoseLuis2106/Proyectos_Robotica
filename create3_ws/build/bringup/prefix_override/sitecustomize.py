import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jlmre/Proyectos_Robotica/create3_ws/install/bringup'
