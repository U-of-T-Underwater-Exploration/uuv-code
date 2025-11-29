import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jeff/Jeff/UTUX/2025-2026/Template/sensor-compass/uuv-code/install/uuv_compass_driver'
