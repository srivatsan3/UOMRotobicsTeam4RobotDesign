import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mscrobotics2425laptop11/interbotix_ws/install/interbotix_common_modules'
