import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/itti_jammy/Desktop/baldybot_cs/src/install/baldybot_cs_commands'
