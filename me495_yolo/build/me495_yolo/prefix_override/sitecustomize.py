import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/catherine-maglione/yolo_home/src/me495_yolo/install/me495_yolo'
