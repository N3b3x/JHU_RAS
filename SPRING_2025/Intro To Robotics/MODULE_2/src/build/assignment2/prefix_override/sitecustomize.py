import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/n3b3x/Data Interchange Drive/SCHOOL/JHU/SPRING_2025/Intro To Robotics/MODULE_2/src/install/assignment2'
