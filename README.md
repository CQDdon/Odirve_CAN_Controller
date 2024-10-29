# Odirve_CAN_Controller
A ROS2 package to control Odrive in closed loop mode by velocity and angle.

Command format: [Device ID],[control mode],[value]

Device ID: hexadecimal

control mode:
- vel: velocity
- ang: angle

value: float

Eg: 1a,vel,100.67

## odrive_can
This is folder contain all the node and some code we use to test some function, which you might need to use for a better understand to our code.

### keyboard_sender_node.py
This is the publisher.

### odrive_sender_node.py

### test_sender.py

### test.py