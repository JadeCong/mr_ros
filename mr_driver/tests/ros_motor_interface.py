#!/usr/bin/env python

import rospy
import time
import serial
from std_msgs.msg import String


ser = serial.Serial(port='/dev/ttyUSB0', baudrate=115200, bytesize=8, parity='N', stopbits=1, timeout=2, rtscts=True, dsrdtr=True)
if ser.isOpen():
    print("Serial has been opened")
else:
    print("Open the serial")
    ser.open()

# Receive command from string
# def MotorPositionDrive():
#     rospy.init_node('cmd_pos_publisher', anonymous=False)
#     pub = rospy.Publisher('cmd_pos', String, queue_size=10)
#     rate = rospy.Rate(2)
    
#     while not rospy.is_shutdown():
#         raw_data = "4154068F4446500000AA"  # 180 degree
#         # raw_data = "4154068F442328000071"  # 90 degree
#         rospy.loginfo(raw_data)
#         send_data = bytes.fromhex(raw_data)
        
#         ser.write(send_data)
#         print(ser.readall())
#         pub.publish(send_data)
#         rate.sleep()

# Receive command from type_pos topic
def CmdCallback(data, args):
    rospy.loginfo("Hex cmd data: %s", data.data)
    args[0].publish(data.data)  # pub_cmd
    send_data = bytes.fromhex(data.data)
    ser.write(send_data)
    
    receive_data = ser.readall().hex()
    rospy.loginfo("Hex rtn data: %s", receive_data)
    args[1].publish(receive_data)  # pub_rtn

def MotorPositionDrive():
    rospy.init_node('motor_position_driver', anonymous=False)
    
    pub_cmd = rospy.Publisher('cmd_pos', String, queue_size=10)
    pub_rtn = rospy.Publisher('rtn_pos', String, queue_size=10)
    rospy.Subscriber('type_pos', String, CmdCallback, (pub_cmd, pub_rtn), queue_size=10)
    
    rospy.spin()


if __name__ == '__main__':
    print("MotorPositionDrive Starting...")
    try:
        MotorPositionDrive()
    except rospy.ROSInterruptException:
        pass
    print("MotorPositionDrive Finished")
