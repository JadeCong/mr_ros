#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import String


def CmdLoop():
    rospy.init_node('CmdLoop', anonymous=False)
    pub = rospy.Publisher('type_pos', String, queue_size=10)
    rate = rospy.Rate(0.25)  # 2Hz
    
    count = 0
    type_pos_cmd1 = "4154068F4446500000AA"
    type_pos_cmd2 = "4154068F442328000071"
    
    while not rospy.is_shutdown():
        if count % 2 == 0:
            # By-hand command: rostopic pub -1 /type_pos std_msgs/String -- "4154068F4446500000AA"
            rospy.loginfo("type_pos_loop: %s", type_pos_cmd1)
            pub.publish(type_pos_cmd1)
        else:
            rospy.loginfo("type_pos_loop: %s", type_pos_cmd2)
            pub.publish(type_pos_cmd2)
        rate.sleep()
        count += 1


if __name__ == '__main__':
    try:
        CmdLoop()
    except rospy.ROSInterruptException:
        pass
