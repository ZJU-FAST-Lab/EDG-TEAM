#!/usr/bin/env python  
import rospy
import pygame
from std_msgs.msg import Empty

if __name__ == '__main__':

    # init pygame
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    rospy.init_node('send_stop')
    rate = rospy.Rate(50) # 50hz
    stop_pub = rospy.Publisher("/mandatory_stop_from_users",Empty,queue_size=10)

    while not rospy.is_shutdown():
        rate.sleep()

        pygame.event.pump()

        if (pygame.joystick.Joystick(0).get_button(0) or \
            pygame.joystick.Joystick(0).get_button(1) or \
            pygame.joystick.Joystick(0).get_button(2) or \
            pygame.joystick.Joystick(0).get_button(3)):
                print("STOP!")
                stop_pub.publish(Empty())

    pygame.quit ()

#      ^                ^
#      |                |
# <-      +0->     <-      +2->        
#    +1|              +3|
#      V                V
