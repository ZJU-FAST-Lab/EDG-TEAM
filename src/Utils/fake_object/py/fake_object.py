#!/usr/bin/env python  
import rospy
import tf
import pygame
from nav_msgs.msg import Odometry

if __name__ == '__main__':

    # init pygame
    pygame.display.init()
    pygame.joystick.init()
    pygame.joystick.Joystick(0).init()

    rospy.init_node('fake_object')
    odom_pub = rospy.Publisher("object_odom", Odometry, queue_size=100)

    x = 0
    y = 0
    z = 1.5
    last_x = 0
    last_y = 0
    last_z = 1.5
    last_t = rospy.Time.now()
    yaw = 0
    rate = rospy.Rate(50) # 50hz
    while not rospy.is_shutdown():
        br = tf.TransformBroadcaster()
        br.sendTransform((x, y, z), tf.transformations.quaternion_from_euler(0, 0, yaw), rospy.Time.now(), "my_view", "world")

        rate.sleep()

        pygame.event.pump()
        x -= pygame.joystick.Joystick(0).get_axis(3) / 3
        y -= pygame.joystick.Joystick(0).get_axis(3) / 3
        z -= pygame.joystick.Joystick(0).get_axis(1) / 3
        yaw += pygame.joystick.Joystick(0).get_axis(0) / 3

        if pygame.joystick.Joystick(0).get_button(0):
            x = 0
            y = 0
            z = 1.5
            yaw = 0
        
        t = rospy.Time.now()
        send_Od_data = Odometry()
        send_Od_data.header.stamp = t
        send_Od_data.header.frame_id = "world"
        send_Od_data.pose.pose.position.x = x
        send_Od_data.pose.pose.position.y = y
        send_Od_data.pose.pose.position.z = z
        send_Od_data.twist.twist.linear.x = (x - last_x) / (t-last_t).to_sec()
        send_Od_data.twist.twist.linear.y = (y - last_y) / (t-last_t).to_sec()
        send_Od_data.twist.twist.linear.z = (z - last_z) / (t-last_t).to_sec()
        odom_pub.publish(send_Od_data)

        last_x = x
        last_y = y
        last_z = z
        last_t = t

    pygame.quit ()

#      ^                ^
#      |                |
# <-      +0->     <-      +2->        
#    +1|              +3|
#      V                V
