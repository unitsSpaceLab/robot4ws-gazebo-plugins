#!/usr/bin/env python3.8

import rospy
import pandas
from geometry_msgs.msg import Twist

def cmd_vel_pub():
    # import the cmd_vel file 
    #cmd_vel_path = '/home/ros/data_test/DLR_surface2/cmd_vel.csv'
    #cmd_vel_path = '/home/ros/data_test/DLR_surface3/cmd_vel.csv'
    cmd_vel_path = '/home/ros/data_test/DLR_origin_path/cmd_vel_origin.csv'
    #cmd_vel_path = '/home/ros/data_test/VRC_terrain/cmd_vel.csv'
    #cmd_vel_path = '/home/ros/data_test/simple_straight_paths/path3/path3_cmd_vel.csv'

    file = pandas.read_csv(cmd_vel_path)

    pub = rospy.Publisher('Archimede/cmd_vel',Twist,queue_size=1)
    rospy.init_node('cmd_vel_pub')
    rospy.loginfo('cmd_vel publisher node initialized')

    # reset the starting time
    row = 674 #674, 5574
    final_row = 884  # = len(file.time)-1 for running until end of file, 884, 5735
    file.time = file.time - file.time[row]
    msg = Twist()
    while not rospy.is_shutdown():
        if row == final_row:
            rospy.signal_shutdown('Last message reached')
            rospy.loginfo('Last message reached! Shutting down cmd_vel publisher node')
            continue

        duration = (file.time[row+1] - file.time[row])/1000000000
        msg.linear.x = file.field_linear_x[row]
        msg.linear.y = file.field_linear_y[row]
        msg.linear.z = file.field_linear_z[row]
        msg.angular.x = file.field_angular_x[row]
        msg.angular.y = file.field_angular_y[row]
        msg.angular.z = file.field_angular_z[row]
        pub.publish(msg)
        row = row + 1

        rospy.sleep(duration)


if __name__ == '__main__':
    try:
        cmd_vel_pub()
    except KeyboardInterrupt:
        rospy.loginfo('Shutting down cmd_vel publisher node')