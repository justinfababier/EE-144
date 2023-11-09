#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()
	# go to 1st waypoint (4.0,0.0): PASSED	
        vel.linear.x = 0.5
        vel.angular.z = 0.01
        for i in range(80):		
            self.vel_pub.publish(vel)
            self.rate.sleep()
	
	# rotate at 1st waypoint
        vel.linear.x = 0.0
        vel.angular.z = 0.325
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()
	
	# go to 2nd waypoint (4.0,4.0): PASSED
        vel.linear.x = 0.5
        vel.angular.z = 0.01
        for i in range(80):		
            self.vel_pub.publish(vel)
            self.rate.sleep()
	
	# rotate at 2nd waypoint
        vel.linear.x = 0.0
        vel.angular.z = 0.325
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()	

	# go to 3rd waypoint (4.0,4.0): PASSED
        vel.linear.x = 0.5
        vel.angular.z = -0.01
        for i in range(80):		
            self.vel_pub.publish(vel)
            self.rate.sleep()

	# rotate at 3rd waypoint
        vel.linear.x = 0.0
        vel.angular.z = 0.325
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()

	# go to 4th waypoint (4.0,4.0): PASSED
        vel.linear.x = 0.5
        vel.angular.z = -0.0455
        for i in range(70):		
            self.vel_pub.publish(vel)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
