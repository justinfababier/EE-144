#!/usr/bin/env python

from math import pi
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
    def __init__(self, P = 0.0, D = 0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # Update controller based on current feedback value
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error	
        return P_term + D_term

    def setPoint(self, set_point):
        # Set a new point for the controller
        self.set_point = set_point
        self.previous_error = 0
    
    def setPD(self, P = 0.0, D = 0.0):
        # Set new P and D gains for the controller
        self.Kp = P
        self.Kd = D

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        # add your code here to adjust your movement based on 2D pose feedback
        # Use the Controller
        ctrl = Controller(0.925, 0.45, 0)
        vel = Twist()

        """
        vel.linear.x = const*                             - set linear velocity in the x-axis
        vel.linear.z = const**                            - set linear velocity in the z-axis
        vel.angular.z = const****                         - set angular velocity about the z-axis
        ctrl.setPoint(const*****)                         - set the control point
        while abs(self.pose.var - const******) > 0.025:   - compare magnitude of pose to 0.025, continues loop until magnitude is < 0.025
            vel.angular.z = ctrl.update(self.pose.theta)    - updates angular velocity about the z-axis
            vel.linear.z = ctrl.update(self.pose.theta)     - updates linear velocity in the z-axis
            self.vel_pub.publish(vel)                       - publish velocity command to control robot's motion
            self.rate.sleep()                               - controls loop rate to 10 Hz
        """

        vel.linear.x = 0.5
        vel.linear.z = 0.0
        ctrl.setPoint(0)
        while abs(self.pose.x - 4) > 0.05:                 # (0,0,0) -> (4,0,0)
            vel.angular.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
        vel.linear.x = 0.0
        vel.angular.z = 0.1
        ctrl.setPoint(pi/2)
        while abs(self.pose.theta - pi/2) > 0.025:           # turn 90 degrees cw, face pi/2
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()

        vel.angular.z = 0.0
        vel.linear.x = 0.5
        ctrl.setPoint(0)
        while abs(self.pose.y - 4) > 0.05:                 # (4,0,0) -> (4,4,pi/2)
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()

        vel.linear.x = 0.0
        vel.angular.z = 0.1
        ctrl.setPoint(pi)
        while abs(self.pose.theta - pi) > 0.025:             # turn 90 degrees cw, face pi
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()

        vel.angular.z = 0.0
        vel.linear.x = 0.5
        while abs(self.pose.x - 0) > 0.05:                 # (4,4,pi/2) -> (4,4,pi)
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()

        vel.linear.x = 0.0
        vel.angular.z = 0.1
        ctrl.setPoint(3*pi/2)
        while abs(self.pose.theta - -pi/2) > 0.025:          # turn 90 degrees cw, face -pi/2
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()

        vel.angular.z = 0
        vel.linear.x = 0.5
        ctrl.setPoint(0)
        while abs(self.pose.y - 0) > 0.05:                 # (4,4,pi) -> (0,0,0)
            vel.linear.z = ctrl.update(self.pose.theta)
            self.vel_pub.publish(vel)
            self.rate.sleep()
        
    pass


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            # display (x, y, theta) on the terminal
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()

import matplotlib.pyplot as plt

def visualization():
    # load csv file and plot trajectory 
    _, ax = plt.subplots(1)
    ax.set_aspect('equal')

    trajectory = np.loadtxt("trajectory.csv", delimiter=',')
    plt.plot(trajectory[:, 0], trajectory[:, 1], linewidth=2)

    plt.xlim(-1, 5)
    plt.ylim(-1, 5)
    plt.show()

if __name__ == '__main__':
    visualization()