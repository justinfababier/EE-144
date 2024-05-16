#!/usr/bin/env python3

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class PDController:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)
        self.previous_error = 0

    def update(self, current_value):
        # calculate P_term and D_term
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error) / 0.1
        self.previous_error = error
        return P_term + D_term

    def set_setpoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def set_parameters(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class TurtlebotController:
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for _ in range(10):
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
        waypoints = [(0, 0), (0.5, 0), (1.0, 0), (1.0, -0.5), (1.5, -0.5), (2.0, -0.5), (2.5, -0.5), (3.0, -0.5),
                      (3.5, -0.5), (3.5, 0.0), (3.5, 0.5), (3.5, 1.0), (4.0, 1.0), (4.5, 1), (4.5, 1)]
        
        self.prev_Vxt = 0.0
        self.prev_Vyt = 0.0

        for i in range(len(waypoints) - 1):
            self.move_to_point(waypoints[i], waypoints[i + 1])

    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint

        vel_ref = 0.2
        t_complete = 2

        pd_controller = PDController()
        pd_controller.set_parameters(0.5, 0.1)

        p_sx, p_ex = current_waypoint[0], next_waypoint[0]
        p_sy, p_ey = current_waypoint[1], next_waypoint[1]

        v_sx, v_sy = self.prev_Vxt, self.prev_Vyt

        diffx = p_ex - p_sx
        diffy = p_ey - p_sy

        angle = atan2(diffy, diffx)

        v_ex = vel_ref * cos(angle)
        v_ey = vel_ref * sin(angle)

        vel = Twist()

        coeff_x = self.polynomial_time_scaling_3rd_order(p_sx, v_sx, p_ex, v_ex, t_complete)
        coeff_y = self.polynomial_time_scaling_3rd_order(p_sy, v_sy, p_ey, v_ey, t_complete)

        for i in range(int(t_complete * 10)):
            t = i * 0.1

            V_x_t = coeff_x[2] + 2 * (coeff_x[1] * t) + 3 * (coeff_x[0] * pow(t, 2))
            V_y_t = coeff_y[2] + 2 * (coeff_y[1] * t) + 3 * (coeff_y[0] * pow(t, 2))

            V_t = sqrt(pow(V_x_t, 2) + pow(V_y_t, 2))

            vel.linear.x = V_t

            theta = atan2(V_y_t, V_x_t)

            if theta <= 0 and self.pose.theta >= 0 and self.pose.theta - theta >= pi:
                theta += 2 * pi
            elif theta > 0 and self.pose.theta < 0 and theta - self.pose.theta > pi:
                theta -= 2 * pi

            pd_controller.set_setpoint(theta)
            vel.angular.z = pd_controller.update(self.pose.theta)

            self.vel_pub.publish(vel)
            self.rate.sleep()

            self.prev_Vxt = V_x_t
            self.prev_Vyt = V_y_t

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial

        t_mat = np.array([[0, 0, 0, 1], [pow(T, 3), pow(T, 2), T, 1], [0, 0, 1, 0], [3 * pow(T, 2), 2 * T, 1, 0]])

        x_mat = np.array([[p_start], [p_end], [v_start], [v_end]])

        t_mat_inv = np.linalg.inv(t_mat)
        coeff = np.dot(t_mat_inv, x_mat)

        return coeff

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=%s;  y=%s;  theta=%s", str(self.pose.x), str(self.pose.y), str(yaw))

if __name__ == '__main__':
    turtlebot_controller = TurtlebotController()
