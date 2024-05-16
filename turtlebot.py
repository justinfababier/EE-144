#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        # Initialize the PD controller with default or provided values
        self.Kp = P
        self.Kd = D
        self.set_point = set_point  # reference (desired value)

    def update(self, current_value):
        # Update the PD controller and calculate the control output
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error) / 0.1  # Derivative term with fixed time step
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        # Set a new reference (desired value) for the controller
        self.set_point = set_point
        self.previous_error = 0  # Reset previous error

    def setPD(self, P=0.0, D=0.0):
        # Set new proportional and derivative gains for the controller
        self.Kp = P
        self.Kd = D

class Turtlebot():
    def __init__(self):
        # Initialize the TurtleBot node and publishers/subscribers
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Control loop rate

        # Reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # Subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()  # Execute the main functionality
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # Save trajectory into a CSV file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        # Main control loop
        start = (0, 0)
        goal = (9, 2)
        obstacles = [(1, 0), (1, 1), (4, 1), (4, 2), (4, 3), (5, 1), (5, 2), (5, 3), (8, 0), (8, -1)]
        waypoints = self.get_path_from_A_star(start, goal, obstacles)
        waypoints.append(waypoints[-1])
        print(waypoints)

        for i in range(len(waypoints) - 1):
            self.move_to_point(waypoints[i], waypoints[i + 1])
            print("Current waypoint")
            print(waypoints[i])

    def move_to_point(self, current_waypoint, next_waypoint):
        # Move the TurtleBot along a polynomial trajectory from current to next waypoint
        vel_ref = 0.1
        t_complete = 5

        pdController = Controller()
        pdController.setPD(0.5, 0.1)

        # Extract waypoint coordinates and calculate velocity components
        p_sx, p_ex = current_waypoint[0], next_waypoint[0]
        p_sy, p_ey = current_waypoint[1], next_waypoint[1]
        v_sx, v_sy = 0.0, 0.0

        diffx, diffy = p_ex - p_sx, p_ey - p_sy
        angle = atan2(diffy, diffx)
        v_ex, v_ey = vel_ref * cos(angle), vel_ref * sin(angle)

        vel = Twist()
        vel.angular.z, vel.linear.x = 0.0, 0.0

        # Calculate coefficients for the polynomial trajectory
        coeff_x = self.polynomial_time_scaling_3rd_order(p_sx, v_sx, p_ex, v_ex, t_complete)
        coeff_y = self.polynomial_time_scaling_3rd_order(p_sy, v_sy, p_ey, v_ey, t_complete)

        for i in range(int(t_complete * 10)):
            t = i * 0.1  # Time step

            # Calculate velocity components from polynomial coefficients
            V_x_t = coeff_x[2] + 2 * (coeff_x[1] * t) + 3 * (coeff_x[0] * pow(t, 2))
            V_y_t = coeff_y[2] + 2 * (coeff_y[1] * t) + 3 * (coeff_y[0] * pow(t, 2))
            V_t = sqrt(pow(V_x_t, 2) + pow(V_y_t, 2))

            vel.linear.x = V_t

            theta = atan2(V_y_t, V_x_t)

            # Handle angle wrapping for smooth angular control
            if theta <= 0 and self.pose.theta >= 0 and self.pose.theta - theta >= pi:
                theta += 2 * pi
            elif theta > 0 and self.pose.theta < 0 and theta - self.pose.theta > pi:
                theta -= 2 * pi
            else:
                pass

            pdController.setPoint(theta)

            # Update angular velocity using the PD controller
            vel.angular.z = pdController.update(self.pose.theta)

            self.vel_pub.publish(vel)
            self.rate.sleep()

    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # Calculate coefficients for a third-order polynomial trajectory
        t_mat = np.array([[0, 0, 0, 1], [pow(T, 3), pow(T, 2), T, 1], [0, 0, 1, 0], [3 * pow(T, 2), 2 * T, 1, 0]])
        x_mat = np.array([[p_start], [p_end], [v_start], [v_end]])
        t_mat_inv = np.linalg.inv(t_mat)
        coeff = np.dot(t_mat_inv, x_mat)
        return coeff

    def odom_callback(self, msg):
        # Extract and log the robot's pose from odometry data
        quaternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Log the trajectory once every 100 times
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # Save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +
                          ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))

    def neighbors(self, current):
        # Define the list of 4 neighbors: up, down, left, and right
        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
        return [(current[0] + nbr[0], current[1] + nbr[1]) for nbr in neighbors]

    def heuristic_distance(self, candidate, goal):
        # Calculate the Manhattan distance heuristic between two points
        dx, dy = abs(candidate[0] - goal[0]), abs(candidate[1] - goal[1])
        return dx + dy

    def get_path_from_A_star(self, start, goal, obstacles):
        # A* algorithm to find the path from start to goal on a grid with obstacles

        # Initialize path and list for open nodes
        path = []
        open_list = [(0, start)]  # List with cost and node tuple

        # Lists to track closed nodes, past costs, and parent nodes
        closed_list = []
        past_cost = {start: 0}
        parent = {start: None}

        while open_list:
            # Sort the open list based on cost
            open_list.sort()
            current = open_list.pop(0)[1]

            if current == goal:
                # Goal reached, reconstruct the path and return
                break

            if current in obstacles:
                # Skip nodes that are obstacles
                continue

            closed_list.append(current)  # Add current node to the closed list

            for nbr in self.neighbors(current):
                if nbr in obstacles:
                    # Skip neighbors that are obstacles
                    continue

                tentative_past_cost = past_cost[current] + 1

                if nbr not in past_cost or tentative_past_cost < past_cost[nbr]:
                    # Update past cost and parent if a better path is found
                    past_cost[nbr] = tentative_past_cost
                    parent[nbr] = current
                    new_cost = past_cost[nbr] + self.heuristic_distance(nbr, goal)
                    open_list.append((new_cost, nbr))

        # Reconstruct the path from goal to start
        while current != start:
            path.append(current)
            current = parent[current]

        path.reverse()  # Reverse the path to start from the beginning

        return path

if __name__ == '__main__':
    whatever = Turtlebot()