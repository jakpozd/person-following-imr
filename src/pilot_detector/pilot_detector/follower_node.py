import numpy as np
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from br_interfaces.msg import WheelsVelocity as IMR_Twist
from tf2_ros import TransformBroadcaster

class PilotFollowerNode(Node):
    def __init__(self):
        super().__init__('pilot_follower')
        # variables for IMR
        self.R = 0.14 #radius of wheels
        self.wheels_distance = 0.49 #distance between wheels
        self.transformation_scalar = 3600.0/(2.0 * np.pi) # scalar to transform wheels velocities into IMR units (3600 = 1 rotation/sec)
        self.max_velocity = 3600.0 # max velocity in IMR units
        self.target_distance = None
        self.distance_to_mantain = 1.5
        self.previous_linear_error = 0.0
        self.linear_error_for_breaking = 0.0
        self.decrease = 0.0
        self.Kp = 0.25
        self.target_angle = None
        self.previous_angular_error = 0.0
        self.angle_to_mantain = 0.0
        self.Kd = 0.0025
        self.previous_time = self.get_clock().now()
        self.first_iteration_flg = True
        
        # subscribers, publishers and callbacks
        self.subscriber = self.create_subscription(PoseStamped, 'pilot_position', self.pilot_position_callback, 10)
        #to IMR:
        self.publisher = self.create_publisher(IMR_Twist, '/real_controller/cmd_wheels_vel', 10)

    def pilot_position_callback(self, msg):
        self.target_distance = msg.pose.position.x # in m
        self.target_angle = msg.pose.orientation.z # in radians

        # on IMR PD controller is used:
        cmd_wheels_vel = IMR_Twist()
        if (not np.isnan(self.target_distance)) and (not np.isnan(self.target_angle)):

            # Linear and angular error
            linear_error = self.target_distance - self.distance_to_mantain
            angular_error = self.target_angle - self.angle_to_mantain

            current_time = self.get_clock().now()
            time_delta = (current_time - self.previous_time).nanoseconds / 1e9

            # protection against dividing by 0
            if time_delta > 0.0:
                dx = (linear_error - self.previous_linear_error)/time_delta
                dalpha = (angular_error - self.previous_angular_error)/time_delta   
            else:
                dx = 0.0
                dalpha = 0.0

            # protection against large dx and dalpha in first iteration 
            if self.first_iteration_flg == True:
                dx = 0.0
                dalpha = 0.0

            if abs(linear_error) >= 0.05 * self.distance_to_mantain:
                linear_velocity_ref = self.Kp * linear_error + self.Kd * dx 
            else: linear_velocity_ref = 0.0
            if abs(angular_error) >= 0.03491: # ca 2 degrees
                angular_velocity_ref = self.Kp * angular_error + self.Kd * dalpha 
            else: angular_velocity_ref = 0.0

            # wheels velocities
            left_wheel_velocity_ref = (linear_velocity_ref - self.wheels_distance/2 * angular_velocity_ref) / self.R
            left_wheel_velocity_ref_scaled = left_wheel_velocity_ref * self.transformation_scalar
            right_wheel_velocity_ref = (linear_velocity_ref + self.wheels_distance/2 * angular_velocity_ref) / self.R
            right_wheel_velocity_ref_scaled = right_wheel_velocity_ref * self.transformation_scalar


            # min-max scaler in case if calculated velocities exceeds assumed maximum velocity 
            if (abs(left_wheel_velocity_ref_scaled) > self.max_velocity) or (abs(right_wheel_velocity_ref_scaled) > self.max_velocity): 
                X_max = max(abs(right_wheel_velocity_ref_scaled), abs(left_wheel_velocity_ref_scaled))
                X_min = 0.0
                right_wheel_velocity_ref_scaled = X_min + right_wheel_velocity_ref_scaled/X_max * (self.max_velocity - X_min)
                left_wheel_velocity_ref_scaled = X_min + left_wheel_velocity_ref_scaled/X_max * (self.max_velocity - X_min)

            # actualisaation of variables
            self.previous_linear_error = linear_error
            self.previous_angular_error = angular_error
            self.previous_time = current_time
            self.first_iteration_flg = False

####################################################################################
            #for breaking when no detection:
            self.linear_error_for_breaking = linear_error
            self.decrease = linear_error*0.1 
####################################################################################

            cmd_wheels_vel.w_l = round(left_wheel_velocity_ref_scaled,4) 
            cmd_wheels_vel.w_r = round(right_wheel_velocity_ref_scaled,4)
            self.publisher.publish(cmd_wheels_vel)

        # when there is no detection, we want to stop. In order not to do this abruptly,  mini PD controller is used, assuming an error reduction of 10% for 0.1 second
        else:
            angular_velocity_ref = 0 # just want to stop, no need for angular velocity
            
            # linear breaking:
            linear_velocity_ref = self.Kp * self.linear_error_for_breaking 

            left_wheel_velocity_ref = (linear_velocity_ref - self.wheels_distance/2 * angular_velocity_ref) / self.R
            left_wheel_velocity_ref_scaled = left_wheel_velocity_ref * self.transformation_scalar
            right_wheel_velocity_ref = (linear_velocity_ref + self.wheels_distance/2 * angular_velocity_ref) / self.R
            right_wheel_velocity_ref_scaled = right_wheel_velocity_ref * self.transformation_scalar

            # actualisation of error for velocity calculation:
            if abs(self.linear_error_for_breaking) >= 0.05 * self.distance_to_mantain:
                self.linear_error_for_breaking -=  self.decrease
            else: self.linear_error_for_breaking = 0.0

            #publish velocities:
            cmd_wheels_vel.w_l = round(left_wheel_velocity_ref_scaled,4) 
            cmd_wheels_vel.w_r = round(right_wheel_velocity_ref_scaled,4)
            self.publisher.publish(cmd_wheels_vel)

def main(args=None):
    rclpy.init()
    pilot_follower_node = PilotFollowerNode()
    rclpy.spin(pilot_follower_node)
    pilot_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
