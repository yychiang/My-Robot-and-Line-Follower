#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from line_follower.srv import DriveToTarget
from line_follower.srv import DriveToTargetResponse
global des_angle
global steering_motor_action
steering_motor_action = 0

def dc_motor(desired_angle, current_angle):
    global steering_motor_action
    e = desired_angle - current_angle
    steering_motor_action = 0.5*e
    
def joint_states_callback(msg):
    global des_angle
    
    if (msg.position[0]!=0):
        current_angle = msg.position[0]
        dc_motor(des_angle, current_angle)
    else:
        dc_motor(des_angle, 0)

def drive_to_target(req):
    global des_angle
    global steering_motor_action

    motor_command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    steering_command_publisher = rospy.Publisher('/cmd_steering', Twist, queue_size=10)

    motor_command = Twist()
    steering_command = Twist()

    joint_states_sub = rospy.Subscriber('/joint_states', JointState, joint_states_callback)
    
    
    motor_command.linear.x = req.linear_x
    motor_command.linear.y = 0
    motor_command.linear.z = 0
    motor_command.angular.x = 0
    motor_command.angular.y = 0
    motor_command.angular.z = 0

    des_angle = req.angular_z

    steering_command.linear.x = steering_motor_action
    steering_command.linear.y = 0
    steering_command.linear.z = 0
    steering_command.angular.x = 0
    steering_command.angular.y = 0
    steering_command.angular.z = 0

    
    motor_command_publisher.publish(motor_command)
    steering_command_publisher.publish(steering_command)
    s = "linear: "+str(motor_command.linear.x)+" "+"angular: "+ str(steering_command.linear.x)
    return s
    
def drive_bot_server():
    rospy.init_node('drive_bot')
    print("Initialize a ROS node named drive_bot.")
    s = rospy.Service('/line_follower/command_robot', DriveToTarget, drive_to_target)
    print("Ready to drive to target.")
    rospy.spin()

if __name__ == "__main__":
    drive_bot_server()
    
    
