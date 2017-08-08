#!/usr/bin/env python

'''
ackermann_drive_joyop.py:
    A ros joystick teleoperation script for ackermann steering based robots
'''

__author__ = 'George Kouros'
__license__ = 'GPLv3'
__maintainer__ = 'George Kouros'
__email__ = 'gkourosg@yahoo.gr'

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import Joy
import sys
import math

class AckermannDriveJoyop:

    def __init__(self, args):
        #print("args length:",len(args))
        #print("args:",args)
        self.enable_button = rospy.get_param('~enable_button', -1)
        self.speed_axis = rospy.get_param('~speed_axis', 5)
        self.speed_axis_invert = rospy.get_param('~speed_axis_invert', True)
        self.speed_axis_centered = rospy.get_param('~speed_axis_centered', False)
        self.steer_axis = rospy.get_param('~steer_axis', 0)
        self.steer_axis_invert = rospy.get_param('~steer_axis_invert', False)
        self.steer_vel_axis = rospy.get_param('~steer_vel_axis', 4)
        self.max_speed = rospy.get_param('~max_speed', 1.0)
        self.max_steering_angle = rospy.get_param('~max_steering_angle', 0.7)
        self.max_angle_vel = rospy.get_param('~max_angle_vel', 20)
        self.min_angle_vel = rospy.get_param('~min_angle_vel', 5)
        cmd_topic = rospy.get_param('~cmd_topic', '/ackermann_cmd')
        self.acceleration_axis = rospy.get_param('~acceleration_axis', 2)
        self.max_acceleration = rospy.get_param('~max_acceleration', 1.5)
        self.acceleration_axis_centered = rospy.get_param('~acceleration_axis_centered', False)
        self.acceleration_axis_invert = rospy.get_param('~acceleration_axis_invert', True)

        #List of Parm
        self.speed = 0
        self.steering_angle = 0
        self.acceleration = 0
        self.steer_angle_vel =0
        self.got_true_val_speed_axis=False
        self.got_true_val_acceleration_axis=False
        self.joy_sub = rospy.Subscriber('/joy', Joy, self.joy_callback)
        self.drive_pub = rospy.Publisher(cmd_topic, AckermannDriveStamped,queue_size=1)
        rospy.Timer(rospy.Duration(1.0/5.0), self.pub_callback, oneshot=False)
        rospy.loginfo('ackermann_drive_joyop_node initialized')

    def joy_callback(self, joy_msg):

        if(joy_msg.axes[self.speed_axis] != 0) :      # New code segment to set default speed & acceleration axis value to Zero
                self.got_true_val_speed_axis=True
        if(joy_msg.axes[self.acceleration_axis] != 0) :
                self.got_true_val_acceleration_axis=True

        speed_axis_val=0;
        if self.speed_axis_centered == False and self.got_true_val_speed_axis== True :
            speed_axis_val= 1;
        if self.speed_axis_invert == True :
            speed_axis_val=speed_axis_val-joy_msg.axes[self.speed_axis];
        else:
            speed_axis_val=joy_msg.axes[self.speed_axis]-speed_axis_val;

        if self.speed_axis_centered == False and self.got_true_val_speed_axis== True :
            speed_axis_val=speed_axis_val/2.0;
        self.speed = speed_axis_val * self.max_speed;

        if self.steer_axis_invert ==True:
            self.steering_angle = -joy_msg.axes[self.steer_axis] * self.max_steering_angle;
        else:
            self.steering_angle = joy_msg.axes[self.steer_axis] * self.max_steering_angle;

        if joy_msg.axes[self.steer_vel_axis] >=0 :
          self.steer_angle_vel=  self.min_angle_vel + joy_msg.axes[self.steer_vel_axis] * (self.max_angle_vel -self.min_angle_vel)
        else:
          self.steer_angle_vel=  self.min_angle_vel

        acceleration_axis_val=0;
        if self.acceleration_axis_centered == False and self.got_true_val_acceleration_axis == True :
            acceleration_axis_val= 1;
        if self.acceleration_axis_invert == True :
            acceleration_axis_val=acceleration_axis_val-joy_msg.axes[self.acceleration_axis];
        else:
            acceleration_axis_val=joy_msg.axes[self.acceleration_axis]-acceleration_axis_val;

        if self.acceleration_axis_centered ==  False and self.got_true_val_acceleration_axis == True :
            acceleration_axis_val=acceleration_axis_val/2.0;
        self.acceleration = acceleration_axis_val * self.max_acceleration;


    def pub_callback(self, event):
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = rospy.Time.now()
        ackermann_cmd_msg.drive.speed = self.speed
        ackermann_cmd_msg.drive.steering_angle = self.steering_angle
        ackermann_cmd_msg.drive.steering_angle_velocity = self.steer_angle_vel
        ackermann_cmd_msg.drive.acceleration = self.acceleration
        self.drive_pub.publish(ackermann_cmd_msg)
        self.print_state()

    def print_state(self):
        sys.stderr.write('\x1b[2J\x1b[H')
        rospy.loginfo('\x1b[1M\r'
                      '\033[34;1mSpeed: \033[32;1m%0.2f m/s, '
                      '\033[34;1mSteering Angle: \033[32;1m%0.2f rad\033[0m,  '
                      '\033[34;1mAcceleration: \033[32;1m%0.2f m/s',
                      self.speed, self.steering_angle, self.acceleration)

    def finalize(self):
        rospy.loginfo('Halting motors, aligning wheels and exiting...')
        ackermann_cmd_msg = AckermannDriveStamped()
        ackermann_cmd_msg.header.stamp = rospy.Time.now()
        ackermann_cmd_msg.drive.speed = 0
        ackermann_cmd_msg.drive.steering_angle = 0
        ackermann_cmd_msg.drive.self.acceleration = 0
        self.drive_pub.publish(ackermann_cmd_msg)
        sys.exit()

if __name__ == '__main__':
    rospy.init_node('ackermann_drive_joyop_node')
    joyop = AckermannDriveJoyop(sys.argv[1:len(sys.argv)])
    rospy.spin()
