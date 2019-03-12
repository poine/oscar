#!/usr/bin/env python
import sys, roslib, rospy, nav_msgs.msg , geometry_msgs.msg, ackermann_msgs.msg
import math, numpy as np
import matplotlib.pyplot as plt

import pdb

import two_d_guidance as tdg
import two_d_guidance.ros_utils as ros_utils
import homere_control.io_dataset as iods


class PWMSetpoint:
    def __init__(self):
        self.pwm_vals = [-0.3, -0.2, -0.1, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        self.step_dur = 10.
        self.full_dur = len(self.pwm_vals)*self.step_dur

    def get_pwm(self, t):
        #pdb.set_trace()
        idx = int(math.fmod(t, self.full_dur)/self.step_dur)
        return self.pwm_vals[idx]


class Node:
    def __init__(self):
        ack_cmd_topic = rospy.get_param('~ack_cmd_topic', '/oscar_ackermann_controller/cmd_ack')
        self.pub_ack = rospy.Publisher(ack_cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)

        path_filename = rospy.get_param('~path_filename', '/home/poine/work/two_d_guidance/paths/demo_z/oval_01.npz')
        param = tdg.pure_pursuit.Param()
        param.L = 0.08
        self.ctl = tdg.pure_pursuit.PurePursuit(path_filename, param)
        self.vel_sp = PWMSetpoint()
        
        self.robot_pose_topic_odom = rospy.get_param('~robot_pose_topic_odom', '/oscar_v0/base_link_truth')
        #self.robot_listener = ros_utils.GazeboTruthListener(topic=self.robot_pose_topic_odom)
        self.robot_listener = ros_utils.SmocapListener()
        
    def periodic(self):
        try:
            p0, psi = self.robot_listener.get_loc_and_yaw()
            _unused, self.alpha = self.ctl.compute_looped(p0, psi)
            self.publish_ackermann_cmd(self.alpha, self.vel_sp.get_pwm(rospy.Time.now().to_sec()))
        except ros_utils.RobotLostException:
            print('robot lost\r')
        except ros_utils.RobotNotLocalizedException:
            print('robot not localized\r')
        
    def publish_ackermann_cmd(self, alpha, v):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = alpha
        msg.drive.speed = v
        self.pub_ack.publish(msg)

    def run(self):
        self.rate = rospy.Rate(20.)
        while not rospy.is_shutdown():
            self.periodic()
            self.rate.sleep()
            
        
if __name__ == '__main__':
    rospy.init_node('lvel_calibration')
    if 1: # send calibration inputs
        Node().run()
    if 0: # compute calibration from recorded data
        ds = iods.DataSet('/tmp/oscar_io.npz', 'oscar')
        iods.plot_encoders(ds)
        plt.show()
