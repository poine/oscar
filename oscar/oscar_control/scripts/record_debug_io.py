#!/usr/bin/env python
import time, math, numpy as np, sys
import rospy, rospkg, sensor_msgs.msg, nav_msgs.msg, geometry_msgs.msg


import pdb
import oscar_control.msg
import julie_misc.utils as jmu

class Node:
    def __init__(self):
        rospy.init_node('record_debug_io')
        rospy.Subscriber('/oscar_ackermann_controller/debug_io', oscar_control.msg.msg_debug_io, self.debug_io_callback)
        self.lw_angle, self.rw_angle = [], []
        self.lw_pwm, self.rw_pwm = [], []
        self.io_stamp = []

        rospy.Subscriber('/smocap/est_marker', geometry_msgs.msg.PoseWithCovarianceStamped, self.smocap_cbk)
        self.mocap_pos = []
        self.mocap_ori = []
        self.mocap_stamp = []


    def debug_io_callback(self, msg):
        self.lw_angle += msg.lw_angle[:msg.nb_data]
        self.rw_angle += msg.rw_angle[:msg.nb_data]
        self.lw_pwm += msg.lw_pwm[:msg.nb_data]
        self.rw_pwm += msg.rw_pwm[:msg.nb_data]
        self.io_stamp += [_s.to_sec() for _s in msg.stamp[:msg.nb_data]]

    def smocap_cbk(self, msg):
        self.mocap_pos.append(jmu.list_of_xyz(msg.pose.pose.position))
        self.mocap_ori.append(jmu.list_of_xyzw(msg.pose.pose.orientation))
        self.mocap_stamp.append(msg.header.stamp)

        
    def run(self):
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            print('recorded {} io and {} mocap'.format( len(self.io_stamp), len(self.mocap_stamp)))
            rate.sleep()

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename,
                 encoders_lw = np.array(self.lw_angle),
                 encoders_rw = np.array(self.rw_angle),
                 pwm_lw = np.array(self.lw_pwm),
                 pwm_rw = np.array(self.rw_pwm),
                 encoders_stamp = np.array(self.io_stamp),
                 mocap_pos   = np.array(self.mocap_pos),
                 mocap_ori   = np.array(self.mocap_ori),
                 mocap_stamp = np.array(self.mocap_stamp)
        )

        
if __name__ == '__main__':
    node = Node()
    try:
        node.run()
    except rospy.ROSInterruptException:
        print('recorded {} odometry and {} mocap'.format( len(node.io_stamp), len(node.mocap_stamp)))
    output_filename = '/tmp/oscar_io' if len(sys.argv) < 2 else sys.argv[1]
    node.save(output_filename)

