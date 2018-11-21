#!/usr/bin/env python
import sys, roslib, rospy, nav_msgs.msg , geometry_msgs.msg, visualization_msgs.msg
import math, numpy as np

import two_d_guidance as tdg
import utils

#
# rewrite of pure pursuit
#
# TODO: fix debug publishing

class NodePublisher:
    def __init__(self):
        self.pub_path = rospy.Publisher('pure_pursuit/path', nav_msgs.msg.Path, queue_size=1)
        self.pub_goal =  rospy.Publisher('pure_pursuit/goal', visualization_msgs.msg.Marker, queue_size=1)
        self.pub_arc = rospy.Publisher('pure_pursuit/arc', nav_msgs.msg.Path, queue_size=1)
    
    def publish_path(self, _path):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for l in _path.points:
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

    def publish_debug(self):
        marker_msg = visualization_msgs.msg.Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id="world"
        marker_msg.type = visualization_msgs.msg.Marker.CYLINDER
        marker_msg.pose.position.x = self.ctl.p2[0]
        marker_msg.pose.position.y = self.ctl.p2[1]
        marker_msg.pose.position.z = 0
        marker_msg.pose.orientation.x = 0;
        marker_msg.pose.orientation.y = 0;
        marker_msg.pose.orientation.z = 0;
        marker_msg.pose.orientation.w = 0;
        marker_msg.scale.x = .01
        marker_msg.scale.y = .01
        marker_msg.scale.z = .1
        marker_msg.color.a = 1.0
        marker_msg.color.r = 1.0
        marker_msg.color.g = 1.0
        marker_msg.color.b = 1.0
        self.pub_goal.publish(marker_msg)

        
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="root_link_m0_actual"
        for theta in np.arange(0, 2*math.pi, 0.01):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x =  self.ctl.R*math.sin(theta)
            pose.pose.position.y = -self.ctl.R*math.cos(theta)+self.ctl.R
            path_msg.poses.append(pose)
        self.pub_arc.publish(path_msg)


class Node:
    def __init__(self):
        self.node_pub = NodePublisher()

        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/oscar_ackermann_controller/cmd_vel')
        self.pub_twist = rospy.Publisher(twist_cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        rospy.loginfo(' publishing twist commands on: {}'.format(twist_cmd_topic))
        #path_file = '/home/poine/work/overlay_ws/src/oscar/oscar_control/paths/demo_z/circle_01.npz'
        path_filename = rospy.get_param('~path_filename', '/home/poine/work/oscar.git/oscar/oscar_control/paths/demo_z/track_ethz_cam1_new.npz')
        param = tdg.pure_pursuit.Param()
        self.l = param.L = 0.1
        self.ctl = tdg.pure_pursuit.PurePursuit(path_filename, param)
        self.v = 0.5
        self.smocap_listener = utils.SmocapListener()

    def periodic(self):
        try:
            # get current pose
            p0, psi = self.smocap_listener.get_loc_and_yaw()
            try:
                _unused, self.alpha = self.ctl.compute(p0, psi)
            except tdg.pure_pursuit.EndOfPathException:
                self.ctl.path.reset()
                _unused, self.alpha = self.ctl.compute(p0, psi)
            else:
                self.publish_twist_cmd()
        except utils.RobotLostException:
            print('robot lost')
        except utils.RobotNotLocalizedException:
            print('robot not localized')
            #self.publish_path(self.ctl.path) # expensive...
            #self.publish_debug()

    def publish_twist_cmd(self):
        #print self.alpha
        lin = self.v
        ang = self.v/self.l*math.tan(self.alpha)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub_twist.publish(msg)
        
    def run(self):
        self.rate = rospy.Rate(20.)
        while not rospy.is_shutdown():
            self.periodic()
            self.rate.sleep()
        

def main(args):
  rospy.init_node('pp_guidance')
  Node().run()

  
if __name__ == '__main__':
    main(sys.argv)

