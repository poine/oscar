#!/usr/bin/env python
import roslib
import sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, sensor_msgs.msg, geometry_msgs.msg, visualization_msgs.msg
import tf.transformations
import matplotlib, matplotlib.pyplot as plt
import scipy.stats

import utils, guidance

import pdb



class PurePursuit:
    def __init__(self, path_file):
        self.path = guidance.Path(load=path_file)

        
class PurePursuitNode:
    def __init__(self):
        twist_cmd_topic = rospy.get_param('~twist_cmd_topic', '/oscar_ackermann_controller/cmd_vel')
        path_file = rospy.get_param('~path_file', '/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_5.npz')

        self.controller = PurePursuit(path_file)
        
        self.dt = 1./60.
        self.l = 0.1                  # wheelbase
        self.pub_twist = rospy.Publisher(twist_cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.pub_path = rospy.Publisher('pure_pursuit/path', nav_msgs.msg.Path, queue_size=1)
        self.pub_curpath = rospy.Publisher('pure_pursuit/curpath', nav_msgs.msg.Path, queue_size=1)
        self.pub_goal =  rospy.Publisher('pure_pursuit/goal', visualization_msgs.msg.Marker, queue_size=1)
        self.pub_arc = rospy.Publisher('pure_pursuit/arc', nav_msgs.msg.Path, queue_size=1)
        self.smocap_listener = utils.SmocapListener()
        self.vel_ref = utils.FirstOrdLinRef(0.75)
        rospy.loginfo(' using twist cmd topic: {}'.format(twist_cmd_topic))
        

        
    def publish_curpath(self, _path):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for l in _path.points:
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            path_msg.poses.append(pose)
        self.pub_curpath.publish(path_msg)

    def publish_path(self, paths):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for p in paths:
            for l in p.points:
                pose = geometry_msgs.msg.PoseStamped()
                pose.pose.position.x = l[0]
                pose.pose.position.y = l[1]
                path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)
        

    def publish_goal(self):
        marker_msg = visualization_msgs.msg.Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id="world"
        marker_msg.type = visualization_msgs.msg.Marker.CYLINDER
        marker_msg.pose.position.x = self.goal[0]
        marker_msg.pose.position.y = self.goal[1]
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
        path_msg.header.frame_id="base_link"
        for theta in np.arange(0, 2*math.pi, 0.01):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x = self.R*math.sin(theta)
            pose.pose.position.y = -self.R*math.cos(theta)+self.R
            path_msg.poses.append(pose)
        self.pub_arc.publish(path_msg)

    
    def publish_twist(self):
        lin = self.v
        ang = self.v/self.l*math.tan(self.alpha)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub_twist.publish(msg)



    def compute(self, _path, v_sp, adp_vel):
        if self.smocap_listener.pose is None:
            self.goal = (1, 1)
            self.R = float('inf')
            self.alpha, self.v = 0, 0
            return True
        # get current pose
        p0, psi = self.smocap_listener.get_loc_and_yaw()
        # find closest point and carrot on path
        p1, p2, end_reached, ip1, ip2 = _path.find_carrot_alt(p0)
        if end_reached: return False
            
        self.goal = p2
        p0p2_w = p2 - p0
        cy, sy = math.cos(psi), math.sin(psi)
        w2b = np.array([[cy, sy],[-sy, cy]])
        p0p2_b = np.dot(w2b, p0p2_w)
        l = np.linalg.norm(p0p2_w)
        self.R = (l**2)/(2*p0p2_b[1])
        self.alpha = math.atan(self.l/self.R)
        
        #print 'P0P2 ', p0p2_b, 'l', l, ' R', self.R, ' alpha ', utils.deg_of_rad(self.alpha)
        
        #print('{:.2f} {:.2f} {:.1f}'.format(my_pos[0], my_pos[1], utils.deg_of_rad(my_yaw)))
        if adp_vel:
            v_sp = _path.vel_sp[ip2]
        self.v = self.vel_ref.run(self.dt, v_sp)[0]
        return True
        
    def run(self, paths, v_sp=0.2, adp_vel=True):
        self.rate = rospy.Rate(1./self.dt)

        path_idx = 0
        while not rospy.is_shutdown():
            if not self.compute(paths[path_idx], v_sp, adp_vel):
                path_idx = (path_idx+1)%len(paths)
                paths[path_idx].reset()
                self.compute(paths[path_idx], v_sp, adp_vel)
            self.publish_twist()
            self.publish_curpath(paths[path_idx])
            self.publish_path(paths)
            self.publish_goal()
            self.rate.sleep()
        

def main(args):
  rospy.init_node('pure_pursuit')
  #PurePursuit().run(make_cw_rectangle(), v=0.5)
  #PurePursuit().run(make_oval(), v=0.9)
  #PurePursuit().run(make_height(), v=0.2)
  #PurePursuit().run([FilePath('/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_1.npz')], v=0.1)
  #PurePursuit().run([path.Path(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz')], v=0.3)

  #PurePursuit(twist_cmd_topic='/rosmip_balance_controller/cmd_vel').run([guidance.Path(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_5.npz')], v_sp=0.6, adp_vel=False)
  PurePursuitNode().run([guidance.Path(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_5.npz')], v_sp=0.25, adp_vel=False)

if __name__ == '__main__':
    main(sys.argv)
