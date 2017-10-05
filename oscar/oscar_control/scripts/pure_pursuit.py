#!/usr/bin/env python
import roslib
import sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, sensor_msgs.msg, geometry_msgs.msg, visualization_msgs.msg
import tf.transformations
import matplotlib, matplotlib.pyplot as plt
import scipy.stats

import utils, path

import pdb



class Path:

    def __init__(self):
        self.last_passed_idx = 0 # we don't allow going back

    def reset(self):
        self.last_passed_idx = 0
        
    def find_closest(self, p0):
        i = np.argmin(np.linalg.norm(p0 - self.points[self.last_passed_idx:], axis=1)) + self.last_passed_idx
        self.last_passed_idx = i
        return i, self.points[i]

    def find_carrot(self, i, p1, _d=0.2):
        j=i
        while j<len(self.points) and self.dists[j] - self.dists[i] < _d:
            j += 1
        return (j, self.points[j]) if j < len(self.points) else (None, None)

    def find_carrot_alt(self, p0, _d=0.2):
        i, p1 = self.find_closest(p0)
        j, p2 = self.find_carrot(i, p1, _d)
        end_reached = (j is None)
        return p1, p2, end_reached

    
class LinePath(Path):
    def __init__(self, p0=(0.1, 0.1), p1=(1.9, 1.5), _d=0.05):
        disp = np.array(p1) - np.array(p0)
        _len = np.linalg.norm(disp)
        n_point = int(_len/_d)
        self.points = np.stack((np.linspace(p0[0], p1[0], n_point), np.linspace(p0[1], p1[1], n_point)), axis=1)
        self.dists = np.linspace(0, _len, n_point)
        self.headings = math.atan2(disp[1], disp[0])*np.ones(n_point)
        self.time = np.zeros(n_point)


class CirclePath(Path):
    def __init__(self, center=(1,1), radius=0.5, theta0=0, theta1=math.pi, _d=0.05):
        _len = abs((theta1-theta0)*radius)
        n_point = int(_len/_d)
        thetas = np.linspace(theta0, theta1, n_point)
        self.points = center + radius*np.stack((np.cos(thetas), np.sin(thetas)), axis=1)
        self.dists = np.linspace(0, _len, n_point)
        self.headings = thetas+math.pi/2
        self.time = np.zeros(n_point)


class OvalPath(Path):
    pass


class FilePath(Path):
    def __init__(self, filename):
        Path.__init__(self)
        data = np.load(filename)
        points = []
        for p,y in data['path']:
            print p, y
            points.append(p)
        self.points = np.array(points)
        self.dists = np.zeros(len(self.points))
        for i, p in enumerate(self.points[1:]):
            self.dists[i+1] = self.dists[i] + np.linalg.norm(self.points[i+1]-self.points[i])

def make_cw_rectangle():
    return  [ LinePath((0.4, 0.2),(0.4, 1.8)),
              LinePath((0.4, 1.8),(1.4, 1.8)),
              LinePath((1.4, 1.8),(1.4, 0.2)),
              LinePath((1.4, 0.2),(0.4, 0.2))]

def make_ccw_rectangle():
     return  [ LinePath((0.4, 0.2),(1.4, 0.2)),
               LinePath((1.4, 0.2),(1.4, 1.8)),
               LinePath((1.4, 1.8),(0.4, 1.8)),
               LinePath((0.4, 1.8), (0.4, 0.2))]

def make_oval():
    return [ CirclePath(center=(0.9,1.3), radius=0.5, theta0=0, theta1=math.pi),
             LinePath((0.4, 1.3),(0.4, 0.9)),
             CirclePath(center=(0.9,0.9), radius=0.5, theta0=math.pi, theta1=2*math.pi),
             LinePath((1.4, 0.9),(1.4, 1.3))]


def make_height():
    return [CirclePath(center=(0.9,1.5), radius=0.4, theta0=0-math.pi/3, theta1=math.pi+math.pi/3),
            LinePath((0.7, 1.15),(1.1, 0.94)),
            CirclePath(center=(0.9,0.6), radius=0.4, theta0=2*math.pi+math.pi/3, theta1=math.pi-math.pi/3),
            LinePath((0.7, 0.95),(1.1, 1.17))]


        
class PurePursuit:
    def __init__(self):
        self.l = 0.1                  # wheelbase
        twist_cmd_topic = '/oscar_ackermann_controller/cmd_vel'
        self.pub_twist = rospy.Publisher(twist_cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.pub_path = rospy.Publisher('pure_pursuit/path', nav_msgs.msg.Path, queue_size=1)
        self.pub_curpath = rospy.Publisher('pure_pursuit/curpath', nav_msgs.msg.Path, queue_size=1)
        self.pub_goal =  rospy.Publisher('pure_pursuit/goal', visualization_msgs.msg.Marker, queue_size=1)
        self.pub_arc = rospy.Publisher('pure_pursuit/arc', nav_msgs.msg.Path, queue_size=1)
        self.smocap_listener = utils.SmocapListener()

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



    def compute(self, _path, v):
        if self.smocap_listener.pose is None:
            self.goal = (1, 1)
            self.R = float('inf')
            self.alpha, self.v = 0, 0
            return True
        # get current pose
        p0, psi = self.smocap_listener.get_loc_and_yaw()
        # find closest point and carrot on path
        p1, p2, end_reached = _path.find_carrot_alt(p0)
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
        self.v = v
        return True
        
    def run(self, paths=make_cw_rectangle(), v=0.2):
        self.rate = rospy.Rate(60.)

        path_idx = 0
        while not rospy.is_shutdown():
            if not self.compute(paths[path_idx], v):
                path_idx = (path_idx+1)%len(paths)
                paths[path_idx].reset()
                self.compute(paths[path_idx], v)
            self.publish_twist()
            self.publish_curpath(paths[path_idx])
            self.publish_path(paths)
            self.publish_goal()
            self.rate.sleep()
        

def main(args):
  rospy.init_node('calibrate_ackermann')
  #PurePursuit().run(make_cw_rectangle(), v=0.5)
  #PurePursuit().run(make_oval(), v=0.9)
  #PurePursuit().run(make_height(), v=0.2)
  #PurePursuit().run([FilePath('/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_1.npz')], v=0.1)
  PurePursuit().run([path.Path(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz')], v=0.3)

if __name__ == '__main__':
    main(sys.argv)
