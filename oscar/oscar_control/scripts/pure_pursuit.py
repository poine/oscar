#!/usr/bin/env python
import roslib
import sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, sensor_msgs.msg, geometry_msgs.msg, visualization_msgs.msg
import tf.transformations
import matplotlib, matplotlib.pyplot as plt
import scipy.stats

import utils

import pdb



class Path:
    def __init__(self):
        self.points = np.array([[0.25, 0.25],
                                [0.25, 1.75],
                                [1.75, 1.75],
                                [1.75, 0.25],
                                [0.35, 0.25]])
        self.time = [0, 0.1, 0.2, 0.3, 0.4]
        self.cur_idx = 0   # to avoidd going back

    def find_closest(self, p):
        i = np.argmin(np.linalg.norm(p - self.points, axis=1))
        return i, self.points[i]

    def find_carrot(self, p, _d=0.2):
        i, cp = self.find_closest(p)
        j=i
        while j<len(self.points) and self.dists[j] - self.dists[i] < _d:
            j += 1
        return j, self.points[j]


class LinePath(Path):
    def __init__(self, p0=(0.1, 0.1), p1=(1.9, 1.5), _d=0.05):
        disp = np.array(p1) - np.array(p0)
        _len = np.linalg.norm(disp)
        n_point = int(_len/_d)
        self.points = np.stack((np.linspace(p0[0], p1[0], n_point), np.linspace(p0[1], p1[1], n_point)), axis=1)
        self.dists = np.linspace(0, _len, n_point)
        self.headings = math.atan2(disp[1], disp[0])*np.ones(n_point)
        #print self.points
        print self.dists
        self.time = np.zeros(n_point)


class CirclePath(Path):
    def __init__(self, center=(1,1), radius=0.75, theta0=0, theta1=math.pi):
        

        pass
    

        
        
class PurePursuit:
    def __init__(self):
        self.l = 0.1                  # wheelbase
        twist_cmd_topic = '/oscar_ackermann_controller/cmd_vel'
        self.pub_twist = rospy.Publisher(twist_cmd_topic, geometry_msgs.msg.Twist, queue_size=1)
        self.pub_path = rospy.Publisher('pure_pursuit/path', nav_msgs.msg.Path, queue_size=1)
        self.pub_goal =  rospy.Publisher('pure_pursuit/goal', visualization_msgs.msg.Marker, queue_size=1)
        self.smocap_listener = utils.SmocapListener()

    def publish_path(self, _path):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for t, p in zip(_path.time, _path.points):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
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
        
    def publish_twist(self):
        lin = self.v
        ang = self.v/self.l*math.tan(self.alpha)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub_twist.publish(msg)



    def compute(self, _path):
        if self.smocap_listener.pose is None:
            self.goal = (1, 1)
            return 0, 0
        # get current pose
        my_pos = utils.array_of_xyz(self.smocap_listener.pose.position)[:2]
        my_yaw = tf.transformations.euler_from_quaternion(utils.list_of_xyzw(self.smocap_listener.pose.orientation))[2]
        # find closest point on path
        i, p0 = _path.find_closest(my_pos)
        self.goal = p0
        j, p1 = _path.find_carrot(my_pos)
        self.goal = p1

        
        print('{:.2f} {:.2f} {:.1f}'.format(my_pos[0], my_pos[1], utils.deg_of_rad(my_yaw)))


        return 0, 0
        
    def run(self, n_iter=10000):
        self.rate = rospy.Rate(50.)

        _path = LinePath()
        self.publish_path(_path)
        loops = 0
        while not rospy.is_shutdown() and loops < n_iter:
            self.v, self.alpha = self.compute(_path)
            self.publish_twist()
            self.publish_path(_path)
            self.publish_goal()
            self.rate.sleep()
            loops += 1
        

def main(args):
  rospy.init_node('calibrate_ackermann')
  PurePursuit().run()


if __name__ == '__main__':
    main(sys.argv)
