#!/usr/bin/env python
import sys, os, cv2, numpy as np, rospy, tf, std_msgs.msg, geometry_msgs.msg, nav_msgs.msg
import pdb
import path, utils

class PathEditor:
    def __init__(self, **kwargs):
        self.pub_path = rospy.Publisher('edit_path/path', nav_msgs.msg.Path, queue_size=1) 
        self.smocap_listener = utils.SmocapListener()
        self.rate = rospy.Rate(10.)
        self._path = path.Path(**kwargs)
        self.save = kwargs.get('save', None)
        self.path = []
        rospy.Subscriber("/edit_path/mode", std_msgs.msg.String, self.on_mode)
        rospy.Subscriber("/edit_path/goal", geometry_msgs.msg.PoseStamped, self.on_goal)
        self.set_mode(kwargs.get('mode', 'edit'))

    def set_mode(self, mode):
        self.mode = mode
        print('setting mode to {}'.format(self.mode))
        
        
    def publish_path(self):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for l, y in zip(self._path.points, self._path.headings):
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            o = pose.pose.orientation
            o.x, o.y, o.z, o.w = tf.transformations.quaternion_from_euler(*[0, 0, y])
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)

    def on_goal(self, msg):
        xy = utils.array_of_xyz(msg.pose.position)[:2]
        psi = tf.transformations.euler_from_quaternion(utils.list_of_xyzw(msg.pose.orientation))[2]
        
        if self.mode == "record":
            self.path.append((xy, psi))
        elif self.mode == "edit":
            i, p = self._path.find_closest(xy)
            print("editing pt {} from {} to {}".format(i, p, xy))
            self._path.move_point(i, xy, psi)
            self._path.reset()
        elif self.mode == "insert":
            i, p = self._path.find_closest(xy)
            print("inserting pt {} at {}".format(xy, i))
            self._path.insert_point(i, xy, psi)
            
    def on_mode(self, msg):
        #p0, psi = self.smocap_listener.get_loc_and_yaw()
        self.set_mode(msg.data)
        
    def run(self):
        while not rospy.is_shutdown():
            self.publish_path()
            self.rate.sleep()
        if self.save is not None: self._path.save(self.save)


        
def main(args):
    PathEditor(load='/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz',
               save='/tmp/foo',
               mode='edit').run()


if __name__ == '__main__':
    rospy.init_node('record_path')
    main(sys.argv)
