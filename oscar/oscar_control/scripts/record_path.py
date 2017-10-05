#!/usr/bin/env python
import sys, os, cv2, numpy as np, rospy, std_msgs.msg, geometry_msgs.msg, nav_msgs.msg

import utils

class PathRecorder:
    def __init__(self):
        self.pub_path = rospy.Publisher('record_path/path', nav_msgs.msg.Path, queue_size=1) 
        self.smocap_listener = utils.SmocapListener()
        self.rate = rospy.Rate(10.)
        self.path = []
        rospy.Subscriber("record_path/do_it", std_msgs.msg.String, self.on_record)
        
    def publish_curpath(self):
        path_msg = nav_msgs.msg.Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id="world"
        for l,y in self.path:
            pose = geometry_msgs.msg.PoseStamped()
            pose.pose.position.x, pose.pose.position.y = l
            path_msg.poses.append(pose)
        self.pub_path.publish(path_msg)
        

    def on_record(self, msg):
        p0, psi = self.smocap_listener.get_loc_and_yaw()
        print "hello", p0, psi
        self.path.append((p0, psi))
        

        
    def run(self):
        while not rospy.is_shutdown():
            self.publish_curpath()
            self.rate.sleep()
        np.savez("/tmp/path", path=self.path)

        
def main(args):
    PathRecorder().run()


if __name__ == '__main__':
    rospy.init_node('record_path')
    main(sys.argv)
