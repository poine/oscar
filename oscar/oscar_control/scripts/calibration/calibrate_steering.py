#!/usr/bin/env python
import time, math, numpy as np, matplotlib.pyplot as plt
import rospy, sensor_msgs.msg
import cv2, cv_bridge


import sys, time, math, numpy as np
import rospy, ackermann_msgs.msg

import pdb

class Node:
    def __init__(self, vel=0., alpha=0., alpha_max = np.deg2rad(10.), n_val=5):
        self.alphas = np.linspace(-alpha_max, alpha_max, n_val)
        self.dt = 2.
        self.cur_alpha_idx = -1
        print self.alphas
        self.vel, self.alpha = vel, alpha
        cmd_topic = rospy.get_param('~cmd_topic', '/oscar_ackermann_controller/cmd_ack')
        self.pub = rospy.Publisher(cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        self.t = rospy.get_time()
        self.bridge = cv_bridge.CvBridge()
        
    def publish_ack(self):
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = self.alpha
        msg.drive.speed = self.vel
        self.pub.publish(msg)

    def take_picture(self):
        rospy.loginfo("Getting image...")
        image_msg = rospy.wait_for_message(
            "/ueye_enac_z_1/image_raw",
            sensor_msgs.msg.Image)
        rospy.loginfo("Got image!")
        cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img_file_path = "/tmp/calib_image_{:02d}.png".format(self.cur_alpha_idx)
        cv2.imwrite(img_file_path, cv_image)
        rospy.loginfo("Saved to: " + img_file_path)
        
    def periodic(self):
        now = rospy.get_time()
        if now - self.t >= self.dt:
            self.take_picture()
            self.cur_alpha_idx += 1
            if self.cur_alpha_idx >= len(self.alphas): self.cur_alpha_idx=0
            self.alpha = self.alphas[self.cur_alpha_idx]
            self.t = now
            print(now, self.alphas[self.cur_alpha_idx])
        self.publish_ack()
        
    def run(self):
        self.rate = rospy.Rate(50.)
        while not rospy.is_shutdown():
            self.periodic()
            self.rate.sleep()
            
def record(args):
    rospy.init_node('send_cmd_ack', anonymous=True)
    Node().run()

def calibrate():
    alpha_max = np.deg2rad(10.); n_val=5
    alphas = np.linspace(-alpha_max, alpha_max, n_val)

    images = [cv2.imread("/tmp/calib_image_{:02d}.png".format(_idx), 0) for _idx, alpha in enumerate(alphas)]
    pdb.set_trace()
    for i, image in enumerate(images):
        #roi = slice(0, image.shape[0]), slice(0, image.shape[1])
        roi = slice(350, 800), slice(750, 1200)
        ax = plt.gcf().add_subplot(1,len(images),i+1)
        plt.imshow(image[roi])
        plt.title('Frame {}'.format(i+1))
    plt.show()
    
if __name__ == '__main__':
    #record(sys.argv)
    calibrate()




