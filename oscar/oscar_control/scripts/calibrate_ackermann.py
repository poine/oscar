#!/usr/bin/env python
import roslib
import sys , math, numpy as np, rospy, ackermann_msgs.msg, nav_msgs.msg, sensor_msgs.msg, geometry_msgs.msg
import tf.transformations
import matplotlib, matplotlib.pyplot as plt
import scipy.stats

import pdb

def list_of_xyz(p): return [p.x, p.y, p.z]
def array_of_xyz(p): return np.array(list_of_xyz(p))
def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]

def deg_of_rad(r): return r/math.pi*180

def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        plt.title(title, {'color'    : 'k', 'fontsize'   : 20 })
    if legend <> None:
        plt.legend(legend, loc='best')
    if xlim <> None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim <> None:
        ax.set_ylim(ylim[0], ylim[1])






class SmocapListener:
    def __init__(self):
        rospy.Subscriber('/smocap/est_world', geometry_msgs.msg.PoseWithCovarianceStamped, self.smocap_cbk)
        self.ts = None
        self.pose = None
        self.vel = None
        
    def smocap_cbk(self, msg):
        if self.pose is not None:
            p1 = array_of_xyz(self.pose.position)
            p2 = array_of_xyz(msg.pose.pose.position)
            dt = msg.header.stamp.to_sec() - self.ts
            self.vel = np.linalg.norm((p2-p1)/dt)
        self.pose = msg.pose.pose
        self.ts = msg.header.stamp.to_sec()

    def inside_zone(self):
        

        return True
        
        
class OdomListener:
    def __init__(self):
        rospy.Subscriber('/oscar_ackermann_controller/odom', nav_msgs.msg.Odometry, self.odom_cbk)
        self.vel = None
        
    def odom_cbk(self, msg):
        self.vel = np.linalg.norm(array_of_xyz(msg.twist.twist.linear))




        

class CalibNode:

    def __init__(self, v=0., alpha=0.):
        self.v, self.alpha = v, alpha # linear velocity and steering angle
        self.l = 0.1                  # wheelbase
        print v, alpha
        
        ackermann_cmd_topic = '/oscar_ackermann_controller/cmd_ackermann'
        self.pub = rospy.Publisher(ackermann_cmd_topic, ackermann_msgs.msg.AckermannDriveStamped, queue_size=1)
        twist_cmd_topic = '/oscar_ackermann_controller/cmd_vel'
        self.pub_twist = rospy.Publisher(twist_cmd_topic, geometry_msgs.msg.Twist, queue_size=1)

        self.smocap_listener = SmocapListener()
        self.odom_listener = OdomListener()
        
            
    def periodic(self):
        self.publish_twist()
        if self.smocap_listener.vel is not None and self.odom_listener.vel is not None:
            print('vel mocap/odom: {:.3f} / {:.3f}'.format(self.smocap_listener.vel, self.odom_listener.vel))
   
    
    def publish_twist(self):
        lin = self.v
        ang = self.v/self.l*math.tan(self.alpha)
        msg = geometry_msgs.msg.Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub_twist.publish(msg)
        
    def publish_ack(self):
        steering, vel = 0, 0
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'odom'
        msg.drive.steering_angle = steering
        msg.drive.speed = vel
        self.pub.publish(msg)


def precommand1(yc, yc_inf=-0.06, yc_sup=0.06, k_sup=0.671):
    if yc > 0:
        return (yc+yc_sup)*0.8
    else:
        return (yc+yc_inf)*0.8

def precommand(yc): return yc
    
        
class VelCalib(CalibNode):

    def measure(self, vc, u, n_iter, discard=40):
        print('commanding vc {} m/s with u {}'.format(vc, u))
        self.v = u
        loops, meas = 0, []
        while not rospy.is_shutdown() and loops < n_iter:
            if self.odom_listener.vel is not None:
                self.publish_twist()
                if loops > discard:
                    meas.append(self.odom_listener.vel if vc >= 0 else -self.odom_listener.vel)
                loops += 1
            self.rate.sleep()
        return meas
            
    def run(self, save_path='/tmp/calib_vel1', load_path=None):
        if save_path is not None:
            self.rate = rospy.Rate(50.)
            cmd_vels = [(0.05, 400), (-0.05, 400),
                        (0.1, 400),  (-0.1, 400),
                        (0.15, 400), (-0.15, 400),
                        (0.2, 200),  (-0.2, 200),
                        (0.25, 200), (-0.25, 200),
                        (0.30, 200), (-0.30, 200),
                        (0.35, 200), (-0.35, 200),
                        (0.40, 200), (-0.40, 200),
                        (0.45, 200), (-0.45, 200)]
            cmd_vels = [(0.05, 400), (-0.05, 400),
                        (0.06, 400), (-0.06, 400),
                        (0.07, 400), (-0.07, 400),
                        (0.08, 400), (-0.08, 400),
                        (0.09, 400), (-0.09, 400),
                        (0.10, 400), (-0.10, 400),
                        (0.11, 400), (-0.11, 400),
                        (0.12, 400), (-0.12, 400),
                        (0.13, 400), (-0.13, 400),
                        (0.14, 400), (-0.14, 400),
                        (0.15, 400), (-0.15, 400)]
            
            measured_vels = [self.measure(v, precommand(v), n) for v, n in cmd_vels]
            np.savez(save_path, cmd_vels=cmd_vels, measured_vels=measured_vels)
        if load_path is not None:
            data = np.load(load_path)
            cmd_vels, measured_vels = data['cmd_vels'],  data['measured_vels']


            
        #print measured_vels
        avg_meas = [np.mean(m) for m in measured_vels]
        #print avg_meas
        cmds = [c[0] for c in cmd_vels]
        args = np.argsort(cmds)
        s_cmds = np.sort(cmds)
        s_outputs =  np.array(avg_meas)[args]

        idxp = np.where(s_outputs > 1e-3)
        slope, intercept, r_value, p_value, std_err = scipy.stats.linregress(s_outputs[idxp], s_cmds[idxp])
        
        
        #pdb.set_trace()
        

        plt.plot(s_cmds, s_outputs)
        decorate(plt.gca(), title='velocity', xlab='setpoint', ylab='output')
        plt.show()






class SteeringCalib(CalibNode):

    def measure(self, alpha, v, n_iter=400):
        self.alpha, self.v = alpha, v
        loops, meas_smocap_loc, meas_smocap_ori = 0, [], []
        while not rospy.is_shutdown() and loops < n_iter:
            self.publish_twist()
            if self.smocap_listener.pose is not None:
                meas_smocap_loc.append(list_of_xyz(self.smocap_listener.pose.position))
                meas_smocap_ori.append(list_of_xyzw(self.smocap_listener.pose.orientation))
            loops += 1
            self.rate.sleep()
        return meas_smocap_loc, meas_smocap_ori
    
    def run(self, save_path='/tmp/calib_steering', load_path=None):
        if save_path is not None:
            self.rate = rospy.Rate(50.)
            v, R = 0.15, 0.5
            alpha = math.atan(v/R)
            print('commanding v {} m/s R {} m (alpha {:.2f} rad {:.1f} deg)'.format(v, R, alpha, deg_of_rad(alpha)))
            meas_loc, meas_ori = self.measure(alpha, v, 3000)
            meas_loc = np.array(meas_loc)
            meas_ori = np.array(meas_ori)
            np.savez(save_path, meas_loc=meas_loc, meas_ori=meas_ori)
        if load_path is not None:  
            data = np.load(load_path)
            meas_loc, meas_ori = data['meas_loc'],  data['meas_ori']
        plt.plot(meas_loc[:,0], meas_loc[:,1])
        decorate(plt.gca(), title='steering', xlab='x', ylab='y')
        plt.gca().set_aspect('equal')
        plt.show()
        

def main(args):
  rospy.init_node('calibrate_ackermann')
  VelCalib().run()#save_path=None, load_path='/tmp/calib_vel1.npz')
  #SteeringCalib().run()#save_path=None, load_path='/tmp/calib_steering.npz')

if __name__ == '__main__':
    main(sys.argv)
