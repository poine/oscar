#!/usr/bin/env python
import sys, time
import numpy as np
import matplotlib.pyplot as plt
from sklearn import linear_model

import rospy

import two_d_guidance.ros_utils as tdgru, two_d_guidance.plot_utils as tdgpu
import christine_hwi_ext
import pdb

class Node:
    def __init__(self):
        self.bbbl = christine_hwi_ext.BBBLink()
        self.bbbl.init()
        self.robot_listener = tdgru.SmocapListener()
        self.time, self.inputs, self.mot, self.pos, self.yaw = [], [], [], [], []

    def run(self):
        self.rate = rospy.Rate(30.)
        while not rospy.is_shutdown():
            self.periodic()
            self.rate.sleep()
        print('exiting')
        self.save()

    def save(self, filename='/tmp/foo.npz'):
        print('saving {} sample to {}'.format(len(self.time), filename))
        np.savez(filename, time=self.time, inputs=self.inputs, mot=self.mot, pos=self.pos, yaw=self.yaw)

    def periodic(self):
        self.time.append(time.time())
        try:
            p0, psi = self.robot_listener.get_loc_and_yaw()
            #_unused, self.alpha = self.ctl.compute_looped(p0, psi)
            #self.publish_ackermann_cmd(self.alpha, self.vel_sp.get_pwm(rospy.Time.now().to_sec()))
            self.pos.append(p0); self.yaw.append(psi)
        except tdgru.RobotLostException:
            print('robot lost\r')
            self.pos.append([np.nan, np.nan]); self.yaw.append(np.nan)
        except tdgru.RobotNotLocalizedException:
            print('robot not localized\r')
            self.pos.append([np.nan, np.nan]); self.yaw.append(np.nan)


        steering, throttle = self.bbbl.get_dsm()
        self.inputs.append([steering, throttle])
        self.bbbl.send(steering, throttle)
        mot_pos, mot_vel = self.bbbl.get_motor()
        self.mot.append([mot_pos, mot_vel])
        

class CalibDataset:
    def __init__(self):
        pass

    def load(self, filename):
        data =  np.load(filename)
        self.time, self.inputs, self.mot, self.pos, self.yaw = [data[w] for w in ['time', 'inputs', 'mot', 'pos', 'yaw']]
        self.differentiate_truth_for_vel()

    def differentiate_truth_for_vel(self):
        self.pos_dts = self.time[1:] - self.time[:-1]
        self.dpos = self.pos[1:] - self.pos[:-1]
        self.lvel = np.array([ dp / dt for dp, dt in zip(self.dpos, self.pos_dts)])
        self.lvel_time = (self.time[:-1] + self.time[1:])/2
        
        
def calibrate(filename='/tmp/foo.npz'):
    ds = CalibDataset()
    ds.load(filename)
    if 0:
        ax = plt.subplot(2,1,1)
        plt.plot(ds.time, ds.inputs[:,1])
        tdgpu.decorate(ax, title='throttle', xlab='time', ylab='%', ylim=(0, 0.2))
        ax = plt.subplot(2,1,2)
        plt.plot(ds.time, ds.mot[:,1])
        tdgpu.decorate(ax, title='mot_vel', xlab='time', ylab='tick/s')
    if 0:
        ax = plt.subplot(2,1,1)
        plt.plot(ds.time, ds.pos[:,0])
        plt.plot(ds.time, ds.pos[:,1])
        tdgpu.decorate(ax, title='pos', xlab='time', ylab='m')
        ax = plt.subplot(2,1,2)
        plt.plot(ds.time, ds.yaw)
        tdgpu.decorate(ax, title='yaw', xlab='time', ylab='rad')
    if 1:
        pass
    
def calibrate_lvel(filename='/tmp/foo.npz'):
    ds = CalibDataset()
    ds.load(filename)

    Y = np.linalg.norm(ds.lvel, axis=1)
    Ymask = np.logical_and(np.logical_not(np.isnan(Y)), Y>0.1, Y<2.)
    Y1 = Y[Ymask].reshape(-1, 1)
    X = ds.mot[1:,1][Ymask].reshape(-1, 1)
    ransac = linear_model.RANSACRegressor()
    #pdb.set_trace()
    ransac.fit(X, Y1)
    inlier_mask = ransac.inlier_mask_
    line_X = np.arange(X.min(), X.max())[:, np.newaxis]
    line_y_ransac = ransac.predict(line_X)

    ax = plt.subplot(3,1,1)
    plt.plot(ds.lvel_time, np.linalg.norm(ds.lvel, axis=1), '.')
    tdgpu.decorate(ax, title='lvel', xlab='time', ylab='m/s', ylim=(0, 2))
    ax = plt.subplot(3,1,2)
    plt.plot(ds.time, ds.mot[:,1])
    tdgpu.decorate(ax, title='motor rvel', xlab='time', ylab='tick/s')
    ax = plt.subplot(3,1,3)
    plt.plot(ds.time, ds.mot[:,0])
    tdgpu.decorate(ax, title='motor pos', xlab='time', ylab='tick')
    plt.show()

    plt.scatter(X, Y1)
    plt.scatter(X[inlier_mask], Y1[inlier_mask])
    plt.plot(line_X, line_y_ransac)
    tdgpu.decorate(plt.gca(), title='motor regression', xlab='tick/s', ylab='m/s')
    plt.show()
    
  

    
def measure(filename='/tmp/foo.npz'):
    bbbl = christine_hwi_ext.BBBLink()
    bbbl.init()
    dt, n_samples = 0.02, 500
    _time, throttle, mot_vel = np.zeros(n_samples), np.zeros(n_samples), np.zeros(n_samples) 
    for i in range(n_samples):
        _start = _time[i] = time.time()
        steering, throttle[i] = 0, 0.06 + 0.01*np.sin(0.9*time.time())
        bbbl.send(steering, throttle[i])
        mot_pos, mot_vel[i] = bbbl.get_motor()
        _end = time.time(); elapsed = _end - _start
        time_to_sleep = dt - elapsed
        if time_to_sleep > 0: time.sleep(time_to_sleep)
    np.savez(filename, time=_time, throttle=throttle, mot_vel=mot_vel)
    return time, throttle, mot_vel

def plot(filename='/tmp/foo.npz'):
    data =  np.load(filename)
    _time, throttle, mot_vel = [data[w] for w in ['time', 'throttle', 'mot_vel']]
    #LOG.info('  found {} samples'.format(len(time)))

    plt.plot(_time, mot_vel)
    plt.show()

def main(args):
    #measure()
    #plot()
    if 0: # send calibration inputs
        rospy.init_node('christine_calibration')
        Node().run()
    else:
        calibrate_lvel()
        
if __name__ == '__main__':
    main(sys.argv)

