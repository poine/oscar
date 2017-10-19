#! /usr/bin/env python
# -*- coding: utf-8 -*-

import logging, timeit, math, numpy as np, scipy.integrate, matplotlib.pyplot as plt
import sklearn.neural_network
import pdb

''' plant and controller identification on first order system '''
import test_ann_first_order as tafo

LOG = logging.getLogger('test_ann_first_order_with_ref_model')

class CtlStateFeedbackWithRef:
    def __init__(self, tau_ref, tau_err, tau_plant=0.1, g_plant=2., dt=0.005):
        exp_m_dt_ov_tau_plant = math.exp(-dt/tau_plant)
        a_plant, b_plant = exp_m_dt_ov_tau_plant, (1-exp_m_dt_ov_tau_plant)*g_plant
        exp_m_dt_ov_tau_err = math.exp(-dt/tau_err)
        a_err = exp_m_dt_ov_tau_err
        exp_m_dt_ov_tau_ref = math.exp(-dt/tau_ref)
        a_ref = exp_m_dt_ov_tau_ref
        self.K = (a_plant - a_err)/b_plant
        self.H1 = (a_ref - a_err)/b_plant
        self.H2 = (1- a_ref)/b_plant
        self.ref = tafo.LinPlant(tau=tau_ref, g=1., dt=dt)
        self.Xref = np.array([0])
        self.ysp = None

    def get(self, t, x):
        yc = self.yc(t)
        self.Xref = self.ref.dyn_disc(self.Xref, yc)
        return -self.K*x + self.H1*self.Xref[0] + self.H2*yc, [yc, self.Xref[0]] 

    def reset(self):
        self.Xref = np.array([0])

def test_with_nonlin_plant():
    
    dt = 0.005

    lin_plant = tafo.LinPlant(tau=1., g=3., dt=dt)
    time =  np.arange(0., 10., dt)

    ctl = CtlStateFeedbackWithRef(tau_ref=0.5, tau_err=0.05, tau_plant=lin_plant.tau, g_plant=lin_plant.g, dt=dt)
    #ctl.yc = tafo.sawtooth
    #ctl.yc = tafo.step
    ctl.yc = tafo.sin
    
    Xlin, Ulin, SPlin = lin_plant.sim_with_input_fun(time, ctl)

    nl_plant = tafo.NonLinPlant(tau=1., g=3., dt=dt)
    ctl.reset()
    Xnl, Unl, SPnl = nl_plant.sim_with_input_fun(time, ctl)

    
    plt.subplot(2,1,1)
    plt.plot(time, SPlin[:,0]) # set point
    plt.plot(time, SPlin[:,1]) # ref
#    plt.plot(time, SPnl[:,0]) # set point
#    plt.plot(time, SPnl[:,1]) # ref
    
    tafo.plot(time, Xlin, Ulin)
    tafo.plot(time, Xnl, Unl)
    plt.subplot(2,1,1)
    plt.legend(['setpoint','ref', 'lin plant', 'non lin plant'])
    plt.subplot(2,1,2)
    plt.legend(['lin plant', 'non lin plant'])
    plt.figure()
    plt.plot(time, Xlin-SPlin[:,1])
    plt.plot(time, Xnl-SPnl[:,1])
    plt.title('tracking error'
    )
def main():
    test_with_nonlin_plant()
    
    plt.show()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
