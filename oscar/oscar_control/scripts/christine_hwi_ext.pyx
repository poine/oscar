# distutils: language = c++
#
#
#  cython interface to the C++ hardware interface
#
# Author: Poine-2019
#

# I should look at that https://github.com/longjie/ros_cython_example

import numpy as np

from libc cimport stdint
from libcpp cimport bool

# cdef extern from "oscar_control/christine_hwi_msg.h":
#   cdef struct ChristineHardwareInput:
#       float steering_srv
#       float throttle_servo

#   cdef struct ChristineHardwareInputMsg:
#         uint8_t stx
#         uint8_t len
#         uint16_t seq
#         ChristineHardwareInput data
#         uint8_t ck1
#         uint8_t ck2

cdef extern from "oscar_control/christine_remote_bbb.h":
    # cdef struct ChristineHardwareInputMsg:
    #     float steering_srv
    #     float throttle_servo
    cdef cppclass c_BBB_LINK "BBBLink":
        c_BBB_LINK()
        bool init()
        bool send2(float steering, float throttle)
        void get_bat(float* bat)
        void get_motor(float* pos, float* vel)
        void get_dsm(float* steering, float* throttle)
  
cdef class BBBLink:
    cdef c_BBB_LINK *thisptr

    def __cinit__(self):
        self.thisptr = new c_BBB_LINK()

    def init(self):
        return self.thisptr.init()
    
    def send(self, servo_steering, servo_throttle):
        self.thisptr.send2(servo_steering, servo_throttle)

    def get_bat(self):
        cdef float bat
        self.thisptr.get_bat(&bat)
        return bat
        
    def get_motor(self):
        cdef float mot_pos, mot_vel
        self.thisptr.get_motor(&mot_pos, &mot_vel)
        return mot_pos, mot_vel

    def get_dsm(self):
        cdef float steering, throttle
        self.thisptr.get_dsm(&steering, &throttle)
        return steering, throttle
