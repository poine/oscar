# distutils: language = C++
#
#
# PyRobCape is a python beaglebone robotic cape interface using cython 
#
# Author: Poine-2017
#
import numpy as np

from libc.stdlib cimport malloc, free
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc



cdef extern from "roboticscape.h":
    int rc_initialize()
    int rc_cleanup()
    int rc_get_state()

    int rc_enable_motors()
    int rc_set_motor(int motor, float duty)

    int rc_get_encoder_pos(int ch)
    int rc_set_encoder_pos(int ch, int value)
    
def initialize(): return rc_initialize()
def cleanup(): return rc_cleanup()
def get_state(): return rc_get_state()

def enable_motors(): return rc_enable_motors()
def set_motor(motor, duty): return rc_set_motor(motor, duty)

def get_encoder_pos(ch): return rc_get_encoder_pos(ch)
def set_encoder_pos(ch, value): return rc_set_encoder_pos(ch, value)
