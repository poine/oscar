# distutils: language = c++
#
#
#  cython interface to the C++ hardware interface
#
# Author: Poine-2019
#

# I should look at that https://github.com/longjie/ros_cython_example

import numpy as np

from libcpp cimport bool

cdef extern from "oscar_control/oscar_hardware_interface.h":
    cdef cppclass c_HWI "OscarHardwareInterface":
        c_HWI()
        bool start()
        #void read(ros::Time now);
        void write();
        bool shutdown();

cdef class HWI:
    cdef c_HWI *thisptr

    def __cinit__(self):
        self.thisptr = new c_HWI()

    def start(self):
        return self.thisptr.start()
    
    def write(self):
        self.thisptr.write()

    def shutdown(self):
        return self.thisptr.shutdown()
