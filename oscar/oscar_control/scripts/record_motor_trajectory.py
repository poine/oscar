#! /usr/bin/env python
# -*- coding: utf-8 -*-


import logging, time, timeit, math, numpy as np
import pyrobcape

def main():
    pyrobcape.initialize()
    while pyrobcape.get_state() < 3: # TODO, add enum...
        now = time.time()
        duty = 0.1*math.sin(now)
        print now, duty
        pyrobcape.set_motor(1, duty)
        time.sleep(0.1)
    pyrobcape.cleanup()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    main()
