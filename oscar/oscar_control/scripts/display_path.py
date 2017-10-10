#!/usr/bin/env python

import sys, numpy as np, matplotlib.pyplot as plt, matplotlib.image as mpimg

import guidance

if __name__ == '__main__':

    map_path = sys.argv[1] if len(sys.argv) > 1 else '/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz_2.yaml'
    path_path = sys.argv[2] if len(sys.argv) > 2 else '/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz'

    
    _map = guidance.Map(map_path)
    _path = guidance.Path(load=path_path)
    guidance.plot(_map, _path)
    #    plot(_map, _path)
    
    plt.show()
