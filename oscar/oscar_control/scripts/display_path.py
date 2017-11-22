#!/usr/bin/env python

import sys, numpy as np, matplotlib.pyplot as plt, matplotlib.image as mpimg

import guidance



def plot_path_list():
    map_path = '/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/enac_bench/track_test2.yaml'
    _map = guidance.Map(yaml_path=map_path)

    path_list = ('/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_03.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_04.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_05.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_08.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_10.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_11.npz',
                 '/home/poine/work/oscar.git/oscar/oscar_control/paths/enac_bench/path_08.npz')

    for i,path_path in enumerate(path_list):
        _path = guidance.Path(load=path_path)
        ax = plt.subplot(len(path_list),1,i+1)
        guidance.plot(_map, _path)
    

def main():
    map_path = sys.argv[1] if len(sys.argv) > 1 else '/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz_2.yaml'
    path_path = sys.argv[2] if len(sys.argv) > 2 else '/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_2.npz'
    
    _map = guidance.Map(yaml_path=map_path)
    _path = guidance.Path(load=path_path)

    ax = plt.subplot(2,1,1)
    guidance.plot(_map, _path)

    ax = plt.subplot(2,1,2)
    _path.compute_curvatures()
    print _path.radius
    print _path.dists
    plt.plot(_path.radius)
    ax.set_ylim(0, 3)
    

    
if __name__ == '__main__':

    #main()
    plot_path_list()
    plt.show()
