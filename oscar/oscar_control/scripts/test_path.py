#!/usr/bin/env python

import numpy as np, matplotlib.pyplot as plt, matplotlib.image as mpimg

import guidance

def test_create_insert(verb=True):
    if verb: print('\n## test_create_insert')
    if verb: print('creating empty path')
    _path = guidance.path.Path()
    if verb: print(_path)
    if verb: print('adding one point')
    _path.insert_point(0, [0, 0], 0)
    if verb: print(_path)
    if verb: print('adding another point')
    _path.insert_point(1, [1, 0], 1)
    if verb: print(_path)
    return _path

def test_move():
    print('\n## test_move')
    _path = test_create_insert(verb=False)
    print(_path)
    print('moving point')
    _path.move_point(1, [0,2], 0.5)
    print(_path)

def test_save_load():
    print('\n## test_save_load')
    _path = test_create_insert(verb=False)
    print('saving path')
    print(_path)
    _path.save('/tmp/foo.npz')
    _path = guidance.path.Path(load='/tmp/foo.npz')
    print(_path)
    
    
def test_plot():
    _map = guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz_2.yaml')
    _path = guidance.path.Path(load="/home/poine/work/oscar.git/oscar/oscar_control/path_track_ethz_4.npz")
    guidance.plot(_map, _path)

    plt.show()
    
    

if __name__ == '__main__':
    test_create_insert()
    test_move()
    test_save_load()
    test_plot()
