#!/usr/bin/env python

import sys, numpy as np

import guidance
if __name__ == '__main__':

    p = guidance.Path(load=sys.argv[1])
    p.append([guidance.Path(load=_p) for _p in sys.argv[2:]])

    p.save('/tmp/foo.npz')
