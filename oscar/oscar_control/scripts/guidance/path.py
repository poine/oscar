import numpy as np


import pdb

class Path:

    def __init__(self, **kwargs):
        self.last_passed_idx = 0 # we don't allow going back
        if 'load' in kwargs:
            self.load(kwargs['load'])
        elif 'points' in kwargs:
            self.initialize(kwargs['points'], kwargs.get('headings', []), kwargs.get('dists', []))
        else:
            self.clear()

    def __str__(self):
        return 'points {} headings {} dists {}'.format(self.points, self.headings, self.dists)

    def initialize(self, points, headings=[], dists=[]):
        self.points = points
        if len(headings) > 0:
            self.headings = headings
        else:
            self.compute_headings()
        if len(dists) > 0:
            self.dists = dists
        else:
            self.compute_dists()
            
    def clear(self):
        self.points = np.empty((0, 2))
        self.headings = np.empty((0))
        self.dists = np.empty((0))
            
    def load(self, filename):
        print('loading from {}'.format(filename))
        data =  np.load(filename)
        self.points = np.array([p.tolist() for p in data['points']])
        self.headings = data['headings']
        if 'dists' in data:
            self.dists = data['dists']
        else:
            self.compute_dists()

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename, points=self.points, headings=self.headings, dists=self.dists)

    def reset(self):
        self.last_passed_idx = 0
        
    def compute_headings(self):  # TEST ME
        self.headings = np.zeros(len(self.points))
        for i, p in enumerate(self.points):
            self.headings[i] = numpy.arctan2(self.points[i+1]-self.points[i])

    def compute_dists(self):
        self.dists = np.zeros(len(self.points))
        for i, p in enumerate(self.points[1:]):
            self.dists[i+1] = self.dists[i] + np.linalg.norm(self.points[i+1]-self.points[i])

    def move_point(self, i, p, y):
        self.points[i] = p
        self.headings[i] = y
        if i > 0:
            self.dists[i] = self.dists[i-1] + np.linalg.norm(self.points[i]-self.points[i-1])

    def insert_point(self, i, p, y):
        self.points = np.insert(self.points, i, p, axis=0)
        self.headings = np.insert(self.headings, i, y)
        self.dists = np.insert(self.dists, i, np.linalg.norm(self.points[i]-self.points[i-1]))

    def append_points(self, p, y):
        self.insert_point(len(self.points), p, y)
        
    
    def find_closest(self, p0):
        i = np.argmin(np.linalg.norm(p0 - self.points[self.last_passed_idx:], axis=1)) + self.last_passed_idx
        self.last_passed_idx = i
        return i, self.points[i]

    def find_carrot(self, i, p1, _d=0.2):
        j=i
        while j<len(self.points) and self.dists[j] - self.dists[i] < _d:
            j += 1
        return (j, self.points[j]) if j < len(self.points) else (None, None)

    def find_carrot_alt(self, p0, _d=0.2):
        i, p1 = self.find_closest(p0)
        j, p2 = self.find_carrot(i, p1, _d)
        end_reached = (j is None)
        return p1, p2, end_reached


