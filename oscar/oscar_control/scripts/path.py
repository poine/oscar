import numpy as np


import pdb

class Path:

    def __init__(self, **kwargs):
        self.last_passed_idx = 0 # we don't allow going back
        if "load" in kwargs:
            self.load(kwargs['load'])
            
    def reset(self):
        self.last_passed_idx = 0
        
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


    def load(self, filename):
        print('loading from {}'.format(filename))
        data =  np.load(filename)
        #self.points = data['path'][:,0]
        #self.headings = data['path'][:,1]
        self.points = np.array([p.tolist() for p in data['points']])
        self.headings = data['headings']
        if 'dists' in data:
            self.dists = data['dists']
        else:
            print("computing dists")
            self.dists = np.zeros(len(self.points))
            for i, p in enumerate(self.points[1:]):
                self.dists[i+1] = self.dists[i] + np.linalg.norm(self.points[i+1]-self.points[i])
        #pdb.set_trace()
        #print self.points
        #print self.headings

    def save(self, filename):
        print('saving to {}'.format(filename))
        np.savez(filename, points=self.points, headings=self.headings, dists=self.dists)


    def move_point(self, i, p, y):
        self.points[i] = p
        self.headings[i] = y
        if i > 0:
            self.dists[i] = self.dists[i-1] + np.linalg.norm(self.points[i]-self.points[i-1])
        

    def insert_point(self, i, p, y):
        new_points = np.zeros((len(self.points)+1, 2))
        #pdb.set_trace()
        new_points[:i] = self.points[:i]
        new_points[i] = p
        new_points[i+1:] = self.points[i:]
        self.points = new_points

        new_headings = np.zeros(len(self.headings)+1)
        new_headings[:i] = self.headings[:i]
        new_headings[i] = y
        new_headings[i+1:] = self.headings[i:]
        self.headings = new_headings

        new_dists = np.zeros(len(self.dists)+1)
        new_dists[:i] = self.dists[:i]
        d = np.linalg.norm(self.points[i]-self.points[i-1])
        new_dists[i:] = d + self.dists[i-1:]
        self.dists = new_dists
        
