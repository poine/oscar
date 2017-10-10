
import os, logging, yaml, matplotlib.image, matplotlib.pyplot as plt

LOG = logging.getLogger('guidance')

from path import *

class Map:
    def __init__(self, yaml_path):
        self.load_yaml(yaml_path)
    
    def load_yaml(self, yaml_path):
        LOG.info(' loading map from yaml {}'.format(yaml_path))
        with open(yaml_path, "r") as f:
            _yaml = yaml.load(f)
            map_img_path = os.path.join(os.path.dirname(yaml_path), _yaml['image'])
            self.img = matplotlib.image.imread(map_img_path)
            self.height, self.width = self.img.shape
            self.resolution = _yaml['resolution']
            self.origin = _yaml['origin']
            self.occupied_thresh = _yaml['occupied_thresh']
            self.free_thresh = _yaml['free_thresh']
            #pdb.set_trace()

    def of_world(self, p_w): return [1, -1]*(p_w - self.origin[:2])/self.resolution + [0, self.height]




def plot(_map=None, _path=None):
    if _map is not None:
        imgplot = plt.imshow(_map.img, cmap='gray')
        if _path is not None:
            points_map = _map.of_world(_path.points)
            plt.plot(points_map[:,0], points_map[:,1], color='r', marker='.', markersize=10)
    else:
        if _path is not None:    
            plt.plot(_path.points[:,0], _path.points[:,1], color='r', marker='.', markersize=10)
            
