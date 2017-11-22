
import os, logging, yaml, matplotlib.image, matplotlib.pyplot as plt

LOG = logging.getLogger('guidance')

from path import *

class Map:
    def __init__(self, **kwargs):
        if 'yaml_path' in kwargs:
            self.load_yaml(kwargs['yaml_path'])
        else:
            self.img = kwargs['img']
            self.height, self.width = self.img.shape
            self.resolution = 0.005#kwargs['resolution']
            self.origin = [0., 0., 0]#kwargs['origin']
            self.occupied_thresh =  0.65
            self.free_thresh = 0.196
            
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

    # pixel of world
    def of_world(self, p_w): return [1, -1]*(p_w - self.origin[:2])/self.resolution + [0, self.height]

    # world of pixel
    #def pixel_to_world(self, p_m): return [1, -1]*((p_m - np.array([0, self.height]))*self.resolution) + self.origin[:2]
    def world_of_pixel(self, p_m): return np.array([p_m[0], self.height-p_m[1], 0])*self.resolution +  self.origin

def add_arrow_to_line2D(
    axes, line, arrow_locs=[0.2, 0.4, 0.6, 0.8],
    arrowstyle='-|>', arrowsize=1, transform=None):
    """
    Add arrows to a matplotlib.lines.Line2D at selected locations.

    Parameters:
    -----------
    axes: 
    line: list of 1 Line2D obbject as returned by plot command
    arrow_locs: list of locations where to insert arrows, % of total length
    arrowstyle: style of the arrow
    arrowsize: size of the arrow
    transform: a matplotlib transform instance, default to data coordinates

    Returns:
    --------
    arrows: list of arrows
    """
    if (not(isinstance(line, list)) or not(isinstance(line[0], 
                                                      matplotlib.lines.Line2D))):
        raise ValueError("expected a matplotlib.lines.Line2D object")
    x, y = line[0].get_xdata(), line[0].get_ydata()

    arrow_kw = dict(arrowstyle=arrowstyle, mutation_scale=10 * arrowsize)
    if transform is None:
        transform = axes.transData

    arrows = []
    for loc in arrow_locs:
        s = np.cumsum(np.sqrt(np.diff(x) ** 2 + np.diff(y) ** 2))
        n = np.searchsorted(s, s[-1] * loc)
        arrow_tail = (x[n], y[n])
        arrow_head = (np.mean(x[n:n + 2]), np.mean(y[n:n + 2]))
        p = matplotlib.patches.FancyArrowPatch(
            arrow_tail, arrow_head, transform=transform,
            **arrow_kw)
        axes.add_patch(p)
        arrows.append(p)
    return arrows



def plot(_map=None, _path=None, stride=10):
    ax = plt.gca()
    if _map is not None:
        imgplot = plt.imshow(_map.img, cmap='gray')
        if _path is not None:
            points_map = _map.of_world(_path.points)
            line = plt.plot(points_map[::stride,0], points_map[::stride,1], color='r')#, marker='.', markersize=10)
            add_arrow_to_line2D(ax, line, arrow_locs=[0., 0.1, 0.2, 0.3, 0.4, 0.6, 0.8, 0.99],
                                arrowsize=1.5)
    else:
        if _path is not None:    
            plt.plot(_path.points[:,0::stride], _path.points[:,1::stride], color='r', marker='.', markersize=10)
            
