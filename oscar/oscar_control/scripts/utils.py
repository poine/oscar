
import math, numpy as np, rospy, geometry_msgs.msg, tf, pickle
import matplotlib
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

def list_of_xyz(p): return [p.x, p.y, p.z]
def array_of_xyz(p): return np.array(list_of_xyz(p))
def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]

def deg_of_rad(r): return r/math.pi*180


class SmocapListener:
    def __init__(self):
        rospy.Subscriber('/smocap/est_marker', geometry_msgs.msg.PoseWithCovarianceStamped, self.smocap_cbk)
        self.ts = None
        self.pose = None
        self.vel = None
        
    def smocap_cbk(self, msg):
        if self.pose is not None:
            p1 = array_of_xyz(self.pose.position)
            p2 = array_of_xyz(msg.pose.pose.position)
            dt = msg.header.stamp.to_sec() - self.ts
            self.vel = np.linalg.norm((p2-p1)/dt)
        self.pose = msg.pose.pose
        self.ts = msg.header.stamp.to_sec()

    def get_loc_and_yaw(self):
        l = array_of_xyz(self.pose.position)[:2]
        y = tf.transformations.euler_from_quaternion(list_of_xyzw(self.pose.orientation))[2]
        return l, y



class LinRef:
    ''' Linear Reference Model (with first order integration)'''
    def __init__(self, K):
        '''K: coefficients of the caracteristic polynomial, in ascending powers order,
              highest order ommited (normalized to -1)'''
        self.K = K; self.order = len(K)
        self.X = np.zeros(self.order+1)

    def run(self, dt, sp):
        self.X[:self.order] += self.X[1:self.order+1]*dt
        e =  np.array(self.X[:self.order]); e[0] -= sp
        self.X[self.order] = np.sum(e*self.K)
        return self.X

    def poles(self):
        return np.roots(np.insert(np.array(self.K[::-1]), 0, -1))


class FirstOrdLinRef(LinRef):
    def __init__(self, tau):
        LinRef.__init__(self, [-1/tau])

class SecOrdLinRef(LinRef):
    def __init__(self, omega, xi):
        LinRef.__init__(self, [-omega**2, -2*xi*omega])

            
class MotorRef(LinRef):
    ''' this is a third order ref, driven by its first derivative '''
    def __init__(self, omega, xi):
        LinRef.__init__(self, [0, -omega**2, -2*xi*omega])

    def run(self, dt, sp): 
        self.X[:self.order] += self.X[1:self.order+1]*dt
        e =  np.array(self.X[:self.order]); e[1] -= sp
        self.X[self.order] = np.sum(e*self.K)
        return self.X


def save_trajectory(time, X, U, desc, filename):
    with open(filename, "wb") as f:
        pickle.dump([time, X, U, desc], f)
        
def load_trajectory(filename):
    with open(filename, "rb") as f:
        time, X, U, desc = pickle.load(f)
    return time, X, U, desc


''' input test vectors '''

def make_random_pulses(dt, size, min_nperiod=1, max_nperiod=10, min_intensity=-1, max_intensity=1.):
    ''' make a vector of pulses of randon duration and intensities '''
    npulses = size/max_nperiod*2
    durations = np.random.random_integers(low=min_nperiod, high=max_nperiod, size=npulses)
    intensities =  np.random.uniform(low=min_intensity, high=max_intensity, size=npulses)
    pulses = []
    for duration, intensitie in zip(durations, intensities):
        pulses += [intensitie for i in range(duration)]
    pulses = np.array(pulses)
    time = np.linspace(0, dt*len(pulses), len(pulses))
    return time, pulses

def step(t, a0=-1, a1=1, dt=4, t0=0): return a0 if math.fmod(t+t0, dt) > dt/2 else a1
def sine_sweep(t, omega=2, domega=0.5, domega1=0.5): return math.sin(omega*(1-domega1*math.sin(domega*t))*t)


def random_input_vec(time): return np.random.uniform(low=-1.0, high=1.0, size=len(time))
def step_input_vec(time): return [step(t) for t in time]
def sine_input_vec(time): return np.sin(time)
def sawtooth_input_vec(time): return scipy.signal.sawtooth(time)
def sine_swipe_input_vec(time): return [sine_sweep(t) for t in time]

"""
Compute numerical jacobian 
"""
def num_jacobian(X, U, dyn):
    s_size = len(X)
    i_size = len(U)
    epsilonX = (0.1*np.ones(s_size)).tolist()
    dX = np.diag(epsilonX)
    A = np.zeros((s_size, s_size))
    for i in range(0, s_size):
        dx = dX[i,:]
        delta_f = dyn(X+dx/2, 0, U) - dyn(X-dx/2, 0, U)
        delta_f = delta_f / dx[i]
        A[:,i] = delta_f

    epsilonU = (0.1*np.ones(i_size)).tolist()
    dU = np.diag(epsilonU)
    B = np.zeros((s_size,i_size))
    for i in range(0, i_size):
        du = dU[i,:]
        delta_f = dyn(X, 0, U+du/2) - dyn(X, 0, U-du/2)
        delta_f = delta_f / du[i]
        B[:,i] = delta_f
        
    return A,B

def get_om_xi(lambda1):
    om = math.sqrt(lambda1.real**2+lambda1.imag**2)
    xi = math.cos(np.arctan2(lambda1.imag, -lambda1.real))
    return om, xi

def get_lambdas(om, xi):
    re, im = -om*xi, om*math.sqrt(1-xi**2)
    return [complex(re, im), complex(re, -im)]

def get_precommand(A, B, C, K):
    tmp1 = np.linalg.inv(A - np.dot(B, K))
    tmp2 = np.dot(np.dot(C, tmp1), B)
    nr, nc = tmp2.shape
    H = -np.linalg.inv(tmp2) if nr == nc else -np.linalg.pinv(tmp2)
    return H



'''
   Plotting
'''
my_title_spec = {'color'    : 'k', 'fontsize'   : 20 }

def prepare_fig(fig=None, window_title=None, figsize=(20.48, 10.24), margins=None):
    if fig == None:
        fig = plt.figure(figsize=figsize)
    #else:
    #    plt.figure(fig.number)
    if margins:
        left, bottom, right, top, wspace, hspace = margins
        fig.subplots_adjust(left=left, right=right, bottom=bottom, top=top,
                            hspace=hspace, wspace=wspace)
    if window_title:
         fig.canvas.set_window_title(window_title)
    return fig

def decorate(ax, title=None, xlab=None, ylab=None, legend=None, xlim=None, ylim=None):
    ax.xaxis.grid(color='k', linestyle='-', linewidth=0.2)
    ax.yaxis.grid(color='k', linestyle='-', linewidth=0.2)
    if xlab:
        ax.xaxis.set_label_text(xlab)
    if ylab:
        ax.yaxis.set_label_text(ylab)
    if title:
        ax.set_title(title, my_title_spec)
    if legend <> None:
        ax.legend(legend, loc='best')
    if xlim <> None:
        ax.set_xlim(xlim[0], xlim[1])
    if ylim <> None:
        ax.set_ylim(ylim[0], ylim[1])
