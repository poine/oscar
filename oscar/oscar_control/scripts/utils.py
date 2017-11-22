
import math, numpy as np, rospy, geometry_msgs.msg, tf


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
              highesr order ommited (normalized to -1)'''
        self.K = K; self.order = len(K)
        self.X = np.zeros(self.order+1)

    def run(self, dt, sp):
        self.X[:self.order] += self.X[1:self.order+1]*dt
        e =  np.array(self.X[:self.order]); e[0] -= sp
        self.X[self.order] = np.sum(e*self.K)
        return self.X

    def poles(self):
        return np.roots(np.insert(np.array(self.coefs[::-1]), 0, -1))


class FirstOrdLinRef(LinRef):
    def __init__(self, tau):
        LinRef.__init__(self, [-1/tau])

class SecOrdLinRef(LinRef):
    def __init__(self, omega, xi):
        LinRef.__init__(self, [-omega**2, -2*xi*omega])
