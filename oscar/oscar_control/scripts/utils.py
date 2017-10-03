
import math, numpy as np, rospy, geometry_msgs.msg


def list_of_xyz(p): return [p.x, p.y, p.z]
def array_of_xyz(p): return np.array(list_of_xyz(p))
def list_of_xyzw(q): return [q.x, q.y, q.z, q.w]

def deg_of_rad(r): return r/math.pi*180


class SmocapListener:
    def __init__(self):
        rospy.Subscriber('/smocap/est_world', geometry_msgs.msg.PoseWithCovarianceStamped, self.smocap_cbk)
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
