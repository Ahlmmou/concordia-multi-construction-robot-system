"""ROS-independent occupancy and conservative fleet clearance checks."""
import math
from itertools import permutations
import numpy as np

ROBOTS = ('tb1', 'tb2', 'tb3', 'tb4', 'tb5')
OFFSETS = ((1, 1), (-1, -1), (1, -1), (-1, 1))


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def distance(a, b):
    return math.hypot(a[0]-b[0], a[1]-b[1])


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def rotate(x, y, angle):
    return (math.cos(angle)*x-math.sin(angle)*y, math.sin(angle)*x+math.cos(angle)*y)


def rectangle(leader, half_length, half_width):
    result = {}
    for robot, (sx, sy) in zip(ROBOTS[1:], OFFSETS):
        dx, dy = rotate(sx*half_length, sy*half_width, leader[2])
        result[robot] = (leader[0]+dx, leader[1]+dy, leader[2])
    return result


def flexible_assignment(poses, slots, previous=None, switch_penalty=0.8):
    """Choose interchangeable follower slots with assignment hysteresis."""
    previous = previous or {}
    order = min(permutations(range(4)), key=lambda order: sum(
        distance(poses[r], slots[i]) +
        (switch_penalty if r in previous and previous[r] != i else 0.)
        for r, i in zip(ROBOTS[1:], order)))
    return dict(zip(ROBOTS[1:], order))


def boundary_order(poses):
    """Draw around current positions rather than permanent robot identities."""
    cx = sum(poses[r][0] for r in ROBOTS[1:])/4
    cy = sum(poses[r][1] for r in ROBOTS[1:])/4
    return sorted(ROBOTS[1:], key=lambda r: math.atan2(poses[r][1]-cy, poses[r][0]-cx))


def trail_target(trail, behind):
    """Interpolate an actual travelled trail, newest point last."""
    if not trail:
        return None
    for i in range(len(trail)-1, 0, -1):
        a, b = trail[i-1], trail[i]
        length = distance(a, b)
        if length > 1e-6 and behind <= length:
            fraction = behind/length
            return (b[0]+fraction*(a[0]-b[0]), b[1]+fraction*(a[1]-b[1]), math.atan2(b[1]-a[1], b[0]-a[0]))
        behind -= length
    return trail[0]


class Grid:
    def __init__(self, message):
        self.frame = message.header.frame_id.lstrip('/')
        self.resolution = message.info.resolution
        self.origin = (message.info.origin.position.x, message.info.origin.position.y)
        self.angle = yaw(message.info.origin.orientation)
        self.cells = np.frombuffer(message.data, dtype=np.int8).reshape(message.info.height, message.info.width)

    def footprint_free(self, pose, polygon, padding=0.1, max_cost=70):
        """Reject unknown/out-of-map cells; test padded convex polygon plus cell extent."""
        px, py = rotate(pose[0]-self.origin[0], pose[1]-self.origin[1], -self.angle)
        theta = pose[2]-self.angle
        vertices = np.array([rotate(x, y, theta) for x, y in polygon]) + (px, py)
        margin = padding + self.resolution*math.sqrt(2)/2
        low = np.floor((vertices.min(axis=0)-margin)/self.resolution).astype(int)
        high = np.floor((vertices.max(axis=0)+margin)/self.resolution).astype(int)
        h, w = self.cells.shape
        if low[0]<0 or low[1]<0 or high[0]>=w or high[1]>=h:
            return False
        xs=(np.arange(low[0], high[0]+1)+.5)*self.resolution
        ys=(np.arange(low[1], high[1]+1)+.5)*self.resolution
        xx, yy=np.meshgrid(xs,ys)
        signed_area=sum(vertices[i,0]*vertices[(i+1)%len(vertices),1]-vertices[(i+1)%len(vertices),0]*vertices[i,1] for i in range(len(vertices)))
        orientation=1 if signed_area>0 else -1
        inside=np.ones(xx.shape,dtype=bool)
        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
            dx,dy=b-a
            inside &= orientation*(dx*(yy-a[1])-dy*(xx-a[0])) >= -margin*math.hypot(dx,dy)
        values=self.cells[low[1]:high[1]+1,low[0]:high[0]+1][inside]
        return bool(values.size and np.all((values>=0)&(values<=max_cost)))


def candidates(preferred, radius, step):
    yield preferred
    for ring in range(1, int(radius/step)+1):
        r=ring*step
        count=max(8, int(2*math.pi*r/step))
        for i in range(count):
            a=2*math.pi*i/count
            yield (preferred[0]+r*math.cos(a), preferred[1]+r*math.sin(a), preferred[2])


def swept_distance(a, va, b, vb, horizon):
    """Closest center distance for two constant world velocities over a time interval."""
    px,py=a[0]-b[0],a[1]-b[1]
    vx,vy=va[0]-vb[0],va[1]-vb[1]
    norm=vx*vx+vy*vy
    t=max(0.,min(horizon,-(px*vx+py*vy)/norm)) if norm>1e-10 else 0.
    return math.hypot(px+t*vx,py+t*vy)


def conservative_gate(poses, commands, radii, gap, horizon):
    """Stop commands whose swept discs conflict, including against stationary peers.

    Stop both sides of an imminent conflict rather than allowing the priority robot
    to drive into a stopped peer. Nav2 may subsequently find a different trajectory.
    """
    result=dict(commands);blocked=set()
    for _ in range(len(ROBOTS)):
        newly=set()
        for i,a in enumerate(ROBOTS):
            for b in ROBOTS[i+1:]:
                av=rotate(result[a][0],0,poses[a][2]);bv=rotate(result[b][0],0,poses[b][2])
                if swept_distance(poses[a],av,poses[b],bv,horizon)<radii[a]+radii[b]+gap:
                    # Permit only commands that increase existing separation.
                    if distance(poses[a],poses[b])<radii[a]+radii[b]+gap:
                        rel=(av[0]-bv[0],av[1]-bv[1]);p=(poses[a][0]-poses[b][0],poses[a][1]-poses[b][1])
                        if p[0]*rel[0]+p[1]*rel[1]>0:
                            continue
                    if result[a]!=(0.,0.):newly.add(a)
                    if result[b]!=(0.,0.):newly.add(b)
        if not newly:break
        for r in newly:result[r]=(0.,0.)
        blocked |= newly
    return result,blocked


def boundary_followers_holding(poses, formation_heading, half_length, half_width,
                               tolerance, leader_velocity):
    """Hold translation that worsens excessive corner error; permit catch-up.

    Corner axes belong to the enclosure, independently of the leader's steering
    yaw. Rotation in place leaves this enclosure fixed and is checked by the
    physical collision/footprint gates separately.
    """
    if abs(leader_velocity)<1e-6:return set()
    leader=poses['tb1'];vx,vy=rotate(leader_velocity,0.,leader[2])
    corners=rectangle((leader[0],leader[1],formation_heading),half_length,half_width)
    holding=set()
    for robot,target in corners.items():
        ex,ey=poses[robot][0]-target[0],poses[robot][1]-target[1]
        if math.hypot(ex,ey)>tolerance and ex*vx+ey*vy<=0.:
            holding.add(robot)
    return holding
