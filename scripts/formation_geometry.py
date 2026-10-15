"""ROS-independent occupancy and conservative fleet clearance checks."""
import math
from itertools import permutations
import numpy as np

ROBOTS = ('tb1', 'tb2', 'tb3', 'tb4', 'tb5')
OFFSETS = ((1, 1), (-1, -1), (1, -1), (-1, 1))
MODES = ('OPEN_RECTANGLE', 'COMPACT_RECTANGLE', 'SINGLE_FILE', 'WALL_WORKSPACE')


def formation_slots(mode, leader, dimensions, wall=None):
    """Exact slots; wall normal points from the wall into accessible space.

    Wall coordinates are map coordinates (x, y, normal yaw). The four robots
    occupy two rows outside the leader's reserved workspace on that side.
    Single-file static slots bootstrap assembly only, never a travelled trail.
    """
    if mode not in MODES:
        raise ValueError('Unknown formation mode: '+str(mode))
    if mode in ('OPEN_RECTANGLE', 'COMPACT_RECTANGLE'):
        prefix = 'compact' if mode == 'COMPACT_RECTANGLE' else 'rectangle'
        return rectangle(leader, dimensions[prefix+'_half_length'], dimensions[prefix+'_half_width'])
    if mode == 'SINGLE_FILE':
        return {r: (leader[0]+dx, leader[1]+dy, leader[2])
                for i,r in enumerate(ROBOTS[1:],1)
                for dx,dy in [rotate(-i*dimensions['transit_spacing'],0.,leader[2])]}
    if wall is None or not all(math.isfinite(v) for v in wall):
        raise ValueError('WALL_WORKSPACE requires a finite map wall point and accessible normal')
    nx,ny=rotate(1.,0.,wall[2]);tx,ty=-ny,nx
    depth=(leader[0]-wall[0])*nx+(leader[1]-wall[1])*ny
    if depth <= 0:
        raise ValueError('tb1 is not on the configured accessible side of the wall')
    offset=dimensions['wall_workspace_depth']
    spacing=dimensions['wall_row_spacing']
    lateral=dimensions['wall_half_width']
    return {r:(leader[0]+(offset+row*spacing)*nx+side*lateral*tx,
               leader[1]+(offset+row*spacing)*ny+side*lateral*ty, wall[2]+math.pi)
            for r,(row,side) in zip(ROBOTS[1:],((0,1),(0,-1),(1,1),(1,-1)))}


def layout_clearance(poses, polygons, gap, primary_gap):
    """Return a reason for a footprint-invalid layout, without ROS dependencies."""
    for i,a in enumerate(ROBOTS):
        for b in ROBOTS[i+1:]:
            clearance=footprint_clearance(poses[a],polygons[a],poses[b],polygons[b])
            required=pair_separation(a,b,gap,primary_gap)
            if clearance < required:
                return f'{a}/{b}: footprint clearance {clearance:.3f} m < {required:.3f} m'
    return ''


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


def flexible_assignment(poses, slots, previous=None, switch_penalty=0.8, locked=None):
    """Choose interchangeable follower slots with assignment hysteresis."""
    previous = previous or {}
    order = min((order for order in permutations(range(4)) if all(order[ROBOTS[1:].index(r)]==i for r,i in (locked or {}).items())), key=lambda order: sum(
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
        values=self.footprint_values(pose,polygon,padding)
        return bool(values is not None and values.size and np.all((values>=0)&(values<=max_cost)))

    def footprint_values(self, pose, polygon, padding=0.1):
        """Cells under the padded footprint, or None when outside the map."""
        indices=self.footprint_indices(pose,polygon,padding)
        return None if indices is None else self.cells.ravel()[indices]

    def footprint_indices(self,pose,polygon,padding=.1):
        """Flat map cell indices covered by the same padded footprint check."""
        px, py = rotate(pose[0]-self.origin[0], pose[1]-self.origin[1], -self.angle)
        theta = pose[2]-self.angle
        vertices = np.array([rotate(x, y, theta) for x, y in polygon]) + (px, py)
        margin = padding + self.resolution*math.sqrt(2)/2
        low = np.floor((vertices.min(axis=0)-margin)/self.resolution).astype(int)
        high = np.floor((vertices.max(axis=0)+margin)/self.resolution).astype(int)
        h, w = self.cells.shape
        if low[0]<0 or low[1]<0 or high[0]>=w or high[1]>=h:
            return None
        xs=(np.arange(low[0], high[0]+1)+.5)*self.resolution
        ys=(np.arange(low[1], high[1]+1)+.5)*self.resolution
        xx, yy=np.meshgrid(xs,ys)
        signed_area=sum(vertices[i,0]*vertices[(i+1)%len(vertices),1]-vertices[(i+1)%len(vertices),0]*vertices[i,1] for i in range(len(vertices)))
        orientation=1 if signed_area>0 else -1
        inside=np.ones(xx.shape,dtype=bool)
        for a,b in zip(vertices,np.roll(vertices,-1,axis=0)):
            dx,dy=b-a
            inside &= orientation*(dx*(yy-a[1])-dy*(xx-a[0])) >= -margin*math.hypot(dx,dy)
        rows,cols=np.nonzero(inside)
        return (rows+low[1])*w+cols+low[0]

    def inflation_overlap(self,pose,polygon,padding=.12):
        values=self.footprint_values(pose,polygon,padding)
        if values is None or not values.size or np.any(values<0):return None
        return float(np.count_nonzero(values>=99)/values.size)


def inflation_escape(grid,pose,polygon,commands,padding=.12,horizon=2.,verify=None):
    """Select a slow escape from inflation without crossing lethal body cells.

    The cell-extent allowance remains in the physical footprint check. Every
    sample must be known and physically free. Overlap decreases, or stays level
    along a straight reverse corridor; rotation allows one percent raster noise.
    """
    initial=grid.inflation_overlap(pose,polygon,padding)
    if initial is None or initial<=0.:return None
    for command in commands:
        previous=initial;maximum=initial;safe=True
        for i in range(11):
            projected=advance(pose,command,horizon*i/10)
            risk=grid.inflation_overlap(projected,polygon,padding)
            if (risk is None or risk>initial+.01 or risk>previous+.01 or
                not grid.footprint_free(projected,polygon,.02,99) or
                (verify is not None and not verify(projected))):safe=False;break
            previous=risk
            maximum=max(maximum,risk)
        improving=previous<initial-max(.0001,min(.02,initial*.5))
        # A verified straight reverse corridor may run alongside the inflated
        # zone before reaching its end. Do not require the whole escape to fit
        # inside a two-second horizon, but never deepen the overlap.
        parallel_reverse=command[0]<0. and abs(command[1])<1e-4 and maximum<=initial+.001
        if safe and (improving or parallel_reverse):return command
    return None


def local_obstacle_command(grid,pose,command,polygon,padding=.12):
    """Prefer slower, tighter steering before stopping an obstructed command."""
    v,w=command
    choices=[command]
    if abs(v)>1e-4:choices.extend((v*scale,w) for scale in (.5,.25,.1,0.))
    elif abs(w)>1e-4:choices.extend((0.,w*scale) for scale in (.5,.25,.1))
    for candidate in choices:
        if all(grid.footprint_free(advance(pose,candidate,t),polygon,padding,98) for t in (0.,.3,.6,1.)):
            return candidate
    return None


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


def advance(pose, command, duration):
    """Integrate a differential-drive command, including curved trajectories."""
    x,y,heading=pose;v,w=command
    if abs(w)<1e-8:
        dx,dy=rotate(v*duration,0.,heading)
    else:
        dx=v/w*(math.sin(heading+w*duration)-math.sin(heading))
        dy=v/w*(math.cos(heading)-math.cos(heading+w*duration))
    return x+dx,y+dy,heading+w*duration


def polyline_distance(pose,route):
    """Continuous centre distance, unaffected by route sample spacing."""
    best=min((distance(pose,p) for p in route),default=float('inf'))
    for a,b in zip(route,route[1:]):
        dx,dy=b[0]-a[0],b[1]-a[1];length2=dx*dx+dy*dy
        if length2<1e-12:continue
        fraction=max(0.,min(1.,((pose[0]-a[0])*dx+(pose[1]-a[1])*dy)/length2))
        best=min(best,math.hypot(pose[0]-a[0]-fraction*dx,pose[1]-a[1]-fraction*dy))
    return best


def route_preview(start, path, lookahead, step=.15):
    """Sample the near route, including the footprint swept through its turns."""
    if not path:return [start]
    nearest=min(range(len(path)),key=lambda i:distance(start,path[i]))
    if distance(start,path[nearest])>1.:return [start]
    result=[start];previous=start;remaining=lookahead
    for target in path[nearest:]:
        length=distance(previous,target)
        if length<1e-6:continue
        heading=math.atan2(target[1]-previous[1],target[0]-previous[0])
        angle=wrap(heading-previous[2]);turns=max(1,math.ceil(abs(angle)/.05))
        result.extend((previous[0],previous[1],previous[2]+angle*i/turns) for i in range(1,turns+1))
        travel=min(length,remaining);count=max(1,math.ceil(travel/step))
        result.extend((previous[0]+(target[0]-previous[0])*travel/length*i/count,
                       previous[1]+(target[1]-previous[1])*travel/length*i/count,heading)
                      for i in range(1,count+1))
        remaining-=travel;previous=result[-1]
        if remaining<=1e-6:break
    return result


def obstacle_detour_candidates(start, goal, preferred_side=1.):
    """Independent left/right waypoints, plus straight forward/reverse escapes."""
    heading=math.atan2(goal[1]-start[1],goal[0]-start[0]) if distance(start,goal)>.1 else start[2]
    for radius in (.9,1.2,1.5,2.0):
        for side in (preferred_side,-preferred_side):
            for forward in (0.,radius*.5):
                dx,dy=rotate(forward,side*radius,heading)
                yield (start[0]+dx,start[1]+dy,wrap(heading+side*math.pi/2))
        for direction in (1.,-1.):
            yield advance(start,(direction,0.),radius)


def rolling_turn_candidates(command, speed=.18, turn_rate=.20):
    """Replace near-stationary turning with gentle arcs; preserve reverse intent."""
    v,w=command
    if abs(v)>=speed or abs(w)<.08:return []
    direction=-1. if v<-.02 else 1.
    angular=max(-turn_rate,min(turn_rate,w))
    return [(direction*speed,angular),(-direction*speed,angular)]


def footprint_clearance(a, polygon_a, b, polygon_b):
    """Conservative signed clearance of two convex footprints using separating axes.

    Positive values are lower bounds on physical clearance; negative values
    indicate overlapping footprints. Unlike circumscribed discs this respects
    the long leader's orientation when another robot passes alongside it.
    """
    def vertices(pose,polygon):
        return [(pose[0]+dx,pose[1]+dy) for dx,dy in
                (rotate(x,y,pose[2]) for x,y in polygon)]
    aa,bb=vertices(a,polygon_a),vertices(b,polygon_b)
    clearance=-math.inf
    for points in (aa,bb):
        for p,q in zip(points,points[1:]+points[:1]):
            nx,ny=-(q[1]-p[1]),q[0]-p[0];length=math.hypot(nx,ny)
            if length<1e-10:continue
            nx/=length;ny/=length
            ap=[x*nx+y*ny for x,y in aa];bp=[x*nx+y*ny for x,y in bb]
            clearance=max(clearance,min(bp)-max(ap),min(ap)-max(bp))
    return clearance


def motion_conflict(a, ca, pa, b, cb, pb, gap, horizon):
    """Check swept footprints with a bound for motion between sampled poses."""
    speed_a=abs(ca[0])+abs(ca[1])*max(math.hypot(x,y) for x,y in pa)
    speed_b=abs(cb[0])+abs(cb[1])*max(math.hypot(x,y) for x,y in pb)
    radius=sum(max(math.hypot(x,y) for x,y in p) for p in (pa,pb))
    if distance(a,b)>radius+gap+(speed_a+speed_b)*horizon:return False
    steps=max(1,math.ceil(horizon/.1));dt=horizon/steps
    interpolation_margin=(speed_a+speed_b)*dt/2
    initial=footprint_clearance(a,pa,b,pb)
    previous=initial
    escaping=0.<initial<gap and speed_a+speed_b>0.
    for i in range(steps+1):
        clearance=footprint_clearance(advance(a,ca,i*dt),pa,advance(b,cb,i*dt),pb)
        # Existing safety-margin overlap may be cleared, but never worsened;
        # actual footprints must stay apart throughout the escape.
        if escaping:
            if clearance<previous-1e-6 or clearance<=interpolation_margin:return True
        elif clearance<gap+interpolation_margin:return True
        previous=clearance
    return escaping and previous<=initial+1e-5


def pair_separation(a, b, gap, primary_gap=None):
    """Use a modest extra body-to-body clearance for encounters with tb1."""
    return max(gap,primary_gap if primary_gap is not None else gap) if 'tb1' in (a,b) else gap


def yielding_gate(poses, commands, polygons, gap, horizon, primary_gap=None):
    """Give Hilti right of way, rechecking against the stopped robot.

    Auxiliary ties use robot order, so the decision remains stable across ticks.
    Never authorize a priority robot to drive into its stationary peer. Return
    unresolved pairs for the supervisor to plan temporary clearance waypoints.
    """
    result=dict(commands);blocked=set();yielding={};unresolved=set()
    order=ROBOTS;rank={r:i for i,r in enumerate(order)}
    for _ in range(len(ROBOTS)):
        changed=False
        for i,a in enumerate(ROBOTS):
            for b in ROBOTS[i+1:]:
                if result[a]==result[b]==(0.,0.):continue
                pair_gap=pair_separation(a,b,gap,primary_gap)
                if not motion_conflict(poses[a],result[a],polygons[a],poses[b],result[b],polygons[b],pair_gap,horizon):continue
                winner,loser=sorted((a,b),key=rank.get)
                yielding[loser]=winner
                safe=not motion_conflict(poses[winner],result[winner],polygons[winner],
                    poses[loser],(0.,0.),polygons[loser],pair_gap,horizon)
                stopped=(loser,) if safe else (a,b)
                if not safe:unresolved.add((winner,loser))
                for r in stopped:
                    if result[r]!=(0.,0.):changed=True;blocked.add(r);result[r]=(0.,0.)
        if not changed:break
    return result,blocked,yielding,unresolved


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


def trail_waypoint(trail, pose, behind, lookahead=0.8, cursor=0.):
    """Advance monotonically along a travelled polyline without skipping bends.

    Cursor is metres from the start, retained for each follower. The carrot is
    capped at the next meaningful bend so dense straight samples do not stall it.
    """
    if not trail:return None,cursor
    lengths=[distance(a,b) for a,b in zip(trail,trail[1:])]
    total=sum(lengths);limit=max(0.,total-behind)
    arc=0.;best=None
    for i,(a,b,length) in enumerate(zip(trail,trail[1:],lengths)):
        if length<1e-6:continue
        lower=max(0.,(cursor-arc)/length);upper=min(1.,(limit-arc)/length)
        if lower<=upper:
            projection=((pose[0]-a[0])*(b[0]-a[0])+(pose[1]-a[1])*(b[1]-a[1]))/length**2
            t=max(lower,min(upper,projection));point=(a[0]+t*(b[0]-a[0]),a[1]+t*(b[1]-a[1]))
            candidate=(distance(pose,point),arc+t*length,i)
            if best is None or (candidate[0],-candidate[1],-candidate[2])<(best[0],-best[1],-best[2]):best=candidate
        arc+=length
    if best is None:return trail_target(trail,behind),cursor
    cursor=max(cursor,best[1]);vertex=total
    def at(arc):return trail_target(trail,max(0.,total-min(total,max(0.,arc))))
    origin=at(max(0.,cursor-.5));ahead=at(cursor)
    if distance(origin,ahead)<.1:origin=at(cursor);ahead=at(min(limit,cursor+.5))
    base=math.atan2(ahead[1]-origin[1],ahead[0]-origin[0])
    arc=sum(lengths[:best[2]+1])
    for i in range(best[2]+1,len(lengths)):
        if arc>cursor+1e-6 and arc<limit:
            before=at(max(cursor,arc-.25));after=at(min(limit,arc+.25))
            if distance(before,after)>.2:
                heading=math.atan2(after[1]-before[1],after[0]-before[0])
                if abs(wrap(heading-base))>.35:vertex=arc;break
        arc+=lengths[i]
    carrot=min(limit,cursor+lookahead,vertex)
    point=at(carrot);before=at(max(0.,carrot-.5))
    heading=math.atan2(point[1]-before[1],point[0]-before[0]) if distance(point,before)>.1 else point[2]
    return (point[0],point[1],heading),cursor


def horizontal_first_routes(start, target, axis_yaw=0., clearance_step=1.):
    """Simple assembly candidates: horizontal, then slot; one outward step if blocked.

    Horizontal means map X by default. Axis rotation is configurable. The
    fallback first steps away from the desired line, never toward a blocking
    robot, then uses the same horizontal-and-slot sequence.
    """
    sx,sy=rotate(start[0],start[1],-axis_yaw)
    gx,gy=rotate(target[0],target[1],-axis_yaw)
    heading=axis_yaw if gx>=sx else wrap(axis_yaw+math.pi)
    def pose(x,y,angle):
        px,py=rotate(x,y,axis_yaw)
        return px,py,angle
    yield [pose(gx,sy,heading),target]
    direction=1. if sy>=gy else -1.
    outward=sy+direction*clearance_step
    yield [pose(sx,outward,wrap(axis_yaw+direction*math.pi/2)),
           pose(gx,outward,heading),target]


def assembly_reroute_candidates(start,target,axis_yaw=0.,clearance_step=1.,steps=3):
    """Horizontal-first plus bounded detours on either side of a blocked route."""
    yield from horizontal_first_routes(start,target,axis_yaw,clearance_step)
    sx,sy=rotate(start[0],start[1],-axis_yaw);gx,gy=rotate(target[0],target[1],-axis_yaw)
    heading=axis_yaw if gx>=sx else wrap(axis_yaw+math.pi)
    preferred=1. if sy>=gy else -1.
    for level in range(1,steps+1):
        for direction in (-preferred,preferred):
            if level==1 and direction==preferred:continue
            offset=sy+direction*clearance_step*level
            def pose(x,y,angle):
                px,py=rotate(x,y,axis_yaw);return px,py,angle
            yield [pose(sx,offset,wrap(axis_yaw+direction*math.pi/2)),
                   pose(gx,offset,heading),target]


def stationary_route_clear(start, route, robot, poses, polygons, gap, primary_gap,
                           free=lambda pose:True, sample_step=.15):
    """Check actual footprints through turns and translation, with sample margin.

    Clearance limits are unchanged. The additional margin bounds motion between
    samples; the live velocity gate still checks the commands Nav2 actually uses.
    """
    radius=max(math.hypot(x,y) for x,y in polygons[robot])
    angular_step=.05
    margin=sample_step/2+radius*angular_step/2
    def safe(sample):
        return free(sample) and all(peer==robot or
            footprint_clearance(sample,polygons[robot],pose,polygons[peer])>=
            pair_separation(robot,peer,gap,primary_gap)+margin
            for peer,pose in poses.items())
    def rotation(pose,angle):
        delta=wrap(angle-pose[2]);count=max(1,math.ceil(abs(delta)/angular_step))
        return all(safe((pose[0],pose[1],pose[2]+delta*i/count)) for i in range(count+1))
    previous=start
    for target in route:
        length=distance(previous,target)
        heading=math.atan2(target[1]-previous[1],target[0]-previous[0]) if length>1e-6 else previous[2]
        if not rotation(previous,heading):return False
        count=max(1,math.ceil(length/sample_step))
        for i in range(count+1):
            fraction=i/count
            sample=(previous[0]+fraction*(target[0]-previous[0]),
                    previous[1]+fraction*(target[1]-previous[1]),heading)
            if not safe(sample):return False
        if not rotation((target[0],target[1],heading),target[2]):return False
        previous=target
    return True


def follower_lag_gate(poses,commands,maximum_lag,recovering=(),allow_turning=False):
    """Coordinate healthy followers; independent recovery retains collision gates."""
    result=dict(commands)
    for robot,follower in zip(ROBOTS,ROBOTS[1:]):
        if follower not in recovering and distance(poses[robot],poses[follower])>maximum_lag:
            result[robot]=(0.,result[robot][1] if allow_turning else 0.)
    return result


def maneuver_sequence_search(start,goal,controls,duration,safe,score,depth=3,beam_width=4,terminal=None,parking_distance=0.,parking_rate=.15):
    """Incremental bounded search; yield after each checked segment.

    Keeping several branches permits left/right and forward/reverse changes
    even when a useful first segment temporarily increases goal distance.
    The caller resumes this generator within its planning time budget.
    """
    beam=[(score(start),start,[])];best=None
    def trajectory_cost(end,sequence):
        gears=sum(a[0]*b[0]<0 for a,b in zip(sequence,sequence[1:]))
        steering=sum(a[1]*b[1]<0 for a,b in zip(sequence,sequence[1:]))
        return score(end)+.01*len(sequence)+(.08*gears+.02*steering if parking_distance>0. else 0.)
    if parking_distance>0.:
        speed=max((abs(v) for v,w in controls if v<0),default=0.)
        if speed>0.:
            count=max(2,math.ceil(parking_distance/(speed*duration)))
            for reverse in ((-speed,parking_rate),(-speed,-parking_rate)):
                for forward in ((speed,parking_rate),(speed,-parking_rate),(speed,parking_rate*2),(speed,-parking_rate*2),(speed,0.)):
                    pose=start;sequence=[];valid=True
                    for command in [reverse]*count+[forward]*count:
                        valid=safe(pose,command,len(sequence)*duration)
                        if valid:pose=advance(pose,command,duration);sequence.append(command)
                        yield None
                        if not valid:break
                    if valid and (terminal is None or terminal(pose)):
                        candidate=(trajectory_cost(pose,sequence),pose,sequence)
                        if best is None or candidate[0]<best[0]:best=candidate
    for level in range(depth):
        expanded=[]
        for _,pose,sequence in beam:
            for command in controls:
                # Do not replace a parking arc with rapid reverse/forward
                # shuffling. The swept checks may still stop it at any moment.
                reverse_run=0.
                for v,w in reversed(sequence):
                    if v>=0.:break
                    reverse_run+=abs(v)*duration
                if (command[0]<0 and sequence and sequence[-1][0]<0 and
                    command[1]*sequence[-1][1]<0 and reverse_run<parking_distance):
                    yield None
                    continue
                if command[0]>0 and sequence and sequence[-1][0]<0 and reverse_run<parking_distance:
                    yield None
                    continue
                if safe(pose,command,len(sequence)*duration):
                    end=advance(pose,command,duration)
                    candidate=(trajectory_cost(end,sequence+[command]),end,sequence+[command])
                    expanded.append(candidate)
                    if len(candidate[2])>=2 and (command[0]>=0 or reverse_run+abs(command[0])*duration>=parking_distance) and (terminal is None or terminal(end)) and (best is None or candidate[0]<best[0]):best=candidate
                yield None
        if not expanded:break
        # Preserve different first steering choices instead of filling the beam
        # with nearly identical straight trajectories.
        ordered=sorted(expanded,key=lambda item:item[0]);beam=[];first=set()
        for item in ordered:
            key=item[2][0]
            if key not in first:beam.append(item);first.add(key)
            if len(beam)>=beam_width:break
    return best[2] if best is not None and best[0]<score(start)-.01 else []
