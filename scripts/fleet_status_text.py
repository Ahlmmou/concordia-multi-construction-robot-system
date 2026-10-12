"""Human-readable fleet progress without changing navigation behavior."""
import math

ROBOTS = ('tb1', 'tb2', 'tb3', 'tb4', 'tb5')
TITLES = {'MODE_TRANSITION':'Changing formation','READY':'Ready for a goal','PREFLIGHT':'Checking robot readiness',
          'FORMING_BOUNDARY':'Building the boundary','BOUNDARY_TRAVELLING':'Moving with the boundary',
          'FORMING_LINE':'Preparing the followers','TRAVELLING':'Following tb1',
          'FORMING_RECTANGLE':'Arranging the boundary','WAITING':'Fleet paused','STOPPED':'Goal stopped'}


def arrived(pose, target):
    angle=math.atan2(math.sin(pose[2]-target[2]),math.cos(pose[2]-target[2]))
    return math.dist(pose[:2],target[:2])<=.4 and abs(angle)<=.45


def describe(state, motion, goal=None):
    phase=state.get('state','CONNECTING');poses=state.get('poses',{});targets=state.get('targets',{})
    title=TITLES.get(phase,'Connecting to the fleet')
    detail=state.get('reason','Waiting for live robot updates.')
    for raw,plain in [('Nav2 actions missing','navigation server is unavailable'),
                      ('amcl not verified active','positioning node is starting or unavailable'),
                      ('map-to-base transform not fresh','waiting for a fresh robot position'),
                      ('lidar stale','waiting for fresh laser measurements'),
                      ('global_costmap stale','waiting for the obstacle map'),
                      ('local_costmap stale','waiting for nearby obstacle updates')]:
        detail=detail.replace(raw,plain)
    pending=[r for r in ROBOTS[1:] if r in poses and r in targets and not arrived(poses[r],targets[r])]
    if phase in ('FORMING_BOUNDARY','FORMING_LINE','FORMING_RECTANGLE'):
        detail=f'{pending[0]} is adjusting its corner. tb1 holds position.' if pending else 'Verifying the formation before continuing.'
    elif phase=='BOUNDARY_TRAVELLING':
        if 'tb1' in poses and 'tb1' in targets and arrived(poses['tb1'],targets['tb1']) and pending:
            detail='tb1 reached this stage; waiting for '+', '.join(pending)+'.'
        elif abs(motion.get('tb1',(0.,0.))[0] or 0.)<=.02 and abs(motion.get('tb1',(0.,0.))[1] or 0.)>.04:
            detail='tb1 is turning to face the next section of the route.'
        elif state.get('flexible_boundary'):detail='tb1 is following your goal; followers can change sides and adjust around obstacles.'
        else:detail='Advancing the rectangle toward your goal.'
    elif phase=='STOPPED':detail='Send a new 2D Goal Pose in RViz when you are ready.'
    elif phase=='WAITING' and 'Whole boundary cannot fit' in detail:
        detail='The next section is too tight for the full boundary. Choose a goal with a wider route.'
    holding=state.get('waiting_for_followers',[])
    if holding and phase=='BOUNDARY_TRAVELLING':detail='tb1 is waiting for '+', '.join(holding)+' to catch up; it can still turn in place.'
    blocked=state.get('blocked',[])
    if blocked and phase not in ('WAITING','STOPPED','PREFLIGHT'):detail='Clearance check is holding '+', '.join(blocked)+'.'
    yielding=state.get('yielding_to',{})
    maneuver=state.get('clearance_maneuver')
    if yielding and phase not in ('WAITING','STOPPED','PREFLIGHT'):
        detail='; '.join(f'{r} is yielding to {peer}' for r,peer in sorted(yielding.items()))+'. Movement resumes automatically when clear.'
        if maneuver:detail=f"{maneuver['peer']} holds while {maneuver['robot']} moves to clear the route. The formation resumes automatically."
    if state.get('rolling_turn_active') and phase=='BOUNDARY_TRAVELLING' and not yielding and not blocked:
        direction='backward' if motion.get('tb1',(0.,0.))[0]<0. else 'forward'
        detail=f'tb1 is taking a gentle {direction} arc to turn toward the route.'
    recoveries=state.get('obstacle_recoveries',{})
    if recoveries and phase not in ('WAITING','STOPPED','PREFLIGHT'):
        detail='; '.join(r+': '+recovery.get('state','Checking escape route').lower() for r,recovery in sorted(recoveries.items()))+'. Formation targets resume after clearance.'
    remaining=None
    if goal is not None and 'tb1' in poses and phase not in ('STOPPED','READY','CONNECTING'):
        remaining=math.dist(poses['tb1'][:2],goal[:2])
    rows=[]
    for r in ROBOTS:
        speed,turn=motion.get(r,(None,None));target=targets.get(r);pose=poses.get(r)
        distance=math.dist(pose[:2],target[:2]) if pose is not None and target is not None else None
        if phase in ('WAITING','STOPPED','PREFLIGHT'):label='On hold'
        elif r in yielding:label='Yielding to '+yielding[r]
        elif r in recoveries:label=recoveries[r].get('state','Recovering from obstacle')
        elif maneuver and r==maneuver['robot']:label='Clearing the route'
        elif speed is None:label='Waiting for data'
        elif r=='tb1' and state.get('rolling_turn_active') and abs(speed)>.02:label='Reversing turn' if speed<0. else 'Rolling turn'
        elif speed<-.02:label='Reversing'
        elif speed>.02:label='Moving'
        elif abs(turn or 0.)>.04:label='Turning'
        elif pose is not None and target is not None and arrived(pose,target):label='At its target'
        elif state.get('actions',{}).get(r) in ('active','sending'):label='Starting / paused'
        else:label='Holding position'
        rows.append({'robot':r,'label':label,'speed':abs(speed) if speed is not None else None,'remaining':distance})
    if state.get('requested_mode'):
        title += ' · '+state['requested_mode']
        detail += ' Active: '+str(state.get('active_mode') or 'unverified')+'. Transition: '+state.get('transition_state','IDLE')+'.'
    return {'title':title,'detail':detail,'remaining':remaining,'rows':rows,'phase':phase}
