#!/usr/bin/env python3
"""Bounded coordinated test: verified-free primary goal and actual formation poses.

Records phase changes and movement. Stops at WAITING, timeout or interruption.
An unfinished assembly is reported as incomplete, never as successful transit.
"""
import argparse,json,math,signal,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import String
from std_srvs.srv import Trigger
from formation_geometry import Grid,distance


def main():
    p=argparse.ArgumentParser();p.add_argument('--distance',type=float,default=1.5);p.add_argument('--timeout',type=float,default=600.);p.add_argument('--output',default='/tmp/fleet_mission_validation.json');args=p.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Node('fleet_mission_validation');latest={};grids={};physical={};subs=[];events=[];last_phase=None;initial={};result={'events':events,'passed':False}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def status(m):latest.clear();latest.update(json.loads(m.data))
    def models(m):physical.update({name:(pose.position.x,pose.position.y) for name,pose in zip(m.name,m.pose) if name.startswith('tb')})
    subs.append(n.create_subscription(String,'/formation/state',status,10));subs.append(n.create_subscription(ModelStates,'/gazebo/model_states',models,10))
    for t in ['/map','/tb1/global_costmap/costmap']:subs.append(n.create_subscription(OccupancyGrid,t,lambda m,t=t:grids.update({t:Grid(m)}),qos))
    pub=n.create_publisher(PoseStamped,'/formation/primary_goal',10)
    def save():Path(args.output).write_text(json.dumps(result,indent=2))
    try:
        end=time.monotonic()+120
        while time.monotonic()<end and not (latest.get('state')=='READY' and len(grids)==2 and len(physical)==5):rclpy.spin_once(n,timeout_sec=.1)
        if latest.get('state')!='READY':result['error']='Controller not READY';return
        initial=dict(physical);start=latest['poses']['tb1'];target=(start[0]+args.distance,start[1],0.)
        poly=[[-1.2,-.85],[-1.2,.85],[1.4,.85],[1.4,-.85]]
        if not all(g.frame=='map' and g.footprint_free(target,poly,.12,70) for g in grids.values()):result['error']='Primary target not verified free';return
        msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=target[0];msg.pose.position.y=target[1];msg.pose.orientation.w=1.;pub.publish(msg)
        result['primary_goal']=target;result['start_poses']=initial;print('Mission primary goal:',target,flush=True)
        started=time.monotonic();last_print=started
        while time.monotonic()-started<args.timeout:
            rclpy.spin_once(n,timeout_sec=.1)
            phase=(latest.get('state'),latest.get('reason'))
            if phase!=last_phase:
                events.append({'elapsed_wall':time.monotonic()-started,'state':phase[0],'reason':phase[1],'poses':dict(physical),'targets':latest.get('targets',{})});last_phase=phase;print(phase,flush=True);save()
            if latest.get('state') in ('WAITING','STOPPED'):result['error']=latest.get('reason');break
            if latest.get('state')=='READY' and any(e['state'] in ('TRAVELLING','BOUNDARY_TRAVELLING') for e in events):result['passed']=True;break
            if time.monotonic()-last_print>30:
                print('Progress:',{r:round(distance(initial[r],physical[r]),2) for r in initial},'state',latest.get('state'),flush=True);last_print=time.monotonic()
        if not result['passed'] and 'error' not in result:result['error']='Bounded test timed out before verified mission completion'
        result['end_poses']=dict(physical);result['physical_displacements']={r:distance(initial[r],physical[r]) for r in initial};result['last_state']=dict(latest)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT,signal.SIG_IGN);result['error']='Interrupted';result['last_state']=dict(latest)
    finally:
        result['end_poses']=dict(physical)
        result['physical_displacements']={r:distance(initial[r],physical[r]) for r in initial if r in physical}
        stop=n.create_client(Trigger,'/formation/stop')
        if stop.wait_for_service(timeout_sec=3.):
            f=stop.call_async(Trigger.Request());end=time.monotonic()+5
            while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
        save();print('Mission evidence saved:',args.output,flush=True);n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
