#!/usr/bin/env python3
"""Sequential footprint-checked navigation tests with actual TF/Gazebo evidence."""
import argparse
import json
import math
import signal
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from gazebo_msgs.msg import ModelStates
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import String
from std_srvs.srv import Trigger
from formation_geometry import ROBOTS,Grid,distance


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--robot',choices=ROBOTS);parser.add_argument('--step',type=float,default=.8);parser.add_argument('--output',default='/tmp/fleet_navigation_validation.json');parser.add_argument('--timeout',type=float,default=240.);args=parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('fleet_navigation_validation');status={};maps={};physical={};subs=[];report=[];handle=None;success=False
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def receive(msg):status.clear();status.update(json.loads(msg.data))
    def models(msg):physical.update({name:(p.position.x,p.position.y) for name,p in zip(msg.name,msg.pose) if name in ROBOTS})
    subs.append(node.create_subscription(String,'/formation/state',receive,10));subs.append(node.create_subscription(ModelStates,'/gazebo/model_states',models,10))
    for topic in ['/map']+[f'/{r}/global_costmap/costmap' for r in ROBOTS]:
        subs.append(node.create_subscription(OccupancyGrid,topic,lambda m,t=topic:maps.update({t:Grid(m)}),qos))
    def wait(predicate,timeout):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if predicate():return True
        return False
    def save():Path(args.output).write_text(json.dumps(report,indent=2))
    try:
        if not wait(lambda:status.get('state')=='READY' and len(maps)==6 and len(physical)==5,args.timeout):
            report.append({'readiness_failed':status});print('Readiness failed:',status,flush=True);return
        for r in [args.robot] if args.robot else ROBOTS:
            client=ActionClient(node,NavigateToPose,f'/{r}/navigate_to_pose')
            if not client.wait_for_server(timeout_sec=5.):report.append({'robot':r,'error':'No Nav2 action server'});break
            start=status['poses'][r];physical_start=physical[r];target=None
            polygon=[[-1.2,-.85],[-1.2,.85],[1.4,.85],[1.4,-.85]] if r=='tb1' else [[-.3,-.3],[-.3,.3],[.475,.3],[.475,-.3]]
            for dx,dy in [(args.step,0.),(0.,args.step),(-args.step,0.),(0.,-args.step)]:
                candidate=(start[0]+dx,start[1]+dy,math.atan2(dy,dx))
                if all(maps[t].frame=='map' and maps[t].footprint_free(candidate,polygon,.12,70) for t in ['/map',f'/{r}/global_costmap/costmap']):
                    target=candidate;break
            if target is None:
                report.append({'robot':r,'error':'Chosen target not verified free for full footprint'});break
            msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=float(target[0]);msg.pose.position.y=float(target[1]);msg.pose.orientation.z=math.sin(target[2]/2);msg.pose.orientation.w=math.cos(target[2]/2)
            goal=NavigateToPose.Goal();goal.pose=msg
            print('Testing',r,'from',start,'to',target,flush=True)
            future=client.send_goal_async(goal)
            if not wait(future.done,15.) or not future.result().accepted:
                report.append({'robot':r,'error':'Goal rejected or no response'});break
            handle=future.result();result=handle.get_result_async();finished=wait(result.done,args.timeout)
            end=status.get('poses',{}).get(r,start);physical_end=physical[r]
            entry={'robot':r,'start':start,'target':target,'end':end,'displacement':distance(start,end),'physical_start':physical_start,'physical_end':physical_end,'physical_displacement':distance(physical_start,physical_end),'remaining':distance(target,end),'action_status':result.result().status if finished else None,'controller_state':status.get('state')}
            entry['passed']=finished and entry['action_status']==4 and entry['displacement']>.35 and entry['physical_displacement']>.35 and entry['remaining']<.45
            report.append(entry);save();print(json.dumps(entry),flush=True)
            if not entry['passed']:break
            handle=None
        success=len(report)==(1 if args.robot else 5) and all(e.get('passed') for e in report)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT,signal.SIG_IGN);report.append({'interrupted':True,'last_status':status})
    finally:
        if handle is not None:wait(handle.cancel_goal_async().done,5.)
        if not success:
            stop=node.create_client(Trigger,'/formation/stop')
            if stop.wait_for_service(timeout_sec=3.):wait(stop.call_async(Trigger.Request()).done,5.)
        save();node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
