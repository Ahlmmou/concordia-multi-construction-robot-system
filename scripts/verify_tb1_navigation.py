#!/usr/bin/env python3
"""Observe a goal sent with RViz's Nav2 Goal tool, or send an equivalent CLI goal."""
import argparse
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from action_msgs.msg import GoalStatusArray
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetEntityState
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rosidl_runtime_py.convert import message_to_ordereddict


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--goal', nargs=3, type=float, metavar=('X','Y','YAW'))
    ap.add_argument('--expected-goal', nargs=3, type=float, default=[3.5,1.6,0.0])
    ap.add_argument('--position-tolerance', type=float, default=.25)
    ap.add_argument('--timeout', type=float, default=240)
    ap.add_argument('--output', default='/tmp/tb1_navigation.json')
    args=ap.parse_args()
    rclpy.init(); n=Node('verify_tb1_navigation'); n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    buf=Buffer(); subs=[]; grids={}; poses={}; latest_scan={}; statuses=[]; feedback=[]; trajectory=[]; plans=[]; clocks=[]; commands=[]; result_future=None; goal_handle=None; goal_id=None
    report={'source':'terminal' if args.goal else 'RViz Nav2 Goal tool','expected_goal':args.goal or args.expected_goal}
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def tf(m,static):
        for t in m.transforms:(buf.set_transform_static if static else buf.set_transform)(t,'test')
    def models(m):
        for name,p in zip(m.name,m.pose):
            if name.startswith('tb'):poses[name]=[p.position.x,p.position.y,yaw(p.orientation)]
    def scan(m):latest_scan.update(finite=sum(math.isfinite(v) for v in m.ranges),rays=len(m.ranges),stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9)
    def clock(m):clocks.append([time.monotonic(),m.clock.sec+m.clock.nanosec*1e-9])
    def status(m):
        nonlocal goal_id
        if m.status_list:
            for s in m.status_list:
                if goal_id is None and s.status in [1,2]:goal_id=bytes(s.goal_info.goal_id.uuid).hex()
                if bytes(s.goal_info.goal_id.uuid).hex()==goal_id:statuses.append({'status':s.status,'time':time.monotonic()})
    def receive_feedback(m):
        feedback.append({'goal_id':bytes(m.goal_id.uuid).hex(),**message_to_ordereddict(m.feedback)})
    def plan(m):
        if m.poses:plans.append({'frame':m.header.frame_id,'points':[[p.pose.position.x,p.pose.position.y] for p in m.poses],'goal':[m.poses[-1].pose.position.x,m.poses[-1].pose.position.y,yaw(m.poses[-1].pose.orientation)]})
    subs.append(n.create_subscription(ModelStates,'/gazebo/model_states',models,qos_profile_sensor_data))
    subs.append(n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data))
    subs.append(n.create_subscription(LaserScan,'/tb1/scan',scan,qos_profile_sensor_data))
    subs.append(n.create_subscription(TFMessage,'/tb1/tf',lambda m:tf(m,False),qos_profile_sensor_data))
    subs.append(n.create_subscription(TFMessage,'/tb1/tf_static',lambda m:tf(m,True),qos))
    subs.append(n.create_subscription(GoalStatusArray,'/tb1/navigate_to_pose/_action/status',status,qos))
    subs.append(n.create_subscription(NavigateToPose.Impl.FeedbackMessage,'/tb1/navigate_to_pose/_action/feedback',receive_feedback,qos_profile_sensor_data))
    subs.append(n.create_subscription(NavPath,'/tb1/plan',plan,qos_profile_sensor_data))
    subs.append(n.create_subscription(Twist,'/tb1/cmd_vel',lambda m:commands.append([m.linear.x,m.angular.z]),qos_profile_sensor_data))
    for topic in ['/map','/tb1/global_costmap/costmap','/tb1/local_costmap/costmap']:
        subs.append(n.create_subscription(OccupancyGrid,topic,lambda m,t=topic:grids.update({t:m}),qos))
    client=ActionClient(n,NavigateToPose,'/tb1/navigate_to_pose')
    get_result=n.create_client(NavigateToPose.Impl.GetResultService,'/tb1/navigate_to_pose/_action/get_result')
    entity_client=n.create_client(GetEntityState,'/gazebo/get_entity_state')
    entity_pending=[]
    def request_entities():
        if not entity_client.service_is_ready():return
        for robot in ['tb1','tb2','tb3','tb4','tb5']:
            req=GetEntityState.Request();req.name=robot;req.reference_frame='world'
            entity_pending.append((robot,entity_client.call_async(req)))
    def receive_entities():
        for robot,f in entity_pending[:]:
            if f.done():
                response=f.result()
                if response.success:
                    p=response.state.pose;poses[robot]=[p.position.x,p.position.y,yaw(p.orientation)]
                entity_pending.remove((robot,f))
    def spin(seconds):
        end=time.monotonic()+seconds
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    request_entities();end_ready=time.monotonic()+30
    while time.monotonic()<end_ready:
        rclpy.spin_once(n,timeout_sec=.05);receive_entities()
        if len(poses)==5 and len(grids)==3 and client.server_is_ready() and latest_scan.get('finite',0)>0:break
        if not entity_pending and len(poses)<5:
            request_entities();spin(.25)
    print('Readiness:',{'poses':list(poses),'grids':list(grids),'action':client.server_is_ready(),'scan':latest_scan},flush=True)
    if len(poses)!=5 or len(grids)!=3 or not client.server_is_ready() or latest_scan.get('finite',0)==0:raise RuntimeError('Missing physical poses, costmaps, lidar returns or action server')
    start=poses['tb1'].copy(); auxiliaries={r:poses[r].copy() for r in ['tb2','tb3','tb4','tb5']}
    goal=args.goal or args.expected_goal
    radius=math.hypot(1.4,.85)+.05+.30
    def disk_cost(m,x,y):
        res=m.info.resolution;ox=m.info.origin.position.x;oy=m.info.origin.position.y;k=math.ceil(radius/res)+1;cx=int((x-ox)/res);cy=int((y-oy)/res);lx=cx-k;hx=cx+k+1;ly=cy-k;hy=cy+k+1
        if lx<0 or ly<0 or hx>m.info.width or hy>m.info.height:return 100
        a=np.asarray(m.data,dtype=np.int8).reshape(m.info.height,m.info.width)[ly:hy,lx:hx];xs=ox+(np.arange(lx,hx)+.5)*res;ys=oy+(np.arange(ly,hy)+.5)*res;mask=(xs[None,:]-x)**2+(ys[:,None]-y)**2<(radius+res/2**.5)**2;v=a[mask]
        return int(v.max()) if np.all(v>=0) else 100
    corridor=[]
    for p in np.linspace(start[:2],goal[:2],31):
        c={t:disk_cost(grids[t],*p) for t in ['/map','/tb1/global_costmap/costmap']}; peer=min(math.dist(p,pose[:2])-radius-math.hypot(.475,.3) for pose in auxiliaries.values());corridor.append({'xy':p.tolist(),'max_cost':c,'peer_gap':peer})
    assert all(max(x['max_cost'].values())<99 and x['peer_gap']>.30 for x in corridor),'Goal corridor does not clear full footprint/margin and peers'
    report.update(start_gazebo=start,auxiliary_start=auxiliaries,corridor_check=corridor,scan_at_start=latest_scan.copy(),command_publishers=[e.node_namespace.rstrip('/')+'/'+e.node_name for e in n.get_publishers_info_by_topic('/tb1/cmd_vel')])
    assert report['command_publishers']==['/tb1/velocity_smoother'],'Competing command publisher'
    Path(args.output).write_text(json.dumps(report,indent=2));print('READY: verified goal corridor; waiting for RViz goal' if not args.goal else 'READY: sending equivalent goal',flush=True)
    if args.goal:
        g=NavigateToPose.Goal();g.pose.header.frame_id='map';g.pose.header.stamp=n.get_clock().now().to_msg();g.pose.pose.position.x=float(goal[0]);g.pose.pose.position.y=float(goal[1]);g.pose.pose.orientation.z=math.sin(goal[2]/2);g.pose.pose.orientation.w=math.cos(goal[2]/2)
        f=client.send_goal_async(g)
        end=time.monotonic()+10
        while not f.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        goal_handle=f.result();assert goal_handle.accepted;goal_id=bytes(goal_handle.goal_id.uuid).hex();result_future=goal_handle.get_result_async()
    end=time.monotonic()+args.timeout;last_sample=0;last_entities=0;done_time=None
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        receive_entities()
        if time.monotonic()-last_entities>1 and not entity_pending:
            request_entities();last_entities=time.monotonic()
        if goal_id and result_future is None:
            req=NavigateToPose.Impl.GetResultService.Request();req.goal_id.uuid=list(bytes.fromhex(goal_id));result_future=get_result.call_async(req)
        if time.monotonic()-last_sample>.25:
            last_sample=time.monotonic();entry={'wall_time':last_sample,'simulation_time':clocks[-1][1] if clocks else None,'gazebo':poses['tb1'].copy()}
            try:t=buf.lookup_transform('map','base_footprint',Time());entry['map']=[t.transform.translation.x,t.transform.translation.y,yaw(t.transform.rotation)]
            except Exception as e:entry['tf_error']=str(e)
            trajectory.append(entry)
            report.update(goal_id=goal_id,trajectory=trajectory,feedback=feedback,statuses=statuses,plans=plans,commands=commands,auxiliary_current={r:poses[r] for r in auxiliaries})
            Path(args.output).write_text(json.dumps(report,indent=2))
        if result_future and result_future.done():
            if done_time is None:done_time=time.monotonic()
            if time.monotonic()-done_time>3:break
    result=result_future.result() if result_future and result_future.done() else None
    target=plans[-1]['goal'] if plans else goal;final=poses['tb1'].copy();map_final=next((e['map'] for e in reversed(trajectory) if 'map' in e),None)
    position_error=math.dist(final[:2],target[:2]);angle_error=abs(math.atan2(math.sin(final[2]-target[2]),math.cos(final[2]-target[2])));map_error=math.dist(map_final[:2],target[:2]) if map_final else None
    max_peer_drift=max(math.dist(poses[r][:2],auxiliaries[r][:2]) for r in auxiliaries)
    report.update(actual_goal=target,final_gazebo=final,final_map=map_final,physical_displacement=math.dist(start[:2],final[:2]),position_error=position_error,yaw_error=angle_error,map_position_error=map_error,max_auxiliary_drift=max_peer_drift,action_status=result.status if result else None,action_result=message_to_ordereddict(result.result) if result else None,stopped=bool(commands and max(abs(v) for v in commands[-1])<.001),clock_elapsed_sim=clocks[-1][1]-clocks[0][1],clock_elapsed_wall=clocks[-1][0]-clocks[0][0])
    report['passed']=result is not None and result.status==4 and report['physical_displacement']>2.0 and position_error<=args.position_tolerance and angle_error<=.4 and map_error is not None and map_error<=args.position_tolerance and max_peer_drift<.005 and report['stopped']
    if not report['passed'] and goal_id:
        cancel=n.create_client(NavigateToPose.Impl.CancelGoalService,'/tb1/navigate_to_pose/_action/cancel_goal');req=NavigateToPose.Impl.CancelGoalService.Request();req.goal_info.goal_id.uuid=list(bytes.fromhex(goal_id));cancel.call_async(req);spin(2)
    Path(args.output).write_text(json.dumps(report,indent=2));print(json.dumps({k:v for k,v in report.items() if k in ['passed','start_gazebo','actual_goal','final_gazebo','final_map','physical_displacement','position_error','yaw_error','max_auxiliary_drift','action_status','stopped']},indent=2),flush=True)
    n.destroy_node();rclpy.shutdown();raise SystemExit(0 if report['passed'] else 1)


if __name__=='__main__':main()
