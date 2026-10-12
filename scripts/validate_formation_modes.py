#!/usr/bin/env python3
"""Bounded live formation request with evidence and fail-safe supervisor stop.

Requires matching ROS domain/middleware. Reports completion only from actual
positions plus supervisor cancellation/result verification. Never sends motor
commands or independent navigation goals.
"""
import argparse
import json
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, Twist
from formation_geometry import MODES, distance, Grid, wrap
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--mode',choices=MODES)
    parser.add_argument('--goal',type=float,nargs=3,metavar=('X','Y','YAW'))
    parser.add_argument('--config',default=str(Path(__file__).resolve().parents[1]/'config/fleet_mission.yaml'))
    parser.add_argument('--expect-blocked',action='store_true',help='Pass only when a requested layout is explicitly rejected and held')
    parser.add_argument('--timeout',type=float,default=300.)
    parser.add_argument('--output',required=True)
    args=parser.parse_args()
    if bool(args.mode)==bool(args.goal):parser.error('Specify exactly one of --mode or --goal')
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Node('formation_mode_validation');latest={};events=[];samples=[]
    sub=node.create_subscription(String,'/formation/state',lambda msg:latest.update(json.loads(msg.data)),10)
    mode_pub=node.create_publisher(String,'/formation/mode_request',10)
    goal_pub=node.create_publisher(PoseStamped,'/formation/primary_goal',10)
    result={'passed':False,'request':args.mode or args.goal,'events':events,'samples':samples}
    motor_commands={};motor_subs=[node.create_subscription(Twist,f'/{r}/cmd_vel',lambda m,r=r:motor_commands.update({r:(m.linear.x,m.angular.z)}),10) for r in ('tb1','tb2','tb3','tb4','tb5')]
    grids={};grid_subs=[]
    if args.goal:
        qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        grid_subs=[node.create_subscription(OccupancyGrid,t,lambda m,t=t:grids.update({t:Grid(m)}),qos) for t in ('/map','/tb1/global_costmap/costmap')]
    started=time.monotonic();last=None;initial={};minimum=None
    try:
        deadline=started+60
        while time.monotonic()<deadline and not latest:rclpy.spin_once(node,timeout_sec=.1)
        if not latest:result['error']='No observed supervisor status';return
        if args.goal and latest.get('state')=='PREFLIGHT':
            deadline=time.monotonic()+60
            while time.monotonic()<deadline and latest.get('state')=='PREFLIGHT':
                rclpy.spin_once(node,timeout_sec=.1)
        initial=dict(latest.get('ground_truth_poses',{}));initial_mission=latest.get('mission',0);result['start_simulation_time']=latest.get('simulation_time')
        if args.goal and latest.get('state')=='STOPPED' and latest.get('phase')=='IDLE':
            resume=node.create_client(Trigger,'/formation/resume')
            if resume.wait_for_service(timeout_sec=3.):
                future=resume.call_async(Trigger.Request());deadline=time.monotonic()+40
                while time.monotonic()<deadline and (not future.done() or latest.get('state')!='READY'):
                    rclpy.spin_once(node,timeout_sec=.1)
        if args.goal and latest.get('state')=='STOPPED' and latest.get('phase')!='IDLE':
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and any(v in ('active','sending','planning','cancelling') for v in latest.get('actions',{}).values()):
                rclpy.spin_once(node,timeout_sec=.1)
        if args.goal and latest.get('state') not in ('READY','STOPPED'):result['error']='Travel requires READY or safely stopped replacement mission';return
        if args.goal:
            import ast,yaml
            params=yaml.safe_load(Path(args.config).read_text())['fleet_mission_controller']['ros__parameters']
            deadline=time.monotonic()+30
            while time.monotonic()<deadline and len(grids)<2:rclpy.spin_once(node,timeout_sec=.1)
            result['goal_map_checked']=len(grids)==2 and all(g.frame=='map' and g.footprint_free(args.goal,ast.literal_eval(params['hilti_footprint']),params['footprint_padding'],params['maximum_target_cost']) for g in grids.values())
            if not result['goal_map_checked']:result['error']='Goal footprint not verified free on the actual map and primary global costmap';return
        if args.mode:
            msg=String();msg.data=args.mode;mode_pub.publish(msg)
        else:
            import math
            msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=args.goal[0];msg.pose.position.y=args.goal[1]
            msg.pose.orientation.z=math.sin(args.goal[2]/2);msg.pose.orientation.w=math.cos(args.goal[2]/2);goal_pub.publish(msg)
        started=time.monotonic();observed=False;last_sample=0.
        while time.monotonic()-started<args.timeout:
            rclpy.spin_once(node,timeout_sec=.1)
            state=latest.get('state');transition=latest.get('transition_state')
            if args.mode:observed |= latest.get('requested_mode')==args.mode and latest.get('mission',0)>initial_mission
            else:observed |= state in ('FORMING_LINE','TRAVELLING','FORMING_BOUNDARY','BOUNDARY_TRAVELLING')
            key=(state,transition,latest.get('reason'))
            elapsed=time.monotonic()-started
            if key!=last:
                event={'elapsed':elapsed,'state':state,'transition':transition,'reason':key[2]}
                events.append(event);print(event,flush=True);last=key
            if observed and state in ('WAITING','STOPPED'):
                if args.expect_blocked and state=='WAITING':
                    result['passed']=True;result['blocked_as_expected']=latest.get('reason')
                else:result['error']=latest.get('reason')
                break
            clearance=latest.get('minimum_footprint_clearance')
            if clearance is not None:minimum=clearance if minimum is None else min(minimum,clearance)
            if elapsed-last_sample>=1.:
                samples.append({'elapsed':elapsed,'simulation_time':latest.get('simulation_time'),'poses':latest.get('ground_truth_poses'),
                    'formation_error':latest.get('formation_error'),'assembly_stage':latest.get('assembly_stage'),'assembly_robot':latest.get('assembly_robot'),'targets':latest.get('targets'),'minimum_clearance':clearance,
                    'blocked':latest.get('blocked'),'actions':latest.get('actions')});last_sample=elapsed
            if observed and state=='READY' and (not args.mode or (latest.get('active_mode')==args.mode and transition=='COMPLETE')):
                result['passed']=not args.expect_blocked
                if args.goal:
                    pose=latest.get('poses',{}).get('tb1')
                    result['passed']=bool(pose is not None and distance(pose,args.goal)<=params['arrival_tolerance'] and abs(wrap(pose[2]-args.goal[2]))<=params['yaw_tolerance'])
                    if not result['passed']:result['error']='Supervisor READY but actual requested goal pose not reached'
                if args.expect_blocked:result['error']='Formation completed when rejection was expected'
                break
        if not result['passed'] and 'error' not in result:result['error']='Timeout before observed actual formation/goal completion'
    except KeyboardInterrupt:result['error']='Interrupted'
    finally:
        result['elapsed_wall']=time.monotonic()-started;result['minimum_footprint_clearance']=minimum
        result['last_status']=dict(latest)
        if args.goal and 'tb1' in latest.get('poses',{}):
            import math
            result['goal_position_error']=distance(latest['poses']['tb1'],args.goal)
            result['goal_yaw_error']=abs(math.atan2(math.sin(latest['poses']['tb1'][2]-args.goal[2]),math.cos(latest['poses']['tb1'][2]-args.goal[2])))
        if result.get('start_simulation_time') is not None and latest.get('simulation_time') is not None:
            result['elapsed_simulation']=latest['simulation_time']-result['start_simulation_time']
        result['physical_displacements']={r:distance(p,latest['ground_truth_poses'][r]) for r,p in initial.items() if r in latest.get('ground_truth_poses',{})}
        stop=node.create_client(Trigger,'/formation/stop')
        if stop.wait_for_service(timeout_sec=3.):
            future=stop.call_async(Trigger.Request());deadline=time.monotonic()+5
            while not future.done() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
            result['stop_acknowledged']=future.done() and future.result().success
            motor_commands.clear();deadline=time.monotonic()+3
            while time.monotonic()<deadline and len(motor_commands)<5:rclpy.spin_once(node,timeout_sec=.1)
            result['final_motor_commands']=dict(motor_commands)
            result['safe_stop_observed']=len(motor_commands)==5 and all(abs(v)<1e-6 and abs(w)<1e-6 for v,w in motor_commands.values())
            if not result['safe_stop_observed']:
                result['passed']=False;result['error']='Zero commands not observed for all five robots after stop'
        Path(args.output).parent.mkdir(parents=True,exist_ok=True)
        Path(args.output).write_text(json.dumps(result,indent=2));print('Evidence:',args.output,'passed:',result['passed'],flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
