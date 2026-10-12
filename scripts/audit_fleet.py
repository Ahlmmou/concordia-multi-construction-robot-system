#!/usr/bin/env python3
"""Read-only live audit. Saves evidence without commanding any robot."""
import argparse
import hashlib
import json
import math
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
from rclpy.time import Time


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--seconds',type=float,default=40);parser.add_argument('--output',default='/tmp/fleet_audit.json');parser.add_argument('--map-yaml');args=parser.parse_args()
    rclpy.init();n=Node('fleet_read_only_audit');n.set_parameters([rclpy.parameter.Parameter('use_sim_time',value=True)])
    robots=['tb1','tb2','tb3','tb4','tb5'];report={'robots':{},'maps':{},'services':{},'clock':[]};buffers={r:Buffer() for r in robots};subs=[];clients=[];futures=[];map_arrays={};first_clock=None
    static_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def clock(msg):
        if len(report['clock'])<2:report['clock'].append(msg.clock.sec+msg.clock.nanosec*1e-9)
        else:report['clock'][-1]=msg.clock.sec+msg.clock.nanosec*1e-9
    subs.append(n.create_subscription(Clock,'/clock',clock,qos_profile_sensor_data))
    def grid(msg,topic):
        if topic in map_arrays:return
        a=np.frombuffer(msg.data,dtype=np.int8).copy().reshape(msg.info.height,msg.info.width);map_arrays[topic]=(msg,a)
        report['maps'][topic]={'frame':msg.header.frame_id,'width':msg.info.width,'height':msg.info.height,'resolution':msg.info.resolution,'origin':[msg.info.origin.position.x,msg.info.origin.position.y],'occupied_cells':int(np.count_nonzero(a>=99)),'inflated_cells':int(np.count_nonzero((a>0)&(a<99))),'unknown_cells':int(np.count_nonzero(a<0)),'sha256':hashlib.sha256(a.tobytes()).hexdigest()}
    subs.append(n.create_subscription(OccupancyGrid,'/map',lambda m:grid(m,'/map'),transient))
    def tf(msg,robot,static):
        for t in msg.transforms:
            try:
                (buffers[robot].set_transform_static if static else buffers[robot].set_transform)(t,'audit')
            except Exception:pass
        edges=report['robots'][robot].setdefault('tf_edges',[])
        for t in msg.transforms:
            e=[t.header.frame_id,t.child_frame_id]
            if e not in edges:edges.append(e)
    def scan(msg,robot):report['robots'][robot]['scan']={'frame':msg.header.frame_id,'rays':len(msg.ranges),'finite_returns':sum(math.isfinite(v) for v in msg.ranges),'stamp':msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9}
    def odom(msg,robot):report['robots'][robot]['odom']={'frame':msg.header.frame_id,'child':msg.child_frame_id,'x':msg.pose.pose.position.x,'y':msg.pose.pose.position.y}
    for r in robots:
        report['robots'][r]={}
        for suffix,typ,callback,qos in [('tf',TFMessage,lambda m,r=r:tf(m,r,False),qos_profile_sensor_data),('tf_static',TFMessage,lambda m,r=r:tf(m,r,True),static_qos),('scan',LaserScan,lambda m,r=r:scan(m,r),qos_profile_sensor_data),('odom',Odometry,lambda m,r=r:odom(m,r),qos_profile_sensor_data)]:subs.append(n.create_subscription(typ,f'/{r}/{suffix}',callback,qos))
        for scope in ['global_costmap','local_costmap']:
            t=f'/{r}/{scope}/costmap';subs.append(n.create_subscription(OccupancyGrid,t,lambda m,t=t:grid(m,t),transient))
        clients.append(ActionClient(n,NavigateToPose,f'/{r}/navigate_to_pose'))
    queried=set();end=time.monotonic()+args.seconds
    while time.monotonic()<end:
        rclpy.spin_once(n,timeout_sec=.05)
        available=dict(n.get_service_names_and_types())
        nodes=['/map_server']+[f'/{r}/{x}' for r in robots for x in ['amcl','controller_server','planner_server','bt_navigator','velocity_smoother','global_costmap/global_costmap','local_costmap/local_costmap']]
        for name in nodes:
            service=name+'/get_state'
            if service in available and service not in queried:
                c=n.create_client(GetState,service);clients.append(c);futures.append((service,c.call_async(GetState.Request()),'state'));queried.add(service)
            service=name+'/get_parameters'
            if service in available and service not in queried:
                names=['use_sim_time']
                if name=='/map_server':names+=['yaml_filename','frame_id']
                elif name.endswith('/amcl'):names+=['global_frame_id','odom_frame_id','base_frame_id','scan_topic','map_topic','set_initial_pose']
                elif 'costmap/' in name:names+=['global_frame','robot_base_frame','footprint','plugins','static_layer.map_topic','obstacle_layer.scan.topic'] if 'global_costmap' in name else ['global_frame','robot_base_frame','footprint','plugins','voxel_layer.scan.topic']
                elif name.endswith('/controller_server'):names+=['FollowPath.max_vel_x','FollowPath.max_speed_xy','goal_checker_plugins','goal_checker.xy_goal_tolerance','FollowPath.xy_goal_tolerance']
                elif name.endswith('/velocity_smoother'):names+=['max_velocity','odom_topic']
                elif name.endswith('/bt_navigator'):names+=['global_frame','robot_base_frame','odom_topic']
                c=n.create_client(GetParameters,service);clients.append(c);req=GetParameters.Request();req.names=names;futures.append((service,c.call_async(req),names));queried.add(service)
    for service,future,kind in futures:
        if not future.done():report['services'][service]={'error':'no response within audit window'};continue
        try:
            v=future.result()
            if kind=='state':report['services'][service]={'id':v.current_state.id,'label':v.current_state.label}
            else:
                def value(p):return {1:p.bool_value,2:p.integer_value,3:p.double_value,4:p.string_value,8:list(p.double_array_value),9:list(p.string_array_value)}.get(p.type)
                report['services'][service]={k:value(p) for k,p in zip(kind,v.values)}
        except Exception as e:report['services'][service]={'error':str(e)}
    report['nodes']=[ns.rstrip('/')+'/'+name for name,ns in n.get_node_names_and_namespaces()]
    report['endpoints']={}
    for t in ['/map','/clock','/scan']+[f'/{r}/{s}' for r in robots for s in ['cmd_vel','cmd_vel_nav','cmd_vel_uncoordinated','goal_pose','scan','robot_description']]:
        def endpoint(e):return {'node':e.node_namespace.rstrip('/')+'/'+e.node_name,'reliability':str(e.qos_profile.reliability),'durability':str(e.qos_profile.durability)}
        report['endpoints'][t]={'publishers':[endpoint(e) for e in n.get_publishers_info_by_topic(t)],'subscribers':[endpoint(e) for e in n.get_subscriptions_info_by_topic(t)]}
    for r in robots:
        try:
            child=report['robots'][r]['odom']['child'];t=buffers[r].lookup_transform('map',child,Time());q=t.transform.rotation
            report['robots'][r]['map_pose']={'x':t.transform.translation.x,'y':t.transform.translation.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)),'base_frame':child}
        except Exception as e:report['robots'][r]['pose_error']=str(e)
    report['actions']=[{'robot':r,'ready':next(c for c in clients if isinstance(c,ActionClient) and c._action_name==f'/{r}/navigate_to_pose').server_is_ready()} for r in robots]
    if args.map_yaml and '/map' in map_arrays:
        import yaml
        from PIL import Image
        spec=yaml.safe_load(Path(args.map_yaml).read_text());image=Image.open(Path(args.map_yaml).parent/spec['image']).convert('L');a=np.asarray(image)[::-1];prob=a/255 if spec['negate'] else (255-a)/255
        expected=np.where(prob>spec['occupied_thresh'],100,np.where(prob<spec['free_thresh'],0,-1)).astype(np.int8)
        actual=map_arrays['/map'][1];report['map_file_comparison']={'file':args.map_yaml,'same_shape':expected.shape==actual.shape,'exact_match':expected.shape==actual.shape and bool(np.array_equal(expected,actual))}
        if expected.shape==actual.shape:
            for r in robots:
                t=f'/{r}/global_costmap/costmap'
                if t in map_arrays and map_arrays[t][1].shape==actual.shape:
                    cost=map_arrays[t][1];wall=expected==100;report['maps'][t]['map_wall_cells_present']=int(np.count_nonzero((cost>=99)&wall));report['maps'][t]['map_wall_cells_total']=int(np.count_nonzero(wall))
    Path(args.output).write_text(json.dumps(report,indent=2));print('Audit saved:',args.output)
    print(json.dumps({'clock':report['clock'],'poses':{r:report['robots'][r].get('map_pose',report['robots'][r].get('pose_error')) for r in robots},'map_comparison':report.get('map_file_comparison'),'maps':report['maps'],'lifecycle':{s:v for s,v in report['services'].items() if s.endswith('get_state')}},indent=2))
    n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
