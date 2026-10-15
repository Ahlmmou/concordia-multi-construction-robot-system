#!/usr/bin/env python3
"""Five-robot Nav2 mission supervisor and conservative command gate.

Each robot retains its own Nav2 planners/controllers. Only this node publishes
motor cmd_vel; Nav2's smoothed commands arrive at cmd_vel_uncoordinated.
"""
import ast
import json
import math
import signal
import time
from pathlib import Path
from collections import deque
from functools import partial

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.clock import Clock as RclClock, ClockType
from rclpy.duration import Duration
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from formation_geometry import (ROBOTS, Grid, yaw, wrap, distance, rectangle,
                                candidates, trail_target, rotate, boundary_followers_holding,
                                flexible_assignment, boundary_order, yielding_gate,
                                footprint_clearance, advance, pair_separation,
                                rolling_turn_candidates, obstacle_detour_candidates, route_preview, motion_conflict, inflation_escape, local_obstacle_command, MODES, formation_slots, layout_clearance, trail_waypoint, horizontal_first_routes, stationary_route_clear, assembly_reroute_candidates, follower_lag_gate)


class FleetMissionController(Node):
    def __init__(self):
        super().__init__('fleet_mission_controller')
        defaults = dict(rectangle_half_length=4.2, rectangle_half_width=3.4,
            compact_half_length=2.8, compact_half_width=2.2,
            wall_x=0.0, wall_y=0.0, wall_normal_yaw=0.0, wall_probe_distance=1.0, maximum_wall_distance=6.0,
            wall_workspace_depth=3.0, wall_row_spacing=1.8, wall_half_width=2.2,
            transition_timeout=600.0, trail_lookahead=2.5,
            single_file_assembly_axis_yaw=0.0, single_file_clearance_step=1.0,
            transit_spacing=3.0, minimum_separation=0.4, primary_minimum_separation=0.6, footprint_padding=0.12,
            leader_speed=0.60, auxiliary_speed=0.80, angular_speed=0.50,
            leader_rolling_turn=True, rolling_turn_speed=0.22, rolling_turn_rate=0.15,
            update_distance=0.8, update_angle=0.5, minimum_goal_interval=4.0,
            arrival_tolerance=0.15, yaw_tolerance=0.06, primary_yaw_tolerance=0.05, alternative_radius=1.5,
            candidate_step=0.3, maximum_target_cost=70, data_timeout=15.0,
            command_timeout=2.0, clock_timeout=5.0, lifecycle_timeout=30.0,
            maximum_retries=3, retry_delay=8.0, maximum_follower_lag=5.0,
            action_response_timeout=30.0, progress_timeout=90.0,
            collision_horizon=2.0, yield_recovery_delay=3.0, yield_clearance_distance=1.5,
            obstacle_recovery_delay=3.0, obstacle_escape_speed=0.12,
            obstacle_release_padding=0.20,
            manual_navigation=False, auto_form_rectangle=False,
            maintain_boundary=True, flexible_boundary=True, boundary_step=1.0, boundary_tolerance=0.8,
            verify_gazebo_localization=True, maximum_localization_error=0.35,
            hilti_footprint='[[-1.20,-0.85],[-1.20,0.85],[1.40,0.85],[1.40,-0.85]]',
            auxiliary_footprint='[[-0.30,-0.30],[-0.30,0.30],[0.475,0.30],[0.475,-0.30]]')
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.p={name:self.get_parameter(name).value for name in defaults}
        self.polygons={r:ast.literal_eval(self.p['hilti_footprint'] if r=='tb1' else self.p['auxiliary_footprint']) for r in ROBOTS}
        for name in ('rectangle_half_length','rectangle_half_width','compact_half_length','compact_half_width','wall_workspace_depth','wall_row_spacing','wall_half_width','single_file_clearance_step','wall_probe_distance','maximum_wall_distance','transition_timeout','trail_lookahead'):
            if not math.isfinite(self.p[name]) or self.p[name]<=0.:raise ValueError(name+' must be finite and positive')
        if not all(math.isfinite(self.p[k]) for k in ('wall_x','wall_y','wall_normal_yaw')):raise ValueError('Wall coordinates must be finite')
        self.radii={r:max(math.hypot(x,y) for x,y in poly) for r,poly in self.polygons.items()}
        if not math.isfinite(self.p['primary_yaw_tolerance']) or not 0.<self.p['primary_yaw_tolerance']<=self.p['yaw_tolerance']:
            raise ValueError('Primary yaw tolerance must be positive and no larger than auxiliary yaw tolerance')
        if self.p['trail_lookahead']<=self.p['arrival_tolerance']:raise ValueError('trail_lookahead must exceed arrival_tolerance')
        if self.p['primary_minimum_separation']<self.p['minimum_separation']:raise ValueError('Primary clearance must be at least minimum_separation')
        if not (0.<self.p['rolling_turn_speed']<=self.p['leader_speed'] and
                0.<self.p['rolling_turn_rate']<=self.p['angular_speed']):raise ValueError('Rolling turn speeds must be positive and within motor limits')
        if self.p['obstacle_recovery_delay']<=0. or not 0.<self.p['obstacle_escape_speed']<=min(self.p['leader_speed'],self.p['auxiliary_speed']):raise ValueError('Obstacle escape intervals and speed must be positive and within motor limits')
        if self.p['transit_spacing']<self.radii['tb1']+self.radii['tb2']+self.p['primary_minimum_separation']:
            raise ValueError('transit_spacing is too small for the Hilti and auxiliary footprints')
        if min(self.p['boundary_step'],self.p['boundary_tolerance'],self.p['candidate_step'],self.p['minimum_goal_interval'],self.p['data_timeout'],self.p['yield_recovery_delay'],self.p['yield_clearance_distance'])<=0:raise ValueError('Intervals and candidate step must be positive')
        self.requested_mode='LEGACY';self.active_mode='LEGACY'
        self.legacy_parameters={k:self.p[k] for k in ('rectangle_half_length','rectangle_half_width','flexible_boundary','maintain_boundary')}
        self.transition_state='IDLE';self.transition_started=None;self.transition_duration=None
        self.mode_targets=None;self.mode_anchor=None;self.transition_robot=None;self.assembly_route=None;self.assembly_stage=None;self.assembly_routes={};self.assembly_blocked_since={}
        self.trail_cursors={r:0. for r in ROBOTS[1:]};self.follower_waypoints={};self.trail_blocked_since={};self.trail_recovering={};self.trail_detours={}
        self.open_dimensions=(self.p['rectangle_half_length'],self.p['rectangle_half_width'])
        self.state='PREFLIGHT';self.reason='Waiting for verified live data';self.phase='IDLE'
        self.static_transforms={r:{} for r in ROBOTS}
        self.poses={};self.pose_stamp={};self.odom_frames={};self.buffers={r:Buffer(cache_time=Duration(seconds=30.0),node=self) for r in ROBOTS}
        self.grids={};self.grid_times={};self.scan_times={};self.commands={};self.clock_value=None;self.clock_time=0.
        self.lifecycle={};self.lifecycle_requests={};self.graph_problem='Graph not checked yet'
        self.boundary_waypoint=None;self.boundary_heading=None
        self.flexible_slots={};self.flexible_final_targets=None
        self.primary_route=[];self.primary_route_stamp=0.
        self.primary=None;self.pending_primary=None;self.targets={};self.rectangle_targets=None;self.trail=deque(maxlen=5000);self.trail_seeded=False
        self.boundary_holds=set();self.blocked=set();self.last_tick=0.;self.manual=self.p['manual_navigation'];self.mission_number=0
        self.yielding={};self.unresolved_pairs=set();self.conflict_since={};self.clearance_maneuver=None
        self.local_blocked=set();self.obstacle_since={};self.obstacle_recoveries={}
        self.physical_poses={};self.physical_stamp=0.
        self.slots={r:dict(handle=None,pending=False,cancelling=False,target=None,last_send=0.,retries=0,cooldown=0.,status='idle',progress_time=time.monotonic(),progress_pose=None,plan_token=0) for r in ROBOTS}
        self.nav={r:ActionClient(self,NavigateToPose,f'/{r}/navigate_to_pose') for r in ROBOTS}
        self.planners={r:ActionClient(self,ComputePathToPose,f'/{r}/compute_path_to_pose') for r in ROBOTS}
        self.cmd_pubs={r:self.create_publisher(Twist,f'/{r}/cmd_vel',10) for r in ROBOTS}
        self.markers=self.create_publisher(MarkerArray,'/formation/markers',10)
        self.status_pub=self.create_publisher(String,'/formation/state',10)
        self.subscriptions_kept=[];self.services_kept=[]
        transient=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        static_qos=QoSProfile(depth=100,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.subscriptions_kept.append(self.create_subscription(OccupancyGrid,'/map',partial(self.grid,topic='/map'),transient))
        self.subscriptions_kept.append(self.create_subscription(Clock,'/clock',self.clock_callback,qos_profile_sensor_data))
        if self.p['verify_gazebo_localization']:
            self.subscriptions_kept.append(self.create_subscription(ModelStates,'/gazebo/model_states',self.model_states,10))
        self.subscriptions_kept.append(self.create_subscription(NavPath,'/tb1/plan',self.primary_plan,qos_profile_sensor_data))
        self.subscriptions_kept.append(self.create_subscription(PoseStamped,'/formation/primary_goal',self.primary_goal,10))
        self.subscriptions_kept.append(self.create_subscription(String,'/formation/mode_request',self.mode_request,10))
        for r in ROBOTS:
            for suffix,static,qos in [('tf',False,qos_profile_sensor_data),('tf_static',True,static_qos)]:
                self.subscriptions_kept.append(self.create_subscription(TFMessage,f'/{r}/{suffix}',partial(self.tf,robot=r,static=static),qos))
            self.subscriptions_kept.append(self.create_subscription(Odometry,f'/{r}/odom',partial(self.odom,robot=r),qos_profile_sensor_data))
            self.subscriptions_kept.append(self.create_subscription(LaserScan,f'/{r}/scan',partial(self.scan,robot=r),qos_profile_sensor_data))
            self.subscriptions_kept.append(self.create_subscription(Twist,f'/{r}/cmd_vel_uncoordinated',partial(self.command,robot=r),qos_profile_sensor_data))
            for scope in ('global_costmap','local_costmap'):
                topic=f'/{r}/{scope}/costmap'
                self.subscriptions_kept.append(self.create_subscription(OccupancyGrid,topic,partial(self.grid,topic=topic),transient))
        for name,callback in [('stop',self.stop_service),('resume',self.resume_service),('form_rectangle',self.rectangle_service)]:
            self.services_kept.append(self.create_service(Trigger,f'/formation/{name}',callback))
        self.steady=RclClock(clock_type=ClockType.STEADY_TIME)
        self.create_timer(.05,self.gate,clock=self.steady)
        self.create_timer(.5,self.tick,clock=self.steady)
        self.create_timer(5.,self.check_graph,clock=self.steady)
        self.get_logger().info('Five-robot supervisor: /formation/primary_goal; stop with /formation/stop')

    def clock_callback(self,msg):
        value=msg.clock.sec+msg.clock.nanosec*1e-9
        if value!=self.clock_value:self.clock_time=time.monotonic()
        if self.clock_value is not None and value<self.clock_value:
            self.pending_primary=None;self.primary=None;self.cancel_all();self.set_state('WAITING','Simulation clock reset: restart controller')
        self.clock_value=value

    def tf(self,msg,robot,static):
        for transform in msg.transforms:
            if static:self.static_transforms[robot][transform.child_frame_id]=transform
            try:(self.buffers[robot].set_transform_static if static else self.buffers[robot].set_transform)(transform,'simulation')
            except Exception as exc:self.get_logger().warning(str(exc))

    def odom(self,msg,robot):self.odom_frames[robot]=msg.child_frame_id.lstrip('/')
    def model_states(self,msg):
        self.physical_poses={name:(pose.position.x,pose.position.y,yaw(pose.orientation)) for name,pose in zip(msg.name,msg.pose) if name in ROBOTS}
        self.physical_stamp=time.monotonic()
    def scan(self,msg,robot):self.scan_times[robot]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
    def command(self,msg,robot):self.commands[robot]=(msg,time.monotonic())
    def grid(self,msg,topic):
        self.grids[topic]=Grid(msg);self.grid_times[topic]=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9

    def check_graph(self):
        problems=[]
        for r in ROBOTS:
            motor=self.get_publishers_info_by_topic(f'/{r}/cmd_vel')
            if len(motor)!=1 or any(e.node_name!=self.get_name() for e in motor):problems.append(f'{r}: conflicting cmd_vel publishers')
            if len(self.get_publishers_info_by_topic(f'/{r}/cmd_vel_uncoordinated'))!=1:problems.append(f'{r}: expected one gated Nav2 velocity_smoother')
            if len(self.get_publishers_info_by_topic(f'/{r}/robot_description'))!=1:problems.append(f'{r}: duplicate/missing robot_state_publisher')
            if self.get_publishers_info_by_topic(f'/{r}/goal_pose'):problems.append(f'{r}: goal_pose publisher conflicts with action ownership')
        if len(self.get_publishers_info_by_topic('/clock'))!=1:problems.append('Expected one simulation clock publisher')
        if len(self.get_publishers_info_by_topic('/map'))!=1:problems.append('Expected one map publisher')
        names=[name for name,_ in self.get_node_names_and_namespaces()]
        if names.count(self.get_name())!=1:problems.append('Duplicate mission controller')
        if any(name.startswith(('goal_listener_and_sender','adaptive_workspace_allocator','teleop','turtlebot3_drive')) for name in names):problems.append('Legacy formation or teleoperation node is running')
        self.graph_problem='; '.join(problems)
        for r in ROBOTS:
            for node in ('amcl','planner_server','controller_server','bt_navigator','velocity_smoother','global_costmap/global_costmap','local_costmap/local_costmap'):
                service=f'/{r}/{node}/get_state'
                if service in self.lifecycle_requests:
                    c,f,started=self.lifecycle_requests[service]
                    if f.done():
                        try:self.lifecycle[service]=(f.result().current_state.id,time.monotonic())
                        except Exception:pass
                        del self.lifecycle_requests[service]
                    elif time.monotonic()-started>self.p['lifecycle_timeout']:del self.lifecycle_requests[service]
                    else:continue
                client=getattr(self,'state_clients',{}).get(service)
                if client is None:
                    if not hasattr(self,'state_clients'):self.state_clients={}
                    client=self.create_client(GetState,service);self.state_clients[service]=client
                if client.service_is_ready():self.lifecycle_requests[service]=(client,client.call_async(GetState.Request()),time.monotonic())

    def refresh_poses(self):
        for r in ROBOTS:
            if r not in self.odom_frames:continue
            # tf2 clears its cache when simulation time first activates. Static
            # publishers send only once; retain and restore those fixed edges.
            for transform in self.static_transforms[r].values():
                self.buffers[r].set_transform_static(transform,'simulation')
            try:
                tf=self.buffers[r].lookup_transform('map',self.odom_frames[r],Time())
                stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9
                self.poses[r]=(tf.transform.translation.x,tf.transform.translation.y,yaw(tf.transform.rotation))
                self.pose_stamp[r]=stamp
            except Exception:pass

    def ready(self):
        now=time.monotonic()
        if self.graph_problem:return False,self.graph_problem
        if now-self.clock_time>self.p['clock_timeout']:return False,'Simulation clock stopped or missing'
        if '/map' not in self.grids:return False,'Waiting for /map'
        for r in ROBOTS:
            if r not in self.poses or self.clock_value is None or abs(self.clock_value-self.pose_stamp.get(r,0))>3.:return False,f'{r}: map-to-base transform not fresh'
            if self.p['verify_gazebo_localization']:
                if r not in self.physical_poses or now-self.physical_stamp>self.p['clock_timeout']:return False,f'{r}: Gazebo reference pose missing or stale'
                error=distance(self.poses[r],self.physical_poses[r])
                if error>self.p['maximum_localization_error']:return False,f'{r}: localization differs from Gazebo by {error:.2f} m; resolve localization before moving'
            if r not in self.scan_times or abs(self.clock_value-self.scan_times[r])>self.p['data_timeout']:return False,f'{r}: lidar stale'
            if not self.nav[r].server_is_ready() or not self.planners[r].server_is_ready():return False,f'{r}: Nav2 actions missing'
            for scope in ('global_costmap','local_costmap'):
                t=f'/{r}/{scope}/costmap'
                if t not in self.grid_times or abs(self.clock_value-self.grid_times[t])>self.p['data_timeout']:return False,f'{r}: {scope} stale'
            for node in ('amcl','planner_server','controller_server','bt_navigator','velocity_smoother','global_costmap/global_costmap','local_costmap/local_costmap'):
                state,stamp=self.lifecycle.get(f'/{r}/{node}/get_state',(0,0))
                if state!=3 or now-stamp>self.p['lifecycle_timeout']:return False,f'{r}: {node} not verified active'
        return True,''

    def in_frame(self,robot,pose,frame):
        if frame=='map':return pose
        try:
            t=self.buffers[robot].lookup_transform(frame,'map',Time());dx,dy=rotate(pose[0],pose[1],yaw(t.transform.rotation))
            return (dx+t.transform.translation.x,dy+t.transform.translation.y,pose[2]+yaw(t.transform.rotation))
        except Exception:return None

    def feasible(self,robot,pose,reserved=None,peers=True,reserve_primary=True):
        for topic in ('/map',f'/{robot}/global_costmap/costmap'):
            grid=self.grids.get(topic)
            if grid is None:return False
            local=self.in_frame(robot,pose,grid.frame)
            if local is None or not grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],self.p['maximum_target_cost']):return False
        if robot!='tb1' and reserve_primary:
            corridor=FleetMissionController.primary_corridor(self)
            if FleetMissionController.primary_corridor_gap(self,robot,pose,corridor)<self.p['primary_minimum_separation']:return False
        if peers:
            for other,current in self.poses.items():
                gap=pair_separation(robot,other,self.p['minimum_separation'],self.p['primary_minimum_separation'])
                if other!=robot and footprint_clearance(pose,self.polygons[robot],current,self.polygons[other])<gap:return False
        for other,target in (reserved or {}).items():
            gap=pair_separation(robot,other,self.p['minimum_separation'],self.p['primary_minimum_separation'])
            if other!=robot and footprint_clearance(pose,self.polygons[robot],target,self.polygons[other])<gap:return False
        return True

    def choose(self,robot,preferred,reserved):
        for pose in candidates(preferred,self.p['alternative_radius'],self.p['candidate_step']):
            if self.feasible(robot,pose,reserved):return pose
        return None

    def navigation_reservations(self,robot):
        if self.p.get('flexible_boundary') and self.state in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING'):
            # Flexible corner goals move with the leader. They need not be
            # occupied simultaneously with the leader's final destination.
            if robot=='tb1':return {}
            return {**{r:t for r,t in self.targets.items() if r!='tb1'},'tb1':self.poses['tb1']}
        return self.targets

    def primary_plan(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':return
        self.primary_route=[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in msg.poses]
        self.primary_route_stamp=time.monotonic()

    def primary_corridor(self):
        goal=getattr(self,'primary',None);poses=getattr(self,'poses',{})
        if goal is None or 'tb1' not in poses or self.state not in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING','TRAVELLING'):return []
        start=poses['tb1']
        if distance(start,goal)<=self.p.get('arrival_tolerance',.15):return []
        lookahead=max(2.,self.p.get('leader_speed',.6)*self.p.get('collision_horizon',2.))
        path=getattr(self,'primary_route',[])
        if time.monotonic()-getattr(self,'primary_route_stamp',0.)>self.p.get('data_timeout',15.):path=[]
        if path and distance(path[-1],goal)>max(.5,self.p.get('update_distance',.8)):path=[]
        if not path or min(distance(start,p) for p in path)>1.:
            heading=math.atan2(goal[1]-start[1],goal[0]-start[0])
            length=min(lookahead,distance(start,goal))
            path=[start,(start[0]+length*math.cos(heading),start[1]+length*math.sin(heading),heading)]
        return route_preview(start,path,lookahead)

    def primary_corridor_gap(self,robot,pose,corridor):
        if not corridor:return float('inf')
        gap=self.p.get('primary_minimum_separation',self.p.get('minimum_separation',.6))
        radius=sum(max(math.hypot(x,y) for x,y in self.polygons[r]) for r in (robot,'tb1'))
        # Circle bounds skip only poses that are certainly beyond the required
        # gap; close passes still use the full oriented footprints. Saturating
        # at the required gap avoids spending gate time measuring distant space.
        nearby=(p for p in corridor if distance(pose,p)<=radius+gap)
        return min(gap,min((footprint_clearance(pose,self.polygons[robot],p,self.polygons['tb1']) for p in nearby),default=gap))

    def auxiliary_corridor_safe(self,robot,pose,command,corridor):
        if not corridor:return True
        initial=FleetMissionController.primary_corridor_gap(self,robot,pose,corridor)
        gap=self.p.get('primary_minimum_separation',self.p['minimum_separation'])
        swept=[FleetMissionController.primary_corridor_gap(self,robot,
            advance(pose,command,self.p['collision_horizon']*i/10),corridor) for i in range(1,11)]
        return min(swept)>=min(initial,gap)-1e-6 and (initial>=gap or swept[-1]>initial+1e-4)

    def primary_goal(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':self.get_logger().error('Primary goal must use the verified shared map frame');return
        q=msg.pose.orientation
        if not all(math.isfinite(v) for v in [msg.pose.position.x,msg.pose.position.y,q.x,q.y,q.z,q.w]) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>0.01:
            self.get_logger().error('Primary goal has invalid coordinates/quaternion');return
        if self.transition_state in ('CANCELLING','ASSEMBLING'):
            self.get_logger().warning('Complete or stop the formation transition before requesting travel');return
        if self.requested_mode=='WALL_WORKSPACE':
            self.get_logger().warning('WALL_WORKSPACE is stationary; request a transit formation before travel');return
        target=(msg.pose.position.x,msg.pose.position.y,yaw(q))
        if not self.feasible('tb1',target,peers=False):self.get_logger().error('Primary goal is unknown, occupied, out of bounds, or too tight for Hilti footprint');return
        self.transition_state='IDLE';self.transition_robot=None
        self.primary_route=[];self.primary_route_stamp=0.
        self.manual=False;self.pending_primary=target;self.rectangle_targets=None;self.targets={}
        if self.requested_mode!='SINGLE_FILE':self.trail.clear();self.trail_seeded=False
        self.mission_number+=1;self.cancel_all()
        for slot in self.slots.values():slot['retries']=0;slot['cooldown']=0.;slot['target']=None
        self.boundary_waypoint=None;self.boundary_heading=None
        self.flexible_slots={};self.flexible_final_targets=None
        self.yielding={};self.unresolved_pairs=set();self.conflict_since={};self.clearance_maneuver=None
        self.obstacle_recoveries={};self.obstacle_since={}
        self.phase=('TRAVELLING' if self.active_mode=='SINGLE_FILE' and self.trail_seeded else 'FORMING_LINE') if self.requested_mode=='SINGLE_FILE' else ('FORMING_BOUNDARY' if self.p['maintain_boundary'] else 'FORMING_LINE')
        self.set_state(self.phase,'New primary goal queued; waiting for previous actions to stop')

    def set_state(self,state,reason):
        if (state,reason)!=(self.state,self.reason):self.get_logger().info(f'{state}: {reason}')
        self.state,self.reason=state,reason

    def cancel(self,robot):
        slot=self.slots[robot]
        slot.pop('next_target',None)
        if slot['pending']:slot['cancel_when_accepted']=True
        if slot['handle'] is not None and not slot['cancelling']:
            slot['cancelling']=True
            future=slot['handle'].cancel_goal_async()
            future.add_done_callback(lambda f,r=robot:self.cancelled(f,r))

    def cancelled(self,future,robot):
        try:
            if not future.result().goals_canceling:self.get_logger().warning(f'{robot}: cancellation not acknowledged; waiting for action result')
        except Exception as exc:self.get_logger().error(f'{robot} cancel: {exc}')
        # Ownership stays with this goal until the result callback clears it.

    def cancel_all(self):
        for r in ROBOTS:self.cancel(r)

    def failure(self,robot,reason):
        slot=self.slots[robot];slot['pending']=False;slot['handle']=None;slot['cancelling']=False;slot['status']='failed';slot['retries']+=1;slot['cooldown']=self.get_clock().now().nanoseconds*1e-9+self.p['retry_delay']
        if robot in self.yielding or robot in self.local_blocked or robot in self.obstacle_recoveries:
            slot['retries']=0;slot['cooldown']=0.
            self.get_logger().info(f'{robot}: navigation paused for clearance recovery; retry automatically when clear')
            return
        self.get_logger().warning(f'{robot}: {reason}; retry {slot["retries"]}/{self.p["maximum_retries"]}')
        if slot['retries']>=self.p['maximum_retries'] and self.requested_mode=='SINGLE_FILE' and self.state=='TRAVELLING' and robot!='tb1':
            self.recover_auxiliary(robot,reason);slot['retries']=0;return
        if slot['retries']>=self.p['maximum_retries'] and self.flexible_recovery(robot,reason):return
        if slot['retries']>=self.p['maximum_retries']:
            self.cancel_all()
            self.set_state('WAITING',f'{robot}: retry limit reached; use /formation/resume or a new primary goal')

    def send(self,robot,target):
        slot=self.slots[robot];now=self.get_clock().now().nanoseconds*1e-9
        if self.arrived(robot,target):
            if slot['handle'] is not None:self.cancel(robot)
            return
        if slot['retries']>=self.p['maximum_retries']:return
        if slot['pending'] or slot['cancelling'] or now<slot['cooldown']:return
        if slot['target'] is not None:
            changed=distance(slot['target'],target)>=self.p['update_distance'] or abs(wrap(slot['target'][2]-target[2]))>=self.p['update_angle']
            if slot['handle'] is not None:
                if changed and now-slot['last_send']>=self.p['minimum_goal_interval']:
                    moving_follower=(robot!='tb1' and self.p.get('flexible_boundary') and
                        self.state=='BOUNDARY_TRAVELLING' and self.requested_mode=='LEGACY' and
                        robot not in self.obstacle_recoveries and not self.clearance_maneuver)
                    if moving_follower:
                        if now-slot.get('update_last_plan',slot['last_send'])<self.p['minimum_goal_interval']:return
                        if not self.feasible(robot,target,self.navigation_reservations(robot)):return
                        slot['update_last_plan']=now
                        self.start_plan(robot,target,owner=slot['handle'])
                    else:self.cancel(robot)
                return
            if slot['status']=='succeeded' and not changed:
                # A Nav2 result alone is insufficient: actual pose must meet tolerance.
                self.failure(robot,'Nav2 succeeded but actual pose is outside formation tolerance');return
        interval=0. if self.requested_mode=='SINGLE_FILE' and self.state=='TRAVELLING' and robot!='tb1' else self.p['minimum_goal_interval']
        if now-slot['last_send']<interval:return
        if not self.feasible(robot,target,self.navigation_reservations(robot)):return
        self.start_plan(robot,target)

    def start_plan(self,robot,target,owner=None):
        slot=self.slots[robot]
        slot['pending']=True;slot['pending_since']=time.monotonic();slot['plan_token']+=1;token=slot['plan_token'];slot['cancel_when_accepted']=False
        request=ComputePathToPose.Goal();request.goal=self.pose_message(target);request.use_start=False
        try:
            future=self.planners[robot].send_goal_async(request)
            future.add_done_callback(partial(self.plan_accepted,robot=robot,target=target,token=token,mission=self.mission_number,owner=owner))
        except Exception as exc:self.plan_error(robot,str(exc),owner)

    def plan_error(self,robot,reason,owner=None):
        if owner is None:self.failure(robot,reason);return
        self.slots[robot]['pending']=False
        self.get_logger().info(f'{robot}: moving-slot path update deferred: {reason}')

    def plan_accepted(self,future,robot,target,token,mission,owner=None):
        if token!=self.slots[robot]['plan_token']:return
        try:
            handle=future.result()
            if not handle.accepted:self.plan_error(robot,'Planner rejected target',owner);return
            future=handle.get_result_async();future.add_done_callback(partial(self.plan_done,robot=robot,target=target,token=token,mission=mission,owner=owner))
        except Exception as exc:self.plan_error(robot,str(exc),owner)

    def plan_done(self,future,robot,target,token,mission,owner=None):
        slot=self.slots[robot]
        if token!=slot['plan_token']:return
        if mission!=self.mission_number or slot.get('cancel_when_accepted') or self.state in ('STOPPED','WAITING'):
            slot['pending']=False;return
        try:
            result=future.result()
            if result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:self.plan_error(robot,'No feasible Nav2 path',owner);return
            if not self.feasible(robot,target,self.navigation_reservations(robot)):slot['pending']=False;return
            if owner is not None:
                slot['pending']=False
                desired=self.targets.get(robot)
                if (slot['handle'] is not owner or slot['cancelling'] or
                    robot in self.obstacle_recoveries or self.clearance_maneuver or
                    self.state!='BOUNDARY_TRAVELLING' or self.requested_mode!='LEGACY' or
                    desired is None or distance(desired,target)>=self.p['update_distance'] or
                    abs(wrap(desired[2]-target[2]))>=self.p['update_angle']):return
                self.cancel(robot)
                slot['next_target']=(target,mission)
                slot['updates_prepared']=slot.get('updates_prepared',0)+1
            else:self.dispatch_navigation(robot,target)
        except Exception as exc:self.plan_error(robot,str(exc),owner)

    def dispatch_navigation(self,robot,target):
        slot=self.slots[robot]
        goal=NavigateToPose.Goal();goal.pose=self.pose_message(target)
        if robot=='tb1' or self.requested_mode=='SINGLE_FILE':
            from ament_index_python.packages import get_package_share_directory
            tree='hilti_navigation.xml' if robot=='tb1' else 'single_file_navigation.xml'
            goal.behavior_tree=str(Path(get_package_share_directory('crs_multibot'))/'config'/tree)
        self.commands.pop(robot,None) # Require fresh output from the new action.
        slot['pending']=True;slot['pending_since']=time.monotonic();slot['target']=target;slot['last_send']=self.get_clock().now().nanoseconds*1e-9;slot['status']='sending'
        future=self.nav[robot].send_goal_async(goal)
        future.add_done_callback(partial(self.nav_accepted,robot=robot))

    def nav_accepted(self,future,robot):
        slot=self.slots[robot];slot['pending']=False
        try:
            handle=future.result()
            if not handle.accepted:self.failure(robot,'NavigateToPose rejected goal');return
            slot['handle']=handle;slot['status']='active';slot['progress_time']=time.monotonic();slot['progress_sim']=self.clock_value;slot['progress_pose']=self.poses.get(robot)
            handle.get_result_async().add_done_callback(partial(self.nav_done,robot=robot,handle=handle))
            if slot.get('cancel_when_accepted'):self.cancel(robot)
        except Exception as exc:self.failure(robot,str(exc))

    def nav_done(self,future,robot,handle):
        slot=self.slots[robot]
        if slot['handle'] is not handle:return
        slot['handle']=None;slot['cancelling']=False
        try:
            result=future.result()
            if result.status==GoalStatus.STATUS_SUCCEEDED:slot['status']='succeeded';slot['retries']=0
            elif result.status==GoalStatus.STATUS_CANCELED:slot['status']='cancelled'
            else:self.failure(robot,f'NavigateToPose result status {result.status}')
            queued=slot.pop('next_target',None)
            if queued and result.status in (GoalStatus.STATUS_SUCCEEDED,GoalStatus.STATUS_CANCELED):
                target,mission=queued;desired=self.targets.get(robot)
                if (mission==self.mission_number and self.state=='BOUNDARY_TRAVELLING' and
                    self.requested_mode=='LEGACY' and robot not in self.obstacle_recoveries and
                    not self.clearance_maneuver and desired is not None and
                    distance(desired,target)<self.p['update_distance'] and
                    abs(wrap(desired[2]-target[2]))<self.p['update_angle'] and
                    self.ready()[0] and self.feasible(robot,target,self.navigation_reservations(robot))):
                    self.dispatch_navigation(robot,target)
                    slot['updates_switched']=slot.get('updates_switched',0)+1
        except Exception as exc:self.failure(robot,str(exc))

    def pose_message(self,pose):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg();msg.pose.position.x=float(pose[0]);msg.pose.position.y=float(pose[1]);msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2);return msg

    def arrived(self,robot,target):
        pose=self.poses.get(robot)
        tolerance=self.p.get('primary_yaw_tolerance',self.p['yaw_tolerance']) if robot=='tb1' else self.p['yaw_tolerance']
        return pose is not None and distance(pose,target)<=self.p['arrival_tolerance'] and abs(wrap(pose[2]-target[2]))<=tolerance

    def idle_actions(self):return all(not s['pending'] and s['handle'] is None for s in self.slots.values())

    def tick(self):
        self.refresh_poses();ok,reason=self.ready()
        if not ok:
            if self.state not in ('STOPPED','WAITING'):self.set_state('PREFLIGHT',reason)
            self.publish_markers();return
        if self.state=='PREFLIGHT':
            self.set_state(self.phase if self.phase!='IDLE' else 'READY','Live data and Nav2 verified')
            if self.p['auto_form_rectangle'] and self.phase=='IDLE':self.phase='FORMING_RECTANGLE';self.set_state(self.phase,'Initial work formation')
        if self.state in ('STOPPED','WAITING'):
            if self.transition_state in ('CANCELLING','ASSEMBLING'):self.transition_state='BLOCKED'
            self.publish_markers();return
        if self.p['flexible_boundary'] and self.state in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING'):self.obstacle_tick()
        for robot,slot in self.slots.items():
            if slot['pending'] and time.monotonic()-slot.get('pending_since',time.monotonic())>self.p['action_response_timeout']:
                # A timed-out send may still be accepted remotely. Keep ownership,
                # stop all motor commands, and cancel if its late response arrives.
                slot['cancel_when_accepted']=True
                if self.requested_mode=='SINGLE_FILE' and self.state=='TRAVELLING' and robot!='tb1':
                    self.recover_auxiliary(robot,'action response timeout');continue
                if self.flexible_recovery(robot,'action response timeout'):continue
                self.set_state('WAITING',f'{robot}: action response timeout; awaiting cancellation/result')
                self.publish_markers();return
            if slot['handle'] is not None and not slot['cancelling']:
                if self.requested_mode=='SINGLE_FILE' and self.state=='TRAVELLING' and robot!='tb1' and (robot in self.yielding or robot in self.blocked):
                    since=self.trail_blocked_since.setdefault(robot,self.clock_value)
                    if self.clock_value-since>=self.p['yield_recovery_delay']:self.recover_auxiliary(robot,'route conflict')
                    continue
                if robot in self.yielding or robot in self.obstacle_recoveries:
                    # Intentional right-of-way waits are not navigation stalls.
                    slot['progress_sim']=self.clock_value
                    continue
                previous=slot.get('progress_pose')
                if previous is None or distance(previous,self.poses[robot])>.1:
                    slot['progress_pose']=self.poses[robot];slot['progress_sim']=self.clock_value
                elif self.clock_value-slot.get('progress_sim',self.clock_value)>self.p['progress_timeout']:
                    if self.requested_mode=='SINGLE_FILE' and self.state=='TRAVELLING' and robot!='tb1':
                        self.recover_auxiliary(robot,'actual motion stalled');continue
                    if self.flexible_recovery(robot,'actual motion stalled'):continue
                    self.cancel_all();self.set_state('WAITING',f'{robot}: actual motion stalled; resume after resolving obstruction')
                    self.publish_markers();return
        if self.state=='MODE_TRANSITION':
            self.mode_tick();self.publish_markers();return
        if self.pending_primary is not None:
            if not self.idle_actions():self.publish_markers();return
            self.primary=self.pending_primary;self.pending_primary=None
        if self.state in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING') and self.primary:
            if self.p['flexible_boundary']:
                self.clearance_tick()
                self.flexible_tick()
            else:self.boundary_tick()
        elif self.state=='FORMING_LINE' and self.primary:
            leader=self.poses['tb1'];heading=math.atan2(self.primary[1]-leader[1],self.primary[0]-leader[0])
            desired={r:(leader[0]-i*self.p['transit_spacing']*math.cos(heading),leader[1]-i*self.p['transit_spacing']*math.sin(heading),heading) for i,r in enumerate(ROBOTS[1:],1)}
            reserved={'tb1':leader};targets={}
            for r,preferred in desired.items():
                target=preferred if self.feasible(r,preferred,reserved) else self.choose(r,preferred,reserved)
                if target is None:
                    self.set_state('WAITING',f'{r}: no feasible transit assembly position; change goal/layout or resume');self.cancel_all();self.publish_markers();return
                targets[r]=target;reserved[r]=target
            self.targets=targets
            # Sequential assembly prevents crossing approaches; others remain stopped.
            for r,target in targets.items():
                if self.arrived(r,target):
                    if self.slots[r]['handle'] is not None:self.cancel(r);break
                    if self.slots[r]['pending']:break
                else:self.send(r,target);break
            else:
                if not self.idle_actions():self.cancel_all();return
                if self.requested_mode=='SINGLE_FILE':self.active_mode='SINGLE_FILE'
                self.trail=deque([(leader[0]-i*self.p['transit_spacing']*math.cos(heading),leader[1]-i*self.p['transit_spacing']*math.sin(heading),heading) for i in range(4,-1,-1)],maxlen=5000)
                self.trail_seeded=True;self.phase='TRAVELLING';self.set_state('TRAVELLING','Transit assembly verified from actual poses')
        elif self.state=='TRAVELLING' and self.primary:
            leader=self.poses['tb1']
            if not self.trail or distance(self.trail[-1],leader)>.1:self.trail.append(leader)
            if self.requested_mode=='LEGACY':
                self.legacy_transit_tick(leader);self.publish_markers();return
            finished=self.arrived('tb1',self.primary)
            desired={}
            for i,r in enumerate(ROBOTS[1:],1):
                waypoint,cursor=trail_waypoint(list(self.trail),self.poses[r],i*self.p['transit_spacing'],self.p['trail_lookahead'],self.trail_cursors[r])
                previous=self.follower_waypoints.get(r)
                if previous is not None and not self.arrived(r,previous):
                    desired[r]=previous
                elif self.slots[r]['pending'] or self.slots[r]['handle'] is not None:
                    self.cancel(r);desired[r]=previous or waypoint
                else:
                    if previous is not None:
                        _,completed=trail_waypoint(list(self.trail),previous,0.,0.,self.trail_cursors[r])
                        waypoint,cursor=trail_waypoint(list(self.trail),self.poses[r],i*self.p['transit_spacing'],self.p['trail_lookahead'],completed)
                    self.trail_cursors[r]=cursor;self.follower_waypoints[r]=waypoint
                    self.slots[r]['target']=None;self.slots[r]['status']='idle';desired[r]=waypoint
            targets=self.trail_navigation_targets(leader,desired)
            if targets is None:return
            self.targets=targets
            for r,target in targets.items():self.send(r,target)
            final={r:trail_target(list(self.trail),i*self.p['transit_spacing']) for i,r in enumerate(ROBOTS[1:],1)}
            if finished and all(self.arrived(r,t) for r,t in final.items()):
                self.cancel_all()
                if not self.idle_actions():self.publish_markers();return
                if self.requested_mode=='SINGLE_FILE':
                    self.active_mode='SINGLE_FILE';self.phase='IDLE';self.set_state('READY','Primary arrival and all travelled-trail follower slots verified')
                else:
                    self.phase='FORMING_RECTANGLE';self.rectangle_targets=None;self.set_state(self.phase,'Leader arrival verified; assembling work rectangle')
        elif self.state=='FORMING_RECTANGLE':
            if any(s['cancelling'] or s['pending'] for s in self.slots.values()):self.publish_markers();return
            leader=self.poses['tb1']
            if self.rectangle_targets is None:
                desired=self.work_positions(leader);reserved={'tb1':leader};targets={}
                for r,preferred in desired.items():
                    target=preferred if self.feasible(r,preferred,reserved) else self.choose(r,preferred,reserved)
                    if target is None:
                        self.cancel_all();self.set_state('WAITING',f'{r}: rectangle cannot fit; waiting for /formation/resume or another goal');self.publish_markers();return
                    targets[r]=target;reserved[r]=target
                self.rectangle_targets=targets
            self.targets=self.rectangle_targets
            for r,target in self.targets.items():
                if self.arrived(r,target):
                    if self.slots[r]['handle'] is not None:self.cancel(r);break
                    if self.slots[r]['pending']:break
                else:self.send(r,target);break
            else:
                ideal=self.work_positions(self.poses['tb1'])
                relaxed=any(distance(self.targets[r],ideal[r])>self.p['arrival_tolerance'] for r in ideal)
                # An initial peer may have blocked an otherwise free corner. Once
                # assembly has cleared it, prefer the exact rectangle again.
                if relaxed and all(self.feasible(r,target,ideal) for r,target in ideal.items()):
                    self.rectangle_targets=ideal;self.targets=ideal
                    self.set_state('FORMING_RECTANGLE','Space is now free: restoring exact rectangle')
                else:
                    self.cancel_all();self.phase='IDLE'
                    layout='Feasible alternative work positions' if relaxed else 'Destination rectangle'
                    self.set_state('READY',layout+' verified from all actual robot poses; ready for next goal')
        self.publish_markers()

    def recover_auxiliary(self,robot,reason):
        self.cancel(robot)
        self.trail_recovering[robot]=reason

    def flexible_recovery(self,robot,reason):
        """Recover one flexible follower without releasing action ownership."""
        if (robot=='tb1' or not self.p.get('flexible_boundary') or
            self.state not in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING')):return False
        self.cancel(robot)
        slot=self.slots[robot];slot['retries']=0;slot['cooldown']=0.
        if robot not in self.obstacle_recoveries:
            grid=self.grids.get(f'/{robot}/local_costmap/costmap')
            local=self.in_frame(robot,self.poses[robot],grid.frame) if grid else None
            mode='detour' if local is not None and grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],98) else 'escape'
            self.obstacle_recoveries[robot]={'target':None,'started':self.clock_value,
                'start_pose':self.poses[robot],'mode':mode,'state':'Checking escape route','reason':reason}
        self.get_logger().info(f'{robot}: independent flexible recovery: {reason}')
        return True

    def trail_navigation_targets(self,leader,desired):
        reserved={'tb1':leader};targets={'tb1':self.primary};waiting=[]
        for robot,target in desired.items():
            if robot in self.trail_recovering:
                slot=self.slots[robot];detour=self.trail_detours.get(robot)
                if detour is not None and self.arrived(robot,detour):
                    self.cancel(robot)
                    if slot['pending'] or slot['handle'] is not None:
                        reserved[robot]=self.poses[robot];continue
                    self.trail_detours.pop(robot,None);self.trail_recovering.pop(robot,None)
                    slot['target']=None;slot['status']='idle';slot['retries']=0
                elif detour is not None:
                    targets[robot]=detour;reserved[robot]=detour;waiting.append(robot);continue
                else:
                    if not slot['pending'] and slot['handle'] is None:
                        peer=min((r for r in ROBOTS if r!=robot),key=lambda r:distance(self.poses[robot],self.poses[r]))
                        detour=self.clearance_target(robot,peer)
                        if detour is not None:
                            self.trail_detours[robot]=detour;targets[robot]=detour;reserved[robot]=detour
                            slot['target']=None;slot['status']='idle';slot['retries']=0
                    reserved.setdefault(robot,self.poses[robot]);waiting.append(robot);continue
            if not self.feasible(robot,target,reserved,peers=False):
                self.cancel(robot)
                since=self.trail_blocked_since.setdefault(robot,self.clock_value)
                if self.clock_value-since>=self.p['yield_recovery_delay']:
                    self.recover_auxiliary(robot,'trail waypoint obstructed')
                waiting.append(robot);reserved[robot]=self.poses[robot];continue
            self.trail_blocked_since.pop(robot,None)
            targets[robot]=target;reserved[robot]=target
        self.set_state('TRAVELLING',('Independent recovery / trail retry: '+', '.join(waiting)) if waiting else 'Following the verified travelled trail')
        return targets

    def legacy_transit_tick(self,leader):
        if self.arrived('tb1',self.primary):
            self.cancel_all();self.phase='FORMING_RECTANGLE';self.rectangle_targets=None;self.set_state(self.phase,'Leader arrival verified; assembling work rectangle')
        else:
            desired={r:trail_target(list(self.trail),i*self.p['transit_spacing']) for i,r in enumerate(ROBOTS[1:],1)}
            reserved={'tb1':self.primary};targets={'tb1':self.primary}
            for r,preferred in desired.items():
                target=preferred if self.feasible(r,preferred,reserved) else self.choose(r,preferred,reserved)
                if target is not None:targets[r]=target;reserved[r]=target
            self.targets=targets
            for r,target in targets.items():self.send(r,target)

    def work_positions(self,leader):
        desired=rectangle(leader,self.p['rectangle_half_length'],self.p['rectangle_half_width'])
        if not self.p['flexible_boundary']:return desired
        slots=list(desired.values())
        self.flexible_slots=flexible_assignment(self.poses,slots,self.flexible_slots)
        return {r:slots[i] for r,i in self.flexible_slots.items()}

    def flexible_tick(self):
        # The leader follows Nav2 independently of the visual polygon.
        leader=self.poses['tb1'];finished=self.arrived('tb1',self.primary)
        if self.boundary_heading is None:self.boundary_heading=leader[2]
        self.phase='BOUNDARY_TRAVELLING'
        self.set_state(self.phase,'Flexible followers adjusting around tb1; blue line is visual only')
        targets={'tb1':self.primary}
        maneuver=getattr(self,'clearance_maneuver',None)
        if finished and getattr(self,'flexible_final_targets',None) is not None:
            targets.update(self.flexible_final_targets)
        else:
            # Freeze final slots around the requested pose, rather than the
            # leader's residual position error or the initial travel heading.
            anchor=self.primary if finished else (leader[0],leader[1],self.boundary_heading)
            slots=list(rectangle(anchor,
                self.p['rectangle_half_length'],self.p['rectangle_half_width']).values())
            self.flexible_slots=flexible_assignment(self.poses,slots,getattr(self,'flexible_slots',{}))
            reserved={'tb1':leader}
            for r,i in self.flexible_slots.items():
                recovery=getattr(self,'obstacle_recoveries',{}).get(r)
                if recovery:
                    targets[r]=recovery.get('target') or self.targets.get(r,self.poses[r]);reserved[r]=targets[r];continue
                if maneuver and r==maneuver['robot']:
                    targets[r]=maneuver['target'];reserved[r]=maneuver['target'];continue
                preferred=slots[i]
                if finished:
                    heading=self.primary[2]
                else:
                    heading=(math.atan2(preferred[1]-self.poses[r][1],preferred[0]-self.poses[r][0])
                        if distance(preferred,self.poses[r])>self.p['arrival_tolerance'] else self.poses[r][2])
                preferred=(preferred[0],preferred[1],heading)
                target=self.choose(r,preferred,reserved)
                if target is None:
                    self.cancel(r)
                    continue
                targets[r]=target;reserved[r]=target
            if finished and len(targets)==5 and not maneuver and not getattr(self,'obstacle_recoveries',{}):self.flexible_final_targets={r:t for r,t in targets.items() if r!='tb1'}
        self.targets=targets
        for robot,recovery in getattr(self,'obstacle_recoveries',{}).items():
            if recovery.get('target') is not None:self.targets[robot]=recovery['target']
        if maneuver:
            # Keep a stable temporary target while the peer yields; resume the
            # formation target only once physical arrival is verified.
            self.targets[maneuver['robot']]=maneuver['target']
        for r,target in targets.items():
            recovery=getattr(self,'obstacle_recoveries',{}).get(r)
            if recovery and recovery.get('target') is None:continue
            if maneuver and r==maneuver['peer']:continue
            self.send(r,target)
        if not maneuver and not getattr(self,'obstacle_recoveries',{}) and finished and len(targets)==5 and all(self.arrived(r,t) for r,t in targets.items()):
            self.cancel_all()
            if self.idle_actions():
                self.phase='IDLE';self.set_state('READY','Primary goal and feasible follower positions reached')

    def obstacle_tick(self):
        for robot,recovery in list(self.obstacle_recoveries.items()):
            if self.slots[robot]['pending'] or self.slots[robot]['handle'] is not None:
                if recovery.get('target') is None:self.cancel(robot)
                continue
            grid=self.grids.get(f'/{robot}/local_costmap/costmap')
            local=self.in_frame(robot,self.poses[robot],grid.frame) if grid else None
            if local is None:continue
            release_padding=max(self.p['footprint_padding'],self.p['obstacle_release_padding'])
            if grid.footprint_free(local,self.polygons[robot],release_padding,98):
                if ((recovery.get('target') is None and recovery.get('mode')=='escape' and
                     distance(self.poses[robot],recovery['start_pose'])>=.3) or
                    (recovery.get('target') is not None and self.arrived(robot,recovery['target']) and
                     distance(self.poses[robot],recovery['start_pose'])>=.3)):
                    del self.obstacle_recoveries[robot];self.obstacle_since.pop(robot,None)
                    self.slots[robot]['target']=None;self.slots[robot]['last_send']=0.
                    self.slots[robot]['retries']=0;self.slots[robot]['cooldown']=0.
                    self.get_logger().info(f'{robot}: obstacle zone cleared; resuming formation')
        for robot in list(self.obstacle_since):
            if robot not in self.local_blocked and robot not in self.obstacle_recoveries:self.obstacle_since.pop(robot,None)
        for robot in sorted(self.local_blocked):
            if robot in self.obstacle_recoveries or robot in self.yielding:continue
            since=self.obstacle_since.setdefault(robot,self.clock_value)
            if self.clock_value-since<self.p['obstacle_recovery_delay']:continue
            self.cancel(robot)
            self.slots[robot]['retries']=0;self.slots[robot]['cooldown']=0.
            grid=self.grids[f'/{robot}/local_costmap/costmap'];local=self.in_frame(robot,self.poses[robot],grid.frame)
            mode='detour' if local is not None and grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],98) else 'escape'
            self.obstacle_recoveries[robot]={'target':None,'started':self.clock_value,'start_pose':self.poses[robot],'mode':mode,'state':'Checking escape route'}
            self.get_logger().info(f'{robot}: starting local obstacle escape recovery')

    def obstacle_escape_command(self,robot,poses):
        recovery=self.obstacle_recoveries[robot];slot=self.slots[robot]
        if slot['pending'] or slot['handle'] is not None:return (0.,0.)
        pose=poses[robot];grid=self.grids[f'/{robot}/local_costmap/costmap']
        local=self.in_frame(robot,pose,grid.frame)
        if local is None:return (0.,0.)
        speed=self.p['obstacle_escape_speed']
        choices=[(-speed,0.),(speed,0.),(-speed,.25),(-speed,-.25),(speed,.25),(speed,-.25),(0.,.25),(0.,-.25)]
        if recovery.get('command') in choices:choices.remove(recovery['command']);choices.insert(0,recovery['command'])
        choices=[c for c in choices if all(not motion_conflict(pose,c,self.polygons[robot],
            other,(0.,0.),self.polygons[r],pair_separation(robot,r,self.p['minimum_separation'],self.p['primary_minimum_separation']),
            self.p['collision_horizon']) for r,other in poses.items() if r!=robot)]
        corridor=FleetMissionController.primary_corridor(self) if robot!='tb1' else []
        initial_gap=FleetMissionController.primary_corridor_gap(self,robot,pose,corridor)
        def verify(projected):
            dx,dy=rotate(projected[0]-local[0],projected[1]-local[1],pose[2]-local[2])
            world=(pose[0]+dx,pose[1]+dy,pose[2]+projected[2]-local[2])
            if FleetMissionController.primary_corridor_gap(self,robot,world,corridor)<min(initial_gap,self.p['primary_minimum_separation'])-1e-6:return False
            for topic in ('/map',f'/{robot}/global_costmap/costmap'):
                other=self.grids.get(topic)
                transformed=self.in_frame(robot,world,other.frame) if other else None
                if transformed is None or not other.footprint_free(transformed,self.polygons[robot],.02,99):return False
            return True
        if corridor:
            choices=[c for c in choices if FleetMissionController.auxiliary_corridor_safe(self,robot,pose,c,corridor)]
        command=inflation_escape(grid,local,self.polygons[robot],choices,self.p['footprint_padding'],self.p['collision_horizon'],verify)
        if command is not None:
            recovery['state']='Escaping obstacle buffer';recovery['command']=command;return command
        # If already physically outside the inflated zone, replan to a nearby
        # free position instead of repeatedly trying the obstructed trajectory.
        if grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],98):
            if recovery.get('mode')=='escape':
                for candidate in choices:
                    if abs(candidate[0])<1e-4:continue
                    if all(grid.footprint_free(advance(local,candidate,self.p['collision_horizon']*i/10),self.polygons[robot],self.p['footprint_padding'],98)
                        and verify(advance(local,candidate,self.p['collision_horizon']*i/10)) for i in range(11)):
                        recovery['command']=candidate;return candidate
            if self.clock_value>=recovery.get('next_detour_check',0.):
                recovery['next_detour_check']=self.clock_value+1.
                target=self.obstacle_detour_target(robot,poses)
                if target is not None:
                    recovery['target']=target;recovery['state']='Taking independent obstacle detour'
        else:recovery['state']='Waiting for a safe escape route'
        return (0.,0.)

    def obstacle_detour_target(self,robot,poses):
        """Choose each robot's own side; Nav2 retains the waypoint action."""
        start=poses[robot];goal=self.targets.get(robot,start)
        # Break equally short left/right ties independently, never overriding
        # map or peer clearance to force robots onto opposite sides.
        side=1. if ROBOTS.index(robot)%2 else -1.
        corridor=FleetMissionController.primary_corridor(self) if robot!='tb1' else []
        initial=FleetMissionController.primary_corridor_gap(self,robot,start,corridor)
        def free(pose):
            if FleetMissionController.primary_corridor_gap(self,robot,pose,corridor)<min(initial,self.p['primary_minimum_separation'])-1e-6:return False
            for topic in ('/map',f'/{robot}/global_costmap/costmap',f'/{robot}/local_costmap/costmap'):
                grid=self.grids.get(topic)
                local=self.in_frame(robot,pose,grid.frame) if grid else None
                if local is None or not grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],98):return False
            return True
        targets=sorted(obstacle_detour_candidates(start,goal,side),
            key=lambda target:distance(start,target)+distance(target,goal))
        for target in targets:
            if not self.feasible(robot,target):continue
            if stationary_route_clear(start,[target],robot,poses,self.polygons,
                self.p['minimum_separation'],self.p['primary_minimum_separation'],free=free):return target
        return None

    def clearance_tick(self):
        now=self.clock_value
        if self.clearance_maneuver:
            m=self.clearance_maneuver
            if self.arrived(m['robot'],m['target']):
                self.cancel(m['robot']);self.clearance_maneuver=None;self.conflict_since.clear()
            return
        for pair in list(self.conflict_since):
            if pair not in self.unresolved_pairs:del self.conflict_since[pair]
        for winner,loser in sorted(self.unresolved_pairs):
            pair=(winner,loser);since=self.conflict_since.setdefault(pair,now)
            if now-since<self.p['yield_recovery_delay']:continue
            moving,held=(loser,winner) if winner=='tb1' else (winner,loser)
            target=self.clearance_target(moving,held)
            if target is None:continue
            self.clearance_maneuver={'robot':moving,'peer':held,'target':target}
            self.cancel(moving)
            self.slots[moving]['retries']=0;self.slots[moving]['cooldown']=0.
            self.get_logger().info(f'{moving} taking a clearance waypoint; {held} held until safe')
            return

    def clearance_target(self,robot,peer):
        """Find a locally verified escape route, keeping Nav2 in control."""
        start=self.poses[robot];other=self.poses[peer];poly=self.polygons[robot]
        initial=footprint_clearance(start,poly,other,self.polygons[peer])
        peer_gap=pair_separation(robot,peer,self.p['minimum_separation'],self.p.get('primary_minimum_separation'))
        preferred=self.targets.get(robot,start);options=[]
        corridor=FleetMissionController.primary_corridor(self) if robot!='tb1' else []
        initial_corridor=FleetMissionController.primary_corridor_gap(self,robot,start,corridor)
        for radius in (.6,1.,self.p['yield_clearance_distance']):
            for i in range(16):
                angle=2*math.pi*i/16
                target=(start[0]+radius*math.cos(angle),start[1]+radius*math.sin(angle),start[2])
                clearance=footprint_clearance(target,poly,other,self.polygons[peer])
                if clearance<max(peer_gap+.2,initial+.3):continue
                if not self.feasible(robot,target):continue
                safe=True
                for j in range(1,11):
                    sample=(start[0]+(target[0]-start[0])*j/10,start[1]+(target[1]-start[1])*j/10,start[2])
                    if not self.feasible(robot,sample,peers=False,reserve_primary=False):safe=False;break
                    if FleetMissionController.primary_corridor_gap(self,robot,sample,corridor)<min(initial_corridor,self.p.get('primary_minimum_separation',self.p['minimum_separation']))-1e-6:safe=False;break
                    for r,p in self.poses.items():
                        if r==robot:continue
                        before=footprint_clearance(start,poly,p,self.polygons[r])
                        gap=pair_separation(robot,r,self.p['minimum_separation'],self.p.get('primary_minimum_separation'))
                        threshold=min(gap,before)
                        if footprint_clearance(sample,poly,p,self.polygons[r])<max(.05,threshold)-1e-6:safe=False;break
                    if not safe:break
                if safe:options.append((distance(target,preferred)+radius*.2,target))
        return min(options,key=lambda p:p[0])[1] if options else None

    def boundary_tick(self):
        leader=self.poses['tb1']
        # Preserve the enclosure's axes rather than rotating it to match a
        # small leader steering/physics yaw change when a new goal arrives.
        dx=sum(self.poses[a][0]-self.poses[b][0] for a,b in (('tb2','tb5'),('tb4','tb3')))
        dy=sum(self.poses[a][1]-self.poses[b][1] for a,b in (('tb2','tb5'),('tb4','tb3')))
        enclosure_heading=math.atan2(dy,dx) if math.hypot(dx,dy)>1e-6 else leader[2]
        if self.boundary_heading is None:self.boundary_heading=enclosure_heading
        if self.boundary_waypoint is None:
            if self.state=='FORMING_BOUNDARY':
                self.boundary_waypoint=(leader[0],leader[1],self.boundary_heading)
            else:
                remaining=distance(leader,self.primary)
                fraction=min(1.,self.p['boundary_step']/max(remaining,1e-9))
                self.boundary_waypoint=(leader[0]+fraction*(self.primary[0]-leader[0]),
                    leader[1]+fraction*(self.primary[1]-leader[1]),self.boundary_heading)
                if remaining<=self.p['boundary_step']:
                    turn=max(-0.1,min(0.1,wrap(self.primary[2]-self.boundary_heading)))
                    self.boundary_waypoint=(self.primary[0],self.primary[1],self.boundary_heading+turn)
            final_segment=distance(leader,self.primary)<=self.p['boundary_step']
            driving_heading=self.primary[2] if final_segment else math.atan2(self.primary[1]-leader[1],self.primary[0]-leader[0])
            leader_target=(self.boundary_waypoint[0],self.boundary_waypoint[1],driving_heading)
            targets={'tb1':leader if self.state=='FORMING_BOUNDARY' else leader_target}
            targets.update({r:(p[0],p[1],driving_heading) for r,p in rectangle(
                self.boundary_waypoint,self.p['rectangle_half_length'],self.p['rectangle_half_width']).items()})
            # Check the whole group's swept formation, including intermediate rotations.
            for i in range(11):
                t=i/10.;center=(leader[0]+t*(self.boundary_waypoint[0]-leader[0]),
                    leader[1]+t*(self.boundary_waypoint[1]-leader[1]),
                    leader[2] if self.state=='FORMING_BOUNDARY' else leader[2]+t*wrap(driving_heading-leader[2]))
                sample={'tb1':center}
                enclosure_center=(center[0],center[1],enclosure_heading+t*wrap(self.boundary_waypoint[2]-enclosure_heading))
                sample.update({r:(p[0],p[1],self.poses[r][2]+t*wrap(driving_heading-self.poses[r][2]))
                    for r,p in rectangle(enclosure_center,self.p['rectangle_half_length'],self.p['rectangle_half_width']).items()})
                if not all(self.feasible(r,p,sample,peers=False) for r,p in sample.items()):
                    self.cancel_all();self.set_state('WAITING','Whole boundary cannot fit along the next segment; choose a goal in open space')
                    return
            self.targets=targets
        if self.state=='FORMING_BOUNDARY':
            # Assemble without crossing simultaneous approaches.
            for r in ROBOTS[1:]:
                if not self.arrived(r,self.targets[r]):self.send(r,self.targets[r]);return
                if self.slots[r]['handle'] is not None:self.cancel(r);return
                if self.slots[r]['pending']:return
        else:
            for r,target in self.targets.items():self.send(r,target)
        if not all(self.arrived(r,t) for r,t in self.targets.items()):return
        if not self.idle_actions():self.cancel_all();return
        final=(self.state=='BOUNDARY_TRAVELLING' and self.arrived('tb1',self.primary)
               and abs(wrap(self.boundary_waypoint[2]-self.primary[2]))<=self.p['yaw_tolerance'])
        self.boundary_heading=self.boundary_waypoint[2]
        self.boundary_waypoint=None
        for slot in self.slots.values():slot['target']=None;slot['status']='idle'
        if final:
            self.phase='IDLE';self.set_state('READY','Primary goal and enclosing rectangle verified from actual poses')
        else:
            self.phase='BOUNDARY_TRAVELLING';self.set_state(self.phase,'Moving enclosing rectangle toward primary goal')

    def gate(self):
        now=time.monotonic();ok,_=self.ready()
        permitted=ok and (self.state in ('MODE_TRANSITION','FORMING_LINE','TRAVELLING','FORMING_RECTANGLE','FORMING_BOUNDARY','BOUNDARY_TRAVELLING') or (self.manual and self.state=='READY'))
        commands={r:(0.,0.) for r in ROBOTS}
        self.boundary_holds=set()
        if permitted:
            for r in ROBOTS:
                msg,stamp=self.commands.get(r,(Twist(),0.))
                if now-stamp>self.p['command_timeout']:continue
                # Only owned action goals may move in mission mode.
                if not self.manual and (self.slots[r]['handle'] is None or self.slots[r].get('cancelling',False) or self.slots[r].get('cancel_when_accepted')):continue
                speed=self.p['leader_speed'] if r=='tb1' else self.p['auxiliary_speed']
                # Preserve signed Nav2 velocity: reversing is a valid formation
                # maneuver and passes the same obstacle/yield checks as forward motion.
                commands[r]=(max(-speed,min(speed,msg.linear.x)),max(-self.p['angular_speed'],min(self.p['angular_speed'],msg.angular.z)))
            if self.state=='BOUNDARY_TRAVELLING' and not self.p['flexible_boundary']:
                heading=self.boundary_heading if self.boundary_heading is not None else self.poses['tb1'][2]
                self.boundary_holds=boundary_followers_holding(self.poses,heading,
                    self.p['rectangle_half_length'],self.p['rectangle_half_width'],
                    self.p['boundary_tolerance'],commands['tb1'][0])
                if self.boundary_holds:commands['tb1']=(0.,commands['tb1'][1])
            if self.state=='MODE_TRANSITION':
                for r in ROBOTS:
                    if self.transition_state!='ASSEMBLING' or r not in (set(self.assembly_routes) if self.requested_mode=='SINGLE_FILE' else {self.transition_robot}):commands[r]=(0.,0.)
            if self.state=='FORMING_BOUNDARY' and not self.p['flexible_boundary']:commands['tb1']=(0.,0.)
            if self.state=='TRAVELLING':
                single=self.requested_mode=='SINGLE_FILE'
                recovering=set(self.trail_recovering)|set(self.trail_blocked_since) if single else set()
                commands=follower_lag_gate(self.poses,commands,self.p['maximum_follower_lag'],recovering,allow_turning=single)
            maneuver=self.clearance_maneuver
            if maneuver:commands[maneuver['peer']]=(0.,0.)
            # In verified simulation localization, Gazebo world and map share
            # axes. Use fresh physical poses for peer spacing so small TF lag
            # cannot make a close pass look safer than it really is.
            collision_poses=self.poses
            if (self.p.get('verify_gazebo_localization') and
                now-self.physical_stamp<=self.p['clock_timeout'] and
                all(r in self.physical_poses for r in ROBOTS)):
                collision_poses=self.physical_poses
            self.rolling_turn_active=False
            if (self.p.get('leader_rolling_turn') and self.state=='BOUNDARY_TRAVELLING'
                and 'tb1' not in self.obstacle_recoveries
                and self.p['flexible_boundary'] and not (maneuver and maneuver['peer']=='tb1')
                and self.primary is not None and
                distance(collision_poses['tb1'],self.primary)>self.p['arrival_tolerance']+.6):
                requested=commands['tb1']
                commands['tb1']=self.rolling_turn_command(requested,commands,collision_poses)
                self.rolling_turn_active=commands['tb1']!=requested
            self.blocked=set()
            corridor=FleetMissionController.primary_corridor(self)
            self.local_blocked=set();escape_commands=set()
            for r,recovery in getattr(self,'obstacle_recoveries',{}).items():
                if recovery.get('target') is None:
                    commands[r]=self.obstacle_escape_command(r,collision_poses);escape_commands.add(r)
            for r,(v,w) in list(commands.items()):
                if r in escape_commands:continue # Entire escape sweep was checked above.
                grid=self.grids[f'/{r}/local_costmap/costmap']
                if abs(v)<1e-4 and abs(w)<1e-4:
                    target=self.targets.get(r)
                    local=self.in_frame(r,collision_poses[r],grid.frame)
                    if (self.slots[r]['handle'] is not None and target is not None and not self.arrived(r,target)
                        and (local is None or not grid.footprint_free(local,self.polygons[r],self.p['footprint_padding'],98))):
                        self.blocked.add(r);self.local_blocked.add(r)
                    continue
                # Check predicted footprint against lidar-populated local costmap.
                if r!='tb1':
                    local=self.in_frame(r,collision_poses[r],grid.frame)
                    adjusted=local_obstacle_command(grid,local,(v,w),self.polygons[r],self.p['footprint_padding']) if local is not None else None
                    if adjusted is None:commands[r]=(0.,0.);self.blocked.add(r);self.local_blocked.add(r)
                    else:commands[r]=adjusted
                    continue
                for dt in (.3,.6,1.):
                    projected=advance(self.poses[r],(v,w),dt)
                    local=self.in_frame(r,projected,grid.frame)
                    if local is None or not grid.footprint_free(local,self.polygons[r],self.p['footprint_padding'],98):commands[r]=(0.,0.);self.blocked.add(r);self.local_blocked.add(r);break
            for r in ROBOTS[1:]:
                if commands[r]!=(0.,0.) and not FleetMissionController.auxiliary_corridor_safe(self,r,collision_poses[r],commands[r],corridor):
                    commands[r]=(0.,0.);self.blocked.add(r);self.local_blocked.add(r)
            commands,blocked,self.yielding,self.unresolved_pairs=yielding_gate(
                collision_poses,commands,self.polygons,self.p['minimum_separation'],self.p['collision_horizon'],self.p.get('primary_minimum_separation'))
            self.blocked|=blocked
            # Nav2 itself can output zero when a nearby peer blocks its route.
            # Detect those waits too, rather than waiting for the fleet watchdog.
            for i,a in enumerate(ROBOTS):
                for b in ROBOTS[i+1:]:
                    winner,loser=sorted((a,b),key=lambda r:(r!='tb1',r))
                    watched=winner if self.slots[winner]['handle'] is not None else loser
                    target=self.targets.get(watched)
                    progress_age=self.clock_value-self.slots[watched].get('progress_sim',self.clock_value)
                    if (target is not None and self.slots[watched]['handle'] is not None
                        and not self.arrived(watched,target)
                        and (progress_age>=self.p['yield_recovery_delay'] or
                             (abs(commands[watched][0])<1e-4 and abs(commands[watched][1])<.04))
                        and distance(collision_poses[a],collision_poses[b])<self.radii[a]+self.radii[b]+pair_separation(a,b,self.p['minimum_separation'],self.p.get('primary_minimum_separation'))+.5):
                        self.unresolved_pairs.add((winner,loser));self.yielding[loser]=winner
                        commands[loser]=(0.,0.)
            # A newly held peer can change the safety of a third robot's path.
            commands,blocked,extra_yielding,extra_pairs=yielding_gate(
                collision_poses,commands,self.polygons,self.p['minimum_separation'],self.p['collision_horizon'],self.p.get('primary_minimum_separation'))
            self.blocked|=blocked;self.yielding.update(extra_yielding);self.unresolved_pairs|=extra_pairs
            if maneuver:self.yielding[maneuver['peer']]=maneuver['robot']
        else:
            self.blocked=set();self.local_blocked=set();self.yielding={};self.unresolved_pairs=set();self.rolling_turn_active=False
        if commands['tb1']==(0.,0.):self.rolling_turn_active=False
        for r,(v,w) in commands.items():
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.cmd_pubs[r].publish(msg)

    def rolling_turn_command(self,requested,commands,poses):
        """Prefer a rolling turn only when its entire short arc is verified free."""
        options=[]
        for candidate in rolling_turn_candidates(requested,self.p['rolling_turn_speed'],self.p['rolling_turn_rate']):
            safe=True
            for peer in ROBOTS[1:]:
                gap=pair_separation('tb1',peer,self.p['minimum_separation'],self.p['primary_minimum_separation'])
                if motion_conflict(poses['tb1'],candidate,self.polygons['tb1'],
                    poses[peer],commands[peer],self.polygons[peer],gap,self.p['collision_horizon']):safe=False;break
            if not safe:continue
            for i in range(11):
                projected=advance(poses['tb1'],candidate,self.p['collision_horizon']*i/10)
                for topic in ('/map','/tb1/global_costmap/costmap','/tb1/local_costmap/costmap'):
                    grid=self.grids.get(topic)
                    local=self.in_frame('tb1',projected,grid.frame) if grid is not None else None
                    if local is None or not grid.footprint_free(local,self.polygons['tb1'],self.p['footprint_padding'],98):safe=False;break
                if not safe:break
            if safe:options.append(candidate)
        if not options:return requested
        goal=getattr(self,'primary',None)
        if goal is None:return options[0]
        # Keep angular intent, but select reverse when it gives a better
        # position for the turn. Both complete swept arcs passed every gate.
        return min(options,key=lambda c:distance(advance(poses['tb1'],c,self.p['collision_horizon']),goal))

    def stop_service(self,req,res):
        self.obstacle_recoveries={};self.obstacle_since={}
        self.clearance_maneuver=None;self.conflict_since.clear();self.yielding={}
        self.transition_state='STOPPED';self.transition_robot=None
        self.manual=False;self.pending_primary=None;self.cancel_all();self.set_state('STOPPED','Operator stop; zero motor commands');res.success=True;res.message=self.reason;return res
    def resume_service(self,req,res):
        if self.phase=='IDLE':self.transition_state='COMPLETE' if self.active_mode else 'IDLE'
        self.manual=self.p['manual_navigation'] and self.phase=='IDLE'
        for slot in self.slots.values():slot['retries']=0;slot['cooldown']=0.;slot['status']='idle'
        self.set_state('PREFLIGHT','Rechecking readiness before resuming');res.success=True;res.message=self.reason;return res
    def rectangle_service(self,req,res):
        self.p.update(self.legacy_parameters);self.requested_mode='LEGACY';self.active_mode='LEGACY'
        self.transition_state='IDLE';self.transition_robot=None
        self.obstacle_recoveries={};self.obstacle_since={}
        self.clearance_maneuver=None;self.conflict_since.clear();self.yielding={}
        self.manual=False;self.primary=None;self.pending_primary=None;self.mission_number+=1;self.cancel_all();self.rectangle_targets=None;self.phase='FORMING_RECTANGLE';self.set_state('FORMING_RECTANGLE','Operator requested work rectangle');res.success=True;res.message=self.reason;return res

    def mode_request(self,msg):
        mode=msg.data.strip().upper()
        if mode not in (*MODES,'LEGACY'):
            self.get_logger().error('Unknown formation mode: '+mode);return
        self.follower_waypoints={};self.trail_cursors={r:0. for r in ROBOTS[1:]};self.trail_blocked_since={};self.trail_recovering={};self.trail_detours={}
        self.requested_mode=mode;self.manual=False;self.primary=None;self.pending_primary=None
        self.mission_number+=1;self.cancel_all()
        self.mode_targets=None;self.mode_anchor=None;self.transition_robot=None;self.assembly_route=None;self.assembly_stage=None;self.assembly_routes={};self.assembly_blocked_since={}
        self.transition_started=time.monotonic();self.transition_duration=None
        self.transition_state='CANCELLING';self.phase='MODE_TRANSITION'
        self.clearance_maneuver=None;self.obstacle_recoveries={};self.obstacle_since={}
        self.conflict_since.clear();self.yielding={};self.boundary_waypoint=None;self.boundary_heading=None
        for slot in self.slots.values():
            slot['retries']=0;slot['cooldown']=0.;slot['target']=None
        self.set_state('MODE_TRANSITION','Waiting for all owned actions to finish cancellation')

    def mode_blocked(self,reason):
        self.cancel_all();self.transition_robot=None;self.transition_state='BLOCKED'
        self.set_state('WAITING',self.requested_mode+': '+reason)

    def wall_verified(self):
        grid=self.grids.get('/map')
        if grid is None:return False
        x,y,angle=self.p['wall_x'],self.p['wall_y'],self.p['wall_normal_yaw']
        # A configured wall point must touch an occupied map cell. Probe just
        # into the accessible side with the auxiliary footprint, rejecting unknown.
        dx,dy=rotate(x-grid.origin[0],y-grid.origin[1],-grid.angle)
        ix,iy=int(math.floor(dx/grid.resolution)),int(math.floor(dy/grid.resolution))
        h,w=grid.cells.shape
        if not (0<=ix<w and 0<=iy<h and grid.cells[iy,ix]>=99):return False
        nx,ny=rotate(self.p['wall_probe_distance'],0.,angle)
        return grid.footprint_free((x+nx,y+ny,angle),self.polygons['tb2'],self.p['footprint_padding'],self.p['maximum_target_cost'])

    def mode_tick(self):
        if self.requested_mode=='LEGACY':
            if not self.idle_actions():return
            self.p.update(self.legacy_parameters)
            self.active_mode='LEGACY';self.transition_state='COMPLETE'
            self.transition_duration=time.monotonic()-self.transition_started
            self.phase='IDLE';self.set_state('READY','Original formation behavior restored; ready for a primary goal')
            return
        if time.monotonic()-self.transition_started>self.p['transition_timeout']:
            self.mode_blocked('transition timed out before actual pose verification');return
        if self.transition_state=='CANCELLING':
            if not self.idle_actions():return
            self.mode_anchor=self.poses['tb1']
            if self.mode_targets is None:
                if self.requested_mode=='WALL_WORKSPACE':
                    nx,ny=rotate(1.,0.,self.p['wall_normal_yaw'])
                    wall_distance=(self.mode_anchor[0]-self.p['wall_x'])*nx+(self.mode_anchor[1]-self.p['wall_y'])*ny
                    if not 0.<wall_distance<=self.p['maximum_wall_distance']:
                        self.mode_blocked('tb1 is not within the configured nearby-wall distance on its accessible side');return
                if self.requested_mode=='WALL_WORKSPACE' and not self.wall_verified():
                    self.mode_blocked('configured wall point / accessible normal is not verified on the map');return
                try:
                    self.mode_targets=formation_slots(self.requested_mode,self.mode_anchor,{**self.p,'rectangle_half_length':self.open_dimensions[0],'rectangle_half_width':self.open_dimensions[1]},
                        (self.p['wall_x'],self.p['wall_y'],self.p['wall_normal_yaw']))
                except ValueError as exc:self.mode_blocked(str(exc));return
            reserved={'tb1':self.mode_anchor,**self.mode_targets}
            reason=layout_clearance(reserved,self.polygons,self.p['minimum_separation'],self.p['primary_minimum_separation'])
            if reason:self.mode_blocked(reason);return
            for r,t in self.mode_targets.items():
                if not self.feasible(r,t,reserved,peers=False):
                    self.mode_blocked(r+': exact target footprint cannot fit on map/costmap');return
            self.targets=dict(self.mode_targets);self.transition_state='ASSEMBLING'
            self.set_state('MODE_TRANSITION','Targets verified; assembling auxiliaries while tb1 holds')
        if self.transition_state!='ASSEMBLING':return
        if distance(self.poses['tb1'],self.mode_anchor)>self.p['arrival_tolerance']:
            self.mode_blocked('tb1 moved away from the verified transition anchor');return
        if self.requested_mode=='SINGLE_FILE':
            self.concurrent_line_assembly_tick();return
        # Reuse the supervisor's checked clearance waypoint search, but move
        # only the auxiliary that owns this sequential assembly step.
        maneuver=self.clearance_maneuver
        if maneuver:
            robot=maneuver['robot']
            if self.arrived(robot,maneuver['target']):
                self.cancel(robot)
                if self.idle_actions():self.clearance_maneuver=None;self.conflict_since.clear()
            else:
                self.targets={**self.mode_targets,robot:maneuver['target']}
                self.send(robot,maneuver['target'])
            return
        if self.transition_robot is not None:
            robot=self.transition_robot
            pairs=set(self.unresolved_pairs)
            if robot in self.blocked:
                pairs.update((winner,loser) for loser,winner in self.yielding.items() if robot in (winner,loser))
            for pair in sorted(pairs):
                if robot not in pair:continue
                since=self.conflict_since.setdefault(pair,self.clock_value)
                if self.clock_value-since<self.p['yield_recovery_delay']:continue
                peer=next(r for r in pair if r!=robot)
                target=self.clearance_target(robot,peer)
                if target is not None:
                    self.clearance_maneuver={'robot':robot,'peer':peer,'target':target}
                    self.cancel(robot)
                    self.set_state('MODE_TRANSITION',robot+': taking verified clearance waypoint around '+peer)
                    return
        self.targets=dict(self.mode_targets)
        remaining=[r for r,t in self.mode_targets.items() if not self.arrived(r,t)]
        if not remaining:
            self.complete_mode();return
        if self.transition_robot is not None:
            r=self.transition_robot
            if self.arrived(r,self.mode_targets[r]):
                self.cancel(r)
                if self.idle_actions():self.transition_robot=None
                return
        if self.transition_robot is None:
            # Prefer a currently reachable free target; current peers remain fixed.
            self.transition_robot=next((r for r in remaining if self.feasible(r,self.mode_targets[r],self.mode_targets)),None)
            if self.transition_robot is None:
                self.mode_blocked('all remaining slots obstructed by stationary peers');return
        if not self.feasible(self.transition_robot,self.mode_targets[self.transition_robot],self.mode_targets):
            self.mode_blocked(self.transition_robot+': target or approach obstructed by current peers/map');return
        self.send(self.transition_robot,self.mode_targets[self.transition_robot])

    def complete_mode(self):
        if not self.idle_actions():self.cancel_all();return
        self.active_mode=self.requested_mode;self.transition_state='COMPLETE';self.transition_robot=None
        self.transition_duration=time.monotonic()-self.transition_started
        if self.active_mode=='SINGLE_FILE' and not self.trail_seeded:
            self.trail=deque([self.mode_targets[r] for r in reversed(ROBOTS[1:])]+[self.mode_anchor],maxlen=5000)
            self.trail_seeded=True
        if self.active_mode=='COMPACT_RECTANGLE':
            self.p['rectangle_half_length']=self.p['compact_half_length'];self.p['rectangle_half_width']=self.p['compact_half_width']
        elif self.active_mode=='OPEN_RECTANGLE':
            self.p['rectangle_half_length'],self.p['rectangle_half_width']=self.open_dimensions
        if self.active_mode in ('OPEN_RECTANGLE','COMPACT_RECTANGLE'):self.p['flexible_boundary']=False
        self.phase='IDLE';self.set_state('READY','Formation verified from all actual poses and action results');return

    def concurrent_line_assembly_tick(self):
        self.targets=dict(self.mode_targets)
        waiting=[]
        for robot in ROBOTS[1:]:
            slot=self.slots[robot]
            if self.arrived(robot,self.mode_targets[robot]):
                self.cancel(robot)
                if not slot['pending'] and slot['handle'] is None:
                    self.assembly_routes.pop(robot,None)
                continue
            route=self.assembly_routes.get(robot)
            stalled=(robot in self.blocked or robot in self.yielding or
                     (slot['handle'] is not None and self.clock_value-slot.get('progress_sim',self.clock_value)>self.p['yield_recovery_delay']))
            if stalled:
                since=self.assembly_blocked_since.setdefault(robot,self.clock_value)
                if self.clock_value-since>=self.p['yield_recovery_delay']:
                    self.cancel(robot)
                    if slot['pending'] or slot['handle'] is not None:continue
                    self.assembly_routes.pop(robot,None);route=None
                    slot['target']=None;slot['status']='idle'
                    self.assembly_blocked_since.pop(robot,None)
            else:self.assembly_blocked_since.pop(robot,None)
            if route is None:
                for candidate in assembly_reroute_candidates(self.poses[robot],self.mode_targets[robot],
                        self.p['single_file_assembly_axis_yaw'],self.p['single_file_clearance_step']):
                    if stationary_route_clear(self.poses[robot],candidate,robot,self.poses,self.polygons,
                            self.p['minimum_separation'],self.p['primary_minimum_separation'],
                            free=lambda pose,r=robot:self.feasible(r,pose,peers=False)):
                        route=list(candidate);self.assembly_routes[robot]=route;break
                if route is None:waiting.append(robot);continue
            target=route[0]
            reached=(distance(self.poses[robot],target)<=self.p['arrival_tolerance'] if len(route)>1 else self.arrived(robot,target))
            if reached:
                self.cancel(robot)
                # Wait only for this robot's result, never for another robot.
                if slot['pending'] or slot['handle'] is not None:continue
                route.pop(0);slot['target']=None;slot['status']='idle'
                if not route:self.assembly_routes.pop(robot,None);continue
                target=route[0]
            self.targets[robot]=target
        for robot in list(self.assembly_routes):
            self.send(robot,self.targets[robot])
        self.assembly_stage='CONCURRENT_HORIZONTAL_ASSEMBLY'
        self.set_state('MODE_TRANSITION','Concurrent auxiliary assembly'+('; waiting for clearance: '+', '.join(waiting) if waiting else ''))
        if all(self.arrived(r,t) for r,t in self.mode_targets.items()):self.complete_mode()

    def horizontal_assembly_tick(self):
        remaining=[r for r in ROBOTS[1:] if not self.arrived(r,self.mode_targets[r])]
        if not remaining:
            self.complete_mode();return
        if self.transition_robot is None:
            # Clear the rear robots first. Only pick a robot whose entire simple
            # horizontal-first route is verified around currently stationary peers.
            for robot in reversed(remaining):
                for route in horizontal_first_routes(self.poses[robot],self.mode_targets[robot],
                        self.p['single_file_assembly_axis_yaw'],self.p['single_file_clearance_step']):
                    if stationary_route_clear(self.poses[robot],route,robot,self.poses,self.polygons,
                            self.p['minimum_separation'],self.p['primary_minimum_separation'],
                            free=lambda pose,r=robot:self.feasible(r,pose,peers=False)):
                        self.transition_robot=robot;self.assembly_route=list(route)
                        break
                if self.transition_robot is not None:break
            if self.transition_robot is None:
                self.mode_blocked('no safe horizontal-first approach; current peers or map obstruct both simple routes');return
        robot=self.transition_robot
        if self.arrived(robot,self.mode_targets[robot]):
            self.cancel(robot)
            if self.idle_actions():self.transition_robot=None;self.assembly_route=None
            return
        target=self.assembly_route[0]
        reached=(distance(self.poses[robot],target)<=self.p['arrival_tolerance'] if len(self.assembly_route)>1 else self.arrived(robot,target))
        if reached:
            self.cancel(robot)
            if not self.idle_actions():return
            self.assembly_route.pop(0)
            self.slots[robot]['target']=None;self.slots[robot]['status']='idle'
            if not self.assembly_route:self.transition_robot=None;return
            target=self.assembly_route[0]
        self.assembly_stage='HORIZONTAL_CLEARANCE' if len(self.assembly_route)>1 else 'APPROACH_SLOT'
        self.targets={**self.mode_targets,robot:target}
        self.set_state('MODE_TRANSITION',robot+': '+('moving horizontally clear of stationary robots' if self.assembly_stage=='HORIZONTAL_CLEARANCE' else 'horizontal clearance verified; turning toward line slot'))
        self.send(robot,target)

    def formation_targets(self):
        if self.state=='TRAVELLING' and self.requested_mode=='SINGLE_FILE' and self.trail:
            return {r:trail_target(list(self.trail),i*self.p['transit_spacing']) for i,r in enumerate(ROBOTS[1:],1)}
        return self.mode_targets if self.state=='MODE_TRANSITION' and self.mode_targets is not None else self.targets

    def publish_markers(self):
        desired=self.formation_targets()
        array=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
        def marker(mid,label,pose,color,kind):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.ns='fleet_mission';m.id=mid;m.type=kind;m.action=Marker.ADD;m.pose=self.pose_message(pose).pose;m.pose.position.z=.3;m.scale.x=.3;m.scale.y=.3;m.scale.z=.3;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=label
            if kind==Marker.TEXT_VIEW_FACING:m.scale.z=.35
            return m
        if self.primary:array.markers.append(marker(1,'Primary goal',self.primary,(1.,.3,.1),Marker.SPHERE))
        for i,r in enumerate(ROBOTS):
            if r in self.poses:array.markers.append(marker(10+i,r+(' (primary)' if r=='tb1' else ''),(self.poses[r][0],self.poses[r][1]-.5,self.poses[r][2]),(.1,.35,.75),Marker.TEXT_VIEW_FACING))
            if r in desired:
                array.markers.append(marker(20+i,r+' target',desired[r],(.1,1.,.2),Marker.ARROW))
                array.markers.append(marker(30+i,r+' slot',(desired[r][0],desired[r][1]+.5,desired[r][2]),(.05,.5,.1),Marker.TEXT_VIEW_FACING))
                outline=marker(100+i,'',desired[r],(.1,1.,.2),Marker.LINE_STRIP)
                outline.scale.x=.04;outline.pose.position.z=.08
                for x,y in self.polygons[r]+[self.polygons[r][0]]:
                    point=Point();point.x=float(x);point.y=float(y);outline.points.append(point)
                array.markers.append(outline)
        if self.requested_mode in ('LEGACY','OPEN_RECTANGLE','COMPACT_RECTANGLE') and all(r in self.poses for r in ROBOTS):
            boundary=marker(200,'',(0.,0.,0.),(.2,.8,1.),Marker.LINE_STRIP);boundary.scale.x=.08
            order=boundary_order(self.poses)
            for r in order+[order[0]]:
                point=Point();point.x=float(self.poses[r][0]);point.y=float(self.poses[r][1]);boundary.points.append(point)
            array.markers.append(boundary)
        if self.requested_mode=='WALL_WORKSPACE' and self.mode_anchor and self.wall_verified():
            work=marker(210,'Intended wall workspace',(0.,0.,0.),(1.,.5,.1),Marker.LINE_STRIP);work.scale.x=.08
            nx,ny=rotate(1.,0.,self.p['wall_normal_yaw']);tx,ty=-ny,nx
            leader=self.mode_anchor;depth=(leader[0]-self.p['wall_x'])*nx+(leader[1]-self.p['wall_y'])*ny
            width=self.p['wall_half_width']
            for d,side in ((0,1),(-depth,1),(-depth,-1),(0,-1),(0,1)):
                point=Point();point.x=leader[0]+d*nx+side*width*tx;point.y=leader[1]+d*ny+side*width*ty;work.points.append(point)
            array.markers.append(work)
        self.markers.publish(array)
        msg=String();msg.data=json.dumps({'simulation_time':self.clock_value,'requested_mode':self.requested_mode,'active_mode':self.active_mode,'assembly_stage':self.assembly_stage,'assembly_robot':self.transition_robot,'assembly_route':self.assembly_route,'assembly_routes':self.assembly_routes,'transition_state':self.transition_state,'transition_seconds':(time.monotonic()-self.transition_started if self.transition_started and self.transition_state in ('CANCELLING','ASSEMBLING') else self.transition_duration),'transition_progress':sum(self.arrived(r,t) for r,t in desired.items())/max(1,len(desired)),'formation_error':{r:distance(self.poses[r],t) for r,t in desired.items() if r in self.poses},'position_source':'map TF; simulation ground truth verified separately','recovering_robots':self.trail_recovering,'recovery_waypoints':self.trail_detours,'ground_truth_poses':self.physical_poses,'localization_error':{r:distance(p,self.physical_poses[r]) for r,p in self.poses.items() if r in self.physical_poses},'minimum_footprint_clearance':min((footprint_clearance(self.physical_poses[a],self.polygons[a],self.physical_poses[b],self.polygons[b]) for i,a in enumerate(ROBOTS) for b in ROBOTS[i+1:] if a in self.physical_poses and b in self.physical_poses),default=None),'state':self.state,'reason':self.reason,'phase':self.phase,'manual_navigation':self.manual,'mission':self.mission_number,'poses':self.poses,'desired_slots':desired,'targets':self.targets,'unresolved_pairs':list(self.unresolved_pairs),'blocked':sorted(self.blocked),'yielding_to':self.yielding,'clearance_maneuver':self.clearance_maneuver,'rolling_turn_active':getattr(self,'rolling_turn_active',False),'obstacle_recoveries':self.obstacle_recoveries,'waiting_for_followers':sorted(self.boundary_holds),'formation_heading':self.boundary_heading,'flexible_boundary':self.p['flexible_boundary'],'follower_slots':getattr(self,'flexible_slots',{}),'slot_updates':{r:{'prepared':s.get('updates_prepared',0),'switched':s.get('updates_switched',0)} for r,s in self.slots.items()},'actions':{r:s['status'] for r,s in self.slots.items()}});self.status_pub.publish(msg)

    def shutdown(self):
        self.cancel_all()
        for _ in range(5):
            for pub in self.cmd_pubs.values():pub.publish(Twist())
            rclpy.spin_once(self,timeout_sec=.1)


def main(args=None):
    rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO);node=FleetMissionController()
    try:rclpy.spin(node)
    except KeyboardInterrupt:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
    finally:
        if rclpy.ok():node.shutdown()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
