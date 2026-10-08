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
from nav_msgs.msg import OccupancyGrid, Odometry
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
                                candidates, trail_target, rotate, conservative_gate, boundary_followers_holding,
                                flexible_assignment, boundary_order)


class FleetMissionController(Node):
    def __init__(self):
        super().__init__('fleet_mission_controller')
        defaults = dict(rectangle_half_length=4.2, rectangle_half_width=3.4,
            transit_spacing=3.0, minimum_separation=0.4, footprint_padding=0.12,
            leader_speed=0.60, auxiliary_speed=0.80, angular_speed=0.50,
            update_distance=0.8, update_angle=0.5, minimum_goal_interval=4.0,
            arrival_tolerance=0.4, yaw_tolerance=0.45, alternative_radius=1.5,
            candidate_step=0.3, maximum_target_cost=70, data_timeout=15.0,
            command_timeout=2.0, clock_timeout=5.0, lifecycle_timeout=30.0,
            maximum_retries=3, retry_delay=8.0, maximum_follower_lag=5.0,
            action_response_timeout=30.0, progress_timeout=90.0,
            collision_horizon=2.0, manual_navigation=False, auto_form_rectangle=False,
            maintain_boundary=True, flexible_boundary=True, boundary_step=1.0, boundary_tolerance=0.8,
            verify_gazebo_localization=True, maximum_localization_error=0.35,
            hilti_footprint='[[-1.20,-0.85],[-1.20,0.85],[1.40,0.85],[1.40,-0.85]]',
            auxiliary_footprint='[[-0.30,-0.30],[-0.30,0.30],[0.475,0.30],[0.475,-0.30]]')
        for name,value in defaults.items():self.declare_parameter(name,value)
        self.p={name:self.get_parameter(name).value for name in defaults}
        self.polygons={r:ast.literal_eval(self.p['hilti_footprint'] if r=='tb1' else self.p['auxiliary_footprint']) for r in ROBOTS}
        self.radii={r:max(math.hypot(x,y) for x,y in poly) for r,poly in self.polygons.items()}
        if self.p['transit_spacing']<self.radii['tb1']+self.radii['tb2']+self.p['minimum_separation']:
            raise ValueError('transit_spacing is too small for the Hilti and auxiliary footprints')
        if min(self.p['boundary_step'],self.p['boundary_tolerance'],self.p['candidate_step'],self.p['minimum_goal_interval'],self.p['data_timeout'])<=0:raise ValueError('Intervals and candidate step must be positive')
        self.state='PREFLIGHT';self.reason='Waiting for verified live data';self.phase='IDLE'
        self.static_transforms={r:{} for r in ROBOTS}
        self.poses={};self.pose_stamp={};self.odom_frames={};self.buffers={r:Buffer(cache_time=Duration(seconds=30.0),node=self) for r in ROBOTS}
        self.grids={};self.grid_times={};self.scan_times={};self.commands={};self.clock_value=None;self.clock_time=0.
        self.lifecycle={};self.lifecycle_requests={};self.graph_problem='Graph not checked yet'
        self.boundary_waypoint=None;self.boundary_heading=None
        self.flexible_slots={};self.flexible_final_targets=None
        self.primary=None;self.pending_primary=None;self.targets={};self.rectangle_targets=None;self.trail=deque(maxlen=5000);self.trail_seeded=False
        self.boundary_holds=set();self.blocked=set();self.last_tick=0.;self.manual=self.p['manual_navigation'];self.mission_number=0
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
        self.subscriptions_kept.append(self.create_subscription(PoseStamped,'/formation/primary_goal',self.primary_goal,10))
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

    def feasible(self,robot,pose,reserved=None,peers=True):
        for topic in ('/map',f'/{robot}/global_costmap/costmap'):
            grid=self.grids.get(topic)
            if grid is None:return False
            local=self.in_frame(robot,pose,grid.frame)
            if local is None or not grid.footprint_free(local,self.polygons[robot],self.p['footprint_padding'],self.p['maximum_target_cost']):return False
        if peers:
            for other,current in self.poses.items():
                if other!=robot and distance(pose,current)<self.radii[robot]+self.radii[other]+self.p['minimum_separation']:return False
        for other,target in (reserved or {}).items():
            if other!=robot and distance(pose,target)<self.radii[robot]+self.radii[other]+self.p['minimum_separation']:return False
        return True

    def choose(self,robot,preferred,reserved):
        for pose in candidates(preferred,self.p['alternative_radius'],self.p['candidate_step']):
            if self.feasible(robot,pose,reserved):return pose
        return None

    def primary_goal(self,msg):
        if msg.header.frame_id.lstrip('/')!='map':self.get_logger().error('Primary goal must use the verified shared map frame');return
        q=msg.pose.orientation
        if not all(math.isfinite(v) for v in [msg.pose.position.x,msg.pose.position.y,q.x,q.y,q.z,q.w]) or abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>0.01:
            self.get_logger().error('Primary goal has invalid coordinates/quaternion');return
        target=(msg.pose.position.x,msg.pose.position.y,yaw(q))
        if not self.feasible('tb1',target,peers=False):self.get_logger().error('Primary goal is unknown, occupied, out of bounds, or too tight for Hilti footprint');return
        self.manual=False;self.pending_primary=target;self.rectangle_targets=None;self.targets={};self.trail.clear();self.trail_seeded=False
        self.mission_number+=1;self.cancel_all()
        for slot in self.slots.values():slot['retries']=0;slot['cooldown']=0.;slot['target']=None
        self.boundary_waypoint=None;self.boundary_heading=None
        self.flexible_slots={};self.flexible_final_targets=None
        self.phase='FORMING_BOUNDARY' if self.p['maintain_boundary'] else 'FORMING_LINE'
        self.set_state(self.phase,'New primary goal queued; waiting for previous actions to stop')

    def set_state(self,state,reason):
        if (state,reason)!=(self.state,self.reason):self.get_logger().info(f'{state}: {reason}')
        self.state,self.reason=state,reason

    def cancel(self,robot):
        slot=self.slots[robot]
        if slot['pending']:slot['cancel_when_accepted']=True;return
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
        self.get_logger().warning(f'{robot}: {reason}; retry {slot["retries"]}/{self.p["maximum_retries"]}')
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
                if changed and now-slot['last_send']>=self.p['minimum_goal_interval']:self.cancel(robot)
                return
            if slot['status']=='succeeded' and not changed:
                # A Nav2 result alone is insufficient: actual pose must meet tolerance.
                self.failure(robot,'Nav2 succeeded but actual pose is outside formation tolerance');return
        if now-slot['last_send']<self.p['minimum_goal_interval']:return
        if not self.feasible(robot,target,self.targets):return
        slot['pending']=True;slot['pending_since']=time.monotonic();slot['plan_token']+=1;token=slot['plan_token'];slot['cancel_when_accepted']=False
        request=ComputePathToPose.Goal();request.goal=self.pose_message(target);request.use_start=False
        future=self.planners[robot].send_goal_async(request)
        future.add_done_callback(partial(self.plan_accepted,robot=robot,target=target,token=token,mission=self.mission_number))

    def plan_accepted(self,future,robot,target,token,mission):
        try:
            handle=future.result()
            if not handle.accepted:self.failure(robot,'Planner rejected target');return
            future=handle.get_result_async();future.add_done_callback(partial(self.plan_done,robot=robot,target=target,token=token,mission=mission))
        except Exception as exc:self.failure(robot,str(exc))

    def plan_done(self,future,robot,target,token,mission):
        slot=self.slots[robot]
        if token!=slot['plan_token'] or mission!=self.mission_number or slot.get('cancel_when_accepted') or self.state in ('STOPPED','WAITING'):
            slot['pending']=False;return
        try:
            result=future.result()
            if result.status!=GoalStatus.STATUS_SUCCEEDED or not result.result.path.poses:self.failure(robot,'No feasible Nav2 path');return
            if not self.feasible(robot,target,self.targets):slot['pending']=False;return
            goal=NavigateToPose.Goal();goal.pose=self.pose_message(target)
            slot['pending_since']=time.monotonic();slot['target']=target;slot['last_send']=self.get_clock().now().nanoseconds*1e-9;slot['status']='sending'
            future=self.nav[robot].send_goal_async(goal)
            future.add_done_callback(partial(self.nav_accepted,robot=robot))
        except Exception as exc:self.failure(robot,str(exc))

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
        except Exception as exc:self.failure(robot,str(exc))

    def pose_message(self,pose):
        msg=PoseStamped();msg.header.frame_id='map';msg.header.stamp=self.get_clock().now().to_msg();msg.pose.position.x=float(pose[0]);msg.pose.position.y=float(pose[1]);msg.pose.orientation.z=math.sin(pose[2]/2);msg.pose.orientation.w=math.cos(pose[2]/2);return msg

    def arrived(self,robot,target):
        pose=self.poses.get(robot)
        return pose is not None and distance(pose,target)<=self.p['arrival_tolerance'] and abs(wrap(pose[2]-target[2]))<=self.p['yaw_tolerance']

    def idle_actions(self):return all(not s['pending'] and s['handle'] is None for s in self.slots.values())

    def tick(self):
        self.refresh_poses();ok,reason=self.ready()
        if not ok:
            if self.state not in ('STOPPED','WAITING'):self.set_state('PREFLIGHT',reason)
            self.publish_markers();return
        if self.state=='PREFLIGHT':
            self.set_state(self.phase if self.phase!='IDLE' else 'READY','Live data and Nav2 verified')
            if self.p['auto_form_rectangle'] and self.phase=='IDLE':self.phase='FORMING_RECTANGLE';self.set_state(self.phase,'Initial work formation')
        if self.state in ('STOPPED','WAITING'):self.publish_markers();return
        for robot,slot in self.slots.items():
            if slot['pending'] and time.monotonic()-slot.get('pending_since',time.monotonic())>self.p['action_response_timeout']:
                # A timed-out send may still be accepted remotely. Keep ownership,
                # stop all motor commands, and cancel if its late response arrives.
                slot['cancel_when_accepted']=True
                self.set_state('WAITING',f'{robot}: action response timeout; awaiting cancellation/result')
                self.publish_markers();return
            if slot['handle'] is not None and not slot['cancelling']:
                previous=slot.get('progress_pose')
                if previous is None or distance(previous,self.poses[robot])>.1:
                    slot['progress_pose']=self.poses[robot];slot['progress_sim']=self.clock_value
                elif self.clock_value-slot.get('progress_sim',self.clock_value)>self.p['progress_timeout']:
                    self.cancel_all();self.set_state('WAITING',f'{robot}: actual motion stalled; resume after resolving obstruction')
                    self.publish_markers();return
        if self.pending_primary is not None:
            if not self.idle_actions():self.publish_markers();return
            self.primary=self.pending_primary;self.pending_primary=None
        if self.state in ('FORMING_BOUNDARY','BOUNDARY_TRAVELLING') and self.primary:
            if self.p['flexible_boundary']:self.flexible_tick()
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
                self.trail=deque([(leader[0]-i*self.p['transit_spacing']*math.cos(heading),leader[1]-i*self.p['transit_spacing']*math.sin(heading),heading) for i in range(4,-1,-1)],maxlen=5000)
                self.trail_seeded=True;self.phase='TRAVELLING';self.set_state('TRAVELLING','Transit assembly verified from actual poses')
        elif self.state=='TRAVELLING' and self.primary:
            leader=self.poses['tb1']
            if not self.trail or distance(self.trail[-1],leader)>.1:self.trail.append(leader)
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
        if finished and getattr(self,'flexible_final_targets',None) is not None:
            targets.update(self.flexible_final_targets)
        else:
            slots=list(rectangle((leader[0],leader[1],self.boundary_heading),
                self.p['rectangle_half_length'],self.p['rectangle_half_width']).values())
            self.flexible_slots=flexible_assignment(self.poses,slots,getattr(self,'flexible_slots',{}))
            reserved={'tb1':leader}
            for r,i in self.flexible_slots.items():
                preferred=slots[i]
                heading=(math.atan2(preferred[1]-self.poses[r][1],preferred[0]-self.poses[r][0])
                    if distance(preferred,self.poses[r])>self.p['arrival_tolerance'] else self.poses[r][2])
                preferred=(preferred[0],preferred[1],heading)
                target=self.choose(r,preferred,reserved)
                if target is None:
                    self.cancel(r)
                    continue
                targets[r]=target;reserved[r]=target
            if finished and len(targets)==5:self.flexible_final_targets={r:t for r,t in targets.items() if r!='tb1'}
        self.targets=targets
        for r,target in targets.items():self.send(r,target)
        if finished and len(targets)==5 and all(self.arrived(r,t) for r,t in targets.items()):
            self.cancel_all()
            if self.idle_actions():
                self.phase='IDLE';self.set_state('READY','Primary goal and feasible follower positions reached')

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
        permitted=ok and (self.state in ('FORMING_LINE','TRAVELLING','FORMING_RECTANGLE','FORMING_BOUNDARY','BOUNDARY_TRAVELLING') or (self.manual and self.state=='READY'))
        commands={r:(0.,0.) for r in ROBOTS}
        self.boundary_holds=set()
        if permitted:
            for r in ROBOTS:
                msg,stamp=self.commands.get(r,(Twist(),0.))
                if now-stamp>self.p['command_timeout']:continue
                # Only owned action goals may move in mission mode.
                if not self.manual and self.slots[r]['handle'] is None:continue
                speed=self.p['leader_speed'] if r=='tb1' else self.p['auxiliary_speed']
                commands[r]=(max(-speed,min(speed,msg.linear.x)),max(-self.p['angular_speed'],min(self.p['angular_speed'],msg.angular.z)))
            if self.state=='BOUNDARY_TRAVELLING' and not self.p['flexible_boundary']:
                heading=self.boundary_heading if self.boundary_heading is not None else self.poses['tb1'][2]
                self.boundary_holds=boundary_followers_holding(self.poses,heading,
                    self.p['rectangle_half_length'],self.p['rectangle_half_width'],
                    self.p['boundary_tolerance'],commands['tb1'][0])
                if self.boundary_holds:commands['tb1']=(0.,commands['tb1'][1])
            if self.state=='FORMING_BOUNDARY' and not self.p['flexible_boundary']:commands['tb1']=(0.,0.)
            if self.state=='TRAVELLING':
                for i,r in enumerate(ROBOTS[:-1]):
                    if distance(self.poses[r],self.poses[ROBOTS[i+1]])>self.p['maximum_follower_lag']:commands[r]=(0.,0.)
            commands,self.blocked=conservative_gate(self.poses,commands,self.radii,self.p['minimum_separation'],self.p['collision_horizon'])
            for r,(v,w) in list(commands.items()):
                if abs(v)<1e-4 and abs(w)<1e-4:continue
                grid=self.grids[f'/{r}/local_costmap/costmap']
                # Check predicted footprint against lidar-populated local costmap.
                for dt in (.3,.6,1.):
                    pose=self.poses[r];dx,dy=rotate(v*dt,0,pose[2]);projected=(pose[0]+dx,pose[1]+dy,pose[2]+w*dt)
                    local=self.in_frame(r,projected,grid.frame)
                    if local is None or not grid.footprint_free(local,self.polygons[r],self.p['footprint_padding'],98):commands[r]=(0.,0.);self.blocked.add(r);break
        else:self.blocked=set()
        for r,(v,w) in commands.items():
            msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w);self.cmd_pubs[r].publish(msg)

    def stop_service(self,req,res):
        self.manual=False;self.pending_primary=None;self.cancel_all();self.set_state('STOPPED','Operator stop; zero motor commands');res.success=True;res.message=self.reason;return res
    def resume_service(self,req,res):
        self.manual=self.p['manual_navigation'] and self.phase=='IDLE'
        for slot in self.slots.values():slot['retries']=0;slot['cooldown']=0.;slot['status']='idle'
        self.set_state('PREFLIGHT','Rechecking readiness before resuming');res.success=True;res.message=self.reason;return res
    def rectangle_service(self,req,res):
        self.manual=False;self.primary=None;self.pending_primary=None;self.mission_number+=1;self.cancel_all();self.rectangle_targets=None;self.phase='FORMING_RECTANGLE';self.set_state('FORMING_RECTANGLE','Operator requested work rectangle');res.success=True;res.message=self.reason;return res

    def publish_markers(self):
        array=MarkerArray();clear=Marker();clear.action=Marker.DELETEALL;array.markers.append(clear)
        def marker(mid,label,pose,color,kind):
            m=Marker();m.header.frame_id='map';m.header.stamp=self.get_clock().now().to_msg();m.ns='fleet_mission';m.id=mid;m.type=kind;m.action=Marker.ADD;m.pose=self.pose_message(pose).pose;m.pose.position.z=.3;m.scale.x=.3;m.scale.y=.3;m.scale.z=.3;m.color.r,m.color.g,m.color.b=color;m.color.a=1.;m.text=label
            if kind==Marker.TEXT_VIEW_FACING:m.scale.z=.7
            return m
        if self.primary:array.markers.append(marker(1,'Primary goal',self.primary,(1.,.3,.1),Marker.SPHERE))
        for i,r in enumerate(ROBOTS):
            if r in self.poses:array.markers.append(marker(10+i,r,self.poses[r],(1.,1.,1.),Marker.TEXT_VIEW_FACING))
            if r in self.targets:
                array.markers.append(marker(20+i,r+' target',self.targets[r],(.1,1.,.2),Marker.ARROW))
                array.markers.append(marker(30+i,r+' desired',self.targets[r],(.1,1.,.2),Marker.TEXT_VIEW_FACING))
                outline=marker(100+i,'',self.targets[r],(.1,1.,.2),Marker.LINE_STRIP)
                outline.scale.x=.04;outline.pose.position.z=.08
                for x,y in self.polygons[r]+[self.polygons[r][0]]:
                    point=Point();point.x=float(x);point.y=float(y);outline.points.append(point)
                array.markers.append(outline)
        if all(r in self.poses for r in ROBOTS):
            boundary=marker(200,'',(0.,0.,0.),(.2,.8,1.),Marker.LINE_STRIP);boundary.scale.x=.08
            order=boundary_order(self.poses)
            for r in order+[order[0]]:
                point=Point();point.x=float(self.poses[r][0]);point.y=float(self.poses[r][1]);boundary.points.append(point)
            array.markers.append(boundary)
        center=self.poses.get('tb1',(0.,0.,0.));state=marker(50,self.state,center,(1.,1.,.2),Marker.TEXT_VIEW_FACING);state.pose.position.z=2.;array.markers.append(state)
        self.markers.publish(array)
        msg=String();msg.data=json.dumps({'state':self.state,'reason':self.reason,'phase':self.phase,'manual_navigation':self.manual,'mission':self.mission_number,'poses':self.poses,'targets':self.targets,'blocked':sorted(self.blocked),'waiting_for_followers':sorted(self.boundary_holds),'formation_heading':self.boundary_heading,'flexible_boundary':self.p['flexible_boundary'],'follower_slots':getattr(self,'flexible_slots',{}),'actions':{r:s['status'] for r,s in self.slots.items()}});self.status_pub.publish(msg)

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
