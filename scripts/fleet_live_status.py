#!/usr/bin/env python3
"""Live fleet updates and one-click mission stop alongside RViz."""
import time
import json
from collections import deque
import tkinter as tk
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rcl_interfaces.msg import Log
from visualization_msgs.msg import MarkerArray
from fleet_status_text import describe,ROBOTS


class FleetStatus(Node):
    def __init__(self):
        super().__init__('fleet_live_status')
        self.state={};self.received=0.;self.motion={};self.goal=None;self.clocks=deque(maxlen=2000);self.notice=None
        self.requested_goal=None;self.request_number=0;self.request_mission=0;self.goal_confirmed=False;self.goal_rejected=None
        self.stop_client=self.create_client(Trigger,'/formation/stop')
        self.stop_future=None;self.stop_started=0.;self.stop_message=''
        self.subs=[self.create_subscription(String,'/formation/state',self.status,10),
                   self.create_subscription(PoseStamped,'/formation/primary_goal',self.goal_requested,10),
                   self.create_subscription(MarkerArray,'/formation/markers',self.markers,10),
                   self.create_subscription(Clock,'/clock',self.clock,qos_profile_sensor_data),
                   self.create_subscription(Log,'/rosout',self.log,10)]
        for robot in ROBOTS:
            self.subs.append(self.create_subscription(Odometry,f'/{robot}/odom',lambda msg,r=robot:self.odom(msg,r),qos_profile_sensor_data))
    def status(self,msg):
        try:
            self.state=json.loads(msg.data);self.received=time.monotonic()
            if self.requested_goal is not None and self.state.get('mission',0)>self.request_mission:
                self.goal=self.requested_goal;self.goal_confirmed=True
        except (ValueError,TypeError):pass
    def goal_requested(self,msg):
        self.requested_goal=(msg.pose.position.x,msg.pose.position.y)
        self.request_mission=self.state.get('mission',0);self.request_number+=1
        self.goal_confirmed=False;self.goal_rejected=None;self.notice=None
        self.stop_message=''

    def request_stop(self):
        if self.stop_future is not None:return
        if not self.stop_client.service_is_ready():
            self.stop_message='Stop unavailable: waiting for the fleet controller.';return
        try:
            self.stop_future=self.stop_client.call_async(Trigger.Request())
            self.stop_started=time.monotonic();self.stop_message='Stopping the current goal…'
        except Exception as exc:self.stop_message='Stop request failed: '+str(exc)

    def poll_stop(self):
        if self.stop_future is None:return
        if self.stop_future.done():
            try:
                result=self.stop_future.result()
                self.stop_message=('Goal stopped. Send a new goal in RViz when ready.'
                    if result.success else 'Stop rejected: '+result.message)
            except Exception as exc:self.stop_message='Stop request failed: '+str(exc)
            self.stop_future=None
        elif time.monotonic()-self.stop_started>8.:
            self.stop_future.cancel();self.stop_future=None
            self.stop_message='Stop not confirmed. Check fleet status and click again to retry.'

    def markers(self,msg):
        for marker in msg.markers:
            if marker.ns=='fleet_mission' and marker.id==1 and marker.action==0:
                position=(marker.pose.position.x,marker.pose.position.y)
                if self.goal_confirmed and self.requested_goal is not None and position!=self.requested_goal:continue
                self.goal=position
    def odom(self,msg,robot):self.motion[robot]=(time.monotonic(),msg.twist.twist.linear.x,msg.twist.twist.angular.z)
    def clock(self,msg):self.clocks.append((time.monotonic(),msg.clock.sec+msg.clock.nanosec/1e9))
    def log(self,msg):
        if msg.name.endswith('fleet_mission_controller') and msg.level>=40:
            self.notice=(time.monotonic(),msg.msg)
            if self.requested_goal is not None and msg.msg.startswith('Primary goal'):
                self.goal_rejected=msg.msg;self.goal_confirmed=False


def main():
    rclpy.init();node=FleetStatus();root=tk.Tk();root.title('Fleet — live updates');root.geometry(f'520x740+{max(0,root.winfo_screenwidth()-540)}+70');root.attributes('-topmost',True);root.configure(bg='#16202a');root.minsize(480,650)
    def label(text='',size=12,color='#d9e4ed',**kwargs):
        return tk.Label(root,text=text,font=('DejaVu Sans',size),bg='#16202a',fg=color,anchor='w',justify='left',**kwargs)
    label('FLEET LIVE UPDATES',11,'#91a9bc').pack(fill='x',padx=20,pady=(18,8))
    title=label(size=21);title.pack(fill='x',padx=20)
    detail=label(size=12,wraplength=475);detail.pack(fill='x',padx=20,pady=(8,10))
    goal_label=label(size=11,color='#7ed9ec',wraplength=475);goal_label.pack(fill='x',padx=20,pady=(0,8))
    progress=label(size=15,color='#7ed9ec');progress.pack(fill='x',padx=20,pady=(0,10))
    stop_button=tk.Button(root,text='Stop goal',command=node.request_stop,
        font=('DejaVu Sans',14,'bold'),bg='#b52e3d',fg='white',activebackground='#d33b4d',
        activeforeground='white',relief='flat',cursor='hand2',pady=10)
    stop_button.pack(fill='x',padx=20,pady=(0,5))
    stop_feedback=label(size=10,color='#f0c777',wraplength=475)
    stop_feedback.pack(fill='x',padx=20,pady=(0,8))
    rows={}
    for robot in ROBOTS:
        rows[robot]=label(size=12);rows[robot].pack(fill='x',padx=20,pady=5)
    clock=label(size=10,color='#91a9bc');clock.pack(fill='x',padx=20,pady=(12,8))
    label('RECENT UPDATES',10,'#91a9bc').pack(fill='x',padx=20,pady=(8,4))
    events=tk.Text(root,height=8,font=('DejaVu Sans',10),bg='#101820',fg='#d9e4ed',relief='flat',wrap='word',state='disabled');events.pack(fill='both',expand=True,padx=20,pady=(0,18))
    history=deque(maxlen=8);last_key=None;last_progress=0.;last_request_number=0;closed=False
    def update():
        nonlocal last_key,last_progress,last_request_number
        if closed:return
        if not rclpy.ok():close();return
        for _ in range(30):
            if not rclpy.ok():close();return
            rclpy.spin_once(node,timeout_sec=0.)
        node.poll_stop()
        stopping=node.stop_future is not None
        stopped=node.state.get('state')=='STOPPED'
        stop_button.config(text='Stopping…' if stopping else ('Goal stopped' if stopped else 'Stop goal'),
            state='disabled' if stopping or stopped else 'normal')
        stop_feedback.config(text=node.stop_message)
        now=time.monotonic();motion={r:(v,w) for r,(stamp,v,w) in node.motion.items() if now-stamp<3.}
        view=describe(node.state,motion,node.goal)
        stale=bool(node.received) and now-node.received>3.
        if stale:view.update(title='Waiting for live updates',detail='No recent fleet status received. The display will reconnect automatically.')
        color='#78dbb4' if not stale and view['phase'] in ('READY','BOUNDARY_TRAVELLING','TRAVELLING') else '#f0c777'
        title.config(text=view['title'],fg=color);detail.config(text=view['detail'])
        mission=node.state.get('mission',0)
        if node.requested_goal is not None:
            x,y=node.requested_goal
            confirmation='Rejected: '+node.goal_rejected if node.goal_rejected else ('Confirmed by fleet' if node.goal_confirmed else 'Sent — awaiting confirmation')
            goal_label.config(text=f'Latest goal: ({x:.2f}, {y:.2f}) · {confirmation}')
        elif node.goal is not None:
            goal_label.config(text=f'Goal #{mission}: ({node.goal[0]:.2f}, {node.goal[1]:.2f})')
        else:goal_label.config(text='No goal submitted yet.')
        progress.config(text=f"tb1: {view['remaining']:.1f} m from your goal" if view['remaining'] is not None else 'Use 2D Goal Pose in RViz to set a destination.')
        for row in view['rows']:
            speed=f"{row['speed']:.2f} m/s" if row['speed'] is not None else '—'
            distance=f" · {row['remaining']:.1f} m to stage target" if row['remaining'] is not None and view['phase'] not in ('STOPPED','READY') else ''
            rows[row['robot']].config(text=f"{row['robot']}   {row['label']}   {speed}{distance}")
        recent=[p for p in node.clocks if p[0]>now-5.];ratio=(recent[-1][1]-recent[0][1])/(recent[-1][0]-recent[0][0]) if len(recent)>1 else None
        clock.config(text=f'Simulation speed: {max(0.,ratio):.0%} of real time · speeds use simulation time' if ratio is not None else 'Waiting for simulation clock…')
        if node.request_number!=last_request_number:
            x,y=node.requested_goal
            history.appendleft(time.strftime('%H:%M:%S')+f'  New goal sent: ({x:.2f}, {y:.2f}). Waiting for fleet confirmation.')
            last_request_number=node.request_number
        key=(mission,view['title'],view['detail'])
        note=None
        if node.notice is not None and now-node.notice[0]<12.:note='Goal notice: '+node.notice[1];key=key+(note,)
        if key!=last_key:
            history.appendleft(time.strftime('%H:%M:%S')+'  '+(note or (f'Goal #{mission}: ' if mission else '')+view['title']+' — '+view['detail']));last_key=key
        elif view['remaining'] is not None and now-last_progress>15.:
            history.appendleft(time.strftime('%H:%M:%S')+f"  tb1 is {view['remaining']:.1f} m from the goal.");last_progress=now
        events.config(state='normal');events.delete('1.0','end');events.insert('1.0','\n\n'.join(history));events.config(state='disabled')
        root.after(250,update)
    def close():
        nonlocal closed
        closed=True;root.destroy()
    root.protocol('WM_DELETE_WINDOW',close)
    try:update();root.mainloop()
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':main()
