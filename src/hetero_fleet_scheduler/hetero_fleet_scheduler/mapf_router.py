"""Online CBS/EECBS route broker for the native swept-body Nav2 planner.

The planner supplies native, kinematically validated alternatives. Selected
routes really are returned by ComputePathToPose. Live reservations still veto
unsafe execution when tracking differs from nominal time predictions.
"""
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from ament_index_python.packages import get_package_share_directory
import yaml

from .cbs import Candidate, solve
from .reservations import ChannelReservations, intersects, angle
from .channel_node import odom_pose


def compile_route(path, body, speed, regions, tick=.5, headway=.5, grid=.2):
    """Full oriented-body sweeps, conservative space cells and section windows.

    Temporal cells include both endpoint bodies throughout each swept segment.
    There is no point-robot approximation. Adjacent cells merge only contiguous
    visits; repeated entries retain separate windows.
    """
    l,w,p=body; radius=math.hypot(l/2+p,w/2+p); samples=[tuple(path[0])];times=[0.]
    for a,b in zip(path,path[1:]):
        turn=angle(b[2]-a[2]);dist=math.dist(a[:2],b[:2])
        n=max(1,math.ceil((dist+radius*abs(turn))/.08))
        dt=max(dist/speed[0],abs(turn)/speed[1],1e-6)
        begin=times[-1]
        for k in range(1,n+1):
            f=k/n;samples.append((a[0]+f*(b[0]-a[0]),a[1]+f*(b[1]-a[1]),a[2]+f*turn));times.append(begin+dt*f)
    windows={};start=set()
    for i,pose in enumerate(samples):
        c,s=math.cos(pose[2]),math.sin(pose[2]);extra=p+.045
        hx=l/2+extra;hy=w/2+extra
        ax=abs(c)*hx+abs(s)*hy;ay=abs(s)*hx+abs(c)*hy
        resources=set()
        for x in range(math.floor((pose[0]-ax)/grid),math.ceil((pose[0]+ax)/grid)):
            for y in range(math.floor((pose[1]-ay)/grid),math.ceil((pose[1]+ay)/grid)):
                if intersects(pose,(l,w),extra,(x*grid,y*grid,(x+1)*grid,(y+1)*grid)):
                    resources.add(f'cell:{x}:{y}')
        for group,bounds in regions.items():
            if any(intersects(pose,(l,w),p+.075,b) for b in bounds):resources.add('section:'+group)
        a=max(0,math.floor(times[max(0,i-1)]/tick));b=max(a+1,math.ceil((times[min(i+1,len(times)-1)]+headway)/tick))
        for r in resources:
            spans=windows.setdefault(r,[])
            if spans and a<=spans[-1][1]:spans[-1]=(spans[-1][0],max(b,spans[-1][1]))
            else:spans.append((a,b))
        if i==0:start=resources
    return Candidate(max(1,math.ceil(times[-1]/tick)),tuple((r,a,b) for r,spans in windows.items() for a,b in spans),frozenset(start),path)


def committed_suffix(path, pose, decision):
    """Rebase only an actually moving commitment; a queue is a stopped body.

    In particular a queued robot cannot reserve the corridor ahead of its
    physical incumbent. The incumbent must first be able to leave.
    """
    if not path or decision.get('waiting',False) or not decision.get('allowed',False):
        return None
    cursor=min(range(len(path)),key=lambda k:math.dist(pose[:2],path[k][:2])+.1*abs(angle(pose[2]-path[k][2])))
    suffix=[pose]+path[cursor+1:]
    return suffix if len(suffix)>1 and math.dist(pose[:2],path[-1][:2])>.2 else None


class MapfRouter(Node):
    def __init__(self):
        super().__init__('mapf_router')
        cfgpath=self.declare_parameter('channels_file',str(Path(get_package_share_directory('hetero_fleet_scheduler'))/'config/channels.yaml')).value
        cfg=yaml.safe_load(Path(cfgpath).read_text());self.cfg=cfg
        self.mode=self.declare_parameter('mode',cfg.get('scheduling','eecbs')).value
        if self.mode not in ('cbs','eecbs'):raise ValueError('MAPF mode must be cbs or eecbs')
        profiles=yaml.safe_load((Path(get_package_share_directory('hetero_fleet_description'))/'config/robot_profiles.yaml').read_text())
        self.fleet={r['name']:r for r in profiles['fleet']};self.types=profiles['robot_types']
        self.bodies={n:(*self.types[r['type']]['footprint'],self.types[r['type']]['footprint_padding']) for n,r in self.fleet.items()}
        self.regions=ChannelReservations(cfg,self.bodies).regions
        self.tick=float(cfg.get('mapf_tick_sec',.5));self.headway=float(cfg.get('reservation_headway_sec',.5))
        self.channel={}
        self.create_subscription(String,"/channel_reservations",self.channel_snapshot,10)
        self.poses={};self.routes={};self.pending={};self.total=0;self.errors=0;self.alternative=0;self.started=time.monotonic()
        self.pub=self.create_publisher(String,'/mapf/decisions',20)
        self.stats=self.create_publisher(String,'/mapf/status',10)
        self.create_subscription(String,'/mapf/requests',self.request,20)
        for n,r in self.fleet.items():
            self.create_subscription(Odometry,'/'+n+'/odom',lambda m,n=n,r=r:self.poses.update({n:(odom_pose(m,r['spawn']),time.monotonic())}),qos_profile_sensor_data)
        self.create_timer(.15,self.update)
        self.get_logger().info(f'MAPF route broker: {self.mode}, finite native route catalogues, w={cfg.get("eecbs_weight",1.2)}')

    def channel_snapshot(self,message):
        try:self.channel=json.loads(message.data)
        except (ValueError,TypeError):pass

    def request(self,message):
        try:
            req=json.loads(message.data);name=req['robot'];paths=req['candidates']
            if name not in self.fleet or not paths or len(paths)>4:raise ValueError('unknown robot or invalid catalogue')
            if any(len(p)<1 or len(p)>20000 or any(len(q)!=3 or not all(math.isfinite(v) for v in q) for q in p) for p in paths):raise ValueError('invalid path')
            self.pending[name]=(req,time.monotonic())
        except (ValueError,KeyError,TypeError) as exc:self.get_logger().error(f'Invalid MAPF request: {exc}')

    def compile(self,name,path):
        profile=self.types[self.fleet[name]['type']]
        return compile_route(path,self.bodies[name],(profile['max_speed'],profile['max_omega']),self.regions,self.tick,self.headway)

    def update(self):
        if not self.pending:return
        pending=self.pending;self.pending={};now=self.get_clock().now().nanoseconds/1e9;wall=time.monotonic()
        catalogues={n:tuple(self.compile(n,p) for p in req['candidates']) for n,(req,_) in pending.items()}
        fixed=[]
        # Accepted, already executing routes form the committed prefix. Rebase
        # their prediction from measured progress, never reuse an old epoch.
        for n,(pose,stamp) in self.poses.items():
            if n in pending or wall-stamp>1.:continue
            suffix=committed_suffix(self.routes.get(n),pose,self.channel.get('robots',{}).get(n,{}))
            if suffix:
                fixed.extend(self.compile(n,suffix).occupancy);continue
            stationary=self.compile(n,[pose])
            # Coarse spatial cells may intersect beside a physically clear
            # parked body. Native costmaps already include the full parked body;
            # keep exact corridor ownership for idle peers here.
            fixed.extend((r,0,7200) for r in stationary.start_resources if r.startswith('section:'))
        started=time.perf_counter()
        plans,stats=solve(catalogues,self.mode,self.cfg.get('eecbs_weight',1.2),tuple(fixed),
                          self.cfg.get('cbs_max_nodes',512),self.cfg.get('cbs_budget_sec',.35))
        stats['elapsed_sec']=time.perf_counter()-started
        for n,(req,received) in pending.items():
            plan=plans.get(n) if plans else None
            index=req['candidates'].index(plan.candidate.payload) if plan else -1
            if plan:self.routes[n]=plan.candidate.payload
            self.total+=1;self.errors+=int(plan is None);self.alternative+=int(index>0)
            reply=dict(id=req['id'],robot=n,ok=plan is not None,index=index,
                epoch=now,not_before=now+(plan.departure*self.tick if plan else 0),mode=self.mode,stats=stats)
            self.pub.publish(String(data=json.dumps(reply)))
        self.stats.publish(String(data=json.dumps(dict(mode=self.mode,requests=self.total,rejected=self.errors,
            alternative_routes_selected=self.alternative,last_solver=stats,epoch=now))))


def main():
    rclpy.init();node=MapfRouter()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.try_shutdown()
