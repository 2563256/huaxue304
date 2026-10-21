import math
import time
from types import SimpleNamespace
from PIL import Image
from hetero_fleet_scheduler.heading_recovery import heading_sweep_clear
from hetero_fleet_scheduler.geometry import rectangle_is_free
from hetero_fleet_scheduler.task_model import RobotState,Pose2D
from test_goal_geometry import scene
from test_standoff import make_manager


def test_observed_omni_stall_pose_has_safe_full_body_alignment():
    occupancy,profiles=scene();p=profiles['robot_types']['omni_mecanum_amr']
    assert heading_sweep_clear(occupancy,(7.59,-.156,-.392),0.,p['footprint'],p['footprint_padding'])


def test_corner_collision_between_clear_rotation_endpoints_is_rejected():
    image=Image.new('L',(100,100),255)
    # Off the horizontal/vertical end bodies, inside the diagonal sweep.
    image.putpixel((64,35),0)
    occ=dict(px=image.load(),w=100,h=100,res=.05,ox=-2.5,oy=-2.5)
    footprint=(2.,.3)
    assert rectangle_is_free(occ,(0,0,0),footprint)
    assert rectangle_is_free(occ,(0,0,math.pi/2),footprint)
    assert not heading_sweep_clear(occ,(0,0,0),math.pi/2,footprint,0.)


def test_peer_inside_turning_envelope_blocks_recovery():
    occupancy,profiles=scene();p=profiles['robot_types']['omni_mecanum_amr']
    assert not heading_sweep_clear(occupancy,(7.59,-.156,-.392),0.,p['footprint'],p['footprint_padding'],
                                  [((7.8,-.25,0.),(.8,.8))])


def test_stale_pose_or_non_wheeled_never_sends_spin():
    manager=make_manager();robot=manager.robots['r1'];robot.type='omni_mecanum_amr'
    _,profiles=scene();manager.types=profiles['robot_types'];manager._live_xy=lambda name:(7.59,-.156,-.392)
    manager.live_pose_stamps={'r1':time.monotonic()-2.}
    class Never:
        def wait_for_server(self,**kw):raise AssertionError('unsafe Spin sent')
    manager.spin_clients={'r1':Never()}
    assert not manager._recover_heading(robot)
    robot.type='g1_humanoid'
    assert not manager._recover_heading(robot)


def recovery_fixture():
    from test_standoff import resolved,FakeGoalHandle
    manager=make_manager();robot=manager.robots['r1'];robot.type='omni_mecanum_amr'
    occupancy,profiles=scene();manager._occupancy=occupancy;manager.types=profiles['robot_types']
    pose=[7.59,-.156,-.392];manager._live_xy=lambda name:tuple(pose)
    manager.live_pose_stamps={'r1':time.monotonic()};manager.robots={'r1':robot}
    manager._now=lambda:100.
    calls=[]
    manager._request_target_path=lambda *a,**kw:calls.append(kw) or [object()]
    class Client:
        def wait_for_server(self,**kw):return True
        def send_goal_async(self,goal):
            calls.append(goal);pose[2]+=goal.target_yaw
            h=FakeGoalHandle('spin',None);h.result=resolved(SimpleNamespace(status=4))
            return resolved(h)
    manager.spin_clients={'r1':Client()}
    return manager,robot,calls


def test_heading_recovery_commits_route_and_uses_native_spin_without_transport_success():
    manager,robot,calls=recovery_fixture();before=robot.pose
    assert manager._recover_heading(robot)
    assert calls[0]=={'commit':True}
    assert abs(calls[1].target_yaw-.392)<1e-5
    assert robot.pose is before and robot.tasks_completed==0


def test_rejected_cbs_alignment_never_sends_spin():
    manager,robot,calls=recovery_fixture()
    manager._request_target_path=lambda *a,**kw:[]
    assert not manager._recover_heading(robot)
    assert not calls
