import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import rectangle, flexible_assignment, boundary_order
from fleet_mission_controller import FleetMissionController


def test_swapped_followers_keep_nearest_sides():
    slots=list(rectangle((0.,0.,0.),4.2,3.4).values())
    poses=dict(zip(('tb2','tb3','tb4','tb5'),(slots[2],slots[3],slots[0],slots[1])))
    assert flexible_assignment(poses,slots)=={'tb2':2,'tb3':3,'tb4':0,'tb5':1}
    assert set(boundary_order(poses))==set(poses)


def test_small_assignment_savings_do_not_churn_sides():
    slots=[(-1.,1.,0.),(1.,1.,0.),(-1.,-1.,0.),(1.,-1.,0.)]
    poses={'tb2':(.01,1.,0.),'tb3':(-.01,1.,0.),'tb4':slots[2],'tb5':slots[3]}
    previous={'tb2':0,'tb3':1,'tb4':2,'tb5':3}
    assert flexible_assignment(poses,slots,previous)==previous


def test_leader_outside_blue_polygon_moves_even_with_unavailable_corner():
    sent=[];cancelled=[]
    node=NS(poses={'tb1':(10.,0.,0.),**rectangle((0.,0.,0.),4.2,3.4)},
        primary=(20.,0.,0.),boundary_heading=None,flexible_slots={},flexible_final_targets=None,
        p={'rectangle_half_length':4.2,'rectangle_half_width':3.4,'arrival_tolerance':.4},
        arrived=lambda r,t:False,set_state=lambda *a:None,
        choose=lambda r,t,reserved:None if r=='tb2' else t,
        cancel=lambda r:cancelled.append(r),send=lambda r,t:sent.append((r,t)))
    FleetMissionController.flexible_tick(node)
    assert ('tb1',node.primary) in sent
    assert cancelled==['tb2']
    assert len(sent)==4


def test_final_follower_positions_are_frozen_after_leader_arrival():
    targets=rectangle((10.,0.,0.),4.2,3.4);sent=[]
    node=NS(poses={'tb1':(10.,0.,0.)},primary=(10.,0.,0.),boundary_heading=0.,
        flexible_final_targets=targets,arrived=lambda r,t:r=='tb1',
        set_state=lambda *a:None,send=lambda r,t:sent.append((r,t)))
    FleetMissionController.flexible_tick(node)
    assert node.targets=={'tb1':node.primary,**targets}


def test_final_slots_use_requested_position_and_heading_despite_leader_residual_error():
    import math
    primary=(10.,0.,math.pi/2)
    expected=rectangle(primary,4.2,3.4)
    # Followers start at the final positions with unrelated headings. They must
    # still receive the requested orientation, even without translation to do.
    poses={'tb1':(10.12,.02,math.pi/2-.04),
           **{r:(t[0],t[1],-.7) for r,t in expected.items()}}
    node=NS(poses=poses,primary=primary,boundary_heading=0.,flexible_slots={},
        flexible_final_targets=None,p={'rectangle_half_length':4.2,'rectangle_half_width':3.4,'arrival_tolerance':.15},
        arrived=lambda r,t:r=='tb1',set_state=lambda *a:None,choose=lambda r,t,reserved:t,
        send=lambda *a:None,cancel=lambda *a:None)
    FleetMissionController.flexible_tick(node)
    assert node.flexible_final_targets==expected
