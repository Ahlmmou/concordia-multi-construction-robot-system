import sys
from pathlib import Path
from types import SimpleNamespace as NS
import array
import math
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import Grid,ROBOTS,rectangle,trail_target,candidates,conservative_gate


def grid(cells,width=20,resolution=.25,angle=0):
    q=NS(w=math.cos(angle/2),z=math.sin(angle/2),x=0.,y=0.)
    return Grid(NS(header=NS(frame_id='map'),info=NS(resolution=resolution,width=width,height=len(cells)//width,origin=NS(position=NS(x=0.,y=0.),orientation=q)),data=array.array('b',cells)))


POLY=[[-.3,-.3],[-.3,.3],[.5,.3],[.5,-.3]]


def test_footprint_rejects_wall_even_when_center_free():
    cells=[0]*400;cells[8*20+9]=100
    assert not grid(cells).footprint_free((2.,2.,0.),POLY,padding=.1)
    assert grid(cells).footprint_free((1.,1.,0.),POLY,padding=.1)


def test_unknown_boundaries_and_negative_index_are_rejected():
    cells=[0]*400;cells[8*20+8]=-1
    assert not grid(cells).footprint_free((2.,2.,0.),POLY)
    assert not grid([0]*400).footprint_free((-.1,2.,0.),POLY)
    assert not grid([0]*400).footprint_free((4.9,2.,0.),POLY)


def test_rotated_map_origin():
    g=grid([0]*400,angle=math.pi/2)
    assert g.footprint_free((-2.,2.,math.pi/2),POLY)
    assert not g.footprint_free((2.,2.,0.),POLY)


def test_rectangle_rotates_with_leader():
    r=rectangle((10.,20.,math.pi/2),4.,3.)
    assert r['tb2'][:2]==pytest.approx((7.,24.))
    assert r['tb3'][:2]==pytest.approx((13.,16.))
    assert len(r)==4


def test_follow_trail_round_corner_not_straight_through_wall():
    trail=[(0.,0.,0.),(4.,0.,0.),(4.,4.,math.pi/2)]
    assert trail_target(trail,6.)==pytest.approx((2.,0.,0.))
    assert trail_target(trail,2.)==pytest.approx((4.,2.,math.pi/2))


def test_candidate_search_bounded_and_preferred_first():
    points=list(candidates((0.,0.,0.),1.,.25))
    assert points[0]==(0.,0.,0.)
    assert all(math.hypot(x,y)<=1.+1e-8 for x,y,_ in points)


def test_crossing_commands_stop_both_and_stationary_peer_protected():
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses['tb1']=(0.,0.,0.);poses['tb2']=(2.,0.,math.pi)
    cmds={r:(0.,0.) for r in ROBOTS};cmds['tb1']=(.5,0.);cmds['tb2']=(.5,0.)
    radii={r:.5 for r in ROBOTS}
    gated,blocked=conservative_gate(poses,cmds,radii,.4,2.)
    assert gated['tb1']==gated['tb2']==(0.,0.)
    assert blocked=={'tb1','tb2'}
    cmds['tb2']=(0.,0.)
    gated,_=conservative_gate(poses,cmds,radii,.4,2.)
    assert gated['tb1']==(0.,0.)


def test_separating_motion_is_allowed_at_conservative_disc_overlap():
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses['tb1']=(0.,0.,math.pi);poses['tb2']=(1.3,0.,0.)
    cmds={r:(0.,0.) for r in ROBOTS};cmds['tb1']=(.1,0.);cmds['tb2']=(.1,0.)
    radii={r:.5 for r in ROBOTS}
    gated,blocked=conservative_gate(poses,cmds,radii,.4,2.)
    assert not blocked
    assert gated==cmds
