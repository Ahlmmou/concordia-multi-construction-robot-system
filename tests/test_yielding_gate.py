import math
import sys
from pathlib import Path
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import ROBOTS,advance,footprint_clearance,motion_conflict,yielding_gate

BOX=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
LEADER=[(-1.2,-.85),(-1.2,.85),(1.4,.85),(1.4,-.85)]

def fleet():
    return ({r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)},
            {r:(0.,0.) for r in ROBOTS},{r:BOX for r in ROBOTS})

@pytest.mark.parametrize('aux',ROBOTS[1:])
def test_crossing_auxiliary_yields_to_hilti(aux):
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p[aux]=(1.5,-1.5,math.pi/2)
    c['tb1']=(1.,0.);c[aux]=(1.,0.)
    gated,blocked,yields,pairs=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==c['tb1'] and gated[aux]==(0.,0.)
    assert blocked=={aux} and yields=={aux:'tb1'} and not pairs
    # After Hilti passes the crossing, both commands are permitted.
    p[aux]=(1.5,2.,math.pi/2)
    gated,blocked,yields,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==c['tb1'] and not blocked and not yields


def test_auxiliary_crossing_has_stable_priority():
    p,c,polys=fleet();p['tb3']=(0.,0.,0.);p['tb5']=(1.5,-1.5,math.pi/2)
    c['tb3']=(1.,0.);c['tb5']=(1.,0.)
    for _ in range(3):
        gated,blocked,yields,_=yielding_gate(p,c,polys,.4,2.)
        assert gated['tb3']==c['tb3'] and gated['tb5']==(0.,0.)
        assert yields=={'tb5':'tb3'}


def test_head_on_does_not_drive_priority_robot_into_stopped_peer():
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p['tb4']=(2.,0.,math.pi)
    c['tb1']=(.8,0.);c['tb4']=(.8,0.)
    gated,blocked,yields,pairs=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==gated['tb4']==(0.,0.)
    assert pairs=={('tb1','tb4')} and blocked=={'tb1','tb4'}


def test_stationary_third_robot_still_protected():
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p['tb2']=(1.5,-1.5,math.pi/2);p['tb3']=(1.5,.7,0.)
    c['tb1']=(1.,0.);c['tb2']=(1.,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb2']==(0.,0.)
    for i,a in enumerate(ROBOTS):
        for b in ROBOTS[i+1:]:
            assert not motion_conflict(p[a],gated[a],polys[a],p[b],gated[b],polys[b],.4,2.)


def test_passing_long_leader_uses_actual_footprint():
    p,c,polys=fleet();polys['tb1']=LEADER;p['tb1']=(0.,0.,0.);p['tb4']=(0.,1.65,0.);c['tb4']=(.5,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated==c and not blocked


def test_rotating_leader_cannot_sweep_into_stationary_auxiliary():
    p,c,polys=fleet();polys['tb1']=LEADER;p['tb1']=(0.,0.,0.);p['tb4']=(0.,1.65,0.);c['tb1']=(0.,.7)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==(0.,0.) and 'tb1' in blocked


def test_existing_margin_overlap_can_escape_without_approaching_peer():
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p['tb2']=(.9,0.,0.);c['tb2']=(.3,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated==c and not blocked
    c['tb2']=(-.3,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb2']==(0.,0.)


def test_unicycle_prediction_and_rotated_clearance():
    assert advance((0.,0.,0.),(1.,1.),math.pi/2)==pytest.approx((1.,1.,math.pi/2))
    assert footprint_clearance((0.,0.,0.),BOX,(2.,0.,0.),BOX)==pytest.approx(1.4)


def test_reverse_escape_is_allowed_but_rear_peer_remains_protected():
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p['tb4']=(.9,0.,0.);c['tb1']=(-.3,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==(-.3,0.) and not blocked
    p['tb4']=(-1.5,0.,0.)
    gated,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==(0.,0.) and 'tb1' in blocked


def test_reversing_hilti_keeps_priority_over_crossing_auxiliary():
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p['tb3']=(-1.5,-1.5,math.pi/2)
    c['tb1']=(-1.,0.);c['tb3']=(1.,0.)
    gated,blocked,yields,pairs=yielding_gate(p,c,polys,.4,2.)
    assert gated['tb1']==c['tb1'] and gated['tb3']==(0.,0.)
    assert yields=={'tb3':'tb1'} and not pairs
    assert advance((0.,0.,0.),(-1.,1.),math.pi/2)==pytest.approx((-1.,-1.,math.pi/2))


@pytest.mark.parametrize('aux',ROBOTS[1:])
def test_extra_primary_clearance_applies_to_every_auxiliary(aux):
    p,c,polys=fleet();p['tb1']=(0.,0.,0.);p[aux]=(-2.,1.1,0.);c[aux]=(.5,0.)
    old,blocked,_,_=yielding_gate(p,c,polys,.4,2.)
    assert old[aux]==c[aux] and not blocked
    guarded,blocked,_,_=yielding_gate(p,c,polys,.4,2.,primary_gap=.6)
    assert guarded[aux]==(0.,0.) and aux in blocked
    # A wider passing lane is still allowed; this is not a blanket hold.
    p[aux]=(-2.,1.3,0.)
    guarded,blocked,_,_=yielding_gate(p,c,polys,.4,2.,primary_gap=.6)
    assert guarded[aux]==c[aux] and not blocked


def test_extra_primary_gap_does_not_widen_auxiliary_pairs():
    p,c,polys=fleet();p['tb2']=(0.,0.,0.);p['tb4']=(-2.,1.1,0.);c['tb4']=(.5,0.)
    guarded,blocked,_,_=yielding_gate(p,c,polys,.4,2.,primary_gap=.6)
    assert guarded['tb4']==c['tb4'] and not blocked
