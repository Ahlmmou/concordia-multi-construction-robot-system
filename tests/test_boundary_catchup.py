import sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import rectangle,boundary_followers_holding


def group(leader=(0.,0.,0.)):
    return {'tb1':leader,**rectangle((0.,0.,0.),4.2,3.4)}


def test_followers_already_ahead_do_not_deadlock_leader_catchup():
    poses=group()
    for r in ('tb2','tb3','tb4','tb5'):
        p=poses[r];poses[r]=(p[0]+1.,p[1],p[2])
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,.6)==set()


def test_lagging_follower_still_holds_translation():
    poses=group();p=poses['tb3'];poses['tb3']=(p[0]-1.,p[1],p[2])
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,.6)=={'tb3'}


def test_leader_can_turn_inside_fixed_rectangle():
    poses=group((0.,0.,math.pi/2))
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,0.)==set()
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,.6)==set()


def test_sideways_catchup_uses_actual_drive_direction():
    poses=group((0.,0.,-math.pi/2))
    for r in ('tb2','tb3','tb4','tb5'):
        p=poses[r];poses[r]=(p[0],p[1]-1.,p[2])
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,.6)==set()
    assert boundary_followers_holding(poses,0.,4.2,3.4,.8,-.6)==set(('tb2','tb3','tb4','tb5'))
