import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_status_text import describe


def test_stopped_fleet_invites_new_goal_without_showing_old_goal_distance():
    state={'state':'STOPPED','poses':{'tb1':(0.,0.,0.)}}
    view=describe(state,{},(8.,0.))
    assert 'new 2D Goal Pose' in view['detail']
    assert view['remaining'] is None
    assert all(row['label']=='On hold' for row in view['rows'])


def test_assembly_identifies_first_robot_still_adjusting():
    state={'state':'FORMING_BOUNDARY','poses':{'tb2':(4.,3.,0.),'tb3':(-3.,-3.,0.)},'targets':{'tb2':(4.,3.,0.),'tb3':(-4.,-3.,0.)}}
    view=describe(state,{'tb3':(.2,0.)})
    assert view['detail'].startswith('tb3 is adjusting')
    assert next(r for r in view['rows'] if r['robot']=='tb3')['label']=='Moving'


def test_travel_reports_wait_for_follower_after_leader_reaches_stage():
    state={'state':'BOUNDARY_TRAVELLING','poses':{'tb1':(1.,0.,0.),'tb2':(4.,3.,0.)},'targets':{'tb1':(1.,0.,0.),'tb2':(5.,3.,0.)}}
    view=describe(state,{},(5.,0.))
    assert view['remaining']==4.
    assert 'waiting for tb2' in view['detail']


def test_clearance_pause_overrides_normal_moving_message():
    view=describe({'state':'BOUNDARY_TRAVELLING','blocked':['tb4']},{})
    assert 'holding tb4' in view['detail']


def test_turning_is_distinguished_from_stationary_robot():
    view=describe({'state':'FORMING_BOUNDARY'},{'tb2':(0.,.3)})
    assert view['rows'][1]['label']=='Turning'


def test_boundary_obstruction_has_actionable_explanation():
    view=describe({'state':'WAITING','reason':'Whole boundary cannot fit along the next segment; choose a goal in open space'},{})
    assert 'wider route' in view['detail']


def test_live_progress_explains_turn_in_place():
    view=describe({'state':'BOUNDARY_TRAVELLING'},{'tb1':(0.,-.3)})
    assert 'turning to face' in view['detail']
    assert view['rows'][0]['label']=='Turning'


def test_yielding_explains_automatic_resume_and_identifies_both_robots():
    view=describe({'state':'BOUNDARY_TRAVELLING','blocked':['tb1'],'yielding_to':{'tb1':'tb4'}},{'tb4':(.3,0.)})
    assert 'tb1 is yielding to tb4' in view['detail']
    assert 'automatically' in view['detail']
    assert view['rows'][0]['label']=='Yielding to tb4'
    assert view['rows'][3]['label']=='Moving'


def test_clearance_maneuver_has_distinct_live_status():
    view=describe({'state':'BOUNDARY_TRAVELLING','yielding_to':{'tb3':'tb2'},'clearance_maneuver':{'robot':'tb2','peer':'tb3','target':(1,2,0)}},{})
    assert 'tb3 holds while tb2 moves' in view['detail']
    assert view['rows'][1]['label']=='Clearing the route'


def test_backward_motion_is_identified_as_reversing():
    view=describe({'state':'BOUNDARY_TRAVELLING'},{'tb1':(-.2,.1),'tb4':(-.3,0.)})
    assert view['rows'][0]['label']=='Reversing'
    assert view['rows'][3]['label']=='Reversing'
    assert view['rows'][0]['speed']==.2


def test_rolling_turn_is_visible_in_live_progress():
    state={'state':'BOUNDARY_TRAVELLING','rolling_turn_active':True}
    view=describe(state,{'tb1':(.18,.2)})
    assert 'gentle forward arc' in view['detail'] and view['rows'][0]['label']=='Rolling turn'
    view=describe(state,{'tb1':(-.18,.2)})
    assert 'backward arc' in view['detail'] and view['rows'][0]['label']=='Reversing turn'
