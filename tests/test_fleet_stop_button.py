import sys,time
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_live_status import FleetStatus
from std_srvs.srv import Trigger


def panel(available=True,response=None):
    requests=[]
    future=NS(done=lambda:False,cancel=lambda:None)
    node=NS(stop_future=None,stop_message='',stop_started=0.,
        stop_client=NS(service_is_ready=lambda:available,call_async=lambda request:requests.append(request) or future))
    return node,requests,future


def test_stop_calls_existing_service_once_without_blocking():
    n,requests,f=panel()
    FleetStatus.request_stop(n);FleetStatus.request_stop(n)
    assert len(requests)==1 and isinstance(requests[0],Trigger.Request)
    assert n.stop_future is f and 'Stopping' in n.stop_message
    FleetStatus.poll_stop(n)
    assert n.stop_future is f


def test_stop_confirmation_requires_successful_response():
    n,_,f=panel();FleetStatus.request_stop(n)
    f.done=lambda:True;f.result=lambda:Trigger.Response(success=True,message='Operator stop')
    FleetStatus.poll_stop(n)
    assert n.stop_future is None and n.stop_message.startswith('Goal stopped.')
    FleetStatus.request_stop(n)
    f.result=lambda:Trigger.Response(success=False,message='unavailable')
    FleetStatus.poll_stop(n)
    assert 'Stop rejected' in n.stop_message


def test_unavailable_and_timed_out_stop_are_reported():
    n,requests,_=panel(False);FleetStatus.request_stop(n)
    assert not requests and 'unavailable' in n.stop_message
    n,_,f=panel();FleetStatus.request_stop(n);n.stop_started=time.monotonic()-9
    cancelled=[];f.cancel=lambda:cancelled.append(True)
    FleetStatus.poll_stop(n)
    assert cancelled and n.stop_future is None and 'not confirmed' in n.stop_message
