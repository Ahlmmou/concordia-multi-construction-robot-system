import math
import xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np

MODEL=Path(__file__).resolve().parents[1]/'models/Hilti_4/Hilti_4.sdf'


def rotation(roll,pitch,yaw):
    cr,sr=math.cos(roll),math.sin(roll);cp,sp=math.cos(pitch),math.sin(pitch);cy,sy=math.cos(yaw),math.sin(yaw)
    return np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])@np.array([[cp,0,sp],[0,1,0],[-sp,0,cp]])@np.array([[1,0,0],[0,cr,-sr],[0,sr,cr]])


def test_all_four_physical_wheels_are_driven_once_with_consistent_pair_geometry():
    model=ET.parse(MODEL).getroot().find('model');plugin=model.find("plugin[@filename='libgazebo_ros_diff_drive.so']")
    count=int(plugin.findtext('num_wheel_pairs'));left=[e.text for e in plugin.findall('left_joint')];right=[e.text for e in plugin.findall('right_joint')]
    wheel_joints={j.attrib['name'] for j in model.findall('joint') if 'wheel' in j.attrib['name']}
    assert count==len(left)==len(right) and set(left+right)==wheel_joints and len(left+right)==len(wheel_joints)
    base=[float(x) for x in model.find("link[@name='base_link']/pose").text.split()];base_r=rotation(*base[3:])
    separations=[float(e.text) for e in plugin.findall('wheel_separation')];diameters=[float(e.text) for e in plugin.findall('wheel_diameter')]
    assert len(separations)==len(diameters)==count
    for l,r,sep,diameter in zip(left,right,separations,diameters):
        lp=[float(x) for x in model.find(f"joint[@name='{l}']/pose").text.split()];rp=[float(x) for x in model.find(f"joint[@name='{r}']/pose").text.split()]
        lpos=base_r@lp[:3];rpos=base_r@rp[:3]
        assert lpos[1]>rpos[1] # Imported CAD joint names are opposite robot left/right.
        assert abs(lpos[1]-rpos[1]-sep)<1e-4 and abs(lpos[0]-rpos[0])<1e-4
        axes=[]
        for name,pose in ((l,lp),(r,rp)):
            axis=[float(x) for x in model.find(f"joint[@name='{name}']/axis/xyz").text.split()]
            axes.append(base_r@rotation(*pose[3:])@axis)
        assert np.allclose(axes[0],axes[1],atol=1e-4) and np.allclose(axes[0],[0,1,0],atol=1e-4)
        assert diameter>0
