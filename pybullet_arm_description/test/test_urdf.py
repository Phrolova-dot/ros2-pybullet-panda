from pathlib import Path
import math
import xml.etree.ElementTree as ET

import xacro


EXPECTED_MOVABLE_JOINTS = {
    *(f'panda_joint{index}' for index in range(1, 8)),
    'panda_finger_joint1', 'panda_finger_joint2',
}


def test_xacro_expands_to_a_consistent_robot():
    xacro_path = Path(__file__).parents[1] / 'urdf' / 'lab_arm.urdf.xacro'
    document = xacro.process_file(str(xacro_path))
    root = ET.fromstring(document.toxml())

    links = [element.attrib['name'] for element in root.findall('link')]
    joints = root.findall('joint')
    movable = {
        joint.attrib['name']
        for joint in joints
        if joint.attrib['type'] in {'revolute', 'continuous', 'prismatic'}
    }

    assert root.attrib['name'] == 'panda'
    assert len(links) == len(set(links))
    assert movable == EXPECTED_MOVABLE_JOINTS
    assert {'base_link', 'tool0', 'tool_tip'}.issubset(links)

    for joint in joints:
        if joint.attrib['type'] not in {'revolute', 'prismatic'}:
            continue
        limit = joint.find('limit')
        assert limit is not None
        values = [float(limit.attrib[key]) for key in ('lower', 'upper', 'effort', 'velocity')]
        assert all(math.isfinite(value) for value in values)
        assert values[0] <= values[1]
        assert values[2] > 0.0
        assert values[3] > 0.0
