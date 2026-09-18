from pathlib import Path

import yaml


def test_simulation_markers_use_published_topic():
    path = Path(__file__).resolve().parents[1] / 'rviz' / 'lab_arm.rviz'
    config = yaml.safe_load(path.read_text())
    displays = config['Visualization Manager']['Displays']
    objects = next(item for item in displays if item['Name'] == 'Simulation Objects')
    assert objects['Topic']['Value'] == '/simulation/objects'
    assert objects['Topic']['Durability Policy'] == 'Transient Local'
