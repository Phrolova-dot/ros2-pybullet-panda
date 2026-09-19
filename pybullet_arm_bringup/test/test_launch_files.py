import importlib.util
from pathlib import Path

from launch import LaunchDescription
import pytest


LAUNCH_DIRECTORY = Path(__file__).parents[1] / 'launch'


@pytest.mark.parametrize(
    'filename',
    ('display.launch.py', 'simulation.launch.py', 'demo.launch.py', 'sorting.launch.py'),
)
def test_launch_file_generates_description(filename):
    path = LAUNCH_DIRECTORY / filename
    spec = importlib.util.spec_from_file_location(filename, path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    description = module.generate_launch_description()
    assert isinstance(description, LaunchDescription)
    assert len(description.entities) > 0
