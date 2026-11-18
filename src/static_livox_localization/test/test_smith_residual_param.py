"""~smith_residual survives rosrun's YAML parsing.

On 2026-11-16 the follower died at start-up: the launch scripts pass
_smith_residual:=off, YAML 1.1 reads a bare `off` as False, and the mode
check saw 'False'. The scripts' default must start the follower.
"""

import importlib.util
from pathlib import Path

import pytest
import yaml


SCRIPTS = Path(__file__).parents[1] / "scripts"


def load(name):
    spec = importlib.util.spec_from_file_location(
        name, SCRIPTS / ("%s.py" % name))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


dwa_follower = load("dwa_follower")
mode = dwa_follower.smith_residual_mode


@pytest.mark.parametrize("text, expected", [
    ("off", "off"), ("wheel", "wheel"), ("imu", "imu"),
])
def test_every_script_value_parses_as_rosrun_hands_it_over(text, expected):
    # rosrun _name:=value -> yaml.safe_load(value)
    assert mode(yaml.safe_load(text)) == expected
    assert mode(yaml.safe_load(text)) in dwa_follower.SMITH_RESIDUAL_MODES


def test_the_default_is_a_valid_mode():
    assert mode(dwa_follower.SMITH_RESIDUAL_DEFAULT) == "off"


@pytest.mark.parametrize("value", [True, "on", "yes", 1, ""])
def test_ambiguous_values_are_not_modes(value):
    assert mode(value) not in dwa_follower.SMITH_RESIDUAL_MODES
