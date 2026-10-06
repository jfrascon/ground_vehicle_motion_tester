"""Check installed profile selection and the executor's ROS launch clock contract."""

import importlib.util
import json
from pathlib import Path
from types import ModuleType

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
import pytest
import ros2_launch_helpers as rlh
import yaml

PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def load_launch_module() -> ModuleType:
    """Load the source launch entry point without starting a process or publishing commands."""
    path = PACKAGE_ROOT / 'launch' / 'ground_vehicle_motion_executor.launch.py'
    spec = importlib.util.spec_from_file_location('motion_executor_launch', path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('simulated', ['True', 'False'])
@pytest.mark.parametrize('custom_params', [False, True])
@pytest.mark.parametrize('remap_topic', ['', 'controller/cmd_vel'])
def test_launch_constructs_real_node_and_resolves_parameters(
    simulated: str,
    custom_params: bool,
    remap_topic: str,
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    """Select the profile through parameter YAML while preserving clock and remappings."""
    module = load_launch_module()
    context = LaunchContext()
    context.launch_configurations['namespace'] = '/adapta/fl1'
    declarations = {
        action.name: action
        for action in module.generate_launch_description().entities
        if isinstance(action, DeclareLaunchArgument)
    }
    assert 'profile_file' not in declarations
    for declaration in declarations.values():
        declaration.visit(context)
    context.launch_configurations['use_sim_time'] = simulated
    if custom_params:
        params_file = tmp_path / 'executor_params.yaml'
        params_file.write_text(
            yaml.safe_dump(
                {
                    '/**/ground_vehicle_motion_executor': {
                        'ros__parameters': {
                            'profile_file': str(PACKAGE_ROOT / 'test/data/executor_profile.yaml'),
                            'frequency': 25.0,
                        }
                    }
                }
            ),
            encoding='utf-8',
        )
        context.launch_configurations['params_file'] = str(params_file)
    if remap_topic:
        context.launch_configurations['node_args'] = json.dumps(
            {
                'output': 'both',
                'ros_arguments': ['--log-level', 'info'],
                'remappings': [['cmd_vel', remap_topic]],
            }
        )
    captured: dict[str, object] = {}

    def capture_node(**kwargs: object) -> Node:
        """Record arguments while still constructing the production launch Node action."""
        captured.update(kwargs)
        return Node(**kwargs)

    monkeypatch.setattr(module, 'Node', capture_node)
    actions = module.launch_profile_executor(context)
    assert len(actions) == 1 and isinstance(actions[0], Node)
    assert captured['executable'] == 'ground_vehicle_motion_executor_node'
    assert captured['namespace'].perform(context) == '/adapta/fl1'
    expected_remappings = [('cmd_vel', remap_topic)] if remap_topic else None
    assert captured.get('remappings') == expected_remappings
    parameters = captured['parameters']
    assert isinstance(parameters[0], ParameterFile)
    evaluated_file = parameters[0].evaluate(context)
    try:
        contents = yaml.safe_load(Path(evaluated_file).read_text(encoding='utf-8'))
        defaults = contents['/**/ground_vehicle_motion_executor']['ros__parameters']
        assert Path(defaults['profile_file']).is_file()
        assert defaults['frequency'] == (25.0 if custom_params else 50.0)
        if custom_params:
            assert defaults['profile_file'] == str(
                PACKAGE_ROOT / 'test/data/executor_profile.yaml'
            )
        assert 'command_topic' not in defaults
    finally:
        parameters[0].cleanup()
    clock = parameters[1]['use_sim_time']
    assert isinstance(clock, ParameterValue)
    assert clock.evaluate(context) is (simulated == 'True')
    assert len(parameters) == 2


def test_namespace_is_owned_by_its_launch_argument() -> None:
    """Reject a second namespace source inside the shared JSON node arguments."""
    with pytest.raises(ValueError, match='namespace'):
        rlh.resolve_node_arguments('{"namespace":"other"}', extra_rejected_arguments={'namespace'})
