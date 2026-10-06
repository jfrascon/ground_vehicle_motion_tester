import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch import LaunchDescription
from launch import LaunchDescriptionEntity
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.utilities.type_utils import normalize_typed_substitution
from launch.utilities.type_utils import perform_typed_substitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.descriptions import ParameterValue
import ros2_launch_helpers as rlh


def generate_launch_description() -> LaunchDescription:
    """Declare the namespace, clock, executor parameter file, and node arguments."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace', description='Full robot namespace supplied by the caller.'
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=os.path.join(
                    get_package_share_directory('ground_vehicle_motion_tester'),
                    'config',
                    'example_executor_params.yaml',
                ),
                description='ROS parameter YAML selecting the profile and frequency.',
            ),
            DeclareLaunchArgument(
                'params_file_allow_substs',
                default_value='True',
                choices=['True', 'true', 'False', 'false'],
                description='Resolve ROS launch substitutions in the parameter YAML.',
            ),
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='False',
                choices=['True', 'true', 'False', 'false'],
                description='Let ROS use the simulator clock when true.',
            ),
            DeclareLaunchArgument(
                'node_args',
                default_value='{"output":"both","ros_arguments":["--log-level","info"]}',
                description=rlh.LAUNCH_ACTION_ARGUMENTS_DESC,
            ),
            rlh.RequireFile(path=LaunchConfiguration('params_file')),
            OpaqueFunction(function=launch_profile_executor),
        ]
    )


def launch_profile_executor(ctx: LaunchContext) -> list[LaunchDescriptionEntity]:
    """Load the executor parameter file and apply the launch-owned clock setting."""
    allow_substs = perform_typed_substitution(
        ctx,
        normalize_typed_substitution(LaunchConfiguration('params_file_allow_substs'), bool),
        bool,
    )
    parameters = [
        ParameterFile(LaunchConfiguration('params_file'), allow_substs=allow_substs),
        {'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)},
    ]
    return [
        Node(
            package='ground_vehicle_motion_tester',
            executable='ground_vehicle_motion_executor_node',
            namespace=LaunchConfiguration('namespace'),
            parameters=parameters,
            **rlh.resolve_node_arguments(
                LaunchConfiguration('node_args').perform(ctx),
                default_arguments={'name': 'ground_vehicle_motion_executor'},
                extra_rejected_arguments={'namespace'},
            ),
        )
    ]
