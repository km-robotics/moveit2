import launch
import launch_ros
import launch_testing
import os
import sys
import unittest
from launch_param_builder import ParameterBuilder

sys.path.append(os.path.dirname(__file__))


def generate_test_description():
    # Get parameters using the demo config file
    servo_params = (
        ParameterBuilder("moveit_servo")
        .yaml("config/panda_simulated_config.yaml", parameter_namespace="moveit_servo")
        .to_dict()
    )

    test_binary_dir_arg = launch.actions.DeclareLaunchArgument(
        name="test_binary_dir",
        description="Binary directory of package " "containing test executables",
    )

    servo_gtest = launch_ros.actions.Node(
        executable=launch.substitutions.PathJoinSubstitution(
            [
                launch.substitutions.LaunchConfiguration("test_binary_dir"),
                "unit_test_servo_calcs",
            ]
        ),
        parameters=[servo_params],
        output="screen",
        # prefix="kitty gdb -e run --args"
    )

    return (
        launch.LaunchDescription(
            [test_binary_dir_arg, servo_gtest, launch_testing.actions.ReadyToTest()]
        ),
        {"servo_gtest": servo_gtest},
    )


@launch_testing.post_shutdown_test()
class TestGTestProcessPostShutdown(unittest.TestCase):
    def test_gtest_pass(self, proc_info, servo_gtest):
        launch_testing.asserts.assertExitCodes(proc_info, process=servo_gtest)
