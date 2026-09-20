import subprocess

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, GroupAction, OpaqueFunction
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def configure_usb_buffer(context):
    try:
        subprocess.run(
            ["sudo", "-n", "tee", "/sys/module/usbcore/parameters/usbfs_memory_mb"],
            input="512\n",
            text=True,
            capture_output=True,
            check=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError) as error:
        detail = getattr(error, "stderr", None) or str(error)
        raise RuntimeError(
            "Failed to set USB buffer to 512 MB. If sudo requires a password "
            "or permission is denied, run 'sudo -v' in the same terminal, "
            "enter your password, then rerun this launch command. "
            "If permission is still denied, ask your administrator to grant "
            "the required sudo permissions. Details: " + detail.strip()
        ) from error
    return []


def change_state(node, transition_id):
    return EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(node),
            transition_id=transition_id,
        )
    )


def generate_launch_description():
    config_file = LaunchConfiguration("config_file")
    robot_type = LaunchConfiguration("robot_type")
    autostart = LaunchConfiguration("autostart")
    start_self_check = LaunchConfiguration("start_self_check")

    default_config = PathJoinSubstitution(
        [
            FindPackageShare("orbbec_camera_lifecycle"),
            "config",
            PythonExpression(["'cameras-' + '", robot_type, "' + '.yaml'"]),
        ]
    )

    camera_node = LifecycleNode(
        package="orbbec_camera_lifecycle",
        executable="orbbec_camera_lifecycle_node",
        name="orbbec_camera_manager",
        namespace="",
        output="screen",
        parameters=[{"config_file": config_file}],
    )
    self_check_node = LifecycleNode(
        package="orbbec_camera_lifecycle",
        executable="camera_self_check",
        name="orbbec_self_test_node",
        namespace="",
        output="screen",
        parameters=[{"config_file": config_file}],
    )

    configure_camera = change_state(
        camera_node, Transition.TRANSITION_CONFIGURE
    )
    activate_camera = change_state(camera_node, Transition.TRANSITION_ACTIVATE)
    configure_self_check = change_state(
        self_check_node, Transition.TRANSITION_CONFIGURE
    )
    activate_self_check = change_state(
        self_check_node, Transition.TRANSITION_ACTIVATE
    )

    camera_autostart_handlers = [
        RegisterEventHandler(
            OnProcessStart(
                target_action=camera_node,
                on_start=[configure_camera],
            ),
            condition=IfCondition(autostart),
        ),
        RegisterEventHandler(
            OnStateTransition(
                target_lifecycle_node=camera_node,
                goal_state="inactive",
                entities=[activate_camera],
            ),
            condition=IfCondition(autostart),
        ),
    ]

    self_check_group = GroupAction(
        condition=IfCondition(start_self_check),
        actions=[
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=camera_node,
                    goal_state="active",
                    entities=[configure_self_check],
                ),
                condition=IfCondition(autostart),
            ),
            RegisterEventHandler(
                OnStateTransition(
                    target_lifecycle_node=self_check_node,
                    goal_state="inactive",
                    entities=[activate_self_check],
                ),
                condition=IfCondition(autostart),
            ),
            self_check_node,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "robot_type",
                default_value="d9",
                choices=["d5", "d7", "d9"],
                description="Robot configuration to load",
            ),
            DeclareLaunchArgument(
                "config_file",
                default_value=default_config,
                description="Camera and self-check configuration file",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description=(
                    "Configure and activate lifecycle nodes automatically"
                ),
            ),
            DeclareLaunchArgument(
                "start_self_check",
                default_value="true",
                description="Start the camera self-check lifecycle node",
            ),
            OpaqueFunction(function=configure_usb_buffer),
            *camera_autostart_handlers,
            self_check_group,
            camera_node,
        ]
    )
