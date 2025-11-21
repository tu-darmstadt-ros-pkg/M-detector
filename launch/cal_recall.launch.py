from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="false",
        description="Launch RViz2 with display config",
    )

    dataset_arg = DeclareLaunchArgument(
        "dataset",
        default_value="-1",
        description="Dataset ID (int)",
    )

    dataset_folder_arg = DeclareLaunchArgument(
        "dataset_folder",
        default_value="",
        description="Path to dataset folder",
    )

    start_param_arg = DeclareLaunchArgument(
        "start_param",
        default_value="-1",
        description="Start param index",
    )

    end_param_arg = DeclareLaunchArgument(
        "end_param",
        default_value="-1",
        description="End param index",
    )

    start_se_arg = DeclareLaunchArgument(
        "start_se",
        default_value="-1",
        description="Start sequence index",
    )

    end_se_arg = DeclareLaunchArgument(
        "end_se",
        default_value="-1",
        description="End sequence index",
    )

    is_origin_arg = DeclareLaunchArgument(
        "is_origin",
        default_value="false",
        description="Whether to use origin as reference",
    )

    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Run cal_recall under gdb",
    )

    dataset = LaunchConfiguration("dataset")
    dataset_folder = LaunchConfiguration("dataset_folder")
    start_param = LaunchConfiguration("start_param")
    end_param = LaunchConfiguration("end_param")
    start_se = LaunchConfiguration("start_se")
    end_se = LaunchConfiguration("end_se")
    is_origin = LaunchConfiguration("is_origin")
    debug = LaunchConfiguration("debug")

    cal_recall_prefix = [
        "gdb", "-ex", "run", "--args"
    ]

    cal_recall_node = Node(
        package="m_detector",
        executable="cal_recall",
        name="cal_recall",
        output="screen",
        parameters=[{
            "dyn_obj.dataset":        ParameterValue(dataset, value_type=int),
            "dyn_obj.dataset_folder": dataset_folder,
            "dyn_obj.start_param":    ParameterValue(start_param, value_type=int),
            "dyn_obj.end_param":      ParameterValue(end_param, value_type=int),
            "dyn_obj.start_se":       ParameterValue(start_se, value_type=int),
            "dyn_obj.end_se":         ParameterValue(end_se, value_type=int),
            "dyn_obj.is_origin":      ParameterValue(is_origin, value_type=bool),
        }],
    )

    pkg_share = get_package_share_directory("m_detector")
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "display.rviz"])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
        output="screen",
        prefix=["nice"],
    )

    rviz_group = GroupAction(
        condition=IfCondition(LaunchConfiguration("rviz")),
        actions=[rviz_node],
    )

    return LaunchDescription([
        rviz_arg,
        dataset_arg,
        dataset_folder_arg,
        start_param_arg,
        end_param_arg,
        start_se_arg,
        end_se_arg,
        is_origin_arg,
        debug_arg,
        cal_recall_node,
        rviz_group,
    ])
