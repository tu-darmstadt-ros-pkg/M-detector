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
        default_value="true",
        description="Launch RViz2 with demo config",
    )

    time_file_arg = DeclareLaunchArgument(
        "time_file",
        default_value="",
        description="Path to time log file",
    )

    out_path_arg = DeclareLaunchArgument(
        "out_path",
        default_value="",
        description="Output path for dynamic object results",
    )

    out_origin_path_arg = DeclareLaunchArgument(
        "out_origin_path",
        default_value="",
        description="Output path for origin-based results",
    )

    pose_log_arg = DeclareLaunchArgument(
        "pose_log",
        default_value="false",
        description="Enable pose logging",
    )

    pose_log_file_arg = DeclareLaunchArgument(
        "pose_log_file",
        default_value="",
        description="Pose log file path",
    )

    cluster_out_file_arg = DeclareLaunchArgument(
        "cluster_out_file",
        default_value="",
        description="Cluster output file path",
    )

    time_breakdown_file_arg = DeclareLaunchArgument(
        "time_breakdown_file",
        default_value="",
        description="Time breakdown output file path",
    )

    time_file = LaunchConfiguration("time_file")
    out_path = LaunchConfiguration("out_path")
    out_origin_path = LaunchConfiguration("out_origin_path")
    pose_log = LaunchConfiguration("pose_log")
    pose_log_file = LaunchConfiguration("pose_log_file")
    cluster_out_file = LaunchConfiguration("cluster_out_file")
    time_breakdown_file = LaunchConfiguration("time_breakdown_file")

    pkg_share = get_package_share_directory("m_detector")

    avia_config_yaml = PathJoinSubstitution(
        [pkg_share, "config", "avia", "avia0.yaml"]
    )

    dynfilter_node = Node(
        package="m_detector",
        executable="dynfilter",
        name="dynfilter",
        output="screen",
        parameters=[
            avia_config_yaml,
            {
                "dyn_obj.out_file":             out_path,
                "dyn_obj.out_file_origin":      out_origin_path,
                "dyn_obj.time_file":            time_file,
                "dyn_obj.pose_log":             ParameterValue(pose_log, value_type=bool),
                "dyn_obj.pose_log_file":        pose_log_file,
                "dyn_obj.cluster_out_file":     cluster_out_file,
                "dyn_obj.time_breakdown_file":  time_breakdown_file,
            },
        ],
    )

    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "demo.rviz"])

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
        time_file_arg,
        out_path_arg,
        out_origin_path_arg,
        pose_log_arg,
        pose_log_file_arg,
        cluster_out_file_arg,
        time_breakdown_file_arg,
        dynfilter_node,
        rviz_group,
    ])

