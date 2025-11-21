from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Launch RViz2 with display config",
    )

    pred_file_arg = DeclareLaunchArgument(
        "pred_file",
        default_value="",
        description="Path to prediction label files",
    )

    pc_file_arg = DeclareLaunchArgument(
        "pc_file",
        default_value="",
        description="Path to point cloud files (if used)",
    )

    pc_topic_arg = DeclareLaunchArgument(
        "pc_topic",
        default_value="",
        description="PointCloud2 topic to subscribe to (overrides YAML if set)",
    )

    pred_file = LaunchConfiguration("pred_file")
    pc_file = LaunchConfiguration("pc_file")
    pc_topic = LaunchConfiguration("pc_topic")

    pkg_share = get_package_share_directory("m_detector")

    kitti_config_yaml = PathJoinSubstitution(
        [pkg_share, "config", "kitti", "kitti.yaml"]
    )

    display_prediction_node = Node(
        package="m_detector",
        executable="display_prediction",
        name="display_prediction",
        output="screen",
        parameters=[
            kitti_config_yaml,
            {
                "dyn_obj.pred_file": pred_file,
                "dyn_obj.pc_file":   pc_file,
                "dyn_obj.pc_topic":  pc_topic,
            },
        ],
    )

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
        pred_file_arg,
        pc_file_arg,
        pc_topic_arg,
        display_prediction_node,
        rviz_group,
    ])
