#include <rclcpp/rclcpp.hpp>
#include <omp.h>
#include <mutex>
#include <cmath>
#include <thread>
#include <fstream>
#include <iostream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <Eigen/Core>
#include <types.h>
#include <m-detector/DynObjFilter.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/vector3.hpp>
#include <pcl/filters/random_sample.h>
#include <Eigen/Eigen>
#include <tf2_eigen/tf2_eigen.hpp>

#include <deque>

// #include "preprocess.h"

using namespace std;

shared_ptr<DynObjFilter> DynObjFilt(new DynObjFilter());
M3D cur_rot = Eigen::Matrix3d::Identity();
V3D cur_pos = Eigen::Vector3d::Zero();

int     QUAD_LAYER_MAX  = 1;
int     occlude_windows = 3;
int     point_index = 0;
float   VER_RESOLUTION_MAX  = 0.01;
float   HOR_RESOLUTION_MAX  = 0.01;
float   angle_noise     = 0.001;
float   angle_occlude     = 0.02;
float   dyn_windows_dur = 0.5;
bool    dyn_filter_en = true, dyn_filter_dbg_en = true;
string  points_topic, odom_topic;
string  out_folder, out_folder_origin;
double  lidar_end_time = 0;
int     dataset = 0;
int     cur_frame = 0;

deque<M3D> buffer_rots;
deque<V3D> buffer_poss;
deque<double> buffer_times;
deque<PointCloudXYZI::Ptr> buffer_pcs;


rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_dyn_extend;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pcl_std;

void OdomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr cur_odom)
  {
    Eigen::Quaterniond cur_q;
    //geometry_msgs::Quaternion tmp_q;
    //tmp_q = cur_odom->pose.pose.orientation;
    tf2::fromMsg(cur_odom->pose.pose.orientation, cur_q);
    cur_rot = cur_q.toRotationMatrix();
    cur_pos << cur_odom->pose.pose.position.x, cur_odom->pose.pose.position.y, cur_odom->pose.pose.position.z;
    buffer_rots.push_back(cur_rot);
    buffer_poss.push_back(cur_pos);
    lidar_end_time = rclcpp::Time(cur_odom->header.stamp).seconds();
    buffer_times.push_back(lidar_end_time);
  }

void PointsCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg_in)
  {
    PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
    pcl::fromROSMsg(*msg_in, *feats_undistort);
    buffer_pcs.push_back(feats_undistort); 
  }


void TimerCallback()
  {
    if(buffer_pcs.size() > 0 && buffer_poss.size() > 0 && buffer_rots.size() > 0 && buffer_times.size() > 0)
    {
        PointCloudXYZI::Ptr cur_pc = buffer_pcs.at(0);
        buffer_pcs.pop_front();
        auto cur_rot = buffer_rots.at(0);
        buffer_rots.pop_front();
        auto cur_pos = buffer_poss.at(0);
        buffer_poss.pop_front();
        auto cur_time = buffer_times.at(0);
        buffer_times.pop_front();
        string file_name = out_folder;
        stringstream ss;
        ss << setw(6) << setfill('0') << cur_frame ;
        file_name += ss.str();
        file_name.append(".label");
        string file_name_origin = out_folder_origin;
        stringstream sss;
        sss << setw(6) << setfill('0') << cur_frame ;
        file_name_origin += sss.str(); 
        file_name_origin.append(".label");

        if(file_name.length() > 15 || file_name_origin.length() > 15)
            DynObjFilt->set_path(file_name, file_name_origin);

        DynObjFilt->filter(cur_pc, cur_rot, cur_pos, cur_time);
        DynObjFilt->publish_dyn(pub_pcl_dyn, pub_pcl_dyn_extend, pub_pcl_std, cur_time);
        cur_frame ++;
  }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("dynfilter_odom");
    points_topic = node->declare_parameter<std::string>("dyn_obj.points_topic", "");
    odom_topic = node->declare_parameter<std::string>("dyn_obj.odom_topic", "");
    out_folder = node->declare_parameter<std::string>("dyn_obj.out_file", "");
    out_folder_origin  = node->declare_parameter<std::string>("dyn_obj.out_file_origin", "");

    DynObjFilt = std::make_shared<DynObjFilter>();
    DynObjFilt->init(node);

    pub_pcl_dyn_extend = node->create_publisher<sensor_msgs::msg::PointCloud2>("/m_detector/frame_out", rclcpp::QoS(10));     // queue depth ~10 (statt 10000)
    pub_pcl_dyn = node->create_publisher<sensor_msgs::msg::PointCloud2>("/m_detector/point_out", rclcpp::QoS(10));
    pub_pcl_std = node->create_publisher<sensor_msgs::msg::PointCloud2>("/m_detector/std_points", rclcpp::QoS(10));

    using std::placeholders::_1;

    auto sub_pcl = node->create_subscription<sensor_msgs::msg::PointCloud2>(points_topic, rclcpp::SensorDataQoS(), &PointsCallback);

    auto sub_odom = node->create_subscription<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(50), &OdomCallback);

    auto timer = node->create_wall_timer(
      std::chrono::milliseconds(10),
      TimerCallback 
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
