// Copyright 2024 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>. Modified by Lihan Chen
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef ROG_MAP_ROS_HPP
#define ROG_MAP_ROS_HPP

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rog_map/rog_map.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "super_utils/color_msg_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace rog_map
{
using namespace super_utils;

class ROGMapROS : public rclcpp::Node, public ROGMap
{
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  const double getSystemWalltimeNow() override;

  void getSystemWalltimeNow(rclcpp::Time & _in);

  struct VisualizeMap
  {
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_pub, unknown_pub, esdf_neg_pub,
      esdf_occ_pub, occ_inf_pub, unknown_inf_pub, frontier_pub, esdf_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mkr_arr_pub;
    rclcpp::TimerBase::SharedPtr viz_timer;
    rclcpp::CallbackGroup::SharedPtr viz_reen_cbk_group;
  } vm_;

  struct ROSCallback
  {
    rclcpp::CallbackGroup::SharedPtr odom_me_cbk_group, cloud_me_cbk_group, update_cbk_group;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub;
    int unfinished_frame_cnt{0};
    Pose pc_pose;
    PointCloud pc;
    rclcpp::TimerBase::SharedPtr update_timer;
    mutex updete_lock;
  } rc_;

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);

  void updateCallback();

  void vizCallback();

  void vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud);

public:
  typedef shared_ptr<ROGMapROS> Ptr;

  explicit ROGMapROS(const rclcpp::NodeOptions & options);

private:
  inline void visualizeBoundingBox(
    visualization_msgs::msg::MarkerArray & mkrarr, const double & stamp, const Vec3f & box_min,
    const Vec3f & box_max, const string & ns, const Color & color, const double & size_x = 0.1,
    const double & alpha = 1.0, const bool & print_ns = true);

  inline void visualizeText(
    visualization_msgs::msg::MarkerArray & mkr_arr, const double & stamp, const std::string & ns,
    const std::string & text, const Vec3f & position, const Color & c = Color::White(),
    const double & size = 0.6, const int & id = -1);

  inline void visualizePoint(
    visualization_msgs::msg::MarkerArray & mkr_arr, const double & stamp, const Vec3f & pt,
    Color color = Color::Pink(), std::string ns = "pt", double size = 0.1, int id = -1,
    const bool & print_ns = true);
};
}  // namespace rog_map
#endif  // ROG_MAP_ROS_HPP
