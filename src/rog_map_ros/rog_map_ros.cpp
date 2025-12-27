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

#include "rog_map_ros/rog_map_ros.hpp"

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

namespace rog_map
{

const double ROGMapROS::getSystemWalltimeNow() { return this->get_clock()->now().seconds(); }

void ROGMapROS::getSystemWalltimeNow(rclcpp::Time & _in) { _in = this->get_clock()->now(); }

void ROGMapROS::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  updateRobotState(
    std::make_pair(
      Vec3f(
        odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z),
      Quatf(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z)));
}

void ROGMapROS::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
  if (!robot_state_.rcv) {
    std::cout << YELLOW << " -- [ROS] No odom received, skip cloud callback." << RESET << '\n';
    return;
  }
  double cbk_t = this->get_clock()->now().seconds();
  if (cbk_t - robot_state_.rcv_time > cfg_.odom_timeout) {
    std::cout << YELLOW << " -- [ROS] Odom timeout, skip cloud callback." << RESET << '\n';
    return;
  }
  PointCloud temp_pc;
  pcl::fromROSMsg(*cloud_msg, temp_pc);
  rc_.updete_lock.lock();
  rc_.pc = temp_pc;
  rc_.pc_pose = std::make_pair(robot_state_.p, robot_state_.q);
  rc_.unfinished_frame_cnt++;
  map_empty_ = false;
  rc_.updete_lock.unlock();
}

void ROGMapROS::updateCallback()
{
  if (map_empty_) {
    static double last_print_t = this->get_clock()->now().seconds();
    double cur_t = this->get_clock()->now().seconds();
    if (cfg_.ros_callback_en && (cur_t - last_print_t > 1.0)) {
      std::cout << YELLOW << " -- [ROG WARN] No point cloud input, check the topic name." << RESET
                << '\n';
      last_print_t = cur_t;
    }
    return;
  }
  if (rc_.unfinished_frame_cnt == 0) {
    return;
  }

  if (rc_.unfinished_frame_cnt > 1) {
    std::cout << YELLOW
              << " -- [ROG WARN] Unfinished frame cnt > 1, the map may not "
                 "work in real-time"
              << RESET << '\n';
  }
  static PointCloud temp_pc;
  static Pose temp_pose;

  rc_.updete_lock.lock();
  temp_pc = rc_.pc;
  temp_pose = rc_.pc_pose;
  rc_.unfinished_frame_cnt = 0;
  rc_.updete_lock.unlock();

  updateProbMap(temp_pc, temp_pose);

  writeTimeConsumingToLog(time_log_file_);
}

void ROGMapROS::vizCallback()
{
  if (!cfg_.visualization_en) {
    return;
  }
  if (map_empty_) {
    return;
  }

  Vec3f box_max = robot_state_.p + cfg_.visualization_range / 2;
  Vec3f box_min = robot_state_.p - cfg_.visualization_range / 2;

  boundBoxByLocalMap(box_min, box_max);
  if ((box_max - box_min).minCoeff() <= 0) {
    cout << YELLOW << " -- [ROGMap] Visualization range is too small." << RESET << '\n';
    return;
  }

  if (cfg_.pub_unknown_map_en && vm_.unknown_pub->get_subscription_count() >= 1) {
    vec_E<Vec3f> unknown_map, inf_unknown_map;
    boxSearch(box_min, box_max, UNKNOWN, unknown_map);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    vecEVec3fToPC2(unknown_map, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    vm_.unknown_pub->publish(cloud_msg);
    if (cfg_.unk_inflation_en && vm_.unknown_inf_pub->get_subscription_count() >= 1) {
      boxSearchInflate(box_min, box_max, UNKNOWN, inf_unknown_map);
      vecEVec3fToPC2(inf_unknown_map, cloud_msg);
      cloud_msg.header.stamp = this->get_clock()->now();
      vm_.unknown_inf_pub->publish(cloud_msg);
    }
  }

  if (cfg_.frontier_extraction_en && vm_.frontier_pub->get_subscription_count() >= 1) {
    vec_E<Vec3f> frontier_map;
    boxSearch(box_min, box_max, FRONTIER, frontier_map);
    sensor_msgs::msg::PointCloud2 cloud_msg;
    vecEVec3fToPC2(frontier_map, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    vm_.frontier_pub->publish(cloud_msg);
  }

  vec_E<Vec3f> occ_map, inf_occ_map;
  sensor_msgs::msg::PointCloud2 cloud_msg;

  if (vm_.occ_pub->get_subscription_count() >= 1) {
    boxSearch(box_min, box_max, OCCUPIED, occ_map);
    vecEVec3fToPC2(occ_map, cloud_msg);
    vm_.occ_pub->publish(cloud_msg);
  }

  if (vm_.occ_inf_pub->get_subscription_count() >= 1) {
    boxSearchInflate(box_min, box_max, OCCUPIED, inf_occ_map);
    vecEVec3fToPC2(inf_occ_map, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    vm_.occ_inf_pub->publish(cloud_msg);
  }

  /* visualize ESDF Map*/
  if (cfg_.esdf_en) {
    if (vm_.esdf_pub->get_subscription_count() >= 1) {
      PointCloud pc;
      esdf_map_->getPositiveESDFPointCloud(box_min, box_max, robot_state_.p.z() - 0.5, pc);
      pcl::toROSMsg(pc, cloud_msg);
      cloud_msg.header.frame_id = cfg_.frame_id;
      cloud_msg.header.stamp = this->get_clock()->now();
      vm_.esdf_pub->publish(cloud_msg);
    }

#ifdef ESDF_MAP_DEBUG
    esdf_map_->getESDFOccPC2(box_min, box_max, cloud_msg);
    cloud_msg.header.stamp = this->get_clock()->now();
    vm_.esdf_occ_pub->publish(cloud_msg);
#endif
  }

  /* Publish visualization range */
  visualization_msgs::msg::MarkerArray mkr_arr;
  visualizeBoundingBox(
    mkr_arr, this->get_clock()->now().seconds(), box_min, box_max, "Visualization Range",
    Color::Purple());
  visualizeText(
    mkr_arr, this->get_clock()->now().seconds(), "Visualization Range Text", "Visualization Range",
    box_max + Vec3f(0, 0, 0.5), Color::Purple(), 0.6, 0);

  /* Publish local map range */
  Vec3f local_map_max(999, 999, 999), local_map_min(-999, -999, -999);
  boundBoxByLocalMap(local_map_min, local_map_max);
  visualizeBoundingBox(
    mkr_arr, this->get_clock()->now().seconds(), local_map_min, local_map_max, "Local Map Range",
    Color::Orange());
  visualizeText(
    mkr_arr, this->get_clock()->now().seconds(), "Local Map Range Text", "Local Map Range",
    local_map_max + Vec3f(0, 0, 1.0), Color::Orange(), 0.6, 0);

  /* Publish Ray-casting range */
  visualizeBoundingBox(
    mkr_arr, this->get_clock()->now().seconds(), raycast_data_.cache_box_min,
    raycast_data_.cache_box_max, "Updating Range", Color::Green());
  visualizeText(
    mkr_arr, this->get_clock()->now().seconds(), "Updating Range Text", "Updating Range",
    raycast_data_.cache_box_max + Vec3f(0, 0, 0.5), Color::Green(), 0.6, 0);

  /* Publish Local map origin */
  visualizePoint(
    mkr_arr, this->get_clock()->now().seconds(), local_map_origin_d_, Color::Red(),
    "Local Map Origin", 0.2, 0);

  if (cfg_.esdf_en) {
    Vec3f esdf_box_max, esdf_box_min;
    esdf_map_->getUpdatedBbox(esdf_box_min, esdf_box_max);
    visualizeText(
      mkr_arr, this->get_clock()->now().seconds(), "ESDF Map Text", "ESDF Map",
      esdf_box_max + Vec3f(0, 0, 1.0), Color::Blue(), 0.6, 0);
    visualizeBoundingBox(
      mkr_arr, this->get_clock()->now().seconds(), esdf_box_min, esdf_box_max,
      "ESDF Updating Range", Color::Blue());
  }

  vm_.mkr_arr_pub->publish(mkr_arr);
}

void ROGMapROS::vecEVec3fToPC2(const vec_E<Vec3f> & points, sensor_msgs::msg::PointCloud2 & cloud)
{
  pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
  pcl_cloud.resize(points.size());
  for (long unsigned int i = 0; i < points.size(); i++) {
    pcl_cloud[i].x = static_cast<float>(points[i][0]);
    pcl_cloud[i].y = static_cast<float>(points[i][1]);
    pcl_cloud[i].z = static_cast<float>(points[i][2]);
  }
  pcl::toROSMsg(pcl_cloud, cloud);
  cloud.header.stamp = this->get_clock()->now();
  cloud.header.frame_id = cfg_.frame_id;
}

ROGMapROS::ROGMapROS(const rclcpp::NodeOptions & options) : rclcpp::Node("rog_map", options)
{
  // parameters
  this->declare_parameter<std::string>("cfg_path", "");
  std::string cfg_path;
  this->get_parameter("cfg_path", cfg_path);

  const rclcpp::QoS qos(rclcpp::QoS(1).best_effort().keep_last(1).durability_volatile());

  cfg_ = rog_map::Config(cfg_path);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  init();
  if (cfg_.visualization_en) {
    vm_.occ_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/occ", qos);
    vm_.unknown_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/unk", qos);
    vm_.occ_inf_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_occ", qos);
    vm_.unknown_inf_pub =
      this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/inf_unk", qos);

    if (cfg_.frontier_extraction_en) {
      vm_.frontier_pub =
        this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/frontier", qos);
    }

    if (cfg_.esdf_en) {
      vm_.esdf_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("rog_map/esdf", qos);
    }

    if (cfg_.viz_time_rate > 0) {
      const int cbk_dt_ms = static_cast<int>(1.0 / cfg_.viz_time_rate * 1000);
      vm_.viz_reen_cbk_group =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      vm_.viz_timer = this->create_wall_timer(
        std::chrono::milliseconds(cbk_dt_ms), std::bind(&ROGMapROS::vizCallback, this),
        vm_.viz_reen_cbk_group);
    }
  }

  vm_.mkr_arr_pub =
    this->create_publisher<visualization_msgs::msg::MarkerArray>("rog_map/map_bound", qos);

  if (cfg_.ros_callback_en) {
    rc_.odom_me_cbk_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rc_.cloud_me_cbk_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions so;
    so.callback_group = rc_.odom_me_cbk_group;
    rc_.odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      cfg_.odom_topic, qos, std::bind(&ROGMapROS::odomCallback, this, std::placeholders::_1), so);
    so.callback_group = rc_.cloud_me_cbk_group;
    rc_.cloud_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cfg_.cloud_topic, qos, std::bind(&ROGMapROS::cloudCallback, this, std::placeholders::_1), so);
    rc_.update_cbk_group =
      this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rc_.update_timer = this->create_wall_timer(
      std::chrono::milliseconds(1), std::bind(&ROGMapROS::updateCallback, this),
      rc_.update_cbk_group);
  }
}

void ROGMapROS::visualizeBoundingBox(
  visualization_msgs::msg::MarkerArray & mkrarr, const double & stamp, const Vec3f & box_min,
  const Vec3f & box_max, const string & ns, const Color & color, const double & size_x,
  const double & alpha, const bool & print_ns)
{
  Vec3f size = (box_max - box_min) / 2;
  Vec3f vis_pos_world = (box_min + box_max) / 2;
  double width = size.x();
  double length = size.y();
  double height = size.z();

  int id = 0;
  visualization_msgs::msg::Marker line_strip;
  line_strip.header.stamp = rclcpp::Time(stamp);
  line_strip.header.frame_id = cfg_.frame_id;
  line_strip.action = visualization_msgs::msg::Marker::ADD;
  line_strip.ns = ns;
  line_strip.pose.orientation.w = 1.0;
  line_strip.id = id++;
  line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strip.scale.x = size_x;
  line_strip.color = color;
  line_strip.color.a = alpha;
  geometry_msgs::msg::Point p[8];

  p[0].x = vis_pos_world(0) - width;
  p[0].y = vis_pos_world(1) + length;
  p[0].z = vis_pos_world(2) + height;
  p[1].x = vis_pos_world(0) - width;
  p[1].y = vis_pos_world(1) - length;
  p[1].z = vis_pos_world(2) + height;
  p[2].x = vis_pos_world(0) - width;
  p[2].y = vis_pos_world(1) - length;
  p[2].z = vis_pos_world(2) - height;
  p[3].x = vis_pos_world(0) - width;
  p[3].y = vis_pos_world(1) + length;
  p[3].z = vis_pos_world(2) - height;
  p[4].x = vis_pos_world(0) + width;
  p[4].y = vis_pos_world(1) + length;
  p[4].z = vis_pos_world(2) - height;
  p[5].x = vis_pos_world(0) + width;
  p[5].y = vis_pos_world(1) - length;
  p[5].z = vis_pos_world(2) - height;
  p[6].x = vis_pos_world(0) + width;
  p[6].y = vis_pos_world(1) - length;
  p[6].z = vis_pos_world(2) + height;
  p[7].x = vis_pos_world(0) + width;
  p[7].y = vis_pos_world(1) + length;
  p[7].z = vis_pos_world(2) + height;

  for (int i = 0; i < 8; i++) {
    line_strip.points.push_back(p[i]);
  }
  line_strip.points.push_back(p[0]);
  line_strip.points.push_back(p[3]);
  line_strip.points.push_back(p[2]);
  line_strip.points.push_back(p[5]);
  line_strip.points.push_back(p[6]);
  line_strip.points.push_back(p[1]);
  line_strip.points.push_back(p[0]);
  line_strip.points.push_back(p[7]);
  line_strip.points.push_back(p[4]);
  mkrarr.markers.push_back(line_strip);
}

void ROGMapROS::visualizeText(
  visualization_msgs::msg::MarkerArray & mkr_arr, const double & stamp, const std::string & ns,
  const std::string & text, const Vec3f & position, const Color & c, const double & size,
  const int & id)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = cfg_.frame_id;
  marker.header.stamp = rclcpp::Time(stamp);
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.ns = ns;
  if (id >= 0) {
    marker.id = id;
  } else {
    static int id = 0;
    marker.id = id++;
  }
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.scale.z = size;
  marker.color = c;
  marker.text = text;
  marker.pose.position.x = position.x();
  marker.pose.position.y = position.y();
  marker.pose.position.z = position.z();
  marker.pose.orientation.w = 1.0;
  mkr_arr.markers.push_back(marker);
}

void ROGMapROS::visualizePoint(
  visualization_msgs::msg::MarkerArray & mkr_arr, const double & stamp, const Vec3f & pt,
  Color color, std::string ns, double size, int id, const bool & print_ns)
{
  visualization_msgs::msg::Marker marker_ball;
  static int cnt = 0;
  Vec3f cur_pos = pt;
  if (isnan(pt.x()) || isnan(pt.y()) || isnan(pt.z())) {
    return;
  }
  marker_ball.header.frame_id = cfg_.frame_id;
  marker_ball.header.stamp = rclcpp::Time(stamp);
  marker_ball.ns = ns;
  marker_ball.id = id >= 0 ? id : cnt++;
  marker_ball.action = visualization_msgs::msg::Marker::ADD;
  marker_ball.pose.orientation.w = 1.0;
  marker_ball.type = visualization_msgs::msg::Marker::SPHERE;
  marker_ball.scale.x = size;
  marker_ball.scale.y = size;
  marker_ball.scale.z = size;
  marker_ball.color = color;

  geometry_msgs::msg::Point p;
  p.x = cur_pos.x();
  p.y = cur_pos.y();
  p.z = cur_pos.z();

  marker_ball.pose.position = p;
  mkr_arr.markers.push_back(marker_ball);

  if (print_ns) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = cfg_.frame_id;
    marker.header.stamp = rclcpp::Time(stamp);
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.ns = ns + "_text";
    if (id >= 0) {
      marker.id = id;
    } else {
      static int id = 0;
      marker.id = id++;
    }
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.scale.z = 0.6;
    marker.color = color;
    marker.text = ns;
    marker.pose.position.x = cur_pos.x();
    marker.pose.position.y = cur_pos.y();
    marker.pose.position.z = cur_pos.z() + 0.5;
    marker.pose.orientation.w = 1.0;
    mkr_arr.markers.push_back(marker);
  }
}

}  // namespace rog_map

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(rog_map::ROGMapROS)
