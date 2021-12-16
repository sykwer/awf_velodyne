/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

#include <velodyne_pointcloud/interpolate.h>

#include <pcl_conversions/pcl_conversions.h>
#include <velodyne_pointcloud/pointcloudXYZIRADT.h>

#include <velodyne_pointcloud/func.h>

#include <yaml-cpp/yaml.h>
#include <regex>

namespace velodyne_pointcloud
{
/** @brief Constructor. */
Interpolate::Interpolate(const rclcpp::NodeOptions & options)
: Node("velodyne_interpolate_node", options),
 tf2_listener_(tf2_buffer_),
 base_link_frame_("base_link")
{
  // advertise
  velodyne_points_interpolate_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_interpolate", rclcpp::SensorDataQoS());
  velodyne_points_interpolate_ex_pub_ =
    this->create_publisher<sensor_msgs::msg::PointCloud2>("velodyne_points_interpolate_ex", rclcpp::SensorDataQoS());

  // subscribe
  twist_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "/vehicle/status/twist", 10, std::bind(&Interpolate::processTwist, this, std::placeholders::_1));
  velodyne_points_ex_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "velodyne_points_ex", rclcpp::SensorDataQoS(), std::bind(&Interpolate::processPoints,this, std::placeholders::_1));

  save_test_vector_ = static_cast<bool>(declare_parameter("save_test_vector", false));
  declare_parameter("test_vector_sampling_start", 0);
  declare_parameter("test_vector_sampling_rate", 0);
  declare_parameter("test_vector_sampling_end", 0);
  test_vector_sampling_start_ = get_parameter("test_vector_sampling_start").as_int();
  test_vector_sampling_rate_ = get_parameter("test_vector_sampling_rate").as_int();
  test_vector_sampling_end_ = get_parameter("test_vector_sampling_end").as_int();
  frame_id_ = 0;
  if (save_test_vector_) {
    // make prefix of test vector file name
    // (e.g. /sensing/lidar/left/crop_box_filter_self -> sensing_lidar_left_crop_box_filter_self)
    std::string prefix =
      std::regex_replace(std::string(get_namespace()), std::regex("/"), "_").erase(0, 1) +
      "_" +
      std::string(get_name());
    test_vector_input_file_ = prefix + "_input_vector.yaml";
    test_vector_output_file_ = prefix + "_output_vector.yaml";
    // make empty files
    (void)std::ofstream(test_vector_input_file_);
    (void)std::ofstream(test_vector_output_file_);
  }
}

void Interpolate::processTwist(const geometry_msgs::msg::TwistStamped::SharedPtr twist_msg)
{
  twist_queue_.push_back(*twist_msg);

  while (!twist_queue_.empty()) {
    //for replay rosbag
    if (rclcpp::Time(twist_queue_.front().header.stamp) > rclcpp::Time(twist_msg->header.stamp)) {
      twist_queue_.pop_front();
    } else if (rclcpp::Time(twist_queue_.front().header.stamp) < rclcpp::Time(twist_msg->header.stamp) - rclcpp::Duration::from_seconds(1.0)) {
      twist_queue_.pop_front();
    } else {
      break;
    }
  }
}

void Interpolate::processPoints(
  const sensor_msgs::msg::PointCloud2::SharedPtr points_xyziradt_msg)
{
  if (
    velodyne_points_interpolate_pub_->get_subscription_count() <= 0 &&
    velodyne_points_interpolate_ex_pub_->get_subscription_count() <= 0) {
    return;
  }

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  pcl::fromROSMsg(*points_xyziradt_msg, *points_xyziradt);

  pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr interpolate_points_xyziradt(
    new pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>);
  tf2::Transform tf2_base_link_to_sensor;
  getTransform(points_xyziradt->header.frame_id, base_link_frame_, &tf2_base_link_to_sensor);
  interpolate_points_xyziradt = interpolate(points_xyziradt, twist_queue_, tf2_base_link_to_sensor);

  // Write input and output to yaml files
  if (save_test_vector_) {
    int frame_id = frame_id_ - test_vector_sampling_start_;
    if (frame_id >= 0 &&
        frame_id % test_vector_sampling_rate_ == test_vector_sampling_rate_ - 1) {
      RCLCPP_INFO(this->get_logger(), "sampling test vector data");
      test_vector_inputs_.push_back(points_xyziradt);
      test_vector_outputs_.push_back(interpolate_points_xyziradt);
    }
    if (frame_id_ == test_vector_sampling_end_) {
      RCLCPP_INFO(this->get_logger(), "start writing test vector");
      writePointCloud(test_vector_input_file_, test_vector_inputs_);
      writePointCloud(test_vector_output_file_, test_vector_outputs_);
      RCLCPP_INFO(this->get_logger(), "complete writing test vector");
    }
    frame_id_++;
  }

  if (velodyne_points_interpolate_pub_->get_subscription_count() > 0) {
    const auto interpolate_points_xyzir = convert(interpolate_points_xyziradt);
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*interpolate_points_xyzir, *ros_pc_msg_ptr);
    velodyne_points_interpolate_pub_->publish(std::move(ros_pc_msg_ptr));
  }
  if (velodyne_points_interpolate_ex_pub_->get_subscription_count() > 0) {
    auto ros_pc_msg_ptr = std::make_unique<sensor_msgs::msg::PointCloud2>();
    pcl::toROSMsg(*interpolate_points_xyziradt, *ros_pc_msg_ptr);
    velodyne_points_interpolate_ex_pub_->publish(std::move(ros_pc_msg_ptr));
  }
}

bool Interpolate::getTransform(
  const std::string & target_frame, const std::string & source_frame,
  tf2::Transform * tf2_transform_ptr)
{
  if (target_frame == source_frame) {
    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return true;
  }

  try {
    const auto transform_msg =
      tf2_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    tf2::convert(transform_msg.transform, *tf2_transform_ptr);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "%s", ex.what());
    RCLCPP_ERROR(this->get_logger(), "Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    tf2_transform_ptr->setOrigin(tf2::Vector3(0, 0, 0));
    tf2_transform_ptr->setRotation(tf2::Quaternion(0, 0, 0, 1));
    return false;
  }
  return true;
}

/** @brief Write in/out pointclouds data to yaml file. */
void Interpolate::writePointCloud(
  const std::string & filename,
  const std::vector<pcl::PointCloud<velodyne_pointcloud::PointXYZIRADT>::Ptr> & clouds)
{
  YAML::Emitter emitter;
  uint32_t frame_id = 0;
  for (auto cloud = clouds.begin(), end = clouds.end(); cloud != end; cloud++, frame_id++) {
    emitter << YAML::BeginSeq;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "frame_id" << YAML::Value << frame_id;
    emitter << YAML::Key << "cloud" << YAML::Value;
    emitter << YAML::BeginMap;
    emitter << YAML::Key << "points" << YAML::Value;
    emitter << YAML::BeginSeq;
    for (auto it = (*cloud)->points.begin(), e = (*cloud)->points.end(); it != e; it++) {
      emitter << YAML::Flow;
      emitter << YAML::BeginSeq;
      emitter << it->x << it->y << it->z << it->intensity << it->return_type
        << it->ring << it->azimuth << it->distance << it->time_stamp;
      emitter << YAML::EndSeq;
    }
    emitter << YAML::EndSeq;
    emitter << YAML::Key << "width" << YAML::Value << (*cloud)->width;
    emitter << YAML::Key << "height" << YAML::Value << (*cloud)->height;
    emitter << YAML::Key << "is_dense" << YAML::Value << (*cloud)->is_dense;
    emitter << YAML::EndMap;
    emitter << YAML::EndMap;
    emitter << YAML::EndSeq;
  }
  std::ofstream output_file(filename, std::ios::app);
  output_file << emitter.c_str() << std::endl;
  output_file.close();
}

}  // namespace velodyne_pointcloud

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(velodyne_pointcloud::Interpolate)
