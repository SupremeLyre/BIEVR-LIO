#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros2/publisher.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

namespace {
template <typename T>
T deserialize(const rclcpp::SerializedMessage& serialized) {
  T msg;
  rclcpp::Serialization<T>().deserialize_message(&serialized, &msg);
  return msg;
}
}  // namespace

int main(int argc, char** argv) {
  srand(1);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bievr_lio_bag_node");

  bievr::Config config;
  // rclcpp::init does not strip ROS arguments from argv; remove_ros_arguments
  // returns just the application arguments (our config-file flags).
  if (!bievr::loadConfigFromArgs(rclcpp::remove_ros_arguments(argc, argv), config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  auto pipeline = std::make_shared<bievr::Pipeline>(config.pipeline_config);
  auto synchronizer = std::make_shared<bievr::Synchronizer>(pipeline);
  auto lio_pub = std::make_shared<bievr::Publisher>(node, pipeline, "bievr_lio");

  rosbag2_cpp::Reader reader;
  reader.open(config.topic_config.bag_path);

  // Map each topic to its message type (needed to pick the right deserializer).
  std::unordered_map<std::string, std::string> topic_types;
  for (const auto& topic : reader.get_all_topics_and_types()) {
    topic_types[topic.name] = topic.type;
  }

  const std::string& pc_topic = config.topic_config.pointcloud_topic;
  const std::string& imu_topic = config.topic_config.imu_topic;

  const auto pc_type = topic_types.find(pc_topic);
  if (pc_type == topic_types.end()) {
    LOG(E, "Pointcloud topic '" << pc_topic << "' not found in bag.");
    rclcpp::shutdown();
    return 1;
  }
  bool supported_pc_type = pc_type->second == "sensor_msgs/msg/PointCloud2";
#ifdef BIEVR_WITH_LIVOX
  supported_pc_type = supported_pc_type || pc_type->second == "livox_ros_driver2/msg/CustomMsg" ||
                      pc_type->second == "livox_ros_driver/msg/CustomMsg";
#endif
  if (!supported_pc_type) {
    LOG(E, "Unsupported pointcloud message type '" << pc_type->second << "' on '" << pc_topic
                                                  << "'. Livox bags require livox_ros_driver2 "
                                                     "to be available at build time.");
    rclcpp::shutdown();
    return 1;
  }
  LOG(I, "Reading pointcloud topic '" << pc_topic << "' with type " << pc_type->second);

  size_t num_pointclouds = 0;
  size_t num_imus = 0;

  while (rclcpp::ok() && reader.has_next()) {
    auto bag_msg = reader.read_next();
    const std::string& topic = bag_msg->topic_name;
    if (topic != pc_topic && topic != imu_topic) {
      continue;
    }

    rclcpp::SerializedMessage serialized(*bag_msg->serialized_data);
    const std::string& type = topic_types[topic];

    if (type == "sensor_msgs/msg/PointCloud2") {
      auto msg = deserialize<sensor_msgs::msg::PointCloud2>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      if (bievr::msgToPointcloud(msg, pointcloud) && synchronizer->addPointcloud(pointcloud)) {
        ++num_pointclouds;
      }
    }
#ifdef BIEVR_WITH_LIVOX
    else if (type == "livox_ros_driver2/msg/CustomMsg" || type == "livox_ros_driver/msg/CustomMsg") {
      // rosbags-converted gen1 messages retain their old package name, but their
      // ROS2 CDR layout matches CustomMsg from driver2. Gen1's timebase can use
      // the sensor clock, so use the header stamp just as the ROS1 wrapper does.
      auto msg = deserialize<livox_ros_driver2::msg::CustomMsg>(serialized);
      bievr::StampedIntensityPointcloud pointcloud;
      const bool converted = type == "livox_ros_driver/msg/CustomMsg"
          ? bievr::conv_detail::livoxToStampedIntensity(
                msg, bievr::conv_detail::stampToNs(msg.header.stamp), "00_conversion_livox",
                pointcloud)
          : bievr::msgToPointcloud(msg, pointcloud);
      if (converted && synchronizer->addPointcloud(pointcloud)) {
        ++num_pointclouds;
      }
    }
#endif
    else if (type == "sensor_msgs/msg/Imu") {
      auto msg = deserialize<sensor_msgs::msg::Imu>(serialized);
      bievr::ImuMeasurement imu;
      if (bievr::msgToImuMeasurement(msg, imu) && synchronizer->addImu(imu)) {
        ++num_imus;
      }
    }
  }

  LOG(I, "Done with bag. Accepted " << num_pointclouds << " pointclouds and " << num_imus
                                    << " IMU messages.");
  reader.close();
  LOG(I, "Bag closed");

  rclcpp::shutdown();
  return 0;
}
