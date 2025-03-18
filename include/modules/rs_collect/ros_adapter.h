/************************************************************
 * Copyright 2025 RoboSense Technology Co., Ltd
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at

 * http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * 
***************************************************************/
#ifndef RS_COLLECT_ROS_ADAPTER_H
#define RS_COLLECT_ROS_ADAPTER_H

#include <memory>
#include <type_traits>
#include <utility>
#include <sstream>
#include <iostream>

#if __ROS2__

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/type_adapter.hpp>
#include <rclcpp/serialization.hpp>

#define ROSTime rclcpp::Clock(RCL_ROS_TIME).now().nanoseconds
#define SystemTime rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds

using NodeHandle = rclcpp::Node;
using ros_time_t = int64_t;
using ROSTimer = rclcpp::TimerBase::SharedPtr;
using GenericSubscriber = rclcpp::GenericSubscription::SharedPtr;
using SerializedMessage = rclcpp::SerializedMessage;
using SerializedMessagePtr = std::shared_ptr<rclcpp::SerializedMessage>;
using SerializedMessageConstPtr = std::shared_ptr<rclcpp::SerializedMessage const>;

#if RCLCPP_VERSION_MAJOR > 16
#define CALLBACK_PARAM_TYPE(T) std::shared_ptr<T const>  // or usual ptr?
#include "rosbag2_storage/qos.hpp"
using Rosbag2QoS = rosbag2_storage::Rosbag2QoS;
#else
#define CALLBACK_PARAM_TYPE(T) std::shared_ptr<T const>
#include "rosbag2_transport/qos.hpp"
using Rosbag2QoS = rosbag2_transport::Rosbag2QoS;
#endif

#elif __ROS1__
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <topic_tools/shape_shifter.h>
#include <ros/master.h>
#include <rosbag/bag.h>

#define ROSTime ros::Time::now().toNSec
#define SystemTime ros::Time::now().toNSec

using NodeHandle = ros::NodeHandle;
using ros_time_t = uint64_t;
using ROSTimer = ros::WallTimer;
using GenericSubscriber = boost::shared_ptr<ros::Subscriber>;
using SerializedMessage = topic_tools::ShapeShifter;
using SerializedMessagePtr = topic_tools::ShapeShifter::Ptr;
using SerializedMessageConstPtr = topic_tools::ShapeShifter::ConstPtr;

#define CALLBACK_PARAM_TYPE(T) boost::shared_ptr<T const>

#endif

// common interface
using NodeHandlePtr = std::shared_ptr<NodeHandle>;
using GenericSubscriberMap = std::map<std::string, GenericSubscriber>;

/************************* LOG ADAPTER *************************/
namespace robosense
{
namespace rs_collect
{
namespace collect
{

#if __ROS2__
inline rclcpp::Logger getLogger(const rclcpp::Logger & logger) { 
  return logger; 
}

inline rclcpp::Logger getLogger(const rclcpp::Node & node) { 
  return node.get_logger(); 
}

inline rclcpp::Logger getLogger(rclcpp::Node * node) { 
  return node->get_logger(); 
}

inline rclcpp::Logger getLogger(const std::shared_ptr<rclcpp::Node> & node) { 
  return node->get_logger(); 
}
#endif

#if __ROS2__

#define RS_STREAM_IMPL(level, logger, ...) do { \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_##level##_STREAM(logger, ss.str()); \
} while(0)

#elif __ROS1__

#define RS_STREAM_IMPL(level, ...) do { \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  ROS_##level##_STREAM(ss.str()); \
} while(0)

#endif
}
}
}  // namespace robosense::rs_collect::collect

#if __ROS1__
#define DECLARE_LOGGER(__LEVEL__)                                                                           \
  template <typename LoggerT, typename... Args>                                                             \
  void RS_##__LEVEL__(LoggerT && logger, const char * fmt, Args &&... args)                                 \
  {                                                                                                         \
    static_cast<void>(logger);                                                                              \
    if (sizeof...(args) == 0) {                                                                             \
      ROS_##__LEVEL__("%s", fmt);                                                                           \
    } else {                                                                                                \
      ROS_##__LEVEL__(fmt, std::forward<Args>(args)...);                                                    \
    }                                                                                                       \
  }

#elif __ROS2__
#define DECLARE_LOGGER(__LEVEL__)                                                                           \
  template <typename LoggerT, typename... Args>                                                             \
  void RS_##__LEVEL__(LoggerT && logger, const char * fmt, Args &&... args)                                 \
  {                                                                                                         \
    auto logger_ptr = robosense::rs_collect::collect::getLogger(logger);                                     \
    if (sizeof...(args) == 0) {                                                                             \
      RCLCPP_##__LEVEL__(logger_ptr, "%s", fmt);                                                            \
    } else {                                                                                                \
      RCLCPP_##__LEVEL__(logger_ptr, fmt, std::forward<Args>(args)...);                                     \
    }                                                                                                       \
  }

#endif

#if __ROS1__
#define RS_INFO_STREAM(logger, ...) RS_STREAM_IMPL(INFO, __VA_ARGS__)
#define RS_WARN_STREAM(logger, ...) RS_STREAM_IMPL(WARN, __VA_ARGS__)
#define RS_ERROR_STREAM(logger, ...) RS_STREAM_IMPL(ERROR, __VA_ARGS__)
#define RS_DEBUG_STREAM(logger, ...) RS_STREAM_IMPL(DEBUG, __VA_ARGS__)
#define RS_FATAL_STREAM(logger, ...) RS_STREAM_IMPL(FATAL, __VA_ARGS__)
#elif __ROS2__
#define RS_INFO_STREAM(logger, ...) do { \
  auto log_obj = robosense::rs_collect::collect::getLogger(logger); \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_INFO_STREAM(log_obj, ss.str()); \
} while(0)

#define RS_WARN_STREAM(logger, ...) do { \
  auto log_obj = robosense::rs_collect::collect::getLogger(logger); \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_WARN_STREAM(log_obj, ss.str()); \
} while(0)

#define RS_ERROR_STREAM(logger, ...) do { \
  auto log_obj = robosense::rs_collect::collect::getLogger(logger); \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_ERROR_STREAM(log_obj, ss.str()); \
} while(0)

#define RS_DEBUG_STREAM(logger, ...) do { \
  auto log_obj = robosense::rs_collect::collect::getLogger(logger); \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_DEBUG_STREAM(log_obj, ss.str()); \
} while(0)

#define RS_FATAL_STREAM(logger, ...) do { \
  auto log_obj = robosense::rs_collect::collect::getLogger(logger); \
  std::stringstream ss; \
  ss << __VA_ARGS__; \
  RCLCPP_FATAL_STREAM(log_obj, ss.str()); \
} while(0)
#endif

DECLARE_LOGGER(INFO)
DECLARE_LOGGER(WARN)
DECLARE_LOGGER(ERROR)
DECLARE_LOGGER(FATAL)
DECLARE_LOGGER(DEBUG)

/*    message type adapter    */

#if __ROS2__
#include <std_msgs/msg/string.hpp>
using std_msgs_String = std_msgs::msg::String;
#elif __ROS1__
#include <std_msgs/String.h>
using std_msgs_String = std_msgs::String;
#endif

#endif  // RS_COLLECT_ROS_ADAPTER_H
