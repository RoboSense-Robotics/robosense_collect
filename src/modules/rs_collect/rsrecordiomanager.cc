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
#include "modules/rs_collect/rsrecordiomanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

std::mutex RSRecordIoManager::ERASE_TIMEOUT_MTX_;
std::map<size_t, RSRecordClipManager::Ptr>
    RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_;

RSRecordIoManager::RSRecordIoManager(
    const RSGlobalConfig::Ptr &pSharedGlobalConfig,
    const RSMessageMonitor::RS_MESSAGE_MONITOR_CALLBACK &callback,
    const RS_RECORD_IO_ERROR_CALLBACK &errorCallback,
    const NodeHandlePtr &pSharedNode) {
  if (pSharedGlobalConfig == nullptr || pSharedNode == nullptr) {
    throw std::runtime_error(
        "GlobalConfig is Nullptr Or pSharedNode is Nullptr !");
  }

  pSharedGlobalConfig_ = pSharedGlobalConfig;
  monitorCallback_ = callback;
  errorCallback_ = errorCallback;
  pSharedNode_ = pSharedNode;
}

RSRecordIoManager::~RSRecordIoManager() {
  StopRecord();
  Stop();
}

bool RSRecordIoManager::Init() {
  // process topic(s)
  need_message_monitor_channel_names_.clear();
  const auto &sensors_config =
      pSharedGlobalConfig_->recordConfig["recordsensorconfig"];
  for (size_t i = 0; i < sensors_config["record_sensors"].size(); ++i) {
    const auto &sensor_config =
        sensors_config["record_sensors"][static_cast<int>(i)];
    const std::string &topic_name =
        sensor_config["topic_name"].as<std::string>();
    if (!sensor_config["enable_topic"].as<bool>()) {
      black_topics_.push_back(topic_name);
      continue;
    } else {
      white_topics_.push_back(topic_name);
    }

    if (sensor_config["enable_timeout_check"].as<bool>()) {
      need_message_monitor_channel_names_.insert(topic_name);
    }
  }

  is_record_ = false;
  global_writer_index_ = 0;
  current_writer_index_ = invalid_writer_index_;

  // message monitor
  is_need_message_monitor_ = !(need_message_monitor_channel_names_.empty());
  pSharedMessageMonitor_ = nullptr;
  if (is_need_message_monitor_) {
    RS_INFO(pSharedNode_, "Enable Message Monitor !");
    try {
      pSharedMessageMonitor_.reset(new RSMessageMonitor());
    } catch (const std::exception &e) {
      return false;
    }

    const auto &local_callback =
        std::bind(&RSRecordIoManager::LocalMonitorCallback, this,
                  std::placeholders::_1, std::placeholders::_2);
    int ret = pSharedMessageMonitor_->init(pSharedGlobalConfig_, pSharedNode_,
                                           local_callback);
    if (ret != 0) {
      return false;
    }
  } else {
    RS_INFO(pSharedNode_, "Disable Message Monitor !");
  }

  serialization_format_ =
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["serialization_format"]
          .as<std::string>();

  enable_clip_erase_timeout_debug_ =
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["enable_clip_erase_timeout_debug"]
          .as<bool>();

  clip_erase_timeout_debug_count_ =
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["clip_erase_timeout_debug_count"]
          .as<uint32_t>();

  RS_INFO_STREAM(pSharedNode_,
                     ", serialization_format_ = "
                         << serialization_format_
                         << ", enable_clip_erase_timeout_debug_ = "
                         << enable_clip_erase_timeout_debug_
                         << ", clip_erase_timeout_debug_count_ = "
                         << clip_erase_timeout_debug_count_);

  return true;
}

bool RSRecordIoManager::Start() {
  RS_INFO(pSharedNode_, "Start: RSRecordIoManager::Start()");
  for (const auto &topic_name : white_topics_) {
    if (std::find(black_topics_.begin(), black_topics_.end(), topic_name) !=
        black_topics_.end()) {
      RS_ERROR_STREAM(pSharedNode_,
          "find channel in both of white list and black list, channel: "
              << topic_name);
      return false;
    }
  }

  // 处理话题名称
  for (auto &topic : white_topics_) {
#if __ROS2__
    topic = rclcpp::expand_topic_or_service_name(
        topic, pSharedNode_->get_name(), pSharedNode_->get_namespace(), false);
#elif __ROS1__
    topic = ros::names::resolve(topic, false);
#endif
  }

  if (!InitSubscribersImpl()) {
    RS_ERROR(pSharedNode_, "InitSubscribersImpl() error.");
    return false;
  }

  // 强制休眠1s
  std::this_thread::sleep_for(std::chrono::seconds(1));

  message_count_ = 0;
  message_time_ = 0;
  is_started_ = true;
  disk_check_thread_ =
      std::make_shared<std::thread>([this]() { this->DiskCheckThread(); });
  if (disk_check_thread_ == nullptr) {
    RS_ERROR(pSharedNode_, "init display thread error.");
    return false;
  }

  need_next_clip_thread_ =
      std::make_shared<std::thread>([this]() { this->NeedNextClipThread(); });
  if (need_next_clip_thread_ == nullptr) {
    RS_ERROR(pSharedNode_, "init next clip thread error.");
    return false;
  }

  record_buffer_thread_ =
      std::make_shared<std::thread>([this]() { this->RecordBufferThread(); });
  if (record_buffer_thread_ == nullptr) {
    RS_ERROR(pSharedNode_, "init record buffer thread error.");
    return false;
  }
  RS_INFO(pSharedNode_, "init record buffer thread successed !");

  erase_timeout_clip_cnt_ = 0;
  for (size_t i = 0; i < RS_ERASE_THREAD_CNT; ++i) {
    auto erase_writer_indexs_thread = std::make_shared<std::thread>(
        [this]() { this->EraseWriterIndexThread(); });
    if (erase_writer_indexs_thread == nullptr) {
      RS_ERROR(pSharedNode_, "init erase clip thread error.");
      return false;
    }
    erase_writer_indexs_thread->detach();
    erase_writer_timeout_threads_.push_back(erase_writer_indexs_thread);
  }

  erase_writer_timeout_thread_ = std::make_shared<std::thread>(
      [this]() { this->EraseWriterIndexTimeoutThread(); });
  if (erase_writer_timeout_thread_ == nullptr) {
    RS_ERROR(pSharedNode_, "init erase clip timeout thread error.");
    return false;
  }

  if (pSharedMessageMonitor_ != nullptr) {
    int ret = pSharedMessageMonitor_->start();
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "start message monitor failed: ret = " << ret);
      return false;
    }
  }

  RS_INFO(pSharedNode_, "Finish: RSRecordIoManager::Start()");

  return true;
}

bool RSRecordIoManager::Stop() {
  RS_INFO(pSharedNode_, "Start: RSRecordIoManager::Stop()");
  if (!is_started_ || is_stopping_) {
    return false;
  }
  is_stopping_ = true;

  if (!FreeSubscribersImpl()) {
    RS_ERROR(pSharedNode_, "FreeSubscribersImpl() error.");
    return false;
  }

  if (disk_check_thread_) {
    if (disk_check_thread_->joinable()) {
      disk_check_thread_->join();
    }
    disk_check_thread_ = nullptr;
  }

  if (pSharedMessageMonitor_ != nullptr) {
    int ret = pSharedMessageMonitor_->stop();
    if (ret != 0) {
      return false;
    }
  }

  is_started_ = false;
  if (need_next_clip_thread_) {
    need_next_clip_buffer_cond_.notify_all();
    if (need_next_clip_thread_->joinable()) {
      need_next_clip_thread_->join();
    }
    need_next_clip_thread_.reset();
  }

  if (erase_writer_timeout_thread_) {
    if (erase_writer_timeout_thread_->joinable()) {
      erase_writer_timeout_thread_->join();
    }
    erase_writer_timeout_thread_.reset();
  }

  if (record_buffer_thread_) {
    record_buffer_cond_.notify_all();
    if (record_buffer_thread_->joinable()) {
      record_buffer_thread_->join();
    }
  }

  // 关闭全部的线程
  erase_writer_cond_.notify_all();

  erase_writer_timeout_threads_.clear();
  // RCLCPP_ERROR_STREAM(pSharedNode_->get_logger(), "erase_writer_indexs_ SIZE
  // = "
  //                                  << erase_writer_indexs_.size() << ", "
  //                                  << (!(is_started_ && !is_stopping_)));

  is_stopping_ = false;

  RS_INFO(pSharedNode_, "Finish: RSRecordIoManager::Stop()");
  return true;
}

bool RSRecordIoManager::CheckSegment() {
  if (!is_record_) {
    return false;
  }

  bool isSegment = false;
  std::lock_guard<std::mutex> lg(current_write_mtx_);
  if (writer_mapper_.find(current_writer_index_) != writer_mapper_.end()) {
    if (writer_mapper_[current_writer_index_] != nullptr) {
      isSegment = writer_mapper_[current_writer_index_]->checkSegment();
    }
  }

  // RCLCPP_ERROR_STREAM(pSharedNode_->get_logger(), "checkSegment
  // current_writer_index_ = "
  //                                  << current_writer_index_
  //                                  << ", isSegment = " << isSegment);

  return isSegment;
}

bool RSRecordIoManager::StartRecord() {
  RS_INFO(pSharedNode_, "Start Record !");
  if (pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["enable_disk_mount_retry"]
          .as<bool>()) {
    is_disk_mount_retry_debug_ =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["enable_disk_mount_retry_debug"]
            .as<bool>();
    if (!InitDeviceRetryMount()) {
      RS_ERROR(pSharedNode_,
          "Enable Device Retry Mount: initial device retry mount failed !");
      return false;
    } else {
      RS_INFO(pSharedNode_,
          "Enable Device Retry Mount: initial device retry mount "
          "successed !");
    }
  } else {
    RS_INFO(pSharedNode_, "Disable Device Retry Mount !");
  }
  RSRecordClipManager::resetCameraH265Buffer();
#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  if (pSharedMessageMonitor_ != nullptr) {
    pSharedMessageMonitor_->updateRecordStatus(true);
  }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  is_record_ = true;

  return true;
}

bool RSRecordIoManager::StopRecord() {
  RS_INFO(pSharedNode_, "Stop Record !");
#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  if (pSharedMessageMonitor_ != nullptr) {
    pSharedMessageMonitor_->updateRecordStatus(false);
  }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  is_record_ = false;
  // std::this_thread::sleep_for(std::chrono::seconds(6));
  {
    std::lock_guard<std::mutex> lg(current_write_mtx_);
    for (auto iterMap = writer_mapper_.begin(); iterMap != writer_mapper_.end();
         ++iterMap) {
      const size_t index = iterMap->first;
      auto &clipWriter = iterMap->second;
      if (clipWriter != nullptr) {
        // 释放超时的不再释放
        {
          std::lock_guard<std::mutex> lg(RSRecordIoManager::ERASE_TIMEOUT_MTX_);
          if (RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_.find(index) !=
              RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_.end()) {
            continue;
          }
        }
#if 0 
        // 同步关闭Clip 
        EraseClipRecord(index, clipWriter);
#else
        // 异步关闭Clip
        std::lock_guard<std::mutex> lg(erase_writer_mtx_);
        erase_writer_indexs_.insert({index, clipWriter});
        erase_writer_cond_.notify_one();
#endif
      }
    }
    writer_mapper_.clear();
    current_writer_index_ = invalid_writer_index_;
  }
  RS_INFO(pSharedNode_, "Stop Record Finish !");
  return true;
}

int RSRecordIoManager::AddClipRecord(const bool isAddNextClip) {
  const uint64_t startTimestampNs = RS_TIMESTAMP_NS;
  size_t old_current_writer_index = invalid_writer_index_;
  RSRecordClipManager::Ptr old_current_writer_ = nullptr;
  size_t need_add_clip_index = invalid_writer_index_;
  bool isNotNeedCreateClip = true;
  {
    old_current_writer_index = current_writer_index_;
    const size_t current_next_writer_index = current_writer_index_ + 1;
    RS_INFO_STREAM(
        pSharedNode_,
        "before current_writer_index_ = "
            << current_writer_index_
            << ", current_next_writer_index = " << current_next_writer_index
            << ", global_writer_index_ = " << global_writer_index_);
    {
      std::lock_guard<std::mutex> lg(current_write_mtx_);
      auto iterMap = writer_mapper_.find(old_current_writer_index);
      if (iterMap != writer_mapper_.end()) {
        old_current_writer_ = writer_mapper_[old_current_writer_index];
        writer_mapper_.erase(iterMap);
      }
      isNotNeedCreateClip = (writer_mapper_.find(current_next_writer_index) !=
                             writer_mapper_.end());
      if (isNotNeedCreateClip) {
        if (writer_mapper_[current_next_writer_index] != nullptr) {
          writer_mapper_[current_next_writer_index]->start();
        }
        current_writer_index_ = current_next_writer_index;
        RS_INFO_STREAM(pSharedNode_,
                           "Match Next Clip: " << current_next_writer_index
                                               << ", global_writer_index_ = "
                                               << global_writer_index_);
      } else {
        RSRecordClipManager::Ptr recordClipInfoPtr = nullptr;
        try {
          recordClipInfoPtr.reset(new RSRecordClipManager());
        } catch (const std::exception &e) {
#if RS_ENABLE_COLLECT_USE_CHINESE
          const std::string &error_info = "分配Record分段管理器内存失败 !";
#else
          const std::string &error_info = "Malloc Record Clip Manager Failed !";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
          RS_ERROR_STREAM(pSharedNode_, error_info);

          // websocket反馈
          LocalErrorCallback(-1, error_info);

          return -1;
        }

        const auto &local_callback =
            std::bind(&RSRecordIoManager::LocalErrorCallback, this,
                      std::placeholders::_1, std::placeholders::_2);
        int ret =
            recordClipInfoPtr->init(pSharedGlobalConfig_, pSharedNode_,
                                    local_callback, topicmetadatas_, false);
        if (ret == 0) {
          recordClipInfoPtr->start();
          // std::lock_guard<std::mutex> lg(current_write_mtx_);
          writer_mapper_[global_writer_index_] = recordClipInfoPtr;
          current_writer_index_ = global_writer_index_;
          ++global_writer_index_;
        } else {
#if RS_ENABLE_COLLECT_USE_CHINESE
          const std::string &error_info =
              "初始化Record分片管理器失败: ret = " + std::to_string(ret);
#else
          const std::string &error_info =
              "Initial Record Clip Manager Failed: ret = " +
              std::to_string(ret);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
          RS_ERROR_STREAM(pSharedNode_, error_info);

          // websocket反馈
          LocalErrorCallback(-1, error_info);

          return -2;
        }
      }

      {
        std::lock_guard<std::mutex> lg(current_writer_mtx2_);
        current_writer_ = writer_mapper_[current_writer_index_];
      }
      RS_INFO_STREAM(
          pSharedNode_,
          "after current_writer_index_ = "
              << current_writer_index_
              << ", current_next_writer_index = " << current_next_writer_index
              << ", global_writer_index_ = " << global_writer_index_);

      need_add_clip_index = global_writer_index_;
    }
  }

  if (isAddNextClip) {
    std::lock_guard<std::mutex> lg(need_next_clip_buffer_mtx_);
    need_next_clip_buffer_.push(need_add_clip_index);
    need_next_clip_buffer_cond_.notify_one();
    RS_INFO_STREAM(pSharedNode_,
                       "need_add_clip_index = " << need_add_clip_index);
  }

  const uint64_t midTimestampNs = RS_TIMESTAMP_NS;
  if (old_current_writer_index != current_writer_index_ &&
      old_current_writer_ != nullptr) {
    std::lock_guard<std::mutex> lg(erase_writer_mtx_);
    erase_writer_indexs_.insert(
        {old_current_writer_index, old_current_writer_});
    erase_writer_cond_.notify_one();
    RS_INFO_STREAM(
        pSharedNode_,
        "old_current_writer_index = " << old_current_writer_index);
  }
  const uint64_t endTimestampNs = RS_TIMESTAMP_NS;

  RS_INFO_STREAM(
      pSharedNode_,
      "New Clip: endTimestampNs - midTimestampNs: "
          << (endTimestampNs - midTimestampNs) * 1e-9
          << ", midTimestampNs - startTimestampNs: "
          << (midTimestampNs - startTimestampNs) * 1e-9
          << ", current_writer_index_ = " << current_writer_index_
          << ", old_current_writer_index = " << old_current_writer_index);

  return 0;
}

bool RSRecordIoManager::CheckIsValidClip() {
  // RCLCPP_ERROR(pSharedNode_->get_logger(),  "current_writer_index_ = " <<
  // current_writer_index_ << ", invalid_writer_index_ = " <<
  // invalid_writer_index_;
  return (current_writer_index_ != invalid_writer_index_);
}

bool RSRecordIoManager::InitSubscribersImpl() {
  use_sim_time_ =
      pSharedGlobalConfig_->recordConfig["recordcollectconfig"]["is_use_sim"]
          .as<bool>();
  if (!use_sim_time_) {
    SubscribeTopics(GetRequestedOrAvailableTopics());
  }
  stop_discovery_ =
      pSharedGlobalConfig_
          ->recordConfig["recordcollectconfig"]["is_discovery_disabled"]
          .as<bool>();
  if (!stop_discovery_) {
    discovery_future_ =
        std::async(std::launch::async,
                   std::bind(&RSRecordIoManager::TopicsDiscovery, this));
  }

  return true;
}

bool RSRecordIoManager::FreeSubscribersImpl() {
  stop_discovery_ = true;
  if (discovery_future_.valid()) {
    auto status = discovery_future_.wait_for(2 * topic_polling_interval_);
    if (status != std::future_status::ready) {
      RS_ERROR_STREAM(pSharedNode_,
          "discovery_future_.wait_for("
              << topic_polling_interval_.count() << ") return status: "
              << (status == std::future_status::timeout ? "timeout"
                                                        : "deferred"));
    }
  }
  return true;
}

void RSRecordIoManager::TopicsDiscovery() {
  // If using sim time - wait until /clock topic received before even creating
  // subscriptions
#if __ROS2__
  if (use_sim_time_) {
    RS_INFO(pSharedNode_,
        "use_sim_time set, waiting for /clock before starting recording...");
    while (rclcpp::ok() && stop_discovery_ == false) {
      if (pSharedNode_->get_clock()->wait_until_started(
              topic_polling_interval_)) {
        break;
      }
    }
    if (pSharedNode_->get_clock()->started()) {
      RS_INFO(pSharedNode_,
          "Sim time /clock found, starting recording.");
    }
  }
  while (rclcpp::ok() && stop_discovery_ == false) {
#elif __ROS1__
  if (use_sim_time_) {
    RS_INFO(pSharedNode_,
        "use_sim_time set, waiting for /clock before starting recording...");
    
    // 检查/clock话题是否存在
    bool clock_ready = false;
    ros::Time start_wait = ros::Time::now();
    while (ros::ok() && stop_discovery_ == false && !clock_ready) {
      ros::master::V_TopicInfo topic_info;
      ros::master::getTopics(topic_info);
      
      for (const auto& info : topic_info) {
        if (info.name == "/clock") {
          clock_ready = true;
          break;
        }
      }
      
      if (!clock_ready) {
        // 超时检查
        ros::Duration elapsed = ros::Time::now() - start_wait;
        if (elapsed.toSec() > 30.0) {  // 30s超时
          RS_WARN(pSharedNode_, "Waited 30 seconds for /clock but not found. Continuing anyway");
          break;
        }
        
        std::this_thread::sleep_for(topic_polling_interval_);
      }
    }
    
    if (clock_ready) {
      RS_INFO(pSharedNode_, "Sim time /clock found, starting recording.");
    }
  }
  
  while (ros::ok() && stop_discovery_ == false) {
#endif
    try {
      // 获取当前可用的话题
      auto topics_to_subscribe = GetRequestedOrAvailableTopics();
#if __ROS2__
      for (const auto &topic_and_type : topics_to_subscribe) {
        WarnIfNewQosForSubscribedTopic(topic_and_type.first);
      }
#endif
      auto missing_topics = GetMissingTopics(topics_to_subscribe);
      SubscribeTopics(missing_topics);

      if (!white_topics_.empty() &&
          subscriptions_.size() == white_topics_.size()) {
        RS_INFO(pSharedNode_,
            "All requested topics are subscribed. Stopping discovery...");
        return;
      }
    } catch (const std::exception &e) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Failure in topics discovery.\nError: " << e.what());
    } catch (...) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Failure in topics discovery.");
    }
    std::this_thread::sleep_for(topic_polling_interval_);
  }
}

std::unordered_map<std::string, std::string>
RSRecordIoManager::GetRequestedOrAvailableTopics() {
  std::unordered_map<std::string, std::string> topic_types;

#if __ROS2__
  auto all_topics_and_types = pSharedNode_->get_topic_names_and_types();
  for (auto iterMap = all_topics_and_types.begin();
       iterMap != all_topics_and_types.end(); ++iterMap) {
    std::set<std::string> types;
    for (size_t i = 0; i < iterMap->second.size(); ++i) {
      types.insert(iterMap->second[i]);
    }
    if (types.size() != 1) {
      continue;
    }
    topic_types.insert({iterMap->first, iterMap->second[0]});
  }
#elif __ROS1__
  ros::master::V_TopicInfo topic_info;
  if (ros::master::getTopics(topic_info)) {
    for (const auto& info : topic_info) {
      topic_types.insert({info.name, info.datatype});
    }
  } else {
    RS_WARN(pSharedNode_, "Failed to get topics from ROS master");
  }
#endif
  return topic_types;
}

std::unordered_map<std::string, std::string>
RSRecordIoManager::GetMissingTopics(
    const std::unordered_map<std::string, std::string> &all_topics) {
  std::unordered_map<std::string, std::string> missing_topics;
  for (const auto &i : all_topics) {
    if (subscriptions_.find(i.first) == subscriptions_.end()) {
      missing_topics.emplace(i.first, i.second);
    }
  }
  return missing_topics;
}

void RSRecordIoManager::SubscribeTopics(
    const std::unordered_map<std::string, std::string> &topics_and_types) {
  for (const auto &topic_with_type : topics_and_types) {
    for (size_t i = 0; i < white_topics_.size(); ++i) {
      if (white_topics_[i] == topic_with_type.first) {
#if __ROS1__
        rosbag::ConnectionInfo metadata {};
        metadata.topic = topic_with_type.first;
        metadata.datatype = topic_with_type.second;
        metadata.md5sum = "";
        metadata.msg_def = "";
#elif __ROS2__
        rosbag2_storage::TopicMetadata metadata {};
        metadata.name = topic_with_type.first;
        metadata.type = topic_with_type.second;
        metadata.serialization_format = serialization_format_;
#if (defined ROS2_ROLLING) || (defined ROS2_JAZZY)
        metadata.offered_qos_profiles = OfferedQosProfilesForTopic(topic_with_type.first);
#else
        metadata.offered_qos_profiles = SerializedOfferedQosProfilesForTopic(topic_with_type.first);
#endif
#endif  // ROS 1 or ROS 2
        SubscribeTopic(metadata);
        break;
      }
    }
  }
}

#if __ROS2__
void RSRecordIoManager::SubscribeTopic(
    const rosbag2_storage::TopicMetadata &topicmetadata) {
    Rosbag2QoS subscription_qos{
      SubscriptionQosForTopic(topicmetadata.name)};
  auto subscription = CreateSubscription(topicmetadata.name, topicmetadata.type,
                                         subscription_qos);

  if (subscription) {
    subscriptions_.insert({topicmetadata.name, subscription});
    topicmetadatas_.insert({topicmetadata.name, topicmetadata});

    RS_INFO_STREAM(pSharedNode_,
                       "Subscribed to topic '" << topicmetadata.name << "'");
  }
}
#elif __ROS1__
void RSRecordIoManager::SubscribeTopic(
    const rosbag::ConnectionInfo &topicmetadata) {

  auto subscription = CreateSubscription(topicmetadata.topic, topicmetadata.datatype);

  if (subscription) {
    subscriptions_.insert({topicmetadata.topic, subscription});
    topicmetadatas_.insert({topicmetadata.topic, topicmetadata});

    RS_INFO_STREAM(pSharedNode_,
                       "Subscribed to topic '" << topicmetadata.topic << "'");
  }
}
#endif

#if __ROS2__
std::shared_ptr<rclcpp::GenericSubscription>
RSRecordIoManager::CreateSubscription(const std::string &topic_name,
                                      const std::string &topic_type,
                                      const rclcpp::QoS &qos) {

  std::weak_ptr<RSRecordIoManager> weak_this = shared_from_this();
  auto callback =
      [weak_this, topic_name, topic_type](CALLBACK_PARAM_TYPE(rclcpp::SerializedMessage) message) {
        // RCLCPP_ERROR(pSharedNode_->get_logger(),  "RUN HERE";
        auto share_this = weak_this.lock();
        if (!share_this) {
          return;
        }
        // RCLCPP_ERROR(pSharedNode_->get_logger(),  "RUN HERE";
        share_this->SubscribeCallback(message, topic_name, topic_type);
      };
  auto subscription = pSharedNode_->create_generic_subscription(
      topic_name, topic_type, qos, callback);
  return subscription;
}
#elif __ROS1__
std::shared_ptr<ros::Subscriber>
RSRecordIoManager::CreateSubscription(const std::string &topic_name,
                                      const std::string &topic_type) {
  std::weak_ptr<RSRecordIoManager> weak_this = shared_from_this();
  
  auto callback = [weak_this, topic_name, topic_type](const topic_tools::ShapeShifter::ConstPtr& message) {
    auto share_this = weak_this.lock();
    if (!share_this) {
      return;
    }
    share_this->SubscribeCallback(message, topic_name, topic_type);
  };
  
  ros::SubscribeOptions ops;
  ops.template init<topic_tools::ShapeShifter>(topic_name, 5, callback);
  ops.transport_hints = ros::TransportHints();
  ops.datatype = topic_type;
  
  return std::make_shared<ros::Subscriber>(pSharedNode_->subscribe(ops));
}
#endif

#if __ROS2__
std::string RSRecordIoManager::SerializedOfferedQosProfilesForTopic(
    const std::string &topic_name) {
  YAML::Node offered_qos_profiles;
  auto endpoints = pSharedNode_->get_publishers_info_by_topic(topic_name);
  for (const auto &info : endpoints) {
    offered_qos_profiles.push_back(
        Rosbag2QoS(info.qos_profile()));
  }
  return YAML::Dump(offered_qos_profiles);
}

std::vector<rclcpp::QoS> RSRecordIoManager::OfferedQosProfilesForTopic(std::string const & topic_name) {
  std::vector<rclcpp::QoS> offered_qos_profiles {};
  auto endpoints = pSharedNode_->get_publishers_info_by_topic(topic_name);
  for (auto const & info : endpoints) {
    offered_qos_profiles.push_back(info.qos_profile());
  }
  return offered_qos_profiles;
}

rclcpp::QoS RSRecordIoManager::SubscriptionQosForTopic(
    const std::string &topic_name) const {
  if (topic_qos_profile_overrides_.count(topic_name)) {
    RS_INFO_STREAM(pSharedNode_,
                       "Overriding subscription profile for " << topic_name);
    return topic_qos_profile_overrides_.at(topic_name);
  }
  return Rosbag2QoS::adapt_request_to_offers(
      topic_name, pSharedNode_->get_publishers_info_by_topic(topic_name));
}

void RSRecordIoManager::WarnIfNewQosForSubscribedTopic(
    const std::string &topic_name) {
  auto existing_subscription = subscriptions_.find(topic_name);
  if (existing_subscription == subscriptions_.end()) {
    // Not subscribed yet
    return;
  }
  if (topics_warned_about_incompatibility_.count(topic_name) > 0) {
    // Already warned about this topic
    return;
  }
  const auto actual_qos = existing_subscription->second->get_actual_qos();
  const auto &used_profile = actual_qos.get_rmw_qos_profile();
  auto publishers_info = pSharedNode_->get_publishers_info_by_topic(topic_name);
  for (const auto &info : publishers_info) {
    auto new_profile = info.qos_profile().get_rmw_qos_profile();
    bool incompatible_reliability =
        new_profile.reliability == RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT &&
        used_profile.reliability != RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    bool incompatible_durability =
        new_profile.durability == RMW_QOS_POLICY_DURABILITY_VOLATILE &&
        used_profile.durability != RMW_QOS_POLICY_DURABILITY_VOLATILE;

    if (incompatible_reliability) {
      RS_WARN_STREAM(pSharedNode_,
          "A new publisher for subscribed topic "
              << topic_name
              << " "
                 "was found offering RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT, "
                 "but rosbag already subscribed requesting "
                 "RMW_QOS_POLICY_RELIABILITY_RELIABLE. "
                 "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    } else if (incompatible_durability) {
      RS_WARN_STREAM(pSharedNode_,
          "A new publisher for subscribed topic "
              << topic_name
              << " "
                 "was found offering RMW_QOS_POLICY_DURABILITY_VOLATILE, "
                 "but rosbag2 already subscribed requesting "
                 "RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL. "
                 "Messages from this new publisher will not be recorded.");
      topics_warned_about_incompatibility_.insert(topic_name);
    }
  }
}
#endif

void RSRecordIoManager::SubscribeCallback(
    CALLBACK_PARAM_TYPE(SerializedMessage) message,
    const std::string &topic_name, const std::string &topic_type) {
  // RCLCPP_ERROR_STREAM(pSharedNode_->get_logger(),
  //                     "topic_name = " << topic_name
  //                                     << ", is_record_ = " << is_record_);
#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  if (!is_record_ || !is_started_ || is_stopping_) {
    // RCLCPP_WARN(pSharedNode_->get_logger(), "record procedure is not started
    // or stopping.");
    return;
  }
#else
  if (!is_started_ || is_stopping_) {
    // RCLCPP_WARN(pSharedNode_->get_logger(), "record procedure is not started
    // or stopping.");
    return;
  }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR

#if (!defined ROS2_JAZZY) && (!defined ROS2_ROLLING)
  if (message == nullptr) {
    RS_ERROR_STREAM(pSharedNode_,
                        "message is nullptr, channel: " << topic_name);
    return;
  }
#endif

  // RCLCPP_ERROR_STREAM(pSharedNode_->get_logger(),
  //                     "topic_name = " << topic_name
  //                                     << ", is_record_ = " << is_record_);
  message_time_ = ROSTime();
  if (is_record_) {
    RSRecordSingleMessage::Ptr buffer_message = nullptr;
    buffer_message.reset(new RSRecordSingleMessage(topic_name, topic_type,
#if __ROS1__
                                                   message_time_, boost::const_pointer_cast<SerializedMessage>(message)));
#elif __ROS2__
                                                   message_time_, std::const_pointer_cast<SerializedMessage>(message)));
#endif
    std::lock_guard<std::mutex> lg(record_buffer_mtx_);
    record_buffer_.push(buffer_message);
    record_buffer_cond_.notify_one();
#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
    if (pSharedMessageMonitor_ != nullptr &&
        need_message_monitor_channel_names_.count(topic_name)) {
      pSharedMessageMonitor_->updateMessageTimestamp(topic_name, message_time_);
    }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  }
#if !ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  if (pSharedMessageMonitor_ != nullptr &&
      need_message_monitor_channel_names_.count(topic_name)) {
    pSharedMessageMonitor_->updateMessageTimestamp(topic_name, message_time_);
  }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
  message_count_++;
}

void RSRecordIoManager::DiskCheckThread() {
  uint64_t timestampNs = RS_TIMESTAMP_NS;
  uint64_t timestampThNs = 10000000000ul; // 10s
  while (is_started_ && !is_stopping_) {
    uint64_t cur_timestampNs = RS_TIMESTAMP_NS;
    if (cur_timestampNs - timestampNs > timestampThNs) {
      RS_INFO_STREAM(pSharedNode_,
                         "Current Record Time: "
                             << std::setprecision(3)
                             << message_time_ / 1000000000
                             << "    Progress: " << subscriptions_.size()
                             << " channels, " << message_count_ << " messages");

      // Step1: 检查保存路径是否合法
      CheckSaveDiskValid();

      timestampNs = cur_timestampNs;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void RSRecordIoManager::CheckSaveDiskValid() {
  // 路径是否存在
  if (is_record_) {
    bool isDiskMount = true;
    if (pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]
                          ["enable_collect_disk_mount_monitor"]
            .as<bool>()) {
      isDiskMount = RSFileSystem::isDirectoryExist(
          pSharedGlobalConfig_
              ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
              .as<std::string>());
      if (isDiskMount == false) {
        // FauleReporter
#if RS_ENABLE_COLLECT_USE_CHINESE
        const std::string &error_info =
            "采集采集保存目录不存在: " +
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                .as<std::string>();
#else
        const std::string &error_info =
            "Collect Save Directory Is Not Exist: " +
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                .as<std::string>();
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);

        LocalErrorCallback(-1, error_info);

        // 重新挂载
        if (pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["enable_disk_mount_retry"]
                .as<bool>()) {
          DeviceRetryMount();
        } else {
          RS_INFO(pSharedNode_,
                      "Disable Device Retry Mount !");
        }
      }
    }

    if (pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["enable_disk_mount_retry"]
            .as<bool>() &&
        is_disk_mount_retry_debug_) {
      RS_INFO(pSharedNode_,
          "Enable Disk Mount Retry Debug !");
      DeviceRetryMount();
    }

    bool isDiskWriteProperty = true;
    if (isDiskMount) {
      // 检查是否为可写/可读
      bool isDiskReadWriteUsrRole = true;
      bool isDiskReadWriteGrpRole = true;
      bool isDiskReadWriteOthRole = true;
      for (size_t i = 0;
           i < pSharedGlobalConfig_
                   ->recordConfig["recorddataconfig"]
                                 ["collect_disk_write_check_modes"]
                   .size();
           ++i) {
        const auto checkMode = fromStringToCollectDiskWriteCheckType(
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]
                              ["collect_disk_write_check_modes"]
                              [static_cast<int>(i)]
                .as<std::string>());
        switch (checkMode) {
        case RS_COLLECT_DISK_WRITE_CHECK_TYPE::
            RS_COLLECT_DISK_WRITE_CHECK_USR: {
          isDiskReadWriteUsrRole = RSFileSystem::checkIsReadWriteWithUsrRole(
              pSharedGlobalConfig_
                  ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                  .as<std::string>());
          break;
        }
        case RS_COLLECT_DISK_WRITE_CHECK_TYPE::
            RS_COLLECT_DISK_WRITE_CHECK_GRP: {
          isDiskReadWriteGrpRole = RSFileSystem::checkIsReadWriteWithGrpRole(
              pSharedGlobalConfig_
                  ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                  .as<std::string>());
          break;
        }
        case RS_COLLECT_DISK_WRITE_CHECK_TYPE::
            RS_COLLECT_DISK_WRITE_CHECK_OTH: {
          isDiskReadWriteOthRole = RSFileSystem::checkIsReadWriteWithOthRole(
              pSharedGlobalConfig_
                  ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                  .as<std::string>());
          break;
        }
        }
      }

      if (!(isDiskReadWriteUsrRole && isDiskReadWriteGrpRole &&
            isDiskReadWriteOthRole)) {
        isDiskWriteProperty = false;
        // FaultReporter
#if RS_ENABLE_COLLECT_USE_CHINESE
        const std::string &error_info =
            "数据采集可写/可读属性检查不通过: usr = " +
            std::to_string(isDiskReadWriteUsrRole ? 1 : 0) +
            ", grp = " + std::to_string(isDiskReadWriteGrpRole ? 1 : 0) +
            ", oth = " + std::to_string(isDiskReadWriteOthRole ? 1 : 0);
#else
        const std::string &error_info =
            "Collect Save Directory Path Check Write/Read Property Not Pass: "
            "usr "
            "= " +
            std::to_string(isDiskReadWriteUsrRole ? 1 : 0) +
            ", grp = " + std::to_string(isDiskReadWriteGrpRole ? 1 : 0) +
            ", oth = " + std::to_string(isDiskReadWriteOthRole ? 1 : 0);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);

        LocalErrorCallback(-1, error_info);

        // 重新挂载
        if (pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["enable_disk_mount_retry"]
                .as<bool>()) {
          DeviceRetryMount();
        } else {
          RS_INFO(pSharedNode_,
                      "Disable Device Retry Mount !");
        }
      }
      RS_INFO_STREAM(
          pSharedNode_,
          "isDiskReadWriteUsrRole = "
              << isDiskReadWriteUsrRole
              << ", isDiskReadWriteGrpRole = " << isDiskReadWriteGrpRole
              << ", isDiskReadWriteOthRole = " << isDiskReadWriteOthRole);
    }

    // 路径空间检查
    if (isDiskMount) {
      if (pSharedGlobalConfig_
              ->recordConfig["recorddataconfig"]
                            ["enable_collect_freespace_monitor"]
              .as<bool>()) {
        double total_space_gbyte = 0;
        double avaiable_spsace_gbyte = 0;
        double free_space_gbyte = 0;
        bool isSuccess = RSFileSystem::getDirectorySpace(
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                .as<std::string>(),
            total_space_gbyte, avaiable_spsace_gbyte, free_space_gbyte);
        if (isSuccess) {
          if (avaiable_spsace_gbyte <
              pSharedGlobalConfig_
                  ->recordConfig["recorddataconfig"]
                                ["collect_freespace_min_gbyte"]
                  .as<uint32_t>()) {
            // FaultReporter
#if RS_ENABLE_COLLECT_USE_CHINESE
            const std::string &error_info =
                "数据采集保存硬盘要求最小可用空间为: " +
                std::to_string(pSharedGlobalConfig_
                                   ->recordConfig["recorddataconfig"]
                                                 ["collect_freespace_min_gbyte"]
                                   .as<uint32_t>()) +
                "GB, 但是当前可用空间为: " +
                std::to_string(avaiable_spsace_gbyte) + " GB !";
#else
            const std::string &error_info =
                "Collect Need Min Freespace Is: " +
                std::to_string(pSharedGlobalConfig_
                                   ->recordConfig["recorddataconfig"]
                                                 ["collect_freespace_min_gbyte"]
                                   .as<uint32_t>()) +
                " GB, But Now Available Is: " +
                std::to_string(avaiable_spsace_gbyte) + " GB !";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
            RS_ERROR_STREAM(pSharedNode_, error_info);

            LocalErrorCallback(-1, error_info);
          }
          RS_INFO_STREAM(
              pSharedNode_,
              "total_space_gbyte = "
                  << total_space_gbyte
                  << ", avaiable_spsace_gbyte = " << avaiable_spsace_gbyte
                  << ", free_space_gbyte = " << free_space_gbyte
                  << ", threshold = "
                  << pSharedGlobalConfig_
                         ->recordConfig["recorddataconfig"]
                                       ["collect_freespace_min_gbyte"]
                         .as<uint32_t>());
        }
      }
    }

    // 硬盘挂载恢复正常
    if (pre_disk_mount_ == false && isDiskMount == true) {
#if RS_ENABLE_COLLECT_USE_CHINESE
      const std::string &error_info = "硬盘挂载恢复正常 !";
#else
      const std::string &error_info = "Disk Mount Return Normal !";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
      RS_INFO_STREAM(pSharedNode_, error_info);
    }
    pre_disk_mount_ = isDiskMount;

    // 硬盘读写属性恢复正常
    if (pre_disk_write_property_ == false && isDiskWriteProperty == true) {
#if RS_ENABLE_COLLECT_USE_CHINESE
      const std::string &error_info = "硬盘读写属性正常 !";
#else
      const std::string &error_info = "Disk Write/Read Property Normal !";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
      RS_INFO_STREAM(pSharedNode_, error_info);
    }
    pre_disk_write_property_ = isDiskWriteProperty;
  }
}

void RSRecordIoManager::EraseClipRecord(
    const size_t write_index, RSRecordClipManager::Ptr &clipWriterPtr) {
  RS_INFO_STREAM(pSharedNode_,
                     "erase clip write_index = " << write_index << ", pid = "
                                                 << std::this_thread::get_id());
  bool isErase = false;
  if (clipWriterPtr != nullptr) {
    // 需要释放
    if (clipWriterPtr != nullptr) {
      if (clipWriterPtr->checkIsNeedFinish()) {
        std::lock_guard<std::mutex> lg(erase_writer_timeout_mtx_);
        bool isInclude = false;
        for (size_t i = 0; i < erase_writer_timeout_indexs_.size(); ++i) {
          if (erase_writer_timeout_indexs_[i].index == write_index) {
            isInclude = true;
            break;
          }
        }
        if (!isInclude) {
          const uint64_t timestampNs = RS_TIMESTAMP_NS;
          RSTimeoutWriterInfo info;
          info.index = write_index;
          info.timestamp = timestampNs;
          info.writer = clipWriterPtr;
          erase_writer_timeout_indexs_.push_back(info);
        }
      }
    }

    // 用于测试
    if (enable_clip_erase_timeout_debug_) {
      static int clip_erase_timeout_count = 0;
      if (clip_erase_timeout_count < clip_erase_timeout_debug_count_) {
        RS_ERROR_STREAM(
            pSharedNode_,
            "enable clip erase timeout debug: clip_erase_timeout_count = "
                << clip_erase_timeout_count
                << ", clip_erase_timeout_debug_count_ = "
                << clip_erase_timeout_debug_count_);
        ++clip_erase_timeout_count;
        std::this_thread::sleep_for(
            std::chrono::seconds(clip_erase_timeout_th_ns_ + 5));
        RS_ERROR_STREAM(
            pSharedNode_,
            "enable clip erase timeout debug: clip_erase_timeout_count = "
                << clip_erase_timeout_count
                << ", clip_erase_timeout_debug_count_ = "
                << clip_erase_timeout_debug_count_);
      }
    }

    // auto& clipWriterPtr = iterMap->second;
    if (clipWriterPtr != nullptr) {
      if (clipWriterPtr->checkIsNeedFinish()) {
        int ret = clipWriterPtr->finish();
        if (ret != 0) {
#if RS_ENABLE_COLLECT_USE_CHINESE
          const std::string &error_info =
              "写Clip数据到目录: " + clipWriterPtr->clipRootDirectory() +
              " 失败: ret = " + std::to_string(ret);
#else
          const std::string &error_info =
              "Write Clip Data To Clip Directory: " +
              clipWriterPtr->clipRootDirectory() +
              " Failed: ret  = " + std::to_string(ret);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
          RS_ERROR_STREAM(pSharedNode_, error_info);

          // websocket反馈
          LocalErrorCallback(-1, error_info);
        } else {
          RS_INFO_STREAM(pSharedNode_,
                             "Write Clip Data To Clip Directory: "
                                 << clipWriterPtr->clipRootDirectory()
                                 << " Successed !");
        }
        isErase = true;
      } else {
        RS_INFO(pSharedNode_, "not need finish clip !");
      }
    }
    clipWriterPtr.reset();
    // 删除erase时间记录
    {
      std::lock_guard<std::mutex> lg(erase_writer_timeout_mtx_);
      for (size_t i = 0; i < erase_writer_timeout_indexs_.size(); ++i) {
        if (erase_writer_timeout_indexs_[i].index == write_index) {
          erase_writer_timeout_indexs_.erase(
              erase_writer_timeout_indexs_.begin() + i);
          break;
        }
      }
    }

    // 超时后又正常释放
    {
      std::lock_guard<std::mutex> lg(RSRecordIoManager::ERASE_TIMEOUT_MTX_);
      auto iterMap =
          RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_.find(write_index);
      if (iterMap != RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_.end()) {
        if (iterMap->second != nullptr) {
          iterMap->second->setIsWriterTimeout(false);
        }

        RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_.erase(iterMap);
        if (erase_timeout_clip_cnt_ > 0) {
          --erase_timeout_clip_cnt_;
        }
      }
    }
  }
  RS_INFO_STREAM(pSharedNode_,
                     "isErase = " << isErase
                                  << ", write_index = " << write_index
                                  << ", pid = " << std::this_thread::get_id());
}

void RSRecordIoManager::NeedNextClipThread() {
  while (is_started_ && !is_stopping_) {
    size_t index = invalid_writer_index_;
    {
      std::unique_lock<std::mutex> lg(need_next_clip_buffer_mtx_);
      need_next_clip_buffer_cond_.wait(lg, [this] {
        return !need_next_clip_buffer_.empty() ||
               !(is_started_ && !is_stopping_);
      });
      if (!(is_started_ && !is_stopping_)) {
        break;
      }
      index = need_next_clip_buffer_.front();
      need_next_clip_buffer_.pop();
    }

    if (index == invalid_writer_index_) {
      continue;
    }

    RS_INFO_STREAM(pSharedNode_,
                       "add next clip writer index = " << index);

    std::lock_guard<std::mutex> lg(current_write_mtx_);
    if (writer_mapper_.find(index) != writer_mapper_.end()) {
      RS_INFO_STREAM(pSharedNode_,
                         "index already exist: index = " << index);
      continue;
    } else {
      RSRecordClipManager::Ptr recordClipInfoPtr = nullptr;
      try {
        recordClipInfoPtr.reset(new RSRecordClipManager());
      } catch (const std::exception &e) {
#if RS_ENABLE_COLLECT_USE_CHINESE
        const std::string &error_info =
            "分配下一个分片管理器失败，添加下一个分片失败: index = " +
            std::to_string(index) +
            ", global_writer_index_ = " + std::to_string(global_writer_index_);
#else
        const std::string &error_info =
            "Malloc Next Record Clip Manager Failed, Add Next Clip Failed: "
            "index = " +
            std::to_string(index) +
            ", global_writer_index_ = " + std::to_string(global_writer_index_);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);

        // websocket反馈
        LocalErrorCallback(-1, error_info);

        continue;
      }

      const auto &local_callback =
          std::bind(&RSRecordIoManager::LocalErrorCallback, this,
                    std::placeholders::_1, std::placeholders::_2);
      int ret = recordClipInfoPtr->init(pSharedGlobalConfig_, pSharedNode_,
                                        local_callback, topicmetadatas_, true);
      if (ret == 0) {
        // std::lock_guard<std::mutex> lg(current_write_mtx_);
        writer_mapper_[index] = recordClipInfoPtr;
        global_writer_index_ = std::max(index, global_writer_index_);
        ++global_writer_index_;
        RS_INFO_STREAM(pSharedNode_,
                           "Add Next Clip Successed: index = "
                               << index << ", global_writer_index_ = "
                               << global_writer_index_);
      } else {
#if RS_ENABLE_COLLECT_USE_CHINESE
        const std::string &error_info =
            "初始化下一个分片管理器失败: ret = " + std::to_string(ret) +
            ", 添加下一个分片管理器失败: index = " + std::to_string(index) +
            ", global_writer_index_ = " + std::to_string(global_writer_index_);
#else
        const std::string &error_info =
            "Initial Next Record Clip Manager Failed: ret = " +
            std::to_string(ret) +
            ", Add Next Clip Failed: index = " + std::to_string(index) +
            ", global_writer_index_ = " + std::to_string(global_writer_index_);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);

        // websocket反馈
        LocalErrorCallback(-1, error_info);
      }
    }
  }
  RS_INFO(pSharedNode_, "Need Next Clip Thread Exit !");
}

void RSRecordIoManager::EraseWriterIndexThread() {
  while ((is_started_ && !is_stopping_) || !erase_writer_indexs_.empty()) {
    size_t index = invalid_writer_index_;
    RSRecordClipManager::Ptr index_writer = nullptr;
    {
      std::unique_lock<std::mutex> lg(erase_writer_mtx_);
      erase_writer_cond_.wait(lg, [this] {
        return !erase_writer_indexs_.empty() || !(is_started_ && !is_stopping_);
      });
      if (!(is_started_ && !is_stopping_) && erase_writer_indexs_.empty()) {
        break;
      } else if (erase_writer_indexs_.empty()) {
        continue;
      }
      const auto &iterMap = erase_writer_indexs_.begin();
      index = iterMap->first;
      index_writer = iterMap->second;
      erase_writer_indexs_.erase(iterMap);
    }

    if (index == invalid_writer_index_ || index_writer == nullptr) {
      continue;
    }

    RS_INFO_STREAM(pSharedNode_,
                       "erase clip writer index = "
                           << index
                           << ", pid = " << std::this_thread::get_id());
    {
      // std::lock_guard<std::mutex> lg(current_write_mtx_);
      EraseClipRecord(index, index_writer);
    }
  }
  RS_INFO(pSharedNode_, "Erase Writer Index Thread Exit !");
}

void RSRecordIoManager::EraseWriterIndexTimeoutThread() {
  while (is_started_ && !is_stopping_) {
    const uint64_t timestampNs = RS_TIMESTAMP_NS;
    {
      std::lock_guard<std::mutex> lg(erase_writer_timeout_mtx_);
      for (size_t i = 0; i < erase_writer_timeout_indexs_.size(); ++i) {
        const auto &eraser_writer_timeout_index =
            erase_writer_timeout_indexs_[i];

        if (eraser_writer_timeout_index.writer != nullptr) {
          if (timestampNs > eraser_writer_timeout_index.timestamp) {
            const uint64_t diff =
                timestampNs - eraser_writer_timeout_index.timestamp;
            if (diff > clip_erase_timeout_th_ns_) {
#if RS_ENABLE_COLLECT_USE_CHINESE
              const std::string &error_info =
                  "分片写编号Index = " +
                  std::to_string(eraser_writer_timeout_index.index) +
                  " 删除超时: diff = " + std::to_string(diff) +
                  " (ns), clip_erase_timeout_th_ns_ = " +
                  std::to_string(clip_erase_timeout_th_ns_);
#else
              const std::string &error_info =
                  "Clip Writer Index = " +
                  std::to_string(eraser_writer_timeout_index.index) +
                  " Erase Timeout: diff = " + std::to_string(diff) +
                  " (ns), clip_erase_timeout_th_ns_ = " +
                  std::to_string(clip_erase_timeout_th_ns_);
#endif // RS_ENABLE_COLLECT_USE_CHINESE
              RS_ERROR_STREAM(pSharedNode_, error_info);

              // websocket反馈
              LocalErrorCallback(-1, error_info);

              // 创建其他关闭线程
              int ret = eraser_writer_timeout_index.writer->writeTiemoutError();
              if (ret != 0) {
                RS_ERROR_STREAM(pSharedNode_,
                    "Write Clip Timeout Error Info Failed: ret = " << ret);
              }

              // 不再保存
              eraser_writer_timeout_index.writer->setIsWriterTimeout(true);

              // 保存到超时的Mapper中
              {
                std::lock_guard<std::mutex> lg(
                    RSRecordIoManager::ERASE_TIMEOUT_MTX_);
                RSRecordIoManager::ERASE_TIMEOUT_WRITER_MAPPER_
                    [eraser_writer_timeout_index.index] =
                        eraser_writer_timeout_index.writer;
              }

              // 删除超时
              erase_writer_timeout_indexs_.erase(
                  erase_writer_timeout_indexs_.begin() + i);
              --i;

              ++erase_timeout_clip_cnt_;
              if (erase_timeout_clip_cnt_ >= RS_ERASE_THREAD_CNT) {
                std::shared_ptr<std::thread> candidate_thread =
                    std::make_shared<std::thread>(
                        [this]() { this->EraseWriterIndexThread(); });
                if (candidate_thread == nullptr) {
#if RS_ENABLE_COLLECT_USE_CHINESE
                  const std::string &error_info =
                      "分片写编号Index = " +
                      std::to_string(eraser_writer_timeout_index.index) +
                      " 删除超时: diff = " + std::to_string(diff) +
                      " (ns), clip_erase_timeout_th_ns_ = " +
                      std::to_string(clip_erase_timeout_th_ns_) +
                      ", 创建后备释放线程失败!";
#else
                  const std::string &error_info =
                      "Clip Writer Index = " +
                      std::to_string(eraser_writer_timeout_index.index) +
                      " Erase Timeout: diff = " + std::to_string(diff) +
                      " (ns), clip_erase_timeout_th_ns_ = " +
                      std::to_string(clip_erase_timeout_th_ns_) +
                      ", create candidate thread failed";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
                  RS_ERROR_STREAM(pSharedNode_, error_info);
                  // websocket反馈
                  LocalErrorCallback(-1, error_info);
                }

                if (candidate_thread != nullptr) {
                  candidate_thread->detach();
                  erase_writer_timeout_threads_.push_back(candidate_thread);

                  RS_INFO(pSharedNode_,
                      "Add Candidate Erase Write Index Thread Successed !");
                } else {
                  RS_ERROR_STREAM(pSharedNode_,
                      "Add Candidate Erase Write Index Thread Failed !");
                }
              }
            }
          }
        } else {
          erase_writer_timeout_indexs_.erase(
              erase_writer_timeout_indexs_.begin() + i);
          --i;
        }
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  RS_INFO(pSharedNode_, "Erase Writer Index Timeout Thread Exit !");
}

void RSRecordIoManager::RecordBufferThread() {
  while (is_started_ && !is_stopping_) {
    RSRecordSingleMessage::Ptr buffer_message = nullptr;
    {
      std::unique_lock<std::mutex> lg(record_buffer_mtx_);
      record_buffer_cond_.wait(lg, [this] {
        return !record_buffer_.empty() || !(is_started_ && !is_stopping_);
      });
      if (!(is_started_ && !is_stopping_)) {
        break;
      } else if (record_buffer_.empty()) {
        continue;
      }
      buffer_message = record_buffer_.front();
      record_buffer_.pop();
    }

    if (buffer_message) {
      std::lock_guard<std::mutex> lg(current_writer_mtx2_);
      if (current_writer_ == nullptr) {
        return;
      }
      current_writer_->writeMessage2(buffer_message);
    }
  }
  RS_INFO(pSharedNode_, "Record Buffer Thread Exit !");
}

void RSRecordIoManager::LocalErrorCallback(const int error_code,
                                           const std::string &error_info) {
  if (errorCallback_ != nullptr) {
    errorCallback_(error_code, error_info);
  }
}

void RSRecordIoManager::LocalMonitorCallback(
    const std::string &topic_name,
    const RS_MESSAGE_MONITOR_STATUS monitor_status) {
  switch (monitor_status) {
  case RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_ERROR: {
#if RS_ENABLE_COLLECT_USE_CHINESE
    const std::string &error_info =
        "话题: " + topic_name + "数据接收数据间隔超时!";
#else
    const std::string &error_info =
        "Channel Name: " + topic_name + " Receive Data Interval Timeout !";
#endif // RS_ENABLE_COLLECT_USE_CHINESE
    RS_ERROR_STREAM(pSharedNode_, error_info);
    break;
  }
  default: {
    break;
  }
  }

  if (monitorCallback_ != nullptr) {
    monitorCallback_(topic_name, monitor_status);
  }

  // 打印Monitor Info
  RS_INFO_STREAM(pSharedNode_,
                     "Monitor Info: topic_name = "
                         << topic_name
                         << ", status = " << static_cast<int>(monitor_status));
}

bool RSRecordIoManager::InitDeviceRetryMount() {
  disk_mount_retry_cnt_ = 0;
  const std::string &rdcs_root_directory_path =
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
          .as<std::string>();
  is_default_apollo_rdcs_root_path_ =
      (rdcs_root_directory_path == "/apollo/RDCS_ROOT/");
  collect_disk_device_name_.clear();
  collect_disk_mount_point_name_.clear();
  if (!is_default_apollo_rdcs_root_path_) {
    if (!RSFileSystem::mapperFilePathToMountInfo(
            rdcs_root_directory_path, collect_disk_device_name_,
            collect_disk_mount_point_name_)) {
      RS_ERROR_STREAM(pSharedNode_,
                          "rdcs_root_directory_path = "
                              << rdcs_root_directory_path
                              << " get matched mount device name failed !");
      return false;
    }
    RS_INFO_STREAM(pSharedNode_,
                       "rdcs_root_directory_path = "
                           << rdcs_root_directory_path
                           << ", match mount device name = "
                           << collect_disk_device_name_
                           << ", match mount point name = "
                           << collect_disk_mount_point_name_);
  } else {
    RS_WARN_STREAM(pSharedNode_,
                       "rdcs_root_directory_path = "
                           << rdcs_root_directory_path
                           << " is default apollo rdcs_root path !");
  }

  return true;
}

void RSRecordIoManager::DeviceRetryMount() {
  if (!is_default_apollo_rdcs_root_path_) {
    ++disk_mount_retry_cnt_;
    const uint32_t max_disk_mount_retry_cnt =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["max_disk_mount_retry_cnt"]
            .as<uint32_t>();
    if (disk_mount_retry_cnt_ <= max_disk_mount_retry_cnt) {
      RS_INFO_STREAM(pSharedNode_,
                         "Start Disk Mount Retry: Cnt = "
                             << disk_mount_retry_cnt_
                             << ", Max Retry Cnt = " << max_disk_mount_retry_cnt
                             << ", RS_DISK_MOUNT_RETRY_FILE_PATH = "
                             << RS_DISK_MOUNT_RETRY_FILE_PATH
                             << ", collect_disk_device_name_ = "
                             << collect_disk_device_name_
                             << ", collect_disk_mount_point_name_ = "
                             << collect_disk_mount_point_name_);
      int32_t ret_value = 0;

      const std::string &mount_cmd = RS_DISK_MOUNT_RETRY_FILE_PATH + " " +
                                     collect_disk_device_name_ + " " +
                                     collect_disk_mount_point_name_;
      bool isSuccess = RSFileSystem::systemCmdReturnCode(mount_cmd, ret_value);
      if (isSuccess) {
        RS_INFO_STREAM(pSharedNode_,
                           "Disk Mount Retry Successed: Cnt = "
                               << disk_mount_retry_cnt_
                               << ", mount_cmd = " << mount_cmd);
      } else {
#if RS_ENABLE_COLLECT_USE_CHINESE
        std::string ret_error;
        if (ret_value == -1) {
          ret_error = "重新挂载时对应设备不存在: DeviceName = " +
                      collect_disk_device_name_ +
                      ", MountPointName = " + collect_disk_mount_point_name_;
        } else if (ret_value == -2) {
          ret_error = "读写模式重新挂载时失败: DeviceName = " +
                      collect_disk_device_name_ +
                      ", MountPointName = " + collect_disk_mount_point_name_;
        } else if (ret_value == -3) {
          ret_error =
              "缺少挂载需要的参数: DeviceName = " + collect_disk_device_name_ +
              ", MountPointName = " + collect_disk_mount_point_name_;
        }

        const std::string &error_info =
            "采集采集保存目录不存在: " +
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                .as<std::string>() +
            ", 重试失败: Retry Cnt = " + std::to_string(disk_mount_retry_cnt_) +
            ", mount_cmd = " + mount_cmd +
            ", ret_value = " + std::to_string(ret_value) +
            ", ret_error = " + ret_error;
#else
        std::string ret_error;
        if (ret_value == -1) {
          ret_error = "Mount Device Is Not Exist: DeviceName = " +
                      collect_disk_device_name_ +
                      ", MountPointName = " + collect_disk_mount_point_name_;
        } else if (ret_value == -2) {
          ret_error = "RW Mode Mount Device Failed: DeviceName = " +
                      collect_disk_device_name_ +
                      ", MountPointName = " + collect_disk_mount_point_name_;
        } else if (ret_value == -3) {
          ret_error =
              "Missing Mount Args: DeviceName = " + collect_disk_device_name_ +
              ", MountPointName = " + collect_disk_mount_point_name_;
        }

        const std::string &error_info =
            "Collect Save Directory Is Not Exist: " +
            pSharedGlobalConfig_
                ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
                .as<std::string>() +
            ", Retry Failed: Retry Cnt = " +
            std::to_string(disk_mount_retry_cnt_) +
            ", mount_cmd = " + mount_cmd +
            ", ret_value = " + std::to_string(ret_value) +
            ", ret_error = " + ret_error;
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);
      }
    } else {
      RS_WARN(pSharedNode_,
                  "Need Disk Mount Retry, But Not Retry Again !");
    }
  } else {
    RS_INFO(pSharedNode_,
                "Collect File Path is Default Apollo RDCS_ROOT Path Not "
                "Need Mount Retry !");
  }
}

} // namespace collect
} // namespace rs_collect
} // namespace robosense
