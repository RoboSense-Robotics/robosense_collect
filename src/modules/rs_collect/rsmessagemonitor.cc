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
#include "modules/rs_collect/rsmessagemonitor.h"

namespace robosense {
namespace rs_collect {
namespace collect {

RSMessageMonitor::RSMessageMonitor() { is_running_ = false; }

RSMessageMonitor::~RSMessageMonitor() { stop(); }

int RSMessageMonitor::init(const RSGlobalConfig::Ptr &pSharedGlobalConfig,
                           const NodeHandlePtr pSharedNode,
                           const RS_MESSAGE_MONITOR_CALLBACK &callback) {
  if (pSharedGlobalConfig == nullptr) {
    return -1;
  } else if (callback == nullptr) {
    return -2;
  } else if (pSharedNode == nullptr) {
    return -3;
  }

  pSharedGlobalConfig_ = pSharedGlobalConfig;
  pSharedNode_ = pSharedNode;
  callback_ = callback;

  int ret = init();
  if (ret != 0) {
    return -3;
  }

  return 0;
}

int RSMessageMonitor::start() {
  is_record_ = false;
  try {
    is_running_ = true;
    check_thread_ = std::thread(&RSMessageMonitor::checkWorkThread, this);
  } catch (const std::exception &e) {
    is_running_ = false;
    return -1;
  }

  return 0;
}

int RSMessageMonitor::stop() {
  if (is_running_) {
    is_running_ = false;
    if (check_thread_.joinable()) {
      check_thread_.join();
    }
  }

  return 0;
}

void RSMessageMonitor::updateMessageTimestamp(
    const std::string &channel_name, const uint64_t message_timestamp) {
  if (!is_running_) {
    return;
  }

  std::lock_guard<std::mutex> lg(message_mtx_);
  auto iterMap = message_last_timestamp_.find(channel_name);
  if (iterMap != message_last_timestamp_.end()) {
    auto &channelInfo = iterMap->second;
    channelInfo.cur_timestamp_ = message_timestamp;
  }
}

int RSMessageMonitor::init() {
  check_th_ms_ = max_check_th_ms;
  const auto &sensors_config =
      pSharedGlobalConfig_->recordConfig["recordsensorconfig"];
  for (int i = 0; i < sensors_config["record_sensors"].size(); ++i) {
    const auto &sensor_config = sensors_config["record_sensors"][i];
    const std::string &channel_name =
        sensor_config["channel_name"].as<std::string>();
    if (!sensor_config["enable_channel"].as<bool>()) {
      continue;
    } else if (!sensor_config["enable_timeout_check"].as<bool>()) {
      continue;
    }

    RSChannelMonitorInfo info;
    info.timeout_warn_th_ =
        sensor_config["timeout_warn_th_ms"].as<uint64_t>() * 1000000;
    info.timeout_error_th_ =
        sensor_config["timeout_error_th_ms"].as<uint64_t>() * 1000000;

    message_last_timestamp_[channel_name] = info;
    check_th_ms_ =
        std::min(static_cast<uint64_t>(std::min(
                     sensor_config["timeout_warn_th_ms"].as<uint64_t>(),
                     sensor_config["timeout_error_th_ms"].as<uint64_t>())),
                 check_th_ms_);
  }

  check_th_ms_ = check_th_ms_ / 2;
  if (check_th_ms_ < min_check_th_ms) {
    check_th_ms_ = min_check_th_ms;
  }
  RS_INFO_STREAM(pSharedNode_,
                     "check_th_ms_ = " << check_th_ms_);

  return 0;
}

void RSMessageMonitor::checkWorkThread() {
  while (is_running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(check_th_ms_));
    const uint64_t current_timestamp = RS_TIMESTAMP_NS;

#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
    if (!is_record_) {
      continue;
    }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR

    {
      std::lock_guard<std::mutex> lg(message_mtx_);
#if ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
      // 开始采集一小段时间不进行检查
      if (current_timestamp < record_switch_timestamp_ ||
          current_timestamp - record_switch_timestamp_ <
              record_skip_timestamp_diff_th_ns) {
        // AINFO_EVERY(10) << "duration skip, not monitor: current_timestamp = "
        //                 << current_timestamp << ", record_switch_timestamp_ =
        //                 "
        //                 << record_switch_timestamp_
        //                 << ", record_skip_timestamp_diff_th_ns = "
        //                 << record_skip_timestamp_diff_th_ns;
        continue;
      }
#endif // ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR
      for (auto iterMap = message_last_timestamp_.begin();
           iterMap != message_last_timestamp_.end(); ++iterMap) {
        auto &channelInfo = iterMap->second;
        if (channelInfo.cur_timestamp_ != 0) {
          const uint64_t diff_timestamp =
              current_timestamp > channelInfo.cur_timestamp_
                  ? current_timestamp - channelInfo.cur_timestamp_
                  : 0;
          channelInfo.pre_status_ = channelInfo.cur_status_;
          channelInfo.diff_timestamp_ = diff_timestamp;
          if (diff_timestamp > channelInfo.timeout_error_th_) {
            channelInfo.cur_status_ =
                RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_ERROR;
          } else if (diff_timestamp > channelInfo.timeout_warn_th_) {
            channelInfo.cur_status_ =
                RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_WARNING;
          } else {
            channelInfo.cur_status_ =
                RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_NORMAL;
          }
        } else {
          channelInfo.cur_timestamp_ =
              current_timestamp; // 作为如果一直收不到数据的保底检查
          channelInfo.diff_timestamp_ = 0;
        }
      }
    }

    for (auto iterMap = message_last_timestamp_.cbegin();
         iterMap != message_last_timestamp_.cend(); ++iterMap) {
      const auto &channelInfo = iterMap->second;
      if (channelInfo.cur_status_ != channelInfo.pre_status_ &&
          callback_ != nullptr) {
        callback_(iterMap->first, channelInfo.cur_status_);

        // 打印时间戳信息，方便定位问题
        if (channelInfo.cur_status_ ==
            RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_ERROR) {
          RS_INFO_STREAM(
              pSharedNode_,
              "Monitor Error: channelName = "
                  << iterMap->first << ", channelInfo.cur_timestamp_ "
                  << std::to_string(channelInfo.cur_timestamp_)
                  << "(ns), diff_timestamp = "
                  << std::to_string(channelInfo.diff_timestamp_) << "(ns) !");
        } else if (channelInfo.cur_status_ ==
                   RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_WARNING) {
          RS_INFO_STREAM(
              pSharedNode_,
              "Monitor Warn: channelName = "
                  << iterMap->first << ", channelInfo.cur_timestamp_ "
                  << std::to_string(channelInfo.cur_timestamp_)
                  << "(ns), diff_timestamp = "
                  << std::to_string(channelInfo.diff_timestamp_) << "(ns) !");
        } else {
          RS_INFO_STREAM(
              pSharedNode_,
              "Monitor Normal: channelName = "
                  << iterMap->first << ", channelInfo.cur_timestamp_ "
                  << std::to_string(channelInfo.cur_timestamp_)
                  << "(ns), diff_timestamp = "
                  << std::to_string(channelInfo.diff_timestamp_) << "(ns) !");
        }
        // AERROR << "========== channel_name = " << iterMap->first
        //        << ", status = " << static_cast<int>(channelInfo.cur_status_);
      }
    }
  }
}

} // namespace collect
} // namespace rs_collect
} // namespace robosense