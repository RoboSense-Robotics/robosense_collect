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
#ifndef RSMESSAGEMONITOR_H
#define RSMESSAGEMONITOR_H

#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

// 是否仅仅在采集录制区间监控
#define ENABLE_ONLY_RECORD_TOPIC_TIMEOUT_MONITOR (1)

enum class RS_MESSAGE_MONITOR_STATUS : int {
  RS_MESSAGE_MONITOR_NORMAL = 0,
  RS_MESSAGE_MONITOR_WARNING,
  RS_MESSAGE_MONITOR_ERROR,
};

class RSMessageMonitor {
public:
  using Ptr = std::shared_ptr<RSMessageMonitor>;
  using ConstPtr = std::shared_ptr<const RSMessageMonitor>;

public:
  using RS_MESSAGE_MONITOR_CALLBACK =
      std::function<void(const std::string &, const RS_MESSAGE_MONITOR_STATUS)>;

private:
  class RSChannelMonitorInfo {
  public:
    RSChannelMonitorInfo() { reset(); }

    void reset() {
      cur_timestamp_ = 0;
      diff_timestamp_ = 0;
      pre_status_ = RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_NORMAL;
      cur_status_ = RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_NORMAL;
    }

  public:
    uint64_t cur_timestamp_;
    RS_MESSAGE_MONITOR_STATUS pre_status_;
    RS_MESSAGE_MONITOR_STATUS cur_status_;
    uint64_t diff_timestamp_;
    uint64_t timeout_warn_th_;
    uint64_t timeout_error_th_;
  };

public:
  RSMessageMonitor();
  ~RSMessageMonitor();

public:
  int init(const RSGlobalConfig::Ptr &pSharedGlobalConfig,
           const NodeHandlePtr pSharedNode,
           const RS_MESSAGE_MONITOR_CALLBACK &callback);
  int start();
  int stop();
  void updateRecordStatus(const bool isRecord) {
    // 当暂停->重新开始时全部状态复位
    if (isRecord) {
      std::lock_guard<std::mutex> lg(message_mtx_);
      for (auto iterMap = message_last_timestamp_.begin();
           iterMap != message_last_timestamp_.end(); ++iterMap) {
        iterMap->second.reset();
      }
      record_switch_timestamp_ = RS_TIMESTAMP_NS;
    }
    is_record_ = isRecord;
  }
  void updateMessageTimestamp(const std::string &channel_name,
                              const uint64_t message_timestamp);

private:
  int init();

private:
  void checkWorkThread();

private:
  std::mutex message_mtx_;
  std::map<std::string, RSChannelMonitorInfo> message_last_timestamp_;
  bool is_running_;
  std::thread check_thread_;
  RSGlobalConfig::Ptr pSharedGlobalConfig_;
  RS_MESSAGE_MONITOR_CALLBACK callback_;
  uint64_t check_th_ms_ = 5000;
  bool is_record_ = false;
  uint64_t record_switch_timestamp_ = 0;
  NodeHandlePtr pSharedNode_ = nullptr;

private:
  const uint64_t max_check_th_ms = 5000;
  const uint64_t min_check_th_ms = 20;
  const uint64_t record_skip_timestamp_diff_th_ns =
      30000000000; // 第一个clip不检查
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSMESSAGEMONITOR_H