#ifndef RSRECORDIOMANAGER_H
#define RSRECORDIOMANAGER_H

#include <future>
#include <unordered_set>

#include "modules/rs_collect/rsmessagemonitor.h"
#include "modules/rs_collect/rsrecordclipmanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

enum class RS_COLLECT_DISK_WRITE_CHECK_TYPE : int {
  RS_COLLECT_DISK_WRITE_CHECK_USR = 1,
  RS_COLLECT_DISK_WRITE_CHECK_GRP = 2,
  RS_COLLECT_DISK_WRITE_CHECK_OTH = 4,
};

inline RS_COLLECT_DISK_WRITE_CHECK_TYPE
fromStringToCollectDiskWriteCheckType(const std::string &checkType) {
  if (checkType == "RS_COLLECT_DISK_WRITE_CHECK_GRP") {
    return RS_COLLECT_DISK_WRITE_CHECK_TYPE::RS_COLLECT_DISK_WRITE_CHECK_GRP;
  } else if (checkType == "RS_COLLECT_DISK_WRITE_CHECK_OTH") {
    return RS_COLLECT_DISK_WRITE_CHECK_TYPE::RS_COLLECT_DISK_WRITE_CHECK_OTH;
  } else {
    return RS_COLLECT_DISK_WRITE_CHECK_TYPE::RS_COLLECT_DISK_WRITE_CHECK_USR;
  }
}

class RSRecordIoManager
    : public std::enable_shared_from_this<RSRecordIoManager> {
public:
  using Ptr = std::shared_ptr<RSRecordIoManager>;
  using ConstPtr = std::shared_ptr<const RSRecordIoManager>;

public:
  RSRecordIoManager(
      const RSGlobalConfig::Ptr &pSharedGlobalConfig,
      const RSMessageMonitor::RS_MESSAGE_MONITOR_CALLBACK &callback,
      const RS_RECORD_IO_ERROR_CALLBACK &errorCallback,
      const NodeHandlePtr &pSharedNode);

  ~RSRecordIoManager();

  bool Init();
  bool Start();
  bool Stop();
  bool CheckSegment();
  bool StartRecord();
  bool StopRecord();
  int AddClipRecord(const bool isAddNextClip);
  bool CheckIsValidClip();
  size_t getTimeoutClipCnt() const { return erase_timeout_clip_cnt_; }

private:
  bool InitSubscribersImpl();

  bool FreeSubscribersImpl();

  void TopicsDiscovery();

  std::unordered_map<std::string, std::string> GetRequestedOrAvailableTopics();

  std::unordered_map<std::string, std::string> GetMissingTopics(
      const std::unordered_map<std::string, std::string> &all_topics);

  void SubscribeTopics(
      const std::unordered_map<std::string, std::string> &topics_and_types);

#if __ROS2__
  void SubscribeTopic(const rosbag2_storage::TopicMetadata &topicmetadata);
#elif __ROS1__
  void SubscribeTopic(const rosbag::ConnectionInfo &topicmetadata);
#endif

  std::string
  SerializedOfferedQosProfilesForTopic(const std::string &topic_name);

#if __ROS2__
  std::vector<rclcpp::QoS>
  OfferedQosProfilesForTopic(std::string const & topic_name);

  std::shared_ptr<rclcpp::GenericSubscription>
  CreateSubscription(const std::string &topic_name,
                     const std::string &topic_type, const rclcpp::QoS &qos);

  rclcpp::QoS SubscriptionQosForTopic(const std::string &topic_name) const;

  void WarnIfNewQosForSubscribedTopic(const std::string &topic_name);

#elif __ROS1__
  std::shared_ptr<ros::Subscriber>
  CreateSubscription(const std::string &topic_name,
                     const std::string &topic_type);
#endif

  void
  SubscribeCallback(CALLBACK_PARAM_TYPE(SerializedMessage) message,
                    const std::string &topic_name,
                    const std::string &topic_type_);

  void DiskCheckThread();

  void CheckSaveDiskValid();

  void EraseClipRecord(const size_t write_index,
                       RSRecordClipManager::Ptr &clipWriterPtr);

  void NeedNextClipThread();

  void EraseWriterIndexThread();

  void EraseWriterIndexTimeoutThread();

  void RecordBufferThread();

  void LocalErrorCallback(const int error_code, const std::string &error_info);

  void LocalMonitorCallback(const std::string &topic_name,
                            const RS_MESSAGE_MONITOR_STATUS monitor_status);

  bool InitDeviceRetryMount();

  void DeviceRetryMount();

private:
  bool is_started_ = false;
  bool is_stopping_ = false;
  NodeHandlePtr pSharedNode_ = nullptr;

private:
  std::shared_ptr<std::thread> disk_check_thread_ = nullptr;
  uint64_t message_count_;
  uint64_t message_time_;

private:
  std::vector<std::string> white_topics_;
  std::vector<std::string> black_topics_;

  bool stop_discovery_ = false;
  bool use_sim_time_ = false;
  std::chrono::milliseconds topic_polling_interval_{100};
  std::future<void> discovery_future_;

#if __ROS2__
  std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericSubscription>>
#elif __ROS1__
  std::unordered_map<std::string, std::shared_ptr<ros::Subscriber>>
#endif
      subscriptions_;

#if __ROS2__
  std::unordered_map<std::string, rosbag2_storage::TopicMetadata>
#elif __ROS1__
  std::unordered_map<std::string, rosbag::ConnectionInfo>
#endif
      topicmetadatas_;

  std::unordered_set<std::string> topics_warned_about_incompatibility_;
  std::string serialization_format_ = "cdr";
#if __ROS2__
  std::unordered_map<std::string, rclcpp::QoS> topic_qos_profile_overrides_;
#endif
  std::set<std::string> need_message_monitor_channel_names_;

  bool is_record_ = false;
  size_t global_writer_index_;
  size_t current_writer_index_;
  std::mutex current_write_mtx_;
  std::map<size_t, RSRecordClipManager::Ptr> writer_mapper_;
  std::mutex current_writer_mtx2_;
  RSRecordClipManager::Ptr current_writer_ = nullptr;
  RSGlobalConfig::Ptr pSharedGlobalConfig_;

  RSMessageMonitor::RS_MESSAGE_MONITOR_CALLBACK monitorCallback_;
  RS_RECORD_IO_ERROR_CALLBACK errorCallback_;
  RSMessageMonitor::Ptr pSharedMessageMonitor_;

  std::condition_variable need_next_clip_buffer_cond_;
  std::mutex need_next_clip_buffer_mtx_;
  std::queue<size_t> need_next_clip_buffer_;
  std::shared_ptr<std::thread> need_next_clip_thread_;

  std::condition_variable erase_writer_cond_;
  std::mutex erase_writer_mtx_;
  std::map<size_t, RSRecordClipManager::Ptr> erase_writer_indexs_;
  std::shared_ptr<std::thread> erase_writer_indexs_thread_;

  // record erase writer information
  std::mutex erase_writer_timeout_mtx_;
  struct RSTimeoutWriterInfo {
    size_t index = static_cast<size_t>(-1);
    uint64_t timestamp = 0;
    RSRecordClipManager::Ptr writer = nullptr;
  };
  std::vector<RSTimeoutWriterInfo> erase_writer_timeout_indexs_;
  std::shared_ptr<std::thread> erase_writer_timeout_thread_;

  std::vector<std::shared_ptr<std::thread>> erase_writer_timeout_threads_;

  std::mutex record_buffer_mtx_;
  std::condition_variable record_buffer_cond_;
  std::shared_ptr<std::thread> record_buffer_thread_;
  std::queue<RSRecordSingleMessage::Ptr> record_buffer_;

  uint64_t clip_erase_timeout_th_ns_ = 20000000000; // 20s

private:
  const uint32_t invalid_writer_index_ = static_cast<uint32_t>(-1);

private:
  bool enable_clip_erase_timeout_debug_ = false;
  int32_t clip_erase_timeout_debug_count_ = 0;

private:
  const size_t RS_ERASE_THREAD_CNT = 2;
  size_t erase_timeout_clip_cnt_ = 0;

private:
  bool pre_disk_mount_ = true;
  bool pre_disk_write_property_ = true;
  uint32_t disk_mount_retry_cnt_ = 0;
  bool is_disk_mount_retry_debug_ = false;
  bool is_default_apollo_rdcs_root_path_ = false;
  std::string collect_disk_device_name_ = "";
  std::string collect_disk_mount_point_name_ = "";
  const std::string RS_DISK_MOUNT_RETRY_FILE_PATH =
      "bash /apollo/scripts/deployment/mount_udisk.sh";

private:
  std::set<std::string> need_message_monitor_topic_names_;
  bool is_need_message_monitor_ = false;

public:
  // 保存删除超时的Clip
  static std::mutex ERASE_TIMEOUT_MTX_;
  static std::map<size_t, RSRecordClipManager::Ptr>
      ERASE_TIMEOUT_WRITER_MAPPER_;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSRECORDIOMANAGER_H
