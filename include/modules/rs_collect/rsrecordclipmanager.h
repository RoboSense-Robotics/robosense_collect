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
#ifndef RSRECORDCLIPMANAGER_H
#define RSRECORDCLIPMANAGER_H

#include "modules/rs_collect/rscollectinfomanager.h"
#include "modules/rs_collect/rsrecordclipchannel.h"

#if __ROS2__
#include "robosense_msgs/msg/rs_compressed_image.hpp"
using RsCompressedImage = robosense_msgs::msg::RsCompressedImage;
using RsCompressedImagePtr = robosense_msgs::msg::RsCompressedImage::SharedPtr;
#elif __ROS1__
#include "robosense_msgs/RsCompressedImage.h"
using RsCompressedImage = robosense_msgs::RsCompressedImage;
using RsCompressedImagePtr = robosense_msgs::RsCompressedImage::Ptr;
#endif

namespace robosense {
namespace rs_collect {
namespace collect {

enum class RS_RECORD_SEGMENT_TYPE : int {
  RS_RECROD_SEGMENT_NO_SEG = 0,
  RS_RECORD_SEGMENT_BY_TIME = 1,
};

inline RS_RECORD_SEGMENT_TYPE
fromStringToRecordSegmentType(const std::string &segmentType) {
  if (segmentType == "RS_RECORD_SEGMENT_BY_TIME") {
    return RS_RECORD_SEGMENT_TYPE::RS_RECORD_SEGMENT_BY_TIME;
  } else {
    return RS_RECORD_SEGMENT_TYPE::RS_RECROD_SEGMENT_NO_SEG;
  }
}

class RSRecordSingleMessage {
public:
  using Ptr = std::shared_ptr<RSRecordSingleMessage>;
  using ConstPtr = std::shared_ptr<const RSRecordSingleMessage>;

public:
  RSRecordSingleMessage() { reset(); }

  RSRecordSingleMessage(const std::string &topic_name,
                        const std::string &topic_type,
                        const ros_time_t &message_time,
                        const SerializedMessagePtr &message) {
    topic_name_ = topic_name;
    topic_type_ = topic_type;
    message_time_ = message_time;
    message_ = message;
    is_h265_message_ = false;
    is_h265_iframe_ = false;
  }

  RSRecordSingleMessage(const std::string &topic_name,
                        const std::string &topic_type,
                        const ros_time_t &message_time,
                        const SerializedMessage &message) {
    topic_name_ = topic_name;
    topic_type_ = topic_type;
    message_time_ = message_time;
    *message_ = message;
    is_h265_message_ = false;
    is_h265_iframe_ = false;
  }

  ~RSRecordSingleMessage() {
    // NOTHING TODO...
  }

  void deepCopy(const RSRecordSingleMessage &src) {
    this->topic_name_ = src.topic_name_;
    this->topic_type_ = src.topic_type_;
    this->message_time_ = src.message_time_;
    this->is_h265_message_ = src.is_h265_message_;
    this->is_h265_iframe_ = src.is_h265_iframe_;
    if (message_ == nullptr) {
      message_.reset(new SerializedMessage);
    }
    *message_ = *(src.message_);
  }

  void reset() {
    topic_name_ = "";
    topic_type_ = "";
    message_time_ = 0;
    message_ = nullptr;
    is_h265_message_ = false;
    is_h265_iframe_ = false;
  }

public:
  std::string topic_name_;
  std::string topic_type_;
  SerializedMessagePtr message_;
  bool is_h265_message_ = false;
  bool is_h265_iframe_ = false;
  ros_time_t message_time_;
};

// 一个record 文件检查信息
enum class RS_RECORD_FILE_CHECK_OP_TYPE {
  RS_RECORD_FILE_CHECK_OP_NOTHING = 0,
  RS_RECORD_FILE_CHECK_OP_DEL_CLIP,
  RS_RECORD_FILE_CHECK_OP_DEL_RECORD,
};

class RSRecordFileCheckInfo {
public:
  using Ptr = std::shared_ptr<RSRecordFileCheckInfo>;
  using ConstPtr = std::shared_ptr<const RSRecordFileCheckInfo>;

public:
  RSRecordFileCheckInfo() { reset(); }

  ~RSRecordFileCheckInfo() {}

public:
  void setRecordFileName(const std::string &recordFileName,
                         const std::string &recordClipRelDirName) {
    record_clip_rel_dir_name = recordClipRelDirName;
    record_file_name = recordFileName;
  }

  void setRecordCheckStatus(const bool status,
                            const RS_RECORD_FILE_CHECK_OP_TYPE opType,
                            const std::string &info) {
    record_file_check_status = status;
    record_op_type = opType;
    record_file_check_info = info;
  }

  void reset() {
    record_clip_rel_dir_name = "";
    record_file_name = "null";
    record_file_check_status = false;
    record_file_check_info = RSRecordFileCheckInfo::RS_RECORD_NOT_COMPLETE_INFO;
    record_op_type =
        RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_NOTHING;
  }

  YAML::Node toYamlNode() const {
    YAML::Node node;
    node["record_file_name"] = record_file_name;
    node["status"] = record_file_check_status;
    node["info"] = record_file_check_info;
    node["clip_rel_dir_name"] = record_clip_rel_dir_name;
    node["op_type"] = static_cast<int>(record_op_type);
    return node;
  }

  int fromYamlNode(const YAML::Node &node) {
    try {
      record_file_name = node["record_file_name"].as<std::string>();
      record_file_check_status = node["status"].as<bool>();
      record_file_check_info = node["info"].as<std::string>();
      record_clip_rel_dir_name = node["clip_rel_dir_name"].as<std::string>();
      record_op_type =
          static_cast<RS_RECORD_FILE_CHECK_OP_TYPE>(node["op_type"].as<int>());
    } catch (...) {
      return -1;
    }

    return 0;
  }

private:
  std::string record_file_name;
  bool record_file_check_status;
  std::string record_file_check_info;
  std::string record_clip_rel_dir_name;
  RS_RECORD_FILE_CHECK_OP_TYPE record_op_type;

public:
  static std::string RS_RECORD_NOT_COMPLETE_INFO;
  static std::string RS_RECORD_ZERO_MESSAGE_COUNT_INFO;
  static std::string RS_RECOED_CHECK_PASS_INFO;
};

class RSRecordFileCheckManager {
public:
  using Ptr = std::shared_ptr<RSRecordFileCheckManager>;
  using ConstPtr = std::shared_ptr<const RSRecordFileCheckManager>;

public:
  RSRecordFileCheckManager() { is_init_ = false; }
  ~RSRecordFileCheckManager() {}

public:
  int init(const NodeHandlePtr pSharedNode, const std::string &yamlFilePath) {
    if (pSharedNode == nullptr) {
      return -1;
    }
    pSharedNode_ = pSharedNode;
    yaml_file_path_ = yamlFilePath;
    is_init_ = true;
    return 0;
  }

  // record文件路径作为输入
  int addRecordCheckInfoFullPath(const std::string &recordFilePath,
                                 const std::string &recordClipDirName) {
    size_t las_slash_pos = recordFilePath.find_last_of('/');
    std::string record_file_name = recordFilePath;
    if (las_slash_pos != std::string::npos) {
      record_file_name = recordFilePath.substr(las_slash_pos + 1);
    }

    return addRecordCheckInfo(record_file_name, recordClipDirName);
  }

  // record文件名称作为输入
  int addRecordCheckInfo(const std::string &recordFileName,
                         const std::string &recordRelClipDirName) {
    if (record_file_check_infos_.find(recordFileName) !=
        record_file_check_infos_.end()) {
      RS_INFO_STREAM(pSharedNode_,
                     "Already Add Check Record File = " << recordFileName);
      return 0;
    }

    RSRecordFileCheckInfo checkInfo;
    checkInfo.setRecordFileName(recordFileName, recordRelClipDirName);
    record_file_check_infos_.insert({recordFileName, checkInfo});

    RS_INFO_STREAM(pSharedNode_, "Add Check Record File: recordFileName = "
                                     << recordFileName
                                     << ", recordRelClipDirName = "
                                     << recordRelClipDirName << " Successed !");
    return 0;
  }

  // record文件路径作为输入
  int setRecordCheckInfoFullPath(const std::string &recordFilePath,
                                 const bool isComplete,
                                 const bool isZeroMessageCnt) {
    size_t las_slash_pos = recordFilePath.find_last_of('/');
    std::string record_file_name = recordFilePath;
    if (las_slash_pos != std::string::npos) {
      record_file_name = recordFilePath.substr(las_slash_pos + 1);
    }

    return setRecordCheckInfo(record_file_name, isComplete, isZeroMessageCnt);
  }

  // record文件名称作为输入
  int setRecordCheckInfo(const std::string &recordFileName,
                         const bool isComplete, const bool isZeroMessageCnt) {
    auto iterMap = record_file_check_infos_.find(recordFileName);
    if (iterMap != record_file_check_infos_.end()) {
      RSRecordFileCheckInfo &info = iterMap->second;
      // Case1: 文件不完整（无论文件有效消息个数是否为0）;
      // Case2: 文件完整，文件有效消息个数为0;
      // Case3: 通过
      if (!isComplete) {
        info.setRecordCheckStatus(
            false,
            RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_DEL_CLIP,
            RSRecordFileCheckInfo::RS_RECORD_NOT_COMPLETE_INFO);
        RS_INFO_STREAM(
            pSharedNode_,
            "Set Record Check Info: recordFileName = "
                << recordFileName
                << ", status = false, "
                   "RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_DEL_"
                   "CLIP, info = "
                << RSRecordFileCheckInfo::RS_RECORD_NOT_COMPLETE_INFO);
      } else if (isZeroMessageCnt) {
        info.setRecordCheckStatus(
            false,
            RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_DEL_RECORD,
            RSRecordFileCheckInfo::RS_RECORD_ZERO_MESSAGE_COUNT_INFO);
        RS_INFO_STREAM(
            pSharedNode_,
            "Set Record Check Info: recordFileName = "
                << recordFileName
                << ", status = false, "
                   "RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_DEL_"
                   "RECORD, info = "
                << RSRecordFileCheckInfo::RS_RECORD_ZERO_MESSAGE_COUNT_INFO);
      } else {
        info.setRecordCheckStatus(
            true, RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_NOTHING,
            RSRecordFileCheckInfo::RS_RECOED_CHECK_PASS_INFO);
        RS_INFO_STREAM(
            pSharedNode_,
            "Set Record Check Info: recordFileName = "
                << recordFileName
                << ", status = true, "
                   "RS_RECORD_FILE_CHECK_OP_TYPE::RS_RECORD_FILE_CHECK_OP_"
                   "NOTHING, info = "
                << RSRecordFileCheckInfo::RS_RECOED_CHECK_PASS_INFO);
      }
    } else {
      RS_ERROR_STREAM(pSharedNode_,
                      "Set Record Check Info Not Exist: record File Name = "
                          << recordFileName);
      return -1;
    }

    return 0;
  }

  int writeRecordCheckInfo() {
    if (is_init_ == false) {
      RS_ERROR(pSharedNode_, "Record File Check Not Initial !");
      return -1;
    }

    YAML::Node checkItemsNode(YAML::NodeType::Sequence);
    for (auto iterMap = record_file_check_infos_.begin();
         iterMap != record_file_check_infos_.end(); ++iterMap) {
      YAML::Node nodeItem = iterMap->second.toYamlNode();
      checkItemsNode.push_back(nodeItem);
    }

    YAML::Node checkNode;
    checkNode["check"] = checkItemsNode;

    std::ofstream ofstr(yaml_file_path_,
                        std::ios_base::out | std::ios_base::binary);
    if (!ofstr.is_open()) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Open Yaml File = " << yaml_file_path_
                                          << " To Write Failed !");
      return -2;
    }

    YAML::Emitter emitter;
    emitter << checkNode;
    ofstr << emitter.c_str();

    ofstr.flush();
    ofstr.close();

    RS_INFO_STREAM(pSharedNode_, "Write Record Info To Yaml File = "
                                     << yaml_file_path_ << " Successed !");
    return 0;
  }

private:
  std::string yaml_file_path_;
  bool is_init_;
  NodeHandlePtr pSharedNode_ = nullptr;
  std::map<std::string, RSRecordFileCheckInfo> record_file_check_infos_;
};

class RSRecordClipManager
    : public std::enable_shared_from_this<RSRecordClipManager> {
public:
  using Ptr = std::shared_ptr<RSRecordClipManager>;
  using ConstPtr = std::shared_ptr<const RSRecordClipManager>;

public:
  RSRecordClipManager() { reset(); }

  ~RSRecordClipManager() {
    if (is_force_finish_) {
      RS_WARN_STREAM(pSharedNode_,
                     "is force finish: clipDirPath_ = " << clipDirPath_);
#if ENABLE_CLIP_MANAGER_DEBUG
      RS_INFO(pSharedNode_, "Buffer Thread Finish Clip");
      RS_INFO_STREAM(pSharedNode_, "STATISTICAL_COUNT = " << STATISTICAL_COUNT
                                                          << ", BUFFER SIZE = "
                                                          << buffer_.size());
#endif // ENABLE_CLIP_MANAGER_DEBUG
      {
        buffer_cond_.notify_all();
        if (writer_thread_ != nullptr) {
          if (writer_thread_->joinable()) {
            writer_thread_->join();
          }
          writer_thread_.reset();
        }
      }
    } else if (checkIsNeedFinish()) {
      finish();
    }
  }

  void reset() {
    clipDirPath_ = "";
    is_writer_finished_ = true;
    outputChannelWriters_.clear();
    pSharedGlobalConfig_ = nullptr;
    pSharedCollectInfoManager_ = nullptr;
    is_init_next_clip_ = false;
    is_writer_running_ = false;
    is_force_finish_ = false;
    is_first_frame_ = true;
    is_clip_already_start_ = false;
  }

  int init(const RSGlobalConfig::Ptr &pSharedGlobalConfig,
           const NodeHandlePtr &pSharedNode,
           const RS_RECORD_IO_ERROR_CALLBACK &errorCallback,
#if __ROS1__
           const std::unordered_map<std::string, rosbag::ConnectionInfo>
#elif __ROS2__
           const std::unordered_map<std::string, rosbag2_storage::TopicMetadata>
#endif
               &topicMetaDataMapper,
           const bool isInitNextClip) {
    if (pSharedGlobalConfig == nullptr) {
      return -1;
    } else if (errorCallback == nullptr) {
      return -2;
    } else if (pSharedNode == nullptr) {
      return -3;
    }

    pSharedGlobalConfig_ = pSharedGlobalConfig;
    pSharedNode_ = pSharedNode;
    topicMetaDataMapper_ = topicMetaDataMapper;
    errorCallback_ = errorCallback;
    is_init_next_clip_ = isInitNextClip;

    int ret = init();
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Initial Record Clip Manager Failed: ret = " << ret);
      return -4;
    }

    is_clip_already_start_ = false;

    return 0;
  }

  int start() {
    clip_write_start_timestamp_ = RS_TIMESTAMP_NS;
    clip_write_end_timestamp_ = clip_write_start_timestamp_;

    is_clip_already_start_ = true;

    return 0;
  }

  int finish() {
    RS_INFO_STREAM(pSharedNode_,
                   "Clip Already Start Status: " << is_clip_already_start_);
#if ENABLE_CLIP_MANAGER_DEBUG
    RS_INFO(pSharedNode_, "Start Finish Clip");
    RS_INFO_STREAM(pSharedNode_, "STATISTICAL_COUNT = "
                                     << STATISTICAL_COUNT
                                     << ", BUFFER SIZE = " << buffer_.size()
                                     << ", clipDirPath_ = " << clipDirPath_);
#endif // ENABLE_CLIP_MANAGER_DEBUG

    // check duration
    bool isRemoveClip = false;
    switch (clip_write_segment_type_) {
    case RS_RECORD_SEGMENT_TYPE::RS_RECROD_SEGMENT_NO_SEG: {
      isRemoveClip = getRecordTotalMessageCnt() ==
                     0; // 本身不删除，但是，如果一个消息都没有则删除
      break;
    }
    case RS_RECORD_SEGMENT_TYPE::RS_RECORD_SEGMENT_BY_TIME: {
      if (clip_write_end_timestamp_ - clip_write_start_timestamp_ <
          (pSharedGlobalConfig_
               ->recordConfig["recorddataconfig"]["segment_time_th_ms"]
               .as<uint64_t>() -
           500) *
              1000000) {
        RS_WARN_STREAM(
            pSharedNode_,
            "Record Duration: "
                << (clip_write_end_timestamp_ - clip_write_start_timestamp_) *
                       1e-9
                << " (s), Threshold: "
                << (((pSharedGlobalConfig_
                          ->recordConfig["recorddataconfig"]
                                        ["segment_time_th_ms"]
                          .as<uint64_t>() -
                      500) *
                     1000000) *
                    1e-9)
                << " (s)");
        isRemoveClip = true;
      }
      break;
    }
    }

    // output statistical info if the clip is not to be removed
    int ret = isRemoveClip ? 0 : writeRecordStatistical();
    if (ret != 0) {
      RS_WARN_STREAM(pSharedNode_,
                     "write record statistical failed: ret = " << ret);
    } else {
      RS_INFO(pSharedNode_, "write record statistical successed !");
    }

    // update record file check cnt mapper
    ret = isRemoveClip ? 0 : updateRecordFileCheckCntMapper();
    if (ret != 0) {
      RS_ERROR_STREAM(
          pSharedNode_,
          "Update Record File Check Count Mapper Failed: ret = " << ret);
      return 0;
    } else {
      RS_INFO(pSharedNode_,
              "Update Record File Check Count Mapper Successed !");
    }

    // close writer thread
    if (is_writer_running_) {
#if ENABLE_CLIP_MANAGER_DEBUG
      RS_INFO(pSharedNode_, "Buffer Thread Finish Clip");
      RS_INFO_STREAM(pSharedNode_, "STATISTICAL_COUNT = "
                                       << STATISTICAL_COUNT
                                       << ", BUFFER SIZE = " << buffer_.size()
                                       << ", clipDirPath_ = " << clipDirPath_);
#endif // ENABLE_CLIP_MANAGER_DEBUG
      {
        std::lock_guard<std::mutex> lg(buffer_mtx_);
        is_writer_running_ = false;
        buffer_cond_.notify_all();
      }

      if (writer_thread_ != nullptr) {
        if (writer_thread_->joinable()) {
          writer_thread_->join();
        }
        writer_thread_.reset();
      }
    }

#if ENABLE_CLIP_MANAGER_DEBUG
    RS_INFO_STREAM(pSharedNode_,
                   "Thread(s) Finish, clipDirPath_ = " << clipDirPath_);
#endif // ENABLE_CLIP_MANAGER_DEBUG

    // close writer(s)
    // 获取文件写出的消息个数
    std::map<std::string, uint64_t> record_statistical;
    for (auto iterMap = outputChannelWriters_.begin();
         iterMap != outputChannelWriters_.end(); ++iterMap) {
      const auto &channelWriter = iterMap->second;
      const std::string &outputFilePath = channelWriter.getOutputFilePath();
      if (record_statistical.find(outputFilePath) == record_statistical.end()) {
        record_statistical[outputFilePath] = 0;
      }

      record_statistical[outputFilePath] +=
          channelWriter.getOutputFileMessageCnt();

      // RCLCPP_ERROR_STREAM(pSharedNode_->get_logger(), "record_statistical["
      //                                  << outputFilePath << "] = "
      //                                  <<
      //                                  record_statistical[outputFilePath]);
    }

    std::set<std::string> closedOuputFilePaths;
    for (auto iterMap = outputChannelWriters_.begin();
         iterMap != outputChannelWriters_.end(); ++iterMap) {
      auto &recordItemInfo = iterMap->second;
      const std::string &outputFilePath = recordItemInfo.getOutputFilePath();
      if (closedOuputFilePaths.find(outputFilePath) ==
          closedOuputFilePaths.end()) {
        // RCLCPP_INFO(pSharedNode_->get_logger(), "outputFilePath = " <<
        // outputFilePath;
        recordItemInfo.closeWriter();
        closedOuputFilePaths.insert(outputFilePath);

        // 更新文件关闭状态
        for (auto iterMap2 = outputChannelWriters_.begin();
             iterMap2 != outputChannelWriters_.end(); ++iterMap2) {
          auto &recordItemInfo2 = iterMap2->second;
          const std::string &outputFilePath2 =
              recordItemInfo2.getOutputFilePath();
          if (outputFilePath2 == outputFilePath) {
            recordItemInfo2.updateWriterClose(true);
          }
        }
      }
    }
    outputChannelWriters_.clear();

#if ENABLE_CLIP_MANAGER_DEBUG
    RS_INFO_STREAM(pSharedNode_, "Close Record File(s) Finish, clipDirPath_ = "
                                     << clipDirPath_);
#endif // ENABLE_CLIP_MANAGER_DEBUG
    if (isRemoveClip) {
      // Not Save Collect Meta Information
      if (pSharedCollectInfoManager_ != nullptr) {
        pSharedCollectInfoManager_->setIsWriteMetaInfo(false);
      }

      int ret = removeClip();
      if (ret != 0) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Remove Current Clip Failed: clip Directory Path: "
                            << clipDirPath_ << ", ret = " << ret);
        return -1;
      } else {
        RS_INFO_STREAM(pSharedNode_,
                       "Remove Current Clip Successed: clip Directory Path: "
                           << clipDirPath_);
      }

      // 条件删除时记录
      updateTaskNameMetaFileOpInfo("Auto_Remove");
    } else {
      if (pSharedCollectInfoManager_ != nullptr) {
        pSharedCollectInfoManager_->updateHeadTime(clip_write_start_timestamp_ *
                                                   1e-9);
        pSharedCollectInfoManager_->updateTailTime(clip_write_end_timestamp_ *
                                                   1e-9);

        int ret = pSharedCollectInfoManager_->stopInfoManager();
        if (ret != 0) {
          RS_ERROR_STREAM(pSharedNode_,
                          "Stop Collect Info Manager Failed: ret = " << ret);
          return -2;
        }
      }

      // save channel timestamp(s)
      int ret = writeChannelTimestamp();
      if (ret != 0) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Write ChannelTimestamp Failed: ret = " << ret);
      }

      // save file check info(s)
      ret = writeRecordFileCheck();
      if (ret != 0) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Write Record File Check Failed: ret= " << ret);
      }
    }

    is_writer_finished_ = true;

    RS_INFO_STREAM(pSharedNode_, "Finish Clip: " << clipDirPath_);

    return 0;
  }

  bool checkIsNeedFinish() const { return !is_writer_finished_; }

  void setIsWriterTimeout(const bool isWriterTimeout) {
    RS_WARN_STREAM(pSharedNode_,
                   "clipDirPath_ = "
                       << clipDirPath_
                       << ", isWriterTimeout = " << isWriterTimeout
                       << ", is_writer_finished_ = " << is_writer_finished_);
    if (isWriterTimeout) {
      if (is_writer_finished_ == false) {
        is_writer_finished_ = true;
      } else {
        is_force_finish_ = false;
        return;
      }
    }
    is_force_finish_ = isWriterTimeout;
  }

  std::string clipRootDirectory() const { return clipDirPath_; }

  RS_RECORD_DATA_TYPE getRecordDataType(const std::string &topic_name) {
    auto iterMap = outputChannelWriters_.find(topic_name);
    if (iterMap == outputChannelWriters_.end()) {
      return RS_RECORD_DATA_TYPE::RS_RECORD_DATA_ANY;
    }

    return iterMap->second.getRecordDataType();
  }

  int writeTiemoutError() {
    const std::string error_file_path = clipDirPath_ + "/.Collect_Error";
    std::ofstream ofstr(error_file_path,
                        std::ios_base::binary | std::ios_base::out);
    if (!ofstr.is_open()) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Open Clip Collect Error File: " << error_file_path
                                                       << " To Write Failed !");
      return -1;
    }

    ofstr << "This Collect Clip: " << clipDirPath_ << " Write Timeout !";
    ofstr.flush();
    ofstr.close();

    return 0;
  }

  bool checkSegment() {
    bool isSegment = false;
    switch (clip_write_segment_type_) {
    case RS_RECORD_SEGMENT_TYPE::RS_RECROD_SEGMENT_NO_SEG: {
      isSegment = false;
      break;
    }
    case RS_RECORD_SEGMENT_TYPE::RS_RECORD_SEGMENT_BY_TIME: {
      uint64_t current_timestamp = RS_TIMESTAMP_NS;
      isSegment = ((current_timestamp - clip_write_start_timestamp_) >
                   clip_write_segment_by_time_th_);
      break;
    }
    }
    return isSegment;
  }

  int writeMessage2(const RSRecordSingleMessage::Ptr &singleMessagePtr) {
    if (is_writer_running_ && singleMessagePtr != nullptr) {
      if (is_first_frame_) {
        int ret = cleanCameraH265Buffer();
        if (ret != 0) {
          RS_ERROR_STREAM(pSharedNode_,
                          "Clear Camera H265 Buffer Failed: ret = " << ret);
          return -1;
        }
        is_first_frame_ = false;
      }

      // 如果是H265, 则更新H265标记
      if (camera_h265_channels_.find(singleMessagePtr->topic_name_) !=
          camera_h265_channels_.end()) {
        bool isH265IFrame = false;
        int ret = addCameraH265Buffer(singleMessagePtr, isH265IFrame);
        if (ret != 0) {
          RS_ERROR_STREAM(pSharedNode_,
                          "Add Camera H265 Buffer Failed: ret = " << ret);
          return -2;
        }
        singleMessagePtr->is_h265_message_ = true;
        singleMessagePtr->is_h265_iframe_ = isH265IFrame;
      }
      // 加入写缓冲区
      std::lock_guard<std::mutex> lg(buffer_mtx_);
      buffer_.push(singleMessagePtr);
      buffer_cond_.notify_one();

      const uint64_t messageTimestampNs = singleMessagePtr->message_time_;
      clip_write_start_timestamp_ =
          std::min(clip_write_start_timestamp_, messageTimestampNs);
      clip_write_end_timestamp_ = messageTimestampNs;
    }

    // RCLCPP_ERROR(pSharedNode_->get_logger(), "Write Message: " <<
    // message_time << ", topic_name = " << topic_name << ", clipDirPath_ = " <<
    // clipDirPath_;
#if ENABLE_CLIP_MANAGER_DEBUG
    {
      ++STATISTICAL_COUNT;
      if (STATISTICAL_COUNT % 2000 == 0) {
        RS_INFO_STREAM(pSharedNode_,
                       "STATISTICAL_COUNT = "
                           << STATISTICAL_COUNT
                           << ", BUFFER SIZE = " << buffer_.size()
                           << ", clipDirPath_ = " << clipDirPath_
                           << ", is_init_next_clip_ = " << is_init_next_clip_);
      }
    }
#endif // ENABLE_CLIP_MANAGER_DEBUG

    return 0;
  }

  int removeClip() {
    bool isSuccess = RSFileSystem::removeDirectory(clipDirPath_);
    if (!isSuccess) {
      return -1;
    }

    return 0;
  }

  int getRecordTotalMessageCnt() const {
    int totalMsgCnt = 0;
    for (auto iterMap = outputChannelWriters_.begin();
         iterMap != outputChannelWriters_.end(); ++iterMap) {
      const auto &channelWriter = iterMap->second;
      totalMsgCnt += channelWriter.getOutputFileMessageCnt();
    }

    return totalMsgCnt;
  }

  int writeRecordStatistical() {
    const std::string &record_statistical_path =
        clipDirPath_ + "/.Collect_Stat";
    std::ofstream ofstr(record_statistical_path,
                        std::ios_base::out | std::ios_base::app);
    if (ofstr.is_open()) {
      ofstr << "ChannelName\t\t\t\t\tCount\t\t\tRawSize(byte)\t\t\tDuration(ns)"
               "\t\t\tBeginTime(ns)\t\t\tEndTime(ns)\t\t\tFilePath"
            << std::endl;
      ofstr.flush();
      for (auto iterMap = outputChannelWriters_.begin();
           iterMap != outputChannelWriters_.end(); ++iterMap) {
        const auto &channelWriter = iterMap->second;
        // ofstr << "runhere" << std::endl;
        ofstr << iterMap->first << "\t"
              << std::to_string(channelWriter.getOutputFileMessageCnt()) << "\t"
              << std::to_string(channelWriter.getOutputFileByteSize()) << "\t"
              << std::to_string(channelWriter.getOutputFileDurationNs()) << "\t"
              << std::to_string(channelWriter.getOutputFileBeginNs()) << "\t"
              << std::to_string(channelWriter.getOutputFileEndNs()) << "\t"
              << channelWriter.getOutputFilePath() << std::endl;
        ofstr.flush();
        // RCLCPP_ERROR(pSharedNode_->get_logger(), "RUN HERE: " <<
        // channelWriter.getOutputFileMessageCnt();
      }

      ofstr.close();
    } else {
      RS_WARN_STREAM(pSharedNode_, "open write record statistical to file: "
                                       << record_statistical_path
                                       << " failed !");
      return -1;
    }

    RS_INFO_STREAM(pSharedNode_, "open write record statistical successed: "
                                     << record_statistical_path);

    return 0;
  }

  static void resetCameraH265Buffer() {
    RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.clear();
  }

private:
  int init() {
    // clip basic directory path
    if (is_init_next_clip_) {
      clipDirPath_ = pSharedGlobalConfig_->getNextClipNameDirPath();
    } else {
      clipDirPath_ = pSharedGlobalConfig_->getClipNameDirPath();
    }

    if (!RSFileSystem::isDirectoryExist(clipDirPath_)) {
      if (!RSFileSystem::makePath(clipDirPath_)) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Clip Directory Path Not Exist: " << clipDirPath_);
        return -1;
      }
    }
    RS_INFO_STREAM(pSharedNode_,
                   "Current Save Clip Directory Path: "
                       << clipDirPath_
                       << ", is_init_next_clip_ = " << is_init_next_clip_);

    // segment mode
    clip_write_segment_type_ = fromStringToRecordSegmentType(
        pSharedGlobalConfig_->recordConfig["recorddataconfig"]["segment_type"]
            .as<std::string>());

    // create writers
    camera_h265_channels_.clear();
    outputFilePaths_.clear();
    outputChannelWriters_.clear();
    clipRelDirToRecordPathMapper_.clear();
    std::map<std::string, std::set<std::string>> fileMatchChannels;
    const auto &sensors_config =
        pSharedGlobalConfig_->recordConfig["recordsensorconfig"];
    for (size_t i = 0; i < sensors_config["record_sensors"].size(); ++i) {
      const auto &sensor_config =
          sensors_config["record_sensors"][static_cast<int>(i)];
      const std::string &topic_name =
          sensor_config["topic_name"].as<std::string>();

      const std::string clipSensorDirPath =
          clipDirPath_ + "/" +
          sensor_config["clip_rel_directory"].as<std::string>();
      if (!RSFileSystem::isDirectoryExist(clipSensorDirPath)) {
        if (!RSFileSystem::makePath(clipSensorDirPath)) {
          RS_ERROR_STREAM(pSharedNode_,
                          "Make Path clipSensorDirPath = " << clipSensorDirPath
                                                           << " Failed !");
          return -3;
        }
      }

      const auto sensor_type = fromStringToRecordDataType(
          sensor_config["sensor_type"].as<std::string>());
#if __ROS2__
      const std::string &clipChannelNameFilePath =
          RSFileSystem::replaceRepeatSlash(
              clipSensorDirPath + "/" +
              sensor_config["record_file_name"].as<std::string>());
#elif __ROS1__
      const std::string &clipChannelNameFilePath =
          RSFileSystem::replaceRepeatSlash(
              clipSensorDirPath + "/" +
              sensor_config["record_file_name"].as<std::string>()) +
          ".bag";
#endif
      RSRecordClipChannel recordItemInfo;
      recordItemInfo.init(sensor_type, topic_name, clipChannelNameFilePath);

      outputChannelWriters_[topic_name] = recordItemInfo;
      outputFilePaths_.insert(clipChannelNameFilePath);

      // Sensor: Clip Relative => Record File Name
      if (clipRelDirToRecordPathMapper_.find(
              sensor_config["clip_rel_directory"].as<std::string>()) !=
          clipRelDirToRecordPathMapper_.end()) {
        clipRelDirToRecordPathMapper_[sensor_config["clip_rel_directory"]
                                          .as<std::string>()]
            .insert(clipChannelNameFilePath);
      } else {
        clipRelDirToRecordPathMapper_.insert(
            {sensor_config["clip_rel_directory"].as<std::string>(),
             std::set<std::string>{clipChannelNameFilePath}});
      }

      if (fileMatchChannels.find(clipChannelNameFilePath) ==
          fileMatchChannels.end()) {
        fileMatchChannels[clipChannelNameFilePath] =
            std::set<std::string>{topic_name};
      } else {
        fileMatchChannels[clipChannelNameFilePath].insert(topic_name);
      }

      if (sensor_type == RS_RECORD_DATA_TYPE::RS_RECORD_DATA_H265) {
        camera_h265_channels_.insert(topic_name);
        RS_INFO_STREAM(pSharedNode_, "camera h265 topic_name = " << topic_name);
      }
    }

    const bool enable_compression =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["enable_compression"]
            .as<bool>();

    const std::string &compression_mode =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["compression_mode"]
            .as<std::string>();

    const std::string &compression_format =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["compression_format"]
            .as<std::string>();

    const uint64_t compression_queue_size =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["compression_quene_size"]
            .as<uint64_t>();

    const uint64_t compression_thread_cnt =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["compression_thread_cnt"]
            .as<uint64_t>();

    const bool is_enable_no_segement =
        (clip_write_segment_type_ ==
         RS_RECORD_SEGMENT_TYPE::RS_RECROD_SEGMENT_NO_SEG);

    const bool enable_no_segment_inner_seg =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["enable_no_segment_inner_seg"]
            .as<bool>();

    const uint64_t no_segment_inner_segment_th_ms =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]
                          ["no_segment_inner_segment_time_th_ms"]
            .as<uint64_t>();

    const uint64_t storage_cache_size =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["storage_cache_size"]
            .as<uint64_t>();

    // std::cout << "storage_cache_size = " << storage_cache_size << std::endl;

    std::set<std::string> compression_exclude_records;
    const auto &compression_exclude_records_node =
        pSharedGlobalConfig_
            ->recordConfig["recorddataconfig"]["compression_exclude_records"];
    for (size_t i = 0; i < compression_exclude_records_node.size(); ++i) {
      const auto &single_node =
          compression_exclude_records_node[static_cast<int>(i)];
      const std::string clipSensorDirPath = RSFileSystem::replaceRepeatSlash(
          clipDirPath_ + "/" +
          single_node["clip_rel_directory"].as<std::string>() + "/" +
          single_node["record_file_name"].as<std::string>());
      compression_exclude_records.insert(clipSensorDirPath);
    }

#if __ROS2__
    // compression/non-compression opts
    rosbag2_transport::RecordOptions compression_opts;
    compression_opts.compression_mode = compression_mode;
    compression_opts.compression_format = compression_format;
    compression_opts.compression_queue_size = compression_queue_size;
    compression_opts.compression_threads = compression_thread_cnt;

    rosbag2_transport::RecordOptions non_compression_opts = compression_opts;
    non_compression_opts.compression_format.clear();
#endif

    // Step3:
    for (auto iterSet = outputFilePaths_.begin();
         iterSet != outputFilePaths_.end(); ++iterSet) {
      const std::string &outputFilePath = *iterSet;
#if __ROS2__
      rosbag2_storage::StorageOptions storage_options;
      storage_options.uri = outputFilePath;
      storage_options.storage_id =
          pSharedGlobalConfig_->recordConfig["recorddataconfig"]["storage_id"]
              .as<std::string>();
      storage_options.max_cache_size = storage_cache_size;
      storage_options.max_bagfile_size = 0; // Fixed
      storage_options.max_bagfile_duration =
          (is_enable_no_segement && enable_no_segment_inner_seg
               ? no_segment_inner_segment_th_ms / 1000
               : 0); // Fixed

#endif
      bool enable_record_compression = enable_compression;
      if (compression_exclude_records.count(outputFilePath)) {
        enable_record_compression = false;
      }

      RS_INFO_STREAM(pSharedNode_,
                     "outputFilePath = " << outputFilePath
                                         << ", enable_record_compression = "
                                         << enable_record_compression);
#if __ROS2__
      std::shared_ptr<rosbag2_cpp::Writer> writer =
          rosbag2_transport::ReaderWriterFactory::make_writer(
              (enable_record_compression ? compression_opts
                                         : non_compression_opts));
#elif __ROS1__
      std::shared_ptr<rosbag::Bag> writer(std::make_shared<rosbag::Bag>());
#endif

      if (writer == nullptr) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Make ROS2 Record File Writer Failed: outputFilePath = "
                            << outputFilePath);
        return -4;
      }

      try {
#if __ROS2__
        writer->open(
            storage_options,
            {rmw_get_serialization_format(),
             pSharedGlobalConfig_
                 ->recordConfig["recorddataconfig"]["serialization_format"]
                 .as<std::string>()});
#elif __ROS1__
        writer->open(outputFilePath, rosbag::bagmode::Write);
#endif
      } catch (std::exception &e) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Open ROS2 Record File: " << outputFilePath
                                                  << " Failed: errorInfo = "
                                                  << e.what());
        return -5;
      }

      // 更新文件对应的话题信息
      const std::set<std::string> &matchedChannelNames =
          fileMatchChannels[outputFilePath];
      for (auto iterMap = topicMetaDataMapper_.begin();
           iterMap != topicMetaDataMapper_.end(); ++iterMap) {
        const auto topicMetaData = iterMap->second;
        // 保存的channel写
#if __ROS2__
        const std::string topic_name = topicMetaData.name;
#elif __ROS1__
        const std::string topic_name = topicMetaData.topic;
#endif
        if (matchedChannelNames.find(topic_name) == matchedChannelNames.end()) {
          continue;
        }

#if __ROS2__
        try {
          writer->create_topic(topicMetaData);

        } catch (...) {
          RS_ERROR_STREAM(pSharedNode_, "write channel fail, channel:"
                                            << topicMetaData.name << ", type = "
                                            << topicMetaData.type);

          return -6;
        }
#endif
      }

      // Update Writer(s)
      for (auto iterMap = outputChannelWriters_.begin();
           iterMap != outputChannelWriters_.end(); ++iterMap) {
        auto &recordItemInfo = iterMap->second;
        const std::string &clipChannelNameOutputFilePath =
            recordItemInfo.getOutputFilePath();
        if (clipChannelNameOutputFilePath == outputFilePath) {
          recordItemInfo.updateWriter(writer);
        }
      }
    }

    clip_write_start_timestamp_ = RS_TIMESTAMP_NS;
    clip_write_end_timestamp_ = clip_write_start_timestamp_;
    clip_write_size_ = 0;
    const auto &recordConfig = pSharedGlobalConfig_->recordConfig;
    const auto &recordDataConfig = recordConfig["recorddataconfig"];
    clip_write_segment_by_time_th_ =
        (recordDataConfig["segment_time_th_ms"].as<uint64_t>() * 1000000);

    try {
      pSharedCollectInfoManager_.reset(new RSCollectInfoManager());
    } catch (const std::exception &e) {
      RS_ERROR(pSharedNode_, "Malloc Collect Info Manager Failed !");
      return -7;
    }

    int ret = pSharedCollectInfoManager_->initInfoManager(
        pSharedGlobalConfig_, pSharedNode_, is_init_next_clip_);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Initial Collect Info Manager Failed: ret = " << ret);
      return -8;
    }

    // writer thread
    is_writer_running_ = false;
    try {
      is_writer_running_ = true;
      writer_thread_.reset(
          new std::thread(&RSRecordClipManager::writeWorkThread, this));
    } catch (const std::exception &e) {
      is_writer_running_ = false;
      RS_ERROR(pSharedNode_, "Create Write Work Thread Failed !");
      return -9;
    }

    // initial record check manager
    const std::string &record_file_check_yaml_file_path =
        clipDirPath_ + "/" + RS_RECORD_CHECK_YAML_NAME;
    ret = initRecordFileCheckManager(record_file_check_yaml_file_path);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_, "Initial Record File Check Manager Failed: "
                                    "record_file_check_yaml_file_path = "
                                        << record_file_check_yaml_file_path
                                        << ", ret = " << ret);
      return -10;
    }

    ret = addRecordFileCheckManager();
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Add Record File Check Manager Failed: ret = " << ret);
      return -11;
    }

    is_writer_finished_ = false;
    RS_INFO_STREAM(pSharedNode_,
                   "Current Save Clip Directory Path: "
                       << clipDirPath_
                       << ", is_init_next_clip_ = " << is_init_next_clip_);
    return 0;
  }

  void writeWorkThread() {
    bool isFirstExitWarn = true;
    while (is_writer_running_ || !buffer_.empty()) {
      RSRecordSingleMessage::Ptr singleMessagePtr = nullptr;
      {
        std::unique_lock<std::mutex> lg(buffer_mtx_);
        buffer_cond_.wait(lg, [this] {
          return !is_writer_running_ || !buffer_.empty() || is_force_finish_;
        });
        if (is_force_finish_) {
          break;
        } else if (!is_writer_running_ && buffer_.empty()) {
          break;
        } else if (buffer_.empty()) {
          // std::this_thread::sleep_for(std::chrono::milliseconds(5));
          continue;
        }
        singleMessagePtr = buffer_.front();
        buffer_.pop();
      }
      if (!is_writer_running_ && isFirstExitWarn) {
        RS_WARN_STREAM(pSharedNode_,
                       "is_writer_running_ = " << is_writer_running_
                                               << ", Write Buffer Size = "
                                               << buffer_.size());
        isFirstExitWarn = false;
      }
      int ret = writeMessage(singleMessagePtr);
      if (ret != 0) {
#if RS_ENABLE_COLLECT_USE_CHINESE
        const std::string &error_info =
            "写单个消息到Record文件失败: ret = " + std::to_string(ret) +
            ", 话题名称: " + singleMessagePtr->topic_name_;
#else
        const std::string &error_info =
            "Write Single Message To Record File Failed: ret = " +
            std::to_string(ret) +
            ", channle_name = " + singleMessagePtr->topic_name_;
#endif // RS_ENABLE_COLLECT_USE_CHINESE
        RS_ERROR_STREAM(pSharedNode_, error_info);
      }
    }
    RS_INFO(pSharedNode_, "Write Thread Exit !");
  }

  int writeMessage(const RSRecordSingleMessage::Ptr &singleMessagePtr) {
    if (singleMessagePtr == nullptr) {
      return -1;
    } else if (singleMessagePtr->message_ == nullptr) {
      return -2;
    }

    // 非全部写
    auto iterWriterMapper =
        outputChannelWriters_.find(singleMessagePtr->topic_name_);
    if (iterWriterMapper == outputChannelWriters_.end()) {
      return 0;
    }

    updateChannelTimestamp(singleMessagePtr->topic_name_,
                           singleMessagePtr->message_time_);

    // RCLCPP_ERROR(pSharedNode_->get_logger(), "singleMessagePtr->topic_name_ =
    // "
    //        << singleMessagePtr->topic_name_;
    auto &recordItemInfo = iterWriterMapper->second;
    if (!singleMessagePtr->is_h265_message_) {
      // 非H265数据
      if (!recordItemInfo.writeMessage(
              singleMessagePtr->topic_name_, singleMessagePtr->topic_type_,
              singleMessagePtr->message_time_, singleMessagePtr->message_)) {
        const std::string &error_info =
            "not need compressed write: write data fail, channel: " +
            singleMessagePtr->topic_name_;
        RS_ERROR_STREAM(pSharedNode_, error_info);
        return -1;
      }
    } else {
      // H265数据
      if (!recordItemInfo.writeMessage(
              singleMessagePtr->topic_name_, singleMessagePtr->topic_type_,
              singleMessagePtr->message_time_, singleMessagePtr->message_,
              singleMessagePtr->is_h265_iframe_)) {
        const std::string &error_info =
            "not need compressed write: write data fail, channel: " +
            singleMessagePtr->topic_name_;
        RS_ERROR_STREAM(pSharedNode_, error_info);
        return -1;
      }
    }

    // 更新数据量
    clip_write_size_ += singleMessagePtr->message_->size();

    return 0;
  }

  int cleanCameraH265Buffer() {
    std::map<uint64_t, RSRecordSingleMessage::Ptr> buffers;
    for (auto iterMap =
             RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.begin();
         iterMap != RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.end();
         ++iterMap) {
      const auto &buffer = iterMap->second;
      RS_INFO_STREAM(pSharedNode_,
                     "topic_name: " << iterMap->first
                                    << ", Camera H265 Buffer Size: "
                                    << buffer.size());
      for (auto iter = buffer.begin(); iter != buffer.end(); ++iter) {
        if (*iter) {
          buffers[(*iter)->message_time_] = *iter;
        }
      }
    }
    RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.clear();

    for (auto iterMap = buffers.begin(); iterMap != buffers.end(); ++iterMap) {
      auto &singleMessagePtr = iterMap->second;
      if (singleMessagePtr) {
        if (singleMessagePtr->message_) {
          // 解析相机消息判断I帧
          RsCompressedImagePtr compressImagePtr;
          if (!this->deserializeMessage(singleMessagePtr->message_,
                                        compressImagePtr)) {
            RS_ERROR_STREAM(pSharedNode_,
                            "Parse Camera H265 Message Failed: topic_name = "
                                << singleMessagePtr->topic_name_);
            return -1;
          }
          compressImagePtr->attach_type = RsCompressedImage::CLIP_NOT_BELONG;
          singleMessagePtr->message_.reset();
          if (!this->serializeMessage(compressImagePtr,
                                      singleMessagePtr->message_)) {
            RS_ERROR_STREAM(
                pSharedNode_,
                "Re-Seralize Camera H265 Message Failed: topic_name = "
                    << singleMessagePtr->topic_name_);
            return -2;
          }
        }
      }
    }

    {
      std::lock_guard<std::mutex> lg(buffer_mtx_);
      for (auto iterMap = buffers.begin(); iterMap != buffers.end();
           ++iterMap) {
        auto &singleMessagePtr = iterMap->second;
        if (singleMessagePtr) {
          buffer_.push(singleMessagePtr);
        }
      }
      buffer_cond_.notify_one();
    }

    return 0;
  }

  int addCameraH265Buffer(const RSRecordSingleMessage::Ptr &singleMessagePtr,
                          bool &isH265IFrame) {
    isH265IFrame = false;
    if (singleMessagePtr != nullptr) {
      if (singleMessagePtr->message_) {
        const std::string &topic_name = singleMessagePtr->topic_name_;
        auto iterMap =
            RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.find(topic_name);
        if (iterMap ==
            RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.end()) {
          RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER[topic_name] =
              std::list<RSRecordSingleMessage::Ptr>();
          iterMap = RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER.find(
              topic_name);
        }

        auto &buffer = iterMap->second;
        RsCompressedImagePtr compressImagePtr;
        if (!this->deserializeMessage(singleMessagePtr->message_,
                                      compressImagePtr)) {
          RS_ERROR_STREAM(
              pSharedNode_,
              "Parse Camera H265 Message Failed: topic_name = " << topic_name);
          return -1;
        }

        if (compressImagePtr->type == RsCompressedImage::H265_I) {
          buffer.clear();
          isH265IFrame = true;
        }

        RSRecordSingleMessage::Ptr copyMessagePtr(new RSRecordSingleMessage());
        copyMessagePtr->deepCopy(*singleMessagePtr);

        buffer.push_back(copyMessagePtr);
      } else {
        RS_ERROR(pSharedNode_, "singleMessagePtr->message_ is Nullptr !");
        return -2;
      }
    } else {
      RS_ERROR(pSharedNode_, "singleMessagePtr is Nullptr !");
      return -3;
    }

    return 0;
  }

  void updateChannelTimestamp(const std::string &topic_name,
                              const uint64_t timestamp) {
    if (channel_timestamps_.find(topic_name) != channel_timestamps_.end()) {
      channel_timestamps_[topic_name].push_back(timestamp);
    } else {
      std::vector<uint64_t> timestamps{timestamp};
      channel_timestamps_.insert({topic_name, timestamps});
    }
  }

  int writeChannelTimestamp() {
    const std::string &timestampRecordFilePath =
        clipDirPath_ + "/" + RS_TIMESTAMP_RECORD_FILE_NAME;
    std::ofstream ofstr(timestampRecordFilePath,
                        std::ios_base::out | std::ios_base::binary);
    if (!ofstr.is_open()) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Open Timestamp Record File = " << timestampRecordFilePath
                                                      << " To Write Failed !");
      return -1;
    }

    YAML::Node timestampNodes;
    for (auto iterMap = channel_timestamps_.begin();
         iterMap != channel_timestamps_.end(); ++iterMap) {
      const std::string &topic_name = iterMap->first;
      const auto &channel_timestamps = iterMap->second;
      YAML::Node timestampNode;

      const size_t timestampCnt = channel_timestamps.size();
      for (size_t i = 0; i < timestampCnt; ++i) {
        timestampNode.push_back(channel_timestamps[i]);
      }
      timestampNodes[topic_name] = timestampNode;
    }

    YAML::Emitter emitter;
    emitter << timestampNodes;

    ofstr << emitter.c_str() << std::endl;

    RS_INFO_STREAM(
        pSharedNode_,
        "Write RSChannelTimestamp Successed, timestampRecordFilePath = "
            << timestampRecordFilePath);

    return 0;
  }

  int updateTaskNameMetaFileOpInfo(const std::string &op) {
    int ret =
        pSharedGlobalConfig_->updateTaskNameMetaFileOpInfo(op, clipDirPath_);
    if (ret == 0) {
      RS_INFO_STREAM(pSharedNode_,
                     "Update TaskName MetaFile Op Info Successed => Op: "
                         << op << " => clipDirPath = " << clipDirPath_);
    } else {
      RS_ERROR_STREAM(pSharedNode_,
                      "Update TaskName MetaFile Op Info Failed => Op: "
                          << op << " => clipDirPath = " << clipDirPath_);
    }
    return ret;
  }

  int initRecordFileCheckManager(
      const std::string &record_file_check_yaml_path) {
    try {
      record_file_check_manager_ptr_.reset(new RSRecordFileCheckManager());
    } catch (...) {
      RS_ERROR(pSharedNode_, "Malloc Record File Check Manager Failed !");
      return -1;
    }

    int ret = record_file_check_manager_ptr_->init(pSharedNode_,
                                                   record_file_check_yaml_path);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_, "Record File Check Manager Initial Failed: "
                                    "record_file_check_yaml_path = "
                                        << record_file_check_yaml_path
                                        << ", ret = " << ret);
    } else {
      RS_INFO_STREAM(pSharedNode_,
                     "Record File Check Manager Initial Successed: "
                     "record_file_check_yaml_path = "
                         << record_file_check_yaml_path);
    }

    return 0;
  }

  int addRecordFileCheckManager() {
    if (record_file_check_manager_ptr_ == nullptr) {
      RS_ERROR(pSharedNode_, "record_file_check_manager_ptr_ is Nullptr !");
      return -1;
    }

    for (auto iterSet = outputFilePaths_.begin();
         iterSet != outputFilePaths_.end(); ++iterSet) {
      const std::string &record_file_path = *iterSet;

      std::string record_clip_rel_dir_name;
      for (auto iterMap = clipRelDirToRecordPathMapper_.begin();
           iterMap != clipRelDirToRecordPathMapper_.end(); ++iterMap) {
        const auto &recordFiles = iterMap->second;
        if (recordFiles.find(record_file_path) != recordFiles.end()) {
          record_clip_rel_dir_name = iterMap->first;
          break;
        }
      }

      int ret = record_file_check_manager_ptr_->addRecordCheckInfoFullPath(
          record_file_path, record_clip_rel_dir_name);

      if (ret != 0) {
        RS_ERROR_STREAM(pSharedNode_, "Record Check Manager Add Record File "
                                      "Path Failed: record_file_path = "
                                          << record_file_path
                                          << ", record_clip_rel_dir_name = "
                                          << record_clip_rel_dir_name);
        return -2;
      }
    }

    return 0;
  }

  int updateRecordFileCheckCntMapper() {
    record_message_cnt_mapper_.clear();
    for (auto iterMap = outputChannelWriters_.begin();
         iterMap != outputChannelWriters_.end(); ++iterMap) {
      const auto &channelWriter = iterMap->second;
      const std::string &record_file_path = channelWriter.getOutputFilePath();

      if (record_message_cnt_mapper_.find(record_file_path) !=
          record_message_cnt_mapper_.end()) {
        record_message_cnt_mapper_[record_file_path] +=
            channelWriter.getOutputFileMessageCnt();
      } else {
        record_message_cnt_mapper_[record_file_path] =
            channelWriter.getOutputFileMessageCnt();
      }
    }

    return 0;
  }

  int writeRecordFileCheck() {
    if (record_file_check_manager_ptr_ == nullptr) {
      RS_ERROR(pSharedNode_, "record_file_check_manager_ptr_ is Nullptr !");
      return -1;
    }

    for (auto iterMap = record_message_cnt_mapper_.begin();
         iterMap != record_message_cnt_mapper_.end(); ++iterMap) {
      const std::string &record_file_path = iterMap->first;
      const bool is_complete = true;
      const bool is_zero_message_cnt = (iterMap->second == 0);

      int ret = record_file_check_manager_ptr_->setRecordCheckInfoFullPath(
          record_file_path, is_complete, is_zero_message_cnt);
      if (ret != 0) {
        RS_ERROR_STREAM(pSharedNode_,
                        "Set record_file_path = "
                            << record_file_path
                            << " Check Info Failed: is_complete = "
                            << is_complete << ", is_zero_message_cnt = "
                            << is_zero_message_cnt << ", ret = " << ret);
        return -2;
      }
    }

    int ret = record_file_check_manager_ptr_->writeRecordCheckInfo();
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                      "Record File Check Manager Write Failed: ret = " << ret);
      return -3;
    } else {
      RS_INFO(pSharedNode_, "Record File Check Manager Write Successed !");
    }

    return 0;
  }

  template <typename MessageT>
  bool deserializeMessage(const SerializedMessagePtr &serialized_msg,
                          MessageT &msg) {
#if __ROS2__
    if (msg == nullptr) {
      msg.reset(new typename MessageT::element_type());
    }

    static auto serializer =
        rclcpp::Serialization<typename MessageT::element_type>();
    serializer.deserialize_message(serialized_msg.get(), msg.get());
#elif __ROS1__
    msg =
        serialized_msg->template instantiate<typename MessageT::element_type>();
#endif

    return true;
  }

  template <typename MessageT>
  bool serializeMessage(const MessageT &msg,
                        SerializedMessagePtr &serialized_msg) {
#if __ROS2__
    if (serialized_msg == nullptr) {
      serialized_msg.reset(new rclcpp::SerializedMessage());
    }
    serialized_msg->reserve(2 * 1024 * 1024);

    static auto serializer =
        rclcpp::Serialization<typename MessageT::element_type>();
    serializer.serialize_message(msg.get(), serialized_msg.get());
#elif __ROS1__
    size_t length = ros::serialization::serializationLength(*msg);

    boost::shared_array<uint8_t> buffer(new uint8_t[length]);
    ros::serialization::OStream stream(buffer.get(), length);

    ros::serialization::serialize(stream, *msg);

    std::string md5 =
        ros::message_traits::MD5Sum<typename MessageT::element_type>::value();
    std::string datatype =
        ros::message_traits::DataType<typename MessageT::element_type>::value();
    std::string msg_def = ros::message_traits::Definition<
        typename MessageT::element_type>::value();
    std::string latching = "0";

    serialized_msg->morph(md5, datatype, msg_def, latching);

    ros::serialization::IStream istream(buffer.get(), length);
    serialized_msg->read(istream);
#endif
    return true;
  }

public:
  std::string clipDirPath_;
  NodeHandlePtr pSharedNode_;
  std::set<std::string> outputFilePaths_;
  std::map<std::string, RSRecordClipChannel> outputChannelWriters_;
#if __ROS1__
  std::unordered_map<std::string, rosbag::ConnectionInfo> topicMetaDataMapper_;
#elif __ROS2__
  std::unordered_map<std::string, rosbag2_storage::TopicMetadata>
      topicMetaDataMapper_;
#endif
  std::map<std::string, std::set<std::string>> clipRelDirToRecordPathMapper_;
  RS_RECORD_IO_ERROR_CALLBACK errorCallback_;
  RSGlobalConfig::Ptr pSharedGlobalConfig_;
  RSCollectInfoManager::Ptr pSharedCollectInfoManager_;
  uint64_t clip_write_start_timestamp_;
  uint64_t clip_write_end_timestamp_;
  uint64_t clip_write_size_;
  uint64_t clip_write_segment_by_time_th_;
  bool is_writer_finished_;
  bool is_force_finish_ = false;
  bool is_init_next_clip_;
  std::set<std::string> camera_h265_channels_;
  RS_RECORD_SEGMENT_TYPE clip_write_segment_type_;

private:
  bool is_writer_running_;
  std::shared_ptr<std::thread> writer_thread_;
  std::mutex buffer_mtx_;
  std::condition_variable buffer_cond_;
  std::queue<RSRecordSingleMessage::Ptr> buffer_;

#if ENABLE_CLIP_MANAGER_DEBUG
private:
  size_t STATISTICAL_COUNT = 0;
#endif // ENABLE_CLIP_MANAGER_DEBUG

private:
  std::map<std::string, std::vector<uint64_t>> channel_timestamps_;

private:
  RSRecordFileCheckManager::Ptr record_file_check_manager_ptr_;
  std::map<std::string, size_t> record_message_cnt_mapper_;

private:
  bool is_first_frame_ = true;

private:
  bool is_clip_already_start_ = false;

private:
  const std::string RS_TIMESTAMP_RECORD_FILE_NAME = "timestamps.yaml";
  const std::string RS_TIMESTAMP_CHANNEL_NAME = "/channel_timestamps";
  const std::string RS_RECORD_CHECK_YAML_NAME = ".clip_check.yaml";

public:
  static std::map<std::string,
                  std::list<std::shared_ptr<RSRecordSingleMessage>>>
      RS_CAMERA_H265_BUFFER_MAPPER;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSRECORDCLIPMANAGER_H