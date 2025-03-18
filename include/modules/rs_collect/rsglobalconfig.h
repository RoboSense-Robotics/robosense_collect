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
#pragma once

#include <chrono>
#include <condition_variable>
#include <csignal>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "nlohmann/json.hpp"
#include "yaml-cpp/yaml.h"

// #include "modules/rs_collect/qos.hpp"
#include "modules/rs_collect/ros_adapter.h"
#include "modules/rs_collect/rsversion.h"
#include "modules/rs_filesystem/rsfilesystem.h"

#include "modules/rs_collect/ros_adapter.h"

#if __ROS2__
#include "robosense_msgs/msg/rs_compressed_image.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_transport/reader_writer_factory.hpp"
#include "rosbag2_transport/record_options.hpp"
#include "std_msgs/msg/string.hpp"
#elif __ROS1__
#endif

#define RS_STD_DEBUG(X)                                                        \
  std::cout << __FILE__ << "=> " << __FUNCTION__ << "(" << __LINE__ << ")=>"   \
            << X << std::endl
#define RS_TIMESTAMP_S                                                         \
  (std::chrono::time_point_cast<std::chrono::seconds>(                         \
       std::chrono::system_clock::now()))                                      \
      .time_since_epoch()                                                      \
      .count();
#define RS_TIMESTAMP_MS                                                        \
  (std::chrono::time_point_cast<std::chrono::milliseconds>(                    \
       std::chrono::system_clock::now()))                                      \
      .time_since_epoch()                                                      \
      .count();
#define RS_TIMESTAMP_US                                                        \
  (std::chrono::time_point_cast<std::chrono::microseconds>(                    \
       std::chrono::system_clock::now()))                                      \
      .time_since_epoch()                                                      \
      .count();
#define RS_TIMESTAMP_NS                                                        \
  (std::chrono::time_point_cast<std::chrono::nanoseconds>(                     \
       std::chrono::system_clock::now()))                                      \
      .time_since_epoch()                                                      \
      .count();
#define RS_UNUSED_PARAM(X) (void)(X)
#define RS_ENABLE_COLLECT_USE_CHINESE (0)

namespace robosense {
namespace rs_collect {
namespace collect {

using json = nlohmann::json;
using RSFileSystem = robosense::filesystem::util::RSFileSystem;

using RS_RECORD_IO_ERROR_CALLBACK =
    std::function<void(const int, const std::string &)>;

enum class RS_DATA_SAVE_MODE : int {
  RS_DATA_SAVE_BY_NOSEGMENT = -1,
  RS_DATA_SAVE_BY_SIZE = 0,
  RS_DATA_SAVE_BY_TIME,
  RS_DATA_SAVE_BY_ODOM,
  RS_DATA_SAVE_TOTAL,
};

class RSApp2CarConfig {
public:
  using Ptr = std::shared_ptr<RSApp2CarConfig>;
  using ConstPtr = std::shared_ptr<const RSApp2CarConfig>;

public:
  RSApp2CarConfig() { reset(); }

  void reset() {
    app2CarTimeoutMs = 604800000;
    car2AppHeartBeatMs = 5000;
    enableDebug = false;
  }

  int fromProtoMessage(const YAML::Node &config) {
    app2CarTimeoutMs = config["timeout_ms"].as<uint32_t>();
    car2AppHeartBeatMs = config["heartbeat_ms"].as<uint32_t>();
    enableDebug = config["enable_debug"].as<bool>();

    return 0;
  }

public:
  unsigned int app2CarTimeoutMs;
  unsigned int car2AppHeartBeatMs;
  bool enableDebug;
};

class RSSystemConfig {
public:
  using Ptr = std::shared_ptr<RSSystemConfig>;
  using ConstPtr = std::shared_ptr<const RSSystemConfig>;

public:
  RSSystemConfig() { reset(); }

  void reset() { vehicleInfo = ""; }

  int fromProtoMessage(const YAML::Node &config) {
    vehicleInfo = config["vehicle_id"].as<std::string>();
    return 0;
  }

public:
  std::string vehicleInfo;
};

class RSVehicleMetaConfig {
public:
  using Ptr = std::shared_ptr<RSVehicleMetaConfig>;
  using ConstPtr = std::shared_ptr<const RSVehicleMetaConfig>;

public:
  RSVehicleMetaConfig() { reset(); }

  void reset() {
    templateVehicleMetaConfigDirPath.clear();
    templateVehicleMetaConfigs.clear();
    templateVehicleMetaFilePaths.clear();
  }

public:
  int getVehicleMetaTemplate(const std::string &vehicleId,
                             YAML::Node &configNode) {
    auto iterMap = templateVehicleMetaConfigs.find(vehicleId);
    if (iterMap == templateVehicleMetaConfigs.end()) {
      return -1;
    }

    configNode = iterMap->second;

    return 0;
  }

  void updateVehicleMetaTemplate(const std::string &vehicleId,
                                 const YAML::Node configNode) {
    templateVehicleMetaConfigs[vehicleId] = configNode;
  }

  int updateVehicleMetaTemplate(const std::string &vehicleId,
                                const std::string &metaTemplateData) {
    auto iterMap = templateVehicleMetaFilePaths.find(vehicleId);
    std::string metaFilePath;
    if (iterMap == templateVehicleMetaFilePaths.end()) {
      uint64_t timestampNs = RS_TIMESTAMP_NS;
      metaFilePath = templateVehicleMetaConfigDirPath + vehicleId + "_" +
                     std::to_string(timestampNs) + ".yaml";
    } else {
      metaFilePath = iterMap->second;
    }

    std::ofstream ofstr(metaFilePath,
                        std::ios_base::out | std::ios_base::binary);
    if (!ofstr.is_open()) {
      return -1;
    }

    ofstr << metaTemplateData;
    ofstr.flush();
    ofstr.close();

    YAML::Node vehicleMetaNode;
    try {
      vehicleMetaNode = YAML::Load(metaTemplateData);
    } catch (...) {
      return -2;
    }

    templateVehicleMetaConfigs[vehicleId] = vehicleMetaNode;
    templateVehicleMetaFilePaths[vehicleId] = metaFilePath;

    return 0;
  }

  int readVehicleMeteConfig(const std::string &vehicleMetaConfigDirPath) {
    reset();

    if (!RSFileSystem::isDirectoryExist(vehicleMetaConfigDirPath)) {
      return -1;
    }

    std::vector<std::string> metaFilePaths;
    if (!RSFileSystem::searchFilesWithFilter(vehicleMetaConfigDirPath, false,
                                             metaFilePaths,
                                             RSFileSystem::isFilterYamlFile)) {
      return -2;
    }
    // // 针对标定文件软链接
    // const std::string &default_car_calibration_file_path =
    //     "/apollo/DEFAULT_CONFIG/META/car_calibration.yaml";
    // if (RSFileSystem::isFileExist(default_car_calibration_file_path)) {
    //   metaFilePaths.push_back(default_car_calibration_file_path);
    // }
    for (size_t i = 0; i < metaFilePaths.size(); ++i) {
      const std::string &metaFilePath = metaFilePaths[i];
      int ret = addTemplateMetaFile(metaFilePath);
      if (ret != 0) {
        return -3;
      }
    }

    templateVehicleMetaConfigDirPath = vehicleMetaConfigDirPath + "/";

    return 0;
  }

  int addTemplateMetaFile(const std::string &metaFilePath) {
    YAML::Node metaConfigNode;
    try {
      metaConfigNode = YAML::LoadFile(metaFilePath);
    } catch (...) {
      return -1;
    }

    std::string vehicleId;
    try {
      vehicleId = metaConfigNode["vehicle_id"].as<std::string>();
    } catch (...) {
      return -2;
    }

    templateVehicleMetaConfigs[vehicleId] = metaConfigNode;
    templateVehicleMetaFilePaths[vehicleId] = metaFilePath;

    return 0;
  }

public:
  std::string templateVehicleMetaConfigDirPath;
  std::map<std::string, YAML::Node> templateVehicleMetaConfigs;
  std::map<std::string, std::string> templateVehicleMetaFilePaths;
};

// 实现对具有延迟性的Clip Meta信息管理
class RSApp2CarClipMetaInfoItem {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMetaInfoItem>;
  using ConstPtr = std::shared_ptr<const RSApp2CarClipMetaInfoItem>;

public:
  RSApp2CarClipMetaInfoItem() { reset(); }

  virtual ~RSApp2CarClipMetaInfoItem() {
    // NOTHING TODO...
  }

public:
  virtual void reset() {
    systemTimestamp = 0;
    APP_2_CAR_CLIP_META_INFO_ITEM_TYPE = "RSApp2CarClipMetaInfoItem";
  }

  virtual YAML::Node toYamlNode() const {
    YAML::Node configNode;
    return configNode;
  }

  std::string getClipMetaInfoItemType() const {
    return APP_2_CAR_CLIP_META_INFO_ITEM_TYPE;
  }

  bool isClipMetaInfoTagIntersectionItem() const {
    return (APP_2_CAR_CLIP_META_INFO_ITEM_TYPE ==
            "RSApp2CarClipMetaInfoTagIntersectionItem");
  }

  bool isClipMetaInfoTagAudioItem() const {
    return (APP_2_CAR_CLIP_META_INFO_ITEM_TYPE ==
            "RSApp2CarClipMateInfoTagAudioItem");
  }

  bool isClipMetaInfoTagRoadNavItem() const {
    return (APP_2_CAR_CLIP_META_INFO_ITEM_TYPE ==
            "RSApp2CarClipMateInfoTagRoadNavigationItem");
  }

  bool isClipMetaInfoTagIconTypeItem() const {
    return (APP_2_CAR_CLIP_META_INFO_ITEM_TYPE ==
            "RSApp2CarClipMetaInfoTagIconTypeItem");
  }

public:
  std::string APP_2_CAR_CLIP_META_INFO_ITEM_TYPE;
  double systemTimestamp;
};

class RSApp2CarClipMetaInfoTagIntersectionItem
    : public RSApp2CarClipMetaInfoItem {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMetaInfoTagIntersectionItem>;
  using ConstPtr =
      std::shared_ptr<const RSApp2CarClipMetaInfoTagIntersectionItem>;

public:
  RSApp2CarClipMetaInfoTagIntersectionItem() { reset(); }

  virtual void reset() {
    systemTimestamp = 0;
    type = "";
    timestamp = 0;
    gps_lon = 0;
    gps_lat = 0;
    amap_gps_lon = 0;
    amap_gps_lat = 0;
    APP_2_CAR_CLIP_META_INFO_ITEM_TYPE =
        "RSApp2CarClipMetaInfoTagIntersectionItem";
  }

  virtual YAML::Node toYamlNode() const {
    YAML::Node configNode;
    configNode["type"] = type;
    configNode["timetamp"] = timestamp;
    configNode["gps_lon"] = gps_lon;
    configNode["gps_lat"] = gps_lat;
    configNode["amap_gps_lon"] = amap_gps_lon;
    configNode["amap_gps_lat"] = amap_gps_lat;
    return configNode;
  }

public:
  std::string type;
  double timestamp;
  double gps_lon;
  double gps_lat;
  double amap_gps_lon;
  double amap_gps_lat;
};

class RSApp2CarClipMateInfoTagAudioItem : public RSApp2CarClipMetaInfoItem {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMateInfoTagAudioItem>;
  using ConstPtr = std::shared_ptr<const RSApp2CarClipMateInfoTagAudioItem>;

public:
  RSApp2CarClipMateInfoTagAudioItem() { reset(); }

  virtual void reset() {
    systemTimestamp = 0;
    name = "";
    timestamp = 0;
    APP_2_CAR_CLIP_META_INFO_ITEM_TYPE = "RSApp2CarClipMateInfoTagAudioItem";
  }

  virtual YAML::Node toYamlNode() const {
    YAML::Node configNode;
    configNode["name"] = name;
    configNode["timetamp"] = timestamp;
    return configNode;
  }

public:
  std::string name;
  double timestamp;
};

class RSApp2CarClipMateInfoTagRoadNavigationItem
    : public RSApp2CarClipMetaInfoItem {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMateInfoTagRoadNavigationItem>;
  using ConstPtr =
      std::shared_ptr<const RSApp2CarClipMateInfoTagRoadNavigationItem>;

public:
  RSApp2CarClipMateInfoTagRoadNavigationItem() { reset(); }

  virtual void reset() {
    systemTimestamp = 0;
    road_class = "";
    road_type = "";
    APP_2_CAR_CLIP_META_INFO_ITEM_TYPE =
        "RSApp2CarClipMateInfoTagRoadNavigationItem";
  }

  virtual YAML::Node toYamlNode() const {
    YAML::Node configNode;
    configNode["road_class"] = road_class;
    configNode["road_type"] = road_type;
    return configNode;
  }

public:
  std::string road_class;
  std::string road_type;
};

class RSApp2CarClipMetaInfoTagIconTypeItem : public RSApp2CarClipMetaInfoItem {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMetaInfoTagIconTypeItem>;
  using ConstPtr = std::shared_ptr<const RSApp2CarClipMetaInfoTagIconTypeItem>;

public:
  RSApp2CarClipMetaInfoTagIconTypeItem() { reset(); }

  virtual void reset() {
    systemTimestamp = 0;
    icon_type = "";
    APP_2_CAR_CLIP_META_INFO_ITEM_TYPE = "RSApp2CarClipMetaInfoTagIconTypeItem";
  }

  virtual YAML::Node toYamlNode() const {
    YAML::Node configNode;
    configNode["icon_type"] = icon_type;
    return configNode;
  }

public:
  std::string icon_type;
};

class RSApp2CarClipMateInfoManager {
public:
  using Ptr = std::shared_ptr<RSApp2CarClipMateInfoManager>;
  using ConstPtr = std::shared_ptr<const RSApp2CarClipMateInfoManager>;

public:
  using CLIP_KEY = std::pair<std::string, std::string>;
  using CLIP_CHECK_INFO = std::pair<int, double>;

public:
  RSApp2CarClipMateInfoManager() {
    saveRootDirPath.clear();
    isRunning = false;
    app2CarClipMetaInfoCnt = 0;
    app2CarClipMetaInfoMapper.clear();
    candidateClipIds.clear();
  }

  ~RSApp2CarClipMateInfoManager() { finish(); }

public:
  int init(const NodeHandlePtr &pSharedNode,
           const std::string &rootDirPath) {
    if (pSharedNode == nullptr) {
      return -1;
    }
    pSharedNode_ = pSharedNode;

    saveRootDirPath = rootDirPath;
    app2CarClipMetaInfoCnt = 0;
    try {
      isRunning = true;
      workThread =
          std::thread(&RSApp2CarClipMateInfoManager::writeWorkThread, this);
    } catch (...) {
      isRunning = false;
      return -2;
    }

    return 0;
  }

  int finish() {
    if (isRunning == true) {
      isRunning = false;
      if (workThread.joinable()) {
        workThread.join();
      }
    }

    return 0;
  }

  std::string getCurrentRootDirPath() { return saveRootDirPath; }

  int addClipMetaInfoItem(const CLIP_KEY &clipKey,
                          const RSApp2CarClipMetaInfoItem::Ptr &itemPtr) {
    if (itemPtr != nullptr && isRunning) {
      std::lock_guard<std::mutex> lg(app2CarClipMetaInfoMtx);
      auto iterMap = app2CarClipMetaInfoMapper.find(clipKey);
      if (iterMap != app2CarClipMetaInfoMapper.end()) {
        app2CarClipMetaInfoMapper[clipKey].push_back(itemPtr);
      } else {
        std::vector<RSApp2CarClipMetaInfoItem::Ptr> items = {itemPtr};
        app2CarClipMetaInfoMapper[clipKey] = items;
      }
      app2CarClipMetaInfoCnt++;
    }

    return 0;
  }

private:
  std::string checkClipMetaFilePath(const CLIP_KEY &clipKey) {
    return saveRootDirPath + "/" + clipKey.first + "/" + clipKey.first + "_" +
           clipKey.second + "/" + clipKey.first + "_" + clipKey.second +
           ".yaml";
  }

  void writeWorkThread() {
    while (isRunning || app2CarClipMetaInfoCnt > 0) {
      // 每1s检查一次
      std::this_thread::sleep_for(std::chrono::seconds(1));

      // Step1: 检查是否满足写出条件
      {
        std::lock_guard<std::mutex> lg(app2CarClipMetaInfoMtx);
        for (auto iterMap = app2CarClipMetaInfoMapper.begin();
             iterMap != app2CarClipMetaInfoMapper.end(); ++iterMap) {
          const CLIP_KEY &clipKey = iterMap->first;
          std::string clipFilePath = checkClipMetaFilePath(clipKey);

          if (RSFileSystem::isFileExist(clipFilePath) &&
              iterMap->second.size() > 0) {
            auto iterMap2 = candidateClipIds.find(clipKey);
            if (iterMap2 != candidateClipIds.end()) {
              candidateClipIds[clipKey].first++;
            } else {
              candidateClipIds[clipKey].first = 1;
            }

            // 更新最后一个消息信息
            auto &clipMetaInfoItemPtr = *(iterMap->second.rbegin());
            if (clipMetaInfoItemPtr != nullptr) {
              candidateClipIds[clipKey].second =
                  clipMetaInfoItemPtr->systemTimestamp;
            }
          }
        }
      }

      // Step2: 对于满足写出条件的进行写出
      {
        std::lock_guard<std::mutex> lg(app2CarClipMetaInfoMtx);
        for (auto iterMap = candidateClipIds.begin();
             iterMap != candidateClipIds.end(); ++iterMap) {
          const CLIP_KEY &clipKey = iterMap->first;
          // 满足输出条件
          if (iterMap->second.first >= RS_CANDIDATE_CHECK_COUNT_TH) {
            auto iterMap2 = app2CarClipMetaInfoMapper.find(clipKey);
            if (iterMap2 == app2CarClipMetaInfoMapper.end()) {
              continue;
            }
            auto &clipMetaInfoItems = iterMap2->second;

            if (!clipMetaInfoItems.empty()) {
              std::string clipFilePath = checkClipMetaFilePath(clipKey);
              YAML::Node configNode;
              try {
                configNode = YAML::LoadFile(clipFilePath);
              } catch (...) {
                RS_ERROR_STREAM(
                    pSharedNode_,
                    "Load Meta File: " << clipFilePath
                                       << " To Write Delay Meta Info Failed !");
                continue;
              }

              YAML::Node intersectionTypeNode;
              YAML::Node audioTagsNode;
              for (auto iterMap3 = configNode.begin();
                   iterMap3 != configNode.end(); ++iterMap3) {
                const std::string &key = iterMap3->first.as<std::string>();
                if (key == "intersection_type") {
                  intersectionTypeNode = configNode["intersection_type"];
                } else if (key == "audio_tags") {
                  audioTagsNode = configNode["audio_tags"];
                }
              }

              for (size_t i = 0; i < clipMetaInfoItems.size(); ++i) {
                const auto &clipMetaInfoItemPtr = clipMetaInfoItems[i];
                if (clipMetaInfoItemPtr != nullptr) {
                  YAML::Node itemNode = clipMetaInfoItemPtr->toYamlNode();
                  if (clipMetaInfoItemPtr
                          ->isClipMetaInfoTagIntersectionItem()) {
                    intersectionTypeNode.push_back(itemNode);
                  } else if (clipMetaInfoItemPtr
                                 ->isClipMetaInfoTagAudioItem()) {
                    audioTagsNode.push_back(itemNode);
                  } else if (clipMetaInfoItemPtr
                                 ->isClipMetaInfoTagRoadNavItem()) {
                    configNode["road_nav"] = itemNode;
                  } else if (clipMetaInfoItemPtr
                                 ->isClipMetaInfoTagIconTypeItem()) {
                    configNode["icon_type"] =
                        itemNode["icon_type"].as<std::string>();
                  } else {
                    RS_WARN_STREAM(
                        pSharedNode_,
                        "Not Support Clip Meta Info Delay Type: "
                            << clipMetaInfoItemPtr->getClipMetaInfoItemType());
                  }
                }
              }

              std::ofstream ofstr(clipFilePath,
                                  std::ios_base::binary | std::ios_base::out);
              if (!ofstr.is_open()) {
                RS_WARN_STREAM(
                    pSharedNode_,
                    "Open Clip Meta File: " << clipFilePath
                                            << " To Write Failed !");
                continue;
              }
              configNode["intersection_type"] = intersectionTypeNode;
              configNode["audio_tags"] = audioTagsNode;

              YAML::Emitter emitter;
              emitter << configNode;
              ofstr << emitter.c_str() << std::endl;
              ofstr.flush();
              ofstr.close();

              app2CarClipMetaInfoCnt -= clipMetaInfoItems.size();
              clipMetaInfoItems.clear();
            }
          }
        }
      }

      // Step3: 删除超时的
      {
        uint64_t timestampNs = RS_TIMESTAMP_NS;
        std::lock_guard<std::mutex> lg(app2CarClipMetaInfoMtx);
        for (auto iterMap = candidateClipIds.begin();
             iterMap != candidateClipIds.end(); ++iterMap) {
          if ((timestampNs * 1e-9 - iterMap->second.second) >
              RS_CANDIDATE_CHECK_TIMEOUT_TH) {
            const CLIP_KEY &clipKey = iterMap->first;

            auto iterMap2 = app2CarClipMetaInfoMapper.find(clipKey);
            if (iterMap2 != app2CarClipMetaInfoMapper.end()) {
              if (!iterMap2->second.empty()) {
                RS_WARN_STREAM(
                    pSharedNode_,
                    "Some Clip Meta Info Delay Item Not Write To Meta File: "
                        << checkClipMetaFilePath(clipKey)
                        << ", Count = " << (iterMap2->second.size()));
              }
              app2CarClipMetaInfoCnt -= iterMap2->second.size();
              app2CarClipMetaInfoMapper.erase(iterMap2);
            }
          }
        }

        for (auto iterMap = candidateClipIds.begin();
             iterMap != candidateClipIds.end();) {
          auto iterMap2 = app2CarClipMetaInfoMapper.find(iterMap->first);
          if (iterMap2 == app2CarClipMetaInfoMapper.end()) {
            iterMap = candidateClipIds.erase(iterMap);
          } else {
            ++iterMap;
          }
        }
      }
    }
  }

private:
  std::string saveRootDirPath;
  std::mutex app2CarClipMetaInfoMtx;
  std::map<CLIP_KEY, std::vector<RSApp2CarClipMetaInfoItem::Ptr>>
      app2CarClipMetaInfoMapper;
  int app2CarClipMetaInfoCnt;
  std::map<CLIP_KEY, CLIP_CHECK_INFO>
      candidateClipIds; // 当检查RS_CANDIDATE_CHECK_COUNT_TH次文件均存在，则可进行延迟信息写入

  bool isRunning;
  std::thread workThread;

  NodeHandlePtr pSharedNode_ = nullptr;

private:
  const int RS_CANDIDATE_CHECK_COUNT_TH = 3;
  const int RS_CANDIDATE_CHECK_TIMEOUT_TH = 35;
};

// 包含全部的配置
class RSGlobalConfig {
public:
  using Ptr = std::shared_ptr<RSGlobalConfig>;
  using ConstPtr = std::shared_ptr<const RSGlobalConfig>;

public:
  RSGlobalConfig();
  ~RSGlobalConfig();

public:
  void increaseClipId() { ++clipId; }

  void resetClipId() { clipId = 0; }

  std::string getTaskNameDirPath() const {
    return configPath + "/" + taskName + "/";
  }

  std::string getTaskClipWithoutClidIdDirPath() const {
    return configPath + "/" + taskName + "/" + taskName + "_";
  }

  std::string getTaskNameJsonFilePath() const {
    return getTaskNameDirPath() + taskName + ".json";
  }

  std::string getClipName() const { return getClipDiffName(0); }

  std::string getClipDiffName(const int diff) const {
    return taskName + "_" + std::to_string(clipId + diff);
  }

  void updateClipId() {
    std::vector<std::string> directoryPaths;
    RSFileSystem::searchDirectoryWithoutFilter(getTaskNameDirPath(), false,
                                               directoryPaths);

    clipId = -1;
    for (size_t i = 0; i < directoryPaths.size(); ++i) {
      std::string directoryPath =
          RSFileSystem::replaceRepeatSlash(directoryPaths[i]);

      if (directoryPath.size() > 0) {
        while (directoryPath[directoryPath.size() - 1] == '/') {
          directoryPath = directoryPath.substr(0, directoryPath.size() - 1);
          if (directoryPath.empty()) {
            break;
          }
        }
      }

      if (!checkIsClipNameDirPath(directoryPath)) {
        continue;
      }

      size_t lastIdx = directoryPath.find_last_of('_');
      if (lastIdx != std::string::npos) {
        if (lastIdx + 1 < directoryPath.size()) {
          std::string stringIdx = directoryPath.substr(lastIdx + 1);
          // AERROR << "stringIdx = " << stringIdx;
          int idx = atoi(stringIdx.c_str());
          if (idx > clipId) {
            clipId = idx;
          }
        }
      }
    }

    std::cout << "clipId = " << clipId << ", app2CarClipId = " << app2CarClipId
              << std::endl;

    // 保留较大的一个
    clipId = std::max(clipId, app2CarClipId);
    if (clipId < 0) {
      clipId = 0;
    } else {
      ++clipId;
    }
  }

  void updateClipId2() {
    std::vector<std::string> directoryPaths;
    RSFileSystem::searchDirectoryWithoutFilter(getTaskNameDirPath(), false,
                                               directoryPaths);

    clipId = -1;
    for (size_t i = 0; i < directoryPaths.size(); ++i) {
      std::string directoryPath =
          RSFileSystem::replaceRepeatSlash(directoryPaths[i]);

      if (directoryPath.size() > 0) {
        while (directoryPath[directoryPath.size() - 1] == '/') {
          directoryPath = directoryPath.substr(0, directoryPath.size() - 1);
          if (directoryPath.empty()) {
            break;
          }
        }
      }

      if (!checkIsClipNameDirPath(directoryPath)) {
        continue;
      }

      size_t lastIdx = directoryPath.find_last_of('_');
      if (lastIdx != std::string::npos) {
        if (lastIdx + 1 < directoryPath.size()) {
          std::string stringIdx = directoryPath.substr(lastIdx + 1);
          // AERROR << "stringIdx = " << stringIdx;
          int idx = atoi(stringIdx.c_str());
          if (idx > clipId) {
            clipId = idx;
          }
        }
      }
    }

    std::cout << "clipId = " << clipId << std::endl;

    if (clipId < 0) {
      clipId = app2CarClipId;
    }
  }

  bool checkIsClipNameDirPath(const std::string &directoryPath) const {
    const std::string &withoutClipIdDirPath =
        RSFileSystem::replaceRepeatSlash(getTaskClipWithoutClidIdDirPath());
    if (directoryPath.size() <= withoutClipIdDirPath.size()) {
      std::cout << "directoryPath = " << directoryPath
                << " size <= withoutClipIdDirPath = " << withoutClipIdDirPath
                << " size !" << std::endl;
      return false;
    } else if (directoryPath.substr(0, withoutClipIdDirPath.size()) !=
               withoutClipIdDirPath) {
      std::cout << "directoryPath = " << directoryPath
                << " substr != withoutClipIdDirPath = " << withoutClipIdDirPath
                << std::endl;
      return false;
    } else {
      std::cout << "Candidate Clip Directory Path = " << directoryPath
                << std::endl;
    }
    return true;
  }

  std::string getClipNameDirPath() const {
    return getTaskNameDirPath() + getClipName() + "/";
  }

  std::string getClipNameMetaFilePath() const {
    return getClipNameDirPath() + getClipName() + ".yaml";
  }

  std::string getNextClipNameDirPath() const {
    return getTaskNameDirPath() + getClipDiffName(1) + "/";
  }

  std::string getNextClipNameMetaFilePath() const {
    return getNextClipNameDirPath() + getClipDiffName(1) + ".yaml";
  }

  std::string getTaskNameMetaFilePath() const {
    return getTaskNameDirPath() + taskName + ".yaml";
  }

  std::string getClipNameDirPathFromClipId(const int32_t clipId) {
    return getTaskNameDirPath() + taskName + "_" + std::to_string(clipId) + "/";
  }

  int updateMetaFileTemplate(const std::string &vehicleId,
                             const std::string &calibFileData) {
    std::lock_guard<std::mutex> lg(metaMtx);
    int ret =
        vehicleMetaConfig.updateVehicleMetaTemplate(vehicleId, calibFileData);
    if (ret != 0) {
      return -1;
    }
    return 0;
  }

  int updateTaskNameMetaFileOpInfo(const std::string &op,
                                   const std::string &clipNameDirPath) {
    std::lock_guard<std::mutex> lg(taskNameMetaFileMtx);
    const std::string &taskname_metafile_path = getTaskNameMetaFilePath();
    YAML::Node rootNode;
    YAML::Node opItemsNode;
    if (RSFileSystem::isFileExist(taskname_metafile_path)) {
      bool isOk = true;
      try {
        rootNode = YAML::LoadFile(taskname_metafile_path);
      } catch (...) {
        std::cout << "load taskname meta file: " << taskname_metafile_path
                  << " failed !" << std::endl;
        isOk = false;
      }
      if (isOk) {
        for (auto iterMap = rootNode.begin(); iterMap != rootNode.end();
             ++iterMap) {
          const std::string &key = iterMap->first.as<std::string>();
          if (key == "op_clips") {
            opItemsNode = rootNode[key];
            break;
          }
        }
      }
    }

    YAML::Node opItemNode;
    opItemNode["op"] = op;
    opItemNode["clipName"] = clipNameDirPath;
    opItemsNode.push_back(opItemNode);

    rootNode["op_clips"] = opItemsNode;

    YAML::Emitter emitter;
    emitter << rootNode;

    std::ofstream ofstr(taskname_metafile_path,
                        std::ios_base::out | std::ios_base::binary);
    if (!ofstr.is_open()) {
      return -1;
    }

    ofstr << emitter.c_str();
    ofstr.flush();
    ofstr.close();

    return 0;
  }

  int updateMetaFileCameraDownsampleTag(const YAML::Node downsampleTag) {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    vehicleMetaConfigNode["camera_downsample"] = downsampleTag;

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    return 0;
  }

  int updateMetaFileEnvTag(const YAML::Node envTag) {
    // RERROR_SHORT << "taskJsonValue[\"real_vehicle_id\"].template
    // get<std::string>() = " << taskJsonValue["real_vehicle_id"].template
    // get<std::string>();
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    vehicleMetaConfigNode["environments"] = envTag;

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

  int updateMetaFileRoadNavTag(const YAML::Node roadNavTag) {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    vehicleMetaConfigNode["road_nav"] = roadNavTag;

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

  int updateMetaFileIconTypeTag(const YAML::Node iconTypeTag) {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    vehicleMetaConfigNode["icon_type"] =
        iconTypeTag["icon_type"].as<std::string>();

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

  int resetMetaFileEnvTags() {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    YAML::Node envTag;
    envTag["light"] = "";
    envTag["weather"] = "";
    envTag["scene"] = "";

    vehicleMetaConfigNode["environments"] = envTag;

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

  int resetMetaFileRoadNavTags() {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    YAML::Node roadNavTag;
    roadNavTag["light"] = "";
    roadNavTag["weather"] = "";

    vehicleMetaConfigNode["road_nav"] = roadNavTag;

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

  int resetMetaFileIconTypeTag() {
    std::lock_guard<std::mutex> lg(metaMtx);
    YAML::Node vehicleMetaConfigNode;
    int ret = vehicleMetaConfig.getVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);
    if (ret != 0) {
      return -1;
    }

    vehicleMetaConfigNode["icon_type"] = "";

    vehicleMetaConfig.updateVehicleMetaTemplate(
        taskJsonValue["real_vehicle_id"].template get<std::string>(),
        vehicleMetaConfigNode);

    return 0;
  }

public:
  std::mutex globalMtx;
  bool isTaskNameValid;
  std::string taskName;   // 任务名称
  int clipId;             // taskName_clipId
  int app2CarClipId;      //
  std::string configPath; // 数据保存根目录
  json taskJsonValue;     //

  bool isRecord;              // 是否进行写文件: 默认为false
  RS_DATA_SAVE_MODE saveMode; // 保存方法
  float saveDataTh;           // 保存数据时的分割的阈值(MB)

  // Config recordConfig;
  YAML::Node recordConfig;

  std::mutex metaMtx;
  RSVehicleMetaConfig vehicleMetaConfig;

  std::mutex systemMtx;
  RSSystemConfig systemConfig;

  std::mutex app2CarMtx;
  RSApp2CarConfig app2CarConfig;

  std::mutex taskNameMetaFileMtx;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense