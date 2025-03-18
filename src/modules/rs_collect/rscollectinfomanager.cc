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

#include "modules/rs_collect/rscollectinfomanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

RSCollectInfoManager::RSCollectInfoManager() {
  m_pSharedGlobalConfig = nullptr;
  m_isStopInfoManager = true;
  m_isWriteMateInfo = true;
  m_isInitNextClip = false;
  m_cameraEncoder = "h265";
}

RSCollectInfoManager::~RSCollectInfoManager() { stopInfoManager(); }

int RSCollectInfoManager::initInfoManager(
    const RSGlobalConfig::Ptr &pSharedGlobalConfig,
    const NodeHandlePtr pSharedNode,
    const bool isInitNextClip) {
  if (pSharedGlobalConfig == nullptr) {
    return -1;
  } else if (pSharedNode == nullptr) {
    return -2;
  }

  m_pSharedGlobalConfig = pSharedGlobalConfig;
  m_pSharedNode = pSharedNode;
  m_pSharedNode = pSharedNode;
  m_isInitNextClip = isInitNextClip;
  m_cameraEncoder =
      m_pSharedGlobalConfig->recordConfig["recorddataconfig"]["encoder_type"]
          .as<std::string>();

  return initInfoManager();
}

int RSCollectInfoManager::stopInfoManager() {
  if (m_isStopInfoManager == true) {
    return 0;
  }
  if (!m_isWriteMateInfo) {
    return 0;
  }

  std::ofstream ofstr(m_yamlFilePath, std::ios_base::out |
                                          std::ios_base::binary |
                                          std::ios_base::trunc);
  if (!ofstr.is_open()) {
    RS_ERROR_STREAM(
        m_pSharedNode,
        "Open Yaml File Path To Write Failed: " << m_yamlFilePath);
    return -1;
  }

  // 获取当前的Mate模板配置
  YAML::Node newVehicleMetaConfigNode; // 避免直接修改模板文件
  {
    std::lock_guard<std::mutex> lg(m_pSharedGlobalConfig->metaMtx);
    YAML::Node vehicleMetaConfigNode;
    if (m_pSharedGlobalConfig->isTaskNameValid) {
      std::string realVehicleId{};
      if (!m_pSharedGlobalConfig->taskJsonValue.is_null()) {
        realVehicleId =
            m_pSharedGlobalConfig->taskJsonValue.value("real_vehicle_id", "");
      }

      int ret = m_pSharedGlobalConfig->vehicleMetaConfig.getVehicleMetaTemplate(
          realVehicleId, vehicleMetaConfigNode);
      if (ret != 0) {
        RS_ERROR(m_pSharedNode,
                     "TaskName Valid, But Real Vehicle Id Matched "
                     "YAML META Not Exist !");
        return -2;
      }
    } else {
      int ret = m_pSharedGlobalConfig->vehicleMetaConfig.getVehicleMetaTemplate(
          m_pSharedGlobalConfig->systemConfig.vehicleInfo,
          vehicleMetaConfigNode);
      if (ret != 0) {
        RS_ERROR(m_pSharedNode,
                     "TaskName InValid, But Real Vehicle Id Matched "
                     "YAML META Not Exist !");
        return -3;
      }
    }
    newVehicleMetaConfigNode = vehicleMetaConfigNode;
  }

  newVehicleMetaConfigNode["validation_errors"] = m_validationErrorsNode;
  newVehicleMetaConfigNode["positions"] = m_gpsInfoNode;
  newVehicleMetaConfigNode["poses"] = m_poseInfoNode;
  newVehicleMetaConfigNode["camera_encoder"] = m_cameraEncoder;
  newVehicleMetaConfigNode["head_time"] = m_headTime;
  newVehicleMetaConfigNode["tail_time"] = m_tailTime;

  YAML::Emitter emitter;
  emitter << newVehicleMetaConfigNode;

  ofstr << emitter.c_str() << std::endl;
  ofstr.flush();
  ofstr.close();

  m_isStopInfoManager = true;

  return 0;
}

void RSCollectInfoManager::setIsWriteMetaInfo(const bool isWriteMetaInfo) {
  m_isWriteMateInfo = isWriteMetaInfo;
}

void RSCollectInfoManager::setValidationErrorsNode(
    YAML::Node &&validationErrorsNode) {
  m_validationErrorsNode = std::move(validationErrorsNode);
}

int RSCollectInfoManager::appendGPSData(const double timestamp,
                                        const double lon, const double lat,
                                        const double alt) {
  YAML::Node gpsNode(YAML::NodeType::value::Sequence);
  {
    gpsNode.push_back(timestamp); // timestamp
    gpsNode.push_back(lon);       // lon
    gpsNode.push_back(lat);       // lat
    gpsNode.push_back(alt);       // alt
  }

  const int gpsCnt = m_gpsInfoNode.size();
  YAML::Node gpsIndexNode;
  gpsIndexNode[std::string("p") + std::to_string(gpsCnt)] = gpsNode;

  m_gpsInfoNode.push_back(gpsIndexNode);

  return 0;
}

int RSCollectInfoManager::appendPoseData(const double timestamp,
                                         const double roll, const double pitch,
                                         const double yaw) {
  YAML::Node poseNode(YAML::NodeType::value::Sequence);
  {
    poseNode.push_back(timestamp); // timestamp
    poseNode.push_back(roll);      // roll
    poseNode.push_back(pitch);     // pitch
    poseNode.push_back(yaw);       // yaw
  }

  const int poseCnt = m_poseInfoNode.size();
  YAML::Node poseIndexNode;
  poseIndexNode[std::string("p") + std::to_string(poseCnt)] = poseNode;

  m_poseInfoNode.push_back(poseIndexNode);

  return 0;
}

int RSCollectInfoManager::updateHeadTime(const double head_time) {
  m_headTime = head_time;

  return 0;
}

int RSCollectInfoManager::updateTailTime(const double tail_time) {
  m_tailTime = tail_time;

  return 0;
}

int RSCollectInfoManager::initInfoManager() {
  if (m_isInitNextClip) {
    m_yamlFilePath = m_pSharedGlobalConfig->getNextClipNameMetaFilePath();
  } else {
    m_yamlFilePath = m_pSharedGlobalConfig->getClipNameMetaFilePath();
  }
  m_isStopInfoManager = false;
  m_isWriteMateInfo = true;
  m_headTime = 0;
  m_tailTime = 0;

  return 0;
}

} // namespace collect
} // namespace rs_collect
} // namespace robosense