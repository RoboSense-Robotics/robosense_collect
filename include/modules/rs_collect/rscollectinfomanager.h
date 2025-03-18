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

#ifndef RSCOLLECTINFOMANAGER_H
#define RSCOLLECTINFOMANAGER_H

/*
 * ModuleName: RSCollectInfoManager
 * Description: 实现采集工具Info.yaml文件的生成
 * Author: hsw
 * Date: 2021-06-15
 *
 */
#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

class RSCollectInfoManager {
public:
  using Ptr = std::shared_ptr<RSCollectInfoManager>;
  using ConstPtr = std::shared_ptr<const RSCollectInfoManager>;

public:
  RSCollectInfoManager();
  ~RSCollectInfoManager();

public:
  int initInfoManager(const RSGlobalConfig::Ptr &pSharedGlobalConfig,
                      const NodeHandlePtr pSharedNode,
                      const bool isInitNextClip);
  int stopInfoManager();
  void setIsWriteMetaInfo(const bool isWriteMetaInfo);
  int appendGPSData(const double timestamp, const double lon, const double lat,
                    const double alt);
  int appendPoseData(const double timestamp, const double roll,
                     const double pitch, const double yaw);
  int updateHeadTime(const double head_time);
  int updateTailTime(const double tail_time);
  double getHeadTime() const { return m_headTime; }
  double getTailTime() const { return m_tailTime; }
  double getDuration() const { return (m_tailTime - m_headTime); }
  void setValidationErrorsNode(YAML::Node &&validationErrorsNode);

private:
  int initInfoManager();

private:
  RSGlobalConfig::Ptr m_pSharedGlobalConfig;
  YAML::Node m_gpsInfoNode;          // GPS节点
  YAML::Node m_poseInfoNode;         // Pose节点
  YAML::Node m_validationErrorsNode; // 校验错误信息节点
  double m_headTime;
  double m_tailTime;
  std::string m_yamlFilePath;
  bool m_isRename;
  bool m_isStopInfoManager;
  bool m_isWriteMateInfo;
  std::string m_renameFilePath;
  bool m_isInitNextClip;
  std::string m_cameraEncoder;
  NodeHandlePtr m_pSharedNode;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSCOLLECTINFOMANAGER_H
