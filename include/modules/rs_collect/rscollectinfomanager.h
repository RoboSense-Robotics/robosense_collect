/******************************************************************************
 * Copyright 2017 RoboSense All rights reserved.
 * Suteng Innovation Technology Co., Ltd. www.robosense.ai

 * This software is provided to you directly by RoboSense and might
 * only be used to access RoboSense LiDAR. Any compilation,
 * modification, exploration, reproduction and redistribution are
 * restricted without RoboSense's prior consent.

 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESSED OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOSENSE BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

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
