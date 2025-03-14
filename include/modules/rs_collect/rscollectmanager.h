#ifndef RSCOLLECTMANAGER_H
#define RSCOLLECTMANAGER_H

#include <memory>
#include <string>

#include "modules/rs_collect/rsappcarmanager.h"
#include "modules/rs_collect/rsbase64.h"
#include "modules/rs_collect/rsrecordiomanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

class RSCollectManager {
public:
  using Ptr = std::shared_ptr<RSCollectManager>;
  using ConstPtr = std::shared_ptr<const RSCollectManager>;

public:
  RSCollectManager(const YAML::Node &config,
                   const NodeHandlePtr &node);

  ~RSCollectManager();

  bool Init();
  bool Start();
  bool Stop();

private:
  int init();
  int startCollect();
  int initAutoStartClipInfo();
  int initConfig();
  int initVehicleMetaConfig();
  int initSystemConfig();
  int initApp2CarConfig();
  int initApp2CarManager();
  int initMetaInfoManager();
  int initRecordIoManager();

private:
  void segmentCheckWorkThread();

private:
  void messageMonitorCallback(const std::string &channel_name,
                              const RS_MESSAGE_MONITOR_STATUS status);
  void recordIoErrorCallback(const int error_code,
                             const std::string &error_info);

private:
  void sendCar2AppMessage(const RSCar2AppMessage::Ptr &msgPtr);
  void app2carCallback(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdSelfCheck();
  int app2CarCmdStartCollection();
  int app2CarCmdPauseCollection();
  int app2CarCmdEndCollection();
  int app2CarCmdOnlyUpdateJson();
  int app2CarCmdUpdateTag(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdUpdateCalibrationFile(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdUpdateCollectSoftware(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdTagRoadNav(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdTagIconType(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdTagIntersectionType(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdUpdateFileTag(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdTagAudioTags(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdGetCollectSetting(const RSApp2CarMessage::Ptr &msgPtr,
                                  json &logger_json);
  int app2CarCmdSetCollectSetting(const RSApp2CarMessage::Ptr &msgPtr);
  int app2CarCmdGetCollectStatus(const RSApp2CarMessage::Ptr &msgPtr,
                                 json &logger_json);
  int writeTaskNameJson();
  int updateTaskNameJsonValue(const RSApp2CarMessage::Ptr &msgPtr,
                              const bool isUpdateClipId);
  int checkCarAppVehicleIdWithLocal();
  int getLocalVehicleId(std::string &vehicleId);

  bool checkIsCollectRecord() const { return pSharedGlobalConfig_->isRecord; }

  // 判断deviceId是否为自检的DeviceId, 如果不是，则不进行处理
  bool checkApp2CarDeviceId(const std::string &deviceId) const {
    if (deviceId.empty() && selfCheckOpDeviceId_.empty()) {
      return true;
    } else if (deviceId == selfCheckOpDeviceId_) {
      return true;
    }
    return false;
  }

  void updateApp2CarDeviceId(const std::string &deviceId) {
    selfCheckOpDeviceId_ = deviceId;
  }

  int updateTaskNameMetaFileOpInfo(const std::string &op) {
    const std::string &clipDirPath =
        pSharedGlobalConfig_->getClipNameDirPathFromClipId(
            pSharedGlobalConfig_->clipId);
    int ret =
        pSharedGlobalConfig_->updateTaskNameMetaFileOpInfo(op, clipDirPath);
    if (ret == 0) {
      RS_INFO_STREAM(pSharedNode_,
                         "Update TaskName MetaFile Op Info Successed => Op: "
                             << op << " => clipDirPath = " << clipDirPath);
    } else {
      RS_ERROR_STREAM(pSharedNode_,
                          "Update TaskName MetaFile Op Info Failed => Op: "
                              << op << " => clipDirPath = " << clipDirPath);
    }
    return ret;
  }

private:
  RSAppCarManager::Ptr pSharedAppCarManager_;
  RSRecordIoManager::Ptr pSharedRecordIoManager_;
  RSApp2CarClipMateInfoManager::Ptr pSharedClipMetaInfoManager_;
  YAML::Node config_;
  NodeHandlePtr pSharedNode_;
  RSGlobalConfig::Ptr pSharedGlobalConfig_;
  std::shared_ptr<std::thread> pSharedCheckSegThread_;
  bool isStart_;
  bool isPlayControlMode_;
  bool isAutoCollect_;

  std::vector<RSRecordIoManager::Ptr> stopRecordIoManagers_;

  std::string selfCheckOpDeviceId_;

private:
  std::string RS_DEFAULT_CALIBRATION_FILE_PATH;
  const std::string RS_DEFAULT_SUPPORT_FILESYSTEM_TYPE = "ext4";
  const std::string RS_DEFAULT_CALIBRATION_FILE_YAML_NAME =
      "car_calibration.yaml";
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSCOLLECTMANAGER_H
