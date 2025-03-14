#include "modules/rs_collect/rscollectmanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

RSCollectManager::RSCollectManager(const YAML::Node &config,
                                   const NodeHandlePtr &node) {

  if (node == nullptr) {
    throw std::runtime_error("RSCollectManager Input node is Nullptr !");
  }

  config_ = config;
  pSharedNode_ = node;
}

RSCollectManager::~RSCollectManager() { Stop(); }

bool RSCollectManager::Init() {
  if (pSharedNode_ == nullptr) {
    RS_ERROR(pSharedNode_, "node is nullptr");
    return false;
  }

  try {
    pSharedGlobalConfig_.reset(new RSGlobalConfig());
  } catch (const std::exception &e) {
    RS_ERROR(pSharedNode_, "malloc global config object failed !");
    return false;
  }

  pSharedGlobalConfig_->recordConfig = config_;
  int ret = init();
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "collect manager initial failed: ret = " << ret);
    return false;
  }

  return true;
}

bool RSCollectManager::Start() {
  if (isAutoCollect_) {
    int ret = initAutoStartClipInfo();
    if (ret != 0) {
      const std::string &error_info =
          "inital auto start clip info failed: ret = " + std::to_string(ret);
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return false;
    }

    ret = startCollect();
    if (ret != 0) {
      const std::string &error_info =
          "Auto Collect Mode: collect manager start failed: ret = " +
          std::to_string(ret);
      RS_ERROR_STREAM(pSharedNode_, error_info);

      return false;
    }

    ret = pSharedRecordIoManager_->AddClipRecord(true);
    if (ret != 0) {
      const std::string &error_info =
          "Auto Collect Mode: add record clip failed: ret = " +
          std::to_string(ret);
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return false;
    }

    pSharedRecordIoManager_->StartRecord();

    RS_INFO(pSharedNode_, "collect manager is auto start !");
  } else {
    RS_INFO(pSharedNode_, "collect manager is control by app !");
  }

  try {
    isStart_ = true;
    pSharedCheckSegThread_.reset(
        new std::thread(&RSCollectManager::segmentCheckWorkThread, this));
  } catch (const std::exception &e) {
    const std::string &error_info =
        "malloc clip segment check work thread failed !";
    RS_ERROR_STREAM(pSharedNode_, error_info);
    isStart_ = false;
    return false;
  }

  return true;
}

bool RSCollectManager::Stop() {
  if (pSharedAppCarManager_ != nullptr) {
    pSharedAppCarManager_.reset();
  }

  isStart_ = false;
  if (pSharedCheckSegThread_ != nullptr) {
    if (pSharedCheckSegThread_->joinable()) {
      pSharedCheckSegThread_->join();
    }
  }

  if (pSharedRecordIoManager_ != nullptr) {
    pSharedRecordIoManager_->StopRecord();

    pSharedRecordIoManager_->Stop();
  }
  pSharedRecordIoManager_.reset();

  return true;
}

int RSCollectManager::init() {
  int ret = 0;

  RS_DEFAULT_CALIBRATION_FILE_PATH =
      config_["recordmetaconfig"]["template_meta_directory_path"]
          .as<std::string>() +
      "/" + RS_DEFAULT_CALIBRATION_FILE_YAML_NAME;
  isAutoCollect_ = config_["recordcollectconfig"]["is_auto_collect"].as<bool>();
  isPlayControlMode_ = false;
  pSharedGlobalConfig_->configPath =
      config_["recorddataconfig"]["rdcs_root_directory_path"].as<std::string>();
  selfCheckOpDeviceId_.clear();

  ret = initConfig();
  if (ret != 0) {
    const std::string &error_info =
        "Initial YAML::Node Failed: ret = " + std::to_string(ret);
    RS_ERROR_STREAM(pSharedNode_, error_info);
    return -2;
  }

  // std::cerr << "initConfig() Successed !" << std::endl;

  if (isAutoCollect_) {
    ret = initRecordIoManager();
    if (ret != 0) {
      const std::string &error_info =
          "Auto Start Mode: Initial Record Io Manager: ret = " +
          std::to_string(ret);
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -5;
    }

    // std::cerr << "initRecordIoManager() Successed !" << std::endl;

  } else {
    // app 控制时启动 App2Car Manager
    ret = initApp2CarManager();
    if (ret != 0) {
      const std::string &error_info =
          "App Control Mode: Initial App2Car Manager Failed: ret = " +
          std::to_string(ret);
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -6;
    }
  }

  return 0;
}

int RSCollectManager::initAutoStartClipInfo() {
  const auto &recordsystemconfig = config_["recordsystemconfig"];
  // 构造日期
  auto now = std::chrono::system_clock::now();
  std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm *now_tm = std::localtime(&now_time);
  std::ostringstream str;
  str << std::put_time(now_tm, "%Y-%m-%d-%H-%M-%S");
  std::string dateString = str.str();
  // 更新起始clipId
  if (recordsystemconfig["task_name"].as<std::string>().empty()) {
    pSharedGlobalConfig_->clipId = 0;
  } else {
    pSharedGlobalConfig_->clipId = recordsystemconfig["clip_id"].as<int32_t>();
  }

  // 更新Vehicle ID信息
  std::string config_vehicle_id =
      recordsystemconfig["vehicle_id"].as<std::string>();
  std::string calib_vehicle_id = "";
  std::string calib_vehicle_alias = "";
  if (RSFileSystem::isFileExist(RS_DEFAULT_CALIBRATION_FILE_PATH)) {
    YAML::Node configNode;
    try {
      configNode = YAML::LoadFile(RS_DEFAULT_CALIBRATION_FILE_PATH);
    } catch (const std::exception &e) {
      const std::string &error_info = "Load Default Calibration File Failed: " +
                                      RS_DEFAULT_CALIBRATION_FILE_PATH;
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -1;
    }

    try {
      calib_vehicle_id = configNode["vehicle_id"].as<std::string>();
    } catch (const std::exception &e) {
      calib_vehicle_id = "";
      const std::string &error_info =
          "Load Default Calibration File Successed, But Parse vehicle_id "
          "Failed !";
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -2;
    }

    try {
      calib_vehicle_alias = configNode["vehicle_alias"].as<std::string>();
    } catch (const std::exception &e) {
      calib_vehicle_alias = "";
      RS_WARN(pSharedNode_,
                  "Load Default Calibration File Successed, But Parse "
                  "vehicle_alias Failed !");
    }
  }
  if (!calib_vehicle_id.empty() && !config_vehicle_id.empty()) {
    if (calib_vehicle_id != config_vehicle_id) {
      const std::string &error_info =
          "Not Match: config_vehicle_id = " + config_vehicle_id +
          ", calib_vehicle_id = " + calib_vehicle_id;
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -3;
    }
  } else if (!calib_vehicle_id.empty()) {
    config_vehicle_id = calib_vehicle_id;
  }
  if (config_vehicle_id.empty()) {
    const std::string &error_info = "Not Setting Vehicle ID";
    RS_ERROR_STREAM(pSharedNode_, error_info);
    return -4;
  }

  if (!recordsystemconfig["task_name"].as<std::string>().empty() &&
      !calib_vehicle_alias.empty()) {
    if (recordsystemconfig["task_name"].as<std::string>() !=
        calib_vehicle_alias) {
      const std::string &error_info =
          "Not Match: task_name = " +
          recordsystemconfig["task_name"].as<std::string>() +
          ", calib_vehicle_alias = " + calib_vehicle_alias;
      RS_ERROR_STREAM(pSharedNode_, error_info);
      return -5;
    }
    pSharedGlobalConfig_->taskName = calib_vehicle_alias;
  } else if (recordsystemconfig["task_name"].as<std::string>().empty() &&
             !calib_vehicle_alias.empty()) {
    pSharedGlobalConfig_->taskName = calib_vehicle_alias;
  } else if (!recordsystemconfig["task_name"].as<std::string>().empty()) {
    pSharedGlobalConfig_->taskName =
        recordsystemconfig["task_name"].as<std::string>();
  }

  // 判断文件夹是否存在，如果存在了，则强制添加时间戳
  const bool is_add_timestamp =
      recordsystemconfig["task_name_add_timestamp"].as<bool>() ||
      RSFileSystem::isDirectoryExist(pSharedGlobalConfig_->configPath + "/" +
                                     pSharedGlobalConfig_->taskName);
  if (pSharedGlobalConfig_->taskName.empty()) {
    pSharedGlobalConfig_->taskName = dateString;
  } else if (is_add_timestamp) {
    pSharedGlobalConfig_->taskName += "_" + dateString;
  }

  config_["recordsystemconfig"]["vehicle_id"] = config_vehicle_id;
  pSharedGlobalConfig_->recordConfig = config_;
  pSharedGlobalConfig_->systemConfig.vehicleInfo = config_vehicle_id;

  RS_INFO_STREAM(pSharedNode_,
                     "task_name = "
                         << pSharedGlobalConfig_->taskName << ", vehicle_id = "
                         << pSharedGlobalConfig_->systemConfig.vehicleInfo);

  pSharedGlobalConfig_->isTaskNameValid = false;

  return 0;
}

int RSCollectManager::startCollect() {
  if (pSharedRecordIoManager_ != nullptr) {
    bool isSuccess = pSharedRecordIoManager_->Start();
    if (!isSuccess) {
      RS_ERROR(pSharedNode_, "Record Io Manager Start Failed !");
      return -1;
    }
    pSharedRecordIoManager_->StopRecord();
  } else {
    RS_ERROR(pSharedNode_, "Record Io Manager Is Nullptr !");
    return -2;
  }

  return 0;
}

int RSCollectManager::initConfig() {
  int ret = 0;
  // load vehicle meta
  ret = initVehicleMetaConfig();
  if (ret != 0) {
    RS_ERROR_STREAM(
        pSharedNode_,
        "Initial Configure Failed: initial vehicle config failed: ret = "
            << ret);
    return -1;
  }

  ret = initSystemConfig();
  if (ret != 0) {
    RS_ERROR_STREAM(
        pSharedNode_,
        "Initial Configure Failed: initial system config failed: ret = "
            << ret);
    return -2;
  }

  ret = initApp2CarConfig();
  if (ret != 0) {
    RS_ERROR_STREAM(
        pSharedNode_,
        "Initial Configure Failed: initial app2car config failed: ret = "
            << ret);
    return -3;
  }

  return 0;
}

int RSCollectManager::initVehicleMetaConfig() {
  const std::string &vehicleMetaConfigDirPath =
      config_["recordmetaconfig"]["template_meta_directory_path"]
          .as<std::string>();
  int ret = pSharedGlobalConfig_->vehicleMetaConfig.readVehicleMeteConfig(
      vehicleMetaConfigDirPath);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "read vehicle meta config from file: "
                            << vehicleMetaConfigDirPath
                            << " failed: ret = " << ret);
    return -1;
  }

  return 0;
}

int RSCollectManager::initSystemConfig() {
  int ret = pSharedGlobalConfig_->systemConfig.fromProtoMessage(
      config_["recordsystemconfig"]);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
      "parse system configure from proto message failed: ret = " << ret);
    return -1;
  }

  return 0;
}

int RSCollectManager::initApp2CarConfig() {
  int ret = pSharedGlobalConfig_->app2CarConfig.fromProtoMessage(
      config_["recordappcarconfig"]);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
      "parse app2car configure from proto message failed: ret = " << ret);
    return -1;
  }

  return 0;
}

int RSCollectManager::initApp2CarManager() {
  try {
    pSharedAppCarManager_.reset(new RSAppCarManager());
  } catch (const std::exception &e) {
    RS_ERROR(pSharedNode_, "Malloc App2Car Manager Failed !");
    return -1;
  }

  int ret = pSharedAppCarManager_->initManager(
      pSharedNode_, pSharedGlobalConfig_->app2CarConfig,
      // vehicleId,
      std::bind(&RSCollectManager::app2carCallback, this,
                std::placeholders::_1));

  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "App2Car Manager Initial Failed: ret = " << ret);
    return -4;
  }

  return 0;
}

int RSCollectManager::initMetaInfoManager() {
  if (pSharedClipMetaInfoManager_ != nullptr) {
    pSharedClipMetaInfoManager_->finish();
    pSharedClipMetaInfoManager_.reset();
  }

  if (pSharedClipMetaInfoManager_ == nullptr) {
    try {
      pSharedClipMetaInfoManager_.reset(new RSApp2CarClipMateInfoManager());
    } catch (const std::exception &e) {
      RS_ERROR(pSharedNode_, "Malloc Clip Meta Info Manager Failed !");
      return -1;
    }
  }

  int ret = pSharedClipMetaInfoManager_->init(pSharedNode_,
                                              pSharedGlobalConfig_->configPath);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Clip Meta Info Manager Initial Failed: ret = " << ret);
    return -2;
  }

  return 0;
}

int RSCollectManager::initRecordIoManager() {
  const auto &callback =
      std::bind(&RSCollectManager::messageMonitorCallback, this,
                std::placeholders::_1, std::placeholders::_2);
  const auto &callback2 =
      std::bind(&RSCollectManager::recordIoErrorCallback, this,
                std::placeholders::_1, std::placeholders::_2);
  try {
    pSharedRecordIoManager_.reset(new RSRecordIoManager(
        pSharedGlobalConfig_, callback, callback2, pSharedNode_));
  } catch (const std::exception &e) {
    RS_ERROR(pSharedNode_, "Malloc Record Io Manager Failed !");
    return -1;
  }

  bool isSuccess = pSharedRecordIoManager_->Init();
  if (!isSuccess) {
    RS_ERROR(pSharedNode_, "Record Io Manager Initial Failed !");
    return -2;
  }

  // std::cerr << "pSharedRecordIoManager_->Init() Successed !" << std::endl;

  return 0;
}

void RSCollectManager::segmentCheckWorkThread() {
  while (isStart_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    if (pSharedRecordIoManager_ && isStart_) {
      if (pSharedRecordIoManager_->CheckSegment()) {
        pSharedGlobalConfig_->increaseClipId(); //
        int ret = pSharedRecordIoManager_->AddClipRecord(true);
        if (ret != 0) {
          const std::string &error_info =
              "Segement Check Add Clip Failed: ret = " + std::to_string(ret);
          RS_ERROR_STREAM(pSharedNode_, error_info);
        }

        if (!isAutoCollect_) {
          auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
              RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID);
          if (car2AppMsgPtr != nullptr) {
            car2AppMsgPtr->content["task_name"] =
                pSharedGlobalConfig_->taskName;
            car2AppMsgPtr->content["clip_id"] = pSharedGlobalConfig_->clipId;
            sendCar2AppMessage(car2AppMsgPtr);
            RS_INFO_STREAM(pSharedNode_,
                               "pSharedGlobalConfig_->clipId = "
                                   << pSharedGlobalConfig_->clipId);
          }
        }
      }
    }
  }
}

void RSCollectManager::messageMonitorCallback(
    const std::string &channel_name, const RS_MESSAGE_MONITOR_STATUS status) {
  // 反馈设备状态信息给APP
  RSCar2AppMessage::Ptr car2AppMessagePtr;
  switch (status) {
  case RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_NORMAL: {
    car2AppMessagePtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO);
    car2AppMessagePtr->content["sensor_id"] = channel_name;
    car2AppMessagePtr->content["status"] = "设备正常";
    break;
  }
  case RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_WARNING: {
    car2AppMessagePtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_WARN);
    car2AppMessagePtr->content["sensor_id"] = channel_name;
    car2AppMessagePtr->content["status"] = "设备暂时异常";
    break;
  }
  case RS_MESSAGE_MONITOR_STATUS::RS_MESSAGE_MONITOR_ERROR: {
    car2AppMessagePtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL);
    car2AppMessagePtr->content["sensor_id"] = channel_name;
    car2AppMessagePtr->content["status"] = "设备异常";
    break;
  }
  }

  if (car2AppMessagePtr != nullptr) {
    sendCar2AppMessage(car2AppMessagePtr);
  }
}

void RSCollectManager::recordIoErrorCallback(const int error_code,
                                             const std::string &error_info) {
  RSCar2AppMessage::Ptr car2AppMessagePtr =
      RSCar2AppMessageFactory::createCar2AppMessage(
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL);

  if (car2AppMessagePtr != nullptr) {
    car2AppMessagePtr->content["error_code"] = error_code;
    car2AppMessagePtr->content["error_info"] = error_info;
    sendCar2AppMessage(car2AppMessagePtr);
  }
}

void RSCollectManager::sendCar2AppMessage(const RSCar2AppMessage::Ptr &msgPtr) {
  if (pSharedAppCarManager_ != nullptr) {
    if (msgPtr != nullptr) {
      pSharedAppCarManager_->addSendMessage(msgPtr);
    } else {
      RS_WARN(pSharedNode_,
          "To Send msgPtr is Nullptr, Send Message From Car To App Failed !");
    }
  } else {
    RS_WARN(pSharedNode_,
        "AppCar Manager is Nullptr, Send Message From Car To App Failed !");
  }
}

void RSCollectManager::app2carCallback(const RSApp2CarMessage::Ptr &msgPtr) {
  if (msgPtr != nullptr) {
    if (isAutoCollect_) {
      RS_WARN(pSharedNode_,
          "采集软件自动开始采集, 不接受APP操作控制 !");
      return;
    }

    // 固定回复Nothing类型消息
    if (msgPtr->cmd_type ==
        RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING) {
      auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING);
      if (car2AppMsgPtr != nullptr) {
        sendCar2AppMessage(car2AppMsgPtr);
      }
    } else {
      auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO);
      if (car2AppMsgPtr != nullptr) {
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "接收到该请求...";
        sendCar2AppMessage(car2AppMsgPtr);
      }
    }

    // RCLCPP_ERROR(pSharedNode_->get_logger(), "=================== cmd_type =
    // " << static_cast<int>(msgPtr->cmd_type);

    switch (msgPtr->cmd_type) {
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING: {
      // NOTHING TODO...
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SELFCHECK: {
      // 更新Device Id
      if (!isPlayControlMode_) {
        updateApp2CarDeviceId(msgPtr->device_id);
      } else if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "旧SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            "并且已经启动设备, 新的SELFCHECK 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      // 更新Json::Value
      updateTaskNameJsonValue(msgPtr, false);

      int ret = 0;
      ret = app2CarCmdSelfCheck();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "设备自检失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "设备自检通过!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STARTCOLLECTION: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", STARTCOLLECTION 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      // 更新Json::Value
      updateTaskNameJsonValue(msgPtr, true);

      int ret = app2CarCmdStartCollection();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "开始采集失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "开始采集成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_PAUSECOLLECTION: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", PAUSECOLLECTION 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      // 更新Json::Value
      updateTaskNameJsonValue(msgPtr, false);

      int ret = app2CarCmdOnlyUpdateJson();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] = static_cast<int>(
            RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新Json失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新Json成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      ret = app2CarCmdPauseCollection();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "暂停采集失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "暂停采集成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ENDCOLLECTION: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", ENDCOLLECTION 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      // 更新Json::Value
      updateTaskNameJsonValue(msgPtr, false);

      int ret = app2CarCmdEndCollection();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "停止采集失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "停止采集成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", ONLYUPDATEJSON 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      // 更新Json::Value
      updateTaskNameJsonValue(msgPtr, false);

      int ret = app2CarCmdOnlyUpdateJson();
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新Json失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新Json成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_TAG: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", UPDATE TAG命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdUpdateTag(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新环境Tag失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新环境Tag成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE: {
      int ret = app2CarCmdUpdateCalibrationFile(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新标定文件失败!";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新标定文件成功!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", UPDATE COLLECT SOFTWARE 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }
      auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
          RS_CAR_2_APP_MESSAGE_TYPE::
              RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS);
      car2AppMsgPtr->content["Receive_cmd_type"] =
          static_cast<int>(msgPtr->cmd_type);
      car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
      car2AppMsgPtr->content["info"] =
          "软件更新失败: Apollo Collect Software Not Support!";
      car2AppMsgPtr->content["error_code"] = -1;
      car2AppMsgPtr->content["progress"] = 0;
      sendCar2AppMessage(car2AppMsgPtr);
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", TAG ROAD NAVIGATION 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdTagRoadNav(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新导航信息失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] =
            "更新导航信息成功, 将在该clip采集完成后约10s写入Meta文件!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ICON_TYPE: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", TAG ICON TYPE 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdTagIconType(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新收费站信息失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] =
            "更新收费站信息成功, 将在该clip采集完成后约10s写入Meta文件!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", TAG INTERSECTION TYPE 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdTagIntersectionType(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "添加路口信息失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] =
            "添加路口信息成功，将在该clip采集完成后约10s写入Meta文件 !";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", UPDATE FILE TAG 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdUpdateFileTag(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新文件数据失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "更新文件数据成功 !";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG: {
      // 设备号检查
      if (!checkApp2CarDeviceId(msgPtr->device_id)) {
        const std::string &error_info =
            "SELFCHECK 命令设备ID: " + selfCheckOpDeviceId_ +
            ", UPDATE AUDIO TAG 命令设备ID:" + msgPtr->device_id +
            ", 该命令将不会被处理 !";
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = error_info;
        car2AppMsgPtr->content["error_code"] = -1;
        sendCar2AppMessage(car2AppMsgPtr);
        return;
      }

      int ret = app2CarCmdTagAudioTags(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "添加语音信息失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] =
            "添加语音信息成功，将在该clip采集完成后约10s写入Meta文件!";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING: {
      json logger_json;
      int ret = app2CarCmdGetCollectSetting(msgPtr, logger_json);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);

        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "获取采集设置失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content = logger_json;
        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "获取采集设置成功 !";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING: {
      int ret = app2CarCmdSetCollectSetting(msgPtr);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);

        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "设置采集设置失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);

        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "设置采集设置成功 !";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS: {
      json logger_json;
      int ret = app2CarCmdGetCollectStatus(msgPtr, logger_json);
      if (ret != 0) {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);

        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "获取采集状态失败 !";
        car2AppMsgPtr->content["error_code"] = ret;
        sendCar2AppMessage(car2AppMsgPtr);
      } else {
        auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
            RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
        car2AppMsgPtr->content = logger_json;

        car2AppMsgPtr->content["Receive_cmd_type"] =
            static_cast<int>(msgPtr->cmd_type);
        car2AppMsgPtr->content["Receive_cmd_id"] = msgPtr->cmd_id;
        car2AppMsgPtr->content["info"] = "获取采集状态成功 !";
        car2AppMsgPtr->content["error_code"] = 0;
        sendCar2AppMessage(car2AppMsgPtr);
      }

      break;
    }
    default: {
      break;
    }
    }
  }
}

int RSCollectManager::app2CarCmdSelfCheck() {
  RS_INFO(pSharedNode_, "PlayControl: 开始");

  if (isPlayControlMode_) {
    return 0;
  }

  if (!isAutoCollect_) {
    int ret = 0;
    ret = checkCarAppVehicleIdWithLocal();
    if (ret != 0) {
      RS_ERROR_STREAM(
          pSharedNode_,
          "SelfCheck: Check CarApp VehicleId With Local Not Equal: ret = "
              << ret);
      return -1;
    }

    ret = initMetaInfoManager();
    if (ret != 0) {
      RS_ERROR_STREAM(
          pSharedNode_,
          "SelfCheck: Meta Info Manager Failed: ret = " << ret);
      return -2;
    }

    ret = initRecordIoManager();
    if (ret != 0) {
      RS_ERROR_STREAM(
          pSharedNode_,
          "SelfCheck: Record Io Manager Failed: ret = " << ret);
      return -3;
    }

    ret = startCollect();
    if (ret != 0) {
      RS_ERROR_STREAM(
          pSharedNode_,
          "SelfCheck: Start Collect Failed: ret = " << ret);
      return -4;
    }
  }

  int ret = updateTaskNameMetaFileOpInfo("SelfCheck");
  RS_INFO_STREAM(pSharedNode_,
                     "Update TaskName MetaFile Op Info: ret = " << ret);

  isPlayControlMode_ = true;

  return 0;
}

int RSCollectManager::app2CarCmdStartCollection() {
  if (!isPlayControlMode_) {
    return 0;
  } else if (pSharedGlobalConfig_->isRecord == true) {
    return 0;
  }

  int ret = 0;
  pSharedGlobalConfig_->isRecord = true;
  if (pSharedRecordIoManager_ != nullptr) {
    if (!pSharedRecordIoManager_->CheckIsValidClip()) {
      pSharedGlobalConfig_->updateClipId();
      ret = pSharedRecordIoManager_->AddClipRecord(true);
      if (ret != 0) {
        RS_ERROR_STREAM(
            pSharedNode_,
            "Start Collection: Add Clip Record Failed: ret = " << ret);
        goto FAILED_CMD_START_COLLECTION;
      }
    }

    bool isSuccess = pSharedRecordIoManager_->StartRecord();
    if (!isSuccess) {
      RS_ERROR(pSharedNode_, "Start Record Failed !");
      goto FAILED_CMD_START_COLLECTION;
    }
  }

  //
  if (!isAutoCollect_) {
    auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID);
    if (car2AppMsgPtr != nullptr) {
      car2AppMsgPtr->content["task_name"] = pSharedGlobalConfig_->taskName;
      car2AppMsgPtr->content["clip_id"] = pSharedGlobalConfig_->clipId;
      sendCar2AppMessage(car2AppMsgPtr);
      RS_INFO_STREAM(pSharedNode_,
          "pSharedGlobalConfig_->clipId = " << pSharedGlobalConfig_->clipId);
    }
  }

  ret = updateTaskNameMetaFileOpInfo("Start_Collect");
  RS_INFO_STREAM(pSharedNode_,
                     "Update TaskName MetaFile Op Info: ret = " << ret);

  return 0;

FAILED_CMD_START_COLLECTION:
  pSharedGlobalConfig_->isRecord = false;
  if (pSharedRecordIoManager_ != nullptr) {
    pSharedRecordIoManager_.reset();
    RS_INFO(pSharedNode_, "Operator Cmd Start Collection Failed !");
  }
  return -1;
}

int RSCollectManager::app2CarCmdPauseCollection() {
  if (!isPlayControlMode_) {
    return 0;
  } else if (pSharedGlobalConfig_->isRecord == false) {
    return 0;
  }

  int ret = 0;
  pSharedGlobalConfig_->isRecord = false;
  if (pSharedRecordIoManager_ != nullptr) {
    bool isSuccess = pSharedRecordIoManager_->StopRecord();
    if (!isSuccess) {
      RS_ERROR(pSharedNode_, "Stop Record Failed !");
      goto FAILED_CMD_PAUSE_COLLECTION;
    }
  }

  // 结束时更新ClipId
  std::this_thread::sleep_for(std::chrono::seconds(15));
  RS_INFO_STREAM(pSharedNode_,
                     "pSharedGlobalConfig_->clipId = " << pSharedGlobalConfig_->clipId);
  pSharedGlobalConfig_->updateClipId2();
  if (!isAutoCollect_) {
    auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID);
    if (car2AppMsgPtr != nullptr) {
      car2AppMsgPtr->content["task_name"] = pSharedGlobalConfig_->taskName;
      car2AppMsgPtr->content["clip_id"] = pSharedGlobalConfig_->clipId;
      sendCar2AppMessage(car2AppMsgPtr);
      RS_INFO_STREAM(pSharedNode_,
          "pSharedGlobalConfig_->clipId = " << pSharedGlobalConfig_->clipId);
    }
  }

  ret = updateTaskNameMetaFileOpInfo("Pause_Collect");
  RS_INFO_STREAM(pSharedNode_,
                     "Update TaskName MetaFile Op Info: ret = " << ret);

  return 0;

FAILED_CMD_PAUSE_COLLECTION:
  pSharedGlobalConfig_->isRecord = true;
  RS_ERROR(pSharedNode_, "Operator Cmd Pause Collection Failed !");
  return -1;
}

int RSCollectManager::app2CarCmdEndCollection() {
  RS_INFO(pSharedNode_, "PlayControl: 停止");
  if (!isPlayControlMode_) {
    return 0;
  }
  // RS_INFO(pSharedNode_, "PlayControl: 停止2";
  if (pSharedGlobalConfig_->isRecord) {
    app2CarCmdPauseCollection();
  }
  // RS_INFO(pSharedNode_, "PlayControl: 停止2";
  // 结束时更新ClipId
  std::this_thread::sleep_for(std::chrono::seconds(5));
  RS_INFO_STREAM(pSharedNode_,
    "pSharedGlobalConfig_->clipId = " << pSharedGlobalConfig_->clipId);
  pSharedGlobalConfig_->updateClipId2();
  if (!isAutoCollect_) {
    auto car2AppMsgPtr = RSCar2AppMessageFactory::createCar2AppMessage(
        RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID);
    if (car2AppMsgPtr != nullptr) {
      car2AppMsgPtr->content["task_name"] = pSharedGlobalConfig_->taskName;
      car2AppMsgPtr->content["clip_id"] = pSharedGlobalConfig_->clipId;
      sendCar2AppMessage(car2AppMsgPtr);
      RS_INFO_STREAM(pSharedNode_,
          "pSharedGlobalConfig_->clipId = " << pSharedGlobalConfig_->clipId);
    }
  }

  int ret = updateTaskNameMetaFileOpInfo("Stop_Collect");
  RS_INFO_STREAM(pSharedNode_,
    "Update TaskName MetaFile Op Info: ret = " << ret);

  // RCLCPP_INFO(pSharedNode_->get_logger(), "PlayControl: 停止2";
  if (pSharedRecordIoManager_ != nullptr) {
    pSharedRecordIoManager_->Stop();
    if (pSharedRecordIoManager_->getTimeoutClipCnt() != 0) {
      stopRecordIoManagers_.push_back(pSharedRecordIoManager_);
    }
    pSharedRecordIoManager_.reset();
  }
  // RCLCPP_INFO(pSharedNode_->get_logger(), "PlayControl: 停止2";
  //
  if (pSharedClipMetaInfoManager_ != nullptr) {
    pSharedClipMetaInfoManager_->finish();
    pSharedClipMetaInfoManager_.reset();
  }

  isPlayControlMode_ = false;
  RS_INFO(pSharedNode_, "PlayControl: 停止2");
  return 0;
}

int RSCollectManager::app2CarCmdOnlyUpdateJson() {
  int ret = writeTaskNameJson();
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Write TaskName Json Failed: ret = " << ret);
    return -1;
  }

  return 0;
}

int RSCollectManager::app2CarCmdUpdateTag(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarUpdateTag::Ptr app2CarUpdateTagMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarUpdateTag>(app2CarMsgPtr);
  if (app2CarUpdateTagMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr!");
    return -1;
  } else if (!checkIsCollectRecord()) {
    RS_ERROR(pSharedNode_, "Not Start Record, Not Support Update Env Tag!");
    return -2;
  }

  YAML::Node envTag(YAML::NodeType::Map);
  const json &content = app2CarUpdateTagMsgPtr->content;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    const std::string &value = content[key].template get<std::string>();
    envTag[key] = value;
  }

  int ret = pSharedGlobalConfig_->updateMetaFileEnvTag(envTag);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Update Meta File Env Tag Failed: ret = " << ret);
    return -2;
  }

  return 0;
}

int RSCollectManager::app2CarCmdUpdateCalibrationFile(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarUpdateCalibrationFile::Ptr app2CarUpdateCalibFileMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarUpdateCalibrationFile>(app2CarMsgPtr);
  if (app2CarUpdateCalibFileMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  }

  std::string vehicle_id;
  std::string calib_file;

  const json &content = app2CarUpdateCalibFileMsgPtr->content;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    const std::string &value = content[key].template get<std::string>();
    if (key == "vehicle_id") {
      vehicle_id = value;
    } else if (key == "calib_file") {
      calib_file = value;
    }
  }

  int ret =
      pSharedGlobalConfig_->updateMetaFileTemplate(vehicle_id, calib_file);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Update Meta File Template Failed: ret = "
                            << ret << ", vehicle_id = " << vehicle_id);
    return -2;
  }

  return 0;
}

int RSCollectManager::app2CarCmdUpdateCollectSoftware(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  (void)(app2CarMsgPtr);
  return 0;
}

int RSCollectManager::app2CarCmdTagRoadNav(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarTagRoadNavigation::Ptr app2CarTagRoadNavigationMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarTagRoadNavigation>(app2CarMsgPtr);
  if (app2CarTagRoadNavigationMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  }

  YAML::Node roadNavTag(YAML::NodeType::Map);
  const json &content = app2CarTagRoadNavigationMsgPtr->content;

  RSApp2CarClipMateInfoManager::CLIP_KEY clipKey = {"", ""};
  RSApp2CarClipMateInfoTagRoadNavigationItem::Ptr itemPtr(
      new RSApp2CarClipMateInfoTagRoadNavigationItem());

  bool isMatchClipId = false;
  bool isMatchTaskName = false;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    if (key == "task_name") {
      isMatchTaskName = true;
      const std::string &value = content[key].template get<std::string>();
      clipKey.first = value;
    } else if (key == "clip_id") {
      isMatchClipId = true;
      const int value = content[key].template get<int32_t>();
      clipKey.second = std::to_string(value);
    } else if (key == "road_class") {
      const std::string &value = content[key].template get<std::string>();
      itemPtr->road_class = value;
      roadNavTag[key] = value;
    } else if (key == "road_type") {
      const std::string &value = content[key].template get<std::string>();
      itemPtr->road_type = value;
      roadNavTag[key] = value;
    }
  }
  uint64_t timestampNs = RS_TIMESTAMP_NS;
  itemPtr->systemTimestamp = timestampNs * 1e-9;

  if (isMatchTaskName && isMatchClipId && pSharedClipMetaInfoManager_) {
    // new version
    int ret =
        pSharedClipMetaInfoManager_->addClipMetaInfoItem(clipKey, itemPtr);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Add Clip Meta Info Item Failed: ret = "
                              << ret << ", clipKey = " << clipKey.first << ", "
                              << clipKey.second);
      return -2;
    }
  } else {
    if (!checkIsCollectRecord()) {
      RS_ERROR(pSharedNode_,
                   "Not Start Record, Not Support Update Road Nav Tag!");
      return -2;
    }

    // old version
    int ret = pSharedGlobalConfig_->updateMetaFileRoadNavTag(roadNavTag);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
          "Update Meta File Road Nav Tag Failed: ret = " << ret);
      return -3;
    }
  }

  return 0;
}

int RSCollectManager::app2CarCmdTagIconType(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarTagIconType::Ptr app2CarTagIconTypeMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarTagIconType>(app2CarMsgPtr);
  if (app2CarTagIconTypeMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  }

  YAML::Node iconTypeTag(YAML::NodeType::Map);
  const json &content = app2CarTagIconTypeMsgPtr->content;
  RSApp2CarClipMateInfoManager::CLIP_KEY clipKey = {"", ""};
  RSApp2CarClipMetaInfoTagIconTypeItem::Ptr itemPtr(
      new RSApp2CarClipMetaInfoTagIconTypeItem());

  bool isMatchClipId = false;
  bool isMatchTaskName = false;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    if (key == "task_name") {
      isMatchTaskName = true;
      const std::string &value = content[key].template get<std::string>();
      clipKey.first = value;
    } else if (key == "clip_id") {
      isMatchClipId = true;
      const int value = content[key].template get<int32_t>();
      clipKey.second = std::to_string(value);
    } else if (key == "icon_type") {
      const std::string &value = content[key].template get<std::string>();
      itemPtr->icon_type = value;
      iconTypeTag[key] = value;
    }
  }
  uint64_t timestampNs = RS_TIMESTAMP_NS;
  itemPtr->systemTimestamp = timestampNs * 1e-9;

  if (isMatchTaskName && isMatchClipId && pSharedClipMetaInfoManager_) {
    // new version
    int ret =
        pSharedClipMetaInfoManager_->addClipMetaInfoItem(clipKey, itemPtr);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Add Clip Meta Info Item Failed: ret = "
                              << ret << ", clipKey = " << clipKey.first << ", "
                              << clipKey.second);
      return -2;
    }
  } else {
    if (!checkIsCollectRecord()) {
      RS_ERROR(pSharedNode_,
                   "Not Start Record, Not Support Update Icon Type Tag!");
      return -2;
    }

    // old version
    int ret = pSharedGlobalConfig_->updateMetaFileIconTypeTag(iconTypeTag);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
          "Update Meta File Icon Type Tag Failed: ret = " << ret);
      return -3;
    }
  }

  return 0;
}

int RSCollectManager::app2CarCmdTagIntersectionType(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarTagIntersectionType::Ptr app2CarTagIntersectionTypeMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarTagIntersectionType>(app2CarMsgPtr);
  if (app2CarTagIntersectionTypeMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  }

  const json &content = app2CarTagIntersectionTypeMsgPtr->content;

  RSApp2CarClipMateInfoManager::CLIP_KEY clipKey = {"", ""};
  RSApp2CarClipMetaInfoTagIntersectionItem::Ptr itemPtr(
      new RSApp2CarClipMetaInfoTagIntersectionItem());
  const uint64_t timestampNs = RS_TIMESTAMP_NS;
  itemPtr->systemTimestamp = timestampNs * 1e-9;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    if (key == "task_name") {
      const std::string &value = content[key].template get<std::string>();
      clipKey.first = value;
    } else if (key == "clip_id") {
      const int value = content[key].template get<int32_t>();
      clipKey.second = std::to_string(value);
    } else if (key == "type") {
      const std::string &value = content[key].template get<std::string>();
      itemPtr->type = value;
    } else if (key == "timestamp") {
      const double value = content[key].template get<double>();
      itemPtr->timestamp = value;
    } else if (key == "gps_lon") {
      const double value = content[key].template get<double>();
      itemPtr->gps_lon = value;
    } else if (key == "gps_lat") {
      const double value = content[key].template get<double>();
      itemPtr->gps_lat = value;
    } else if (key == "amap_gps_lon") {
      const double value = content[key].template get<double>();
      itemPtr->amap_gps_lon = value;
    } else if (key == "amap_gps_lat") {
      const double value = content[key].template get<double>();
      itemPtr->amap_gps_lat = value;
    }
  }

  if (pSharedClipMetaInfoManager_ != nullptr) {
    int ret =
        pSharedClipMetaInfoManager_->addClipMetaInfoItem(clipKey, itemPtr);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Add Clip Meta Info Item Failed: ret = "
                              << ret << ", clipKey = " << clipKey.first << ", "
                              << clipKey.second);
      return -2;
    }
  } else {
    RS_ERROR(pSharedNode_, "pSharedClipMetaInfoManager_ is Nullptr !");
    return -3;
  }

  return 0;
}

int RSCollectManager::app2CarCmdUpdateFileTag(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarUpdateFileTag::Ptr app2CarUpdateFileTagMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarUpdateFileTag>(app2CarMsgPtr);
  if (app2CarUpdateFileTagMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  } else if (!checkIsCollectRecord()) {
    RS_ERROR(pSharedNode_, "Not Start Record, Not Support Update File Tag!");
    return -2;
  }

  const json &content = app2CarUpdateFileTagMsgPtr->content;

  const std::string &saveTaskName =
      content["task_name"].template get<std::string>();
  const std::string &saveClipId =
      std::to_string(content["clip_id"].template get<int32_t>());
  const std::string &saveRelFilePath =
      content["file_name"].template get<std::string>();

  // = 0 表示相对clip的路径, = 1 表示相对task的路径, = 2表示绝对路径
  int path_relative_type = -1;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    if (key == "path_relative_type") {
      path_relative_type = content[key].template get<int32_t>();
      break;
    }
  }

  // 不包含该字段时，按照为0处理以保持兼容性
  if (path_relative_type == -1) {
    RS_INFO(pSharedNode_,
        "Update File Tag Not Include \"path_relative_type\", use default "
        "is path_relative_type = 0");
    path_relative_type = 0;
  } else if (!(path_relative_type == 0 || path_relative_type == 1 ||
               path_relative_type == 2)) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Not Support path_relative_type = "
                            << path_relative_type
                            << ", saveRelFilePath = " << saveRelFilePath);
    return -3;
  }

  std::string saveFullFilePath;
  if (path_relative_type == 0) {
    // 相对clip的路径
    saveFullFilePath = pSharedGlobalConfig_->configPath + "/" + saveTaskName +
                       "/" + saveTaskName + "_" + saveClipId + "/" +
                       saveRelFilePath;
  } else if (path_relative_type == 1) {
    // 相对task的路径
    saveFullFilePath = pSharedGlobalConfig_->configPath + "/" + saveTaskName +
                       "/" + saveRelFilePath;
  } else if (path_relative_type == 2) {
    saveFullFilePath = saveRelFilePath;
  }
  RS_INFO_STREAM(pSharedNode_,
                     "saveFullFilePath = "
                         << saveFullFilePath
                         << ", saveRelFilePath = " << saveRelFilePath
                         << ", path_relative_type = " << path_relative_type);

  size_t lastSlashPos = saveFullFilePath.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    std::string saveFullDirPath = saveFullFilePath.substr(0, lastSlashPos);
    if (saveFullDirPath.size()) {
      if (saveFullDirPath[saveFullDirPath.size() - 1] != '/') {
        saveFullDirPath += "/";
      }
    }

    bool isSuccess = RSFileSystem::makePath(saveFullDirPath);
    if (!isSuccess) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Make Path Failed: " << saveFullDirPath);
      return -4;
    }
  } else {
    RS_ERROR_STREAM(pSharedNode_,
                        "Save Full File Path is Invalid: " << saveFullFilePath);
    return -5;
  }

  std::ofstream ofstr(saveFullFilePath,
                      std::ios_base::out | std::ios_base::binary);
  if (!ofstr.is_open()) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Open File To Write Failed: " << saveFullFilePath);
    return -6;
  }

  const std::string &base64FileData =
      content["file_content"].template get<std::string>();
  // const std::string& fileData =
  // QByteArray::fromBase64(QByteArray::fromRawData(base64FileData.data(),
  // base64FileData.size()));

  std::string fileData;
  int ret = RSBase64Util::decode(base64FileData, fileData);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Base64 Decode Failed: ret = " << ret);
    return -7;
  }

  ofstr << fileData;
  ofstr.flush();
  ofstr.close();

  return 0;
}

int RSCollectManager::app2CarCmdTagAudioTags(
    const RSApp2CarMessage::Ptr &app2CarMsgPtr) {
  RSApp2CarUpdateAudioTag::Ptr app2CarUpdateAudioTagMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarUpdateAudioTag>(app2CarMsgPtr);
  if (app2CarUpdateAudioTagMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_, "App2Car Message Dynamic is Nullptr !");
    return -1;
  }

  const json &content = app2CarUpdateAudioTagMsgPtr->content;
  RSApp2CarClipMateInfoManager::CLIP_KEY clipKey = {"", ""};
  RSApp2CarClipMateInfoTagAudioItem::Ptr itemPtr(
      new RSApp2CarClipMateInfoTagAudioItem());
  const uint64_t timestampNs = RS_TIMESTAMP_NS;
  itemPtr->systemTimestamp = timestampNs * 1e-9;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();
    if (key == "task_name") {
      const std::string &value = content[key].template get<std::string>();
      clipKey.first = value;
    } else if (key == "clip_id") {
      const int value = content[key].template get<int32_t>();
      clipKey.second = std::to_string(value);
    } else if (key == "name") {
      const std::string &value = content[key].template get<std::string>();
      itemPtr->name = value;
    } else if (key == "timestamp") {
      const double value = content[key].template get<double>();
      itemPtr->timestamp = value;
    }
  }

  if (pSharedClipMetaInfoManager_ != nullptr) {
    int ret =
        pSharedClipMetaInfoManager_->addClipMetaInfoItem(clipKey, itemPtr);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Add Clip Meta Info Item Failed: ret = "
                              << ret << ", clipKey = " << clipKey.first << ", "
                              << clipKey.second);
      return -2;
    }
  } else {
    RS_ERROR(pSharedNode_, "pSharedClipMetaInfoManager_ is Nullptr !");
    return -3;
  }

  return 0;
}

int RSCollectManager::app2CarCmdGetCollectSetting(
    const RSApp2CarMessage::Ptr &msgPtr, json &logger_json) {
  RSApp2CarGetCollectSetting::Ptr getCollectSettingMsgPtr =
      std::dynamic_pointer_cast<RSApp2CarGetCollectSetting>(msgPtr);
  if (getCollectSettingMsgPtr == nullptr) {
    RS_ERROR(pSharedNode_,
                 "Get Collect Setting Message Dynamic Convert Is Nullptr !");
    return -1;
  }

  logger_json = json();
  // int collect_seg_mode = 0;
  // if (pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .sub_segment_time_th_ns()) {
  //   collect_seg_mode += 1;
  // }
  // if (pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .sub_segment_chunck_time_th_ns()) {
  //   collect_seg_mode += 2;
  // }
  // pSharedNode_->get_logger()json["collect_file_seg_th_ns"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .sub_segment_time_th_ns();
  // pSharedNode_->get_logger()json["collect_chunk_seg_th_ns"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .sub_segment_chunck_time_th_ns();
  // pSharedNode_->get_logger()json["collect_seg_mode"] = collect_seg_mode;

  const std::string &default_rdcs_root_directory =
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
          .as<std::string>();
  logger_json["collect_save_directory"] = default_rdcs_root_directory;

  json candidate_dirs = json::array();
  // RCLCPP_ERROR(pSharedNode_->get_logger(), "run here: " <<
  // default_rdcs_root_directory;
  // candidate_dirs.push_back(default_rdcs_root_directory);

  std::vector<std::string> candidate_directories;
  char *usr_name = getlogin();
  if (usr_name != nullptr) {
    const std::string &search_path = "/media/" + std::string(usr_name);
    if (RSFileSystem::isDirectoryExist(search_path)) {
      if (!RSFileSystem::searchDirectoryWithoutFilter(search_path, false,
                                                      candidate_directories)) {
        RS_ERROR_STREAM(pSharedNode_,
                            "Search " << search_path
                                      << " Get Candidate Directories Failed !");
        return -2;
      }
    }
  }

  // 对所有可能的用户名检查一边: sti/rhino/nvidia/szcytek
  if (candidate_directories.empty()) {
    if (RSFileSystem::isDirectoryExist("/media/sti")) {
      if (!RSFileSystem::searchDirectoryWithoutFilter("/media/sti", false,
                                                      candidate_directories)) {
        RS_ERROR_STREAM(pSharedNode_,
                            "Search /media/sti Get Candidate Directories Failed !");
        return -2;
      } else {
        RS_INFO(pSharedNode_,
                    "Search /media/sti Get Candidate Directories Successed !");
      }
    }
  }

  if (candidate_directories.empty()) {
    if (RSFileSystem::isDirectoryExist("/media/rhino")) {
      if (!RSFileSystem::searchDirectoryWithoutFilter("/media/rhino", false,
                                                      candidate_directories)) {
        RS_ERROR_STREAM(pSharedNode_,
                            "Search /media/rhino Get Candidate Directories Failed !");
        return -2;
      } else {
        RS_INFO(pSharedNode_,
                "Search /media/rhino Get Candidate Directories Successed !");
      }
    }
  }

  if (candidate_directories.empty()) {
    if (RSFileSystem::isDirectoryExist("/media/nvidia")) {
      if (!RSFileSystem::searchDirectoryWithoutFilter("/media/nvidia", false,
                                                      candidate_directories)) {
        RS_ERROR_STREAM(pSharedNode_,
                            "Search /media/nvidia Get Candidate Directories Failed !");
        return -2;
      } else {
        RS_INFO(pSharedNode_,
                "Search /media/nvidia Get Candidate Directories Successed !");
      }
    }
  }

  if (candidate_directories.empty()) {
    if (RSFileSystem::isDirectoryExist("/media/szcytek")) {
      if (!RSFileSystem::searchDirectoryWithoutFilter("/media/szcytek", false,
                                                      candidate_directories)) {
        RS_ERROR_STREAM(pSharedNode_,
                            "Search /media/szcytek Get Candidate Directories Failed !");
        return -2;
      } else {
        RS_INFO(pSharedNode_,
                "Search /media/szcytek Get Candidate Directories Successed !");
      }
    }
  }

  const std::string &basic_default_rdcs_root_directory = "/apollo/RDCS_ROOT/";
  std::vector<std::string> candidate_directories2;
  if (default_rdcs_root_directory != basic_default_rdcs_root_directory) {
    candidate_directories2.push_back(basic_default_rdcs_root_directory);
  }
  candidate_directories2.push_back(
      default_rdcs_root_directory); // 第一个保持为默认路径
  for (size_t i = 0; i < candidate_directories.size(); ++i) {
    const std::string &candidate_directory = candidate_directories[i];
    const std::string &candidate_rdcs_root_directory =
        candidate_directory + "/RDCS_ROOT/";
    if (candidate_directory == basic_default_rdcs_root_directory ||
        candidate_directory == default_rdcs_root_directory) {
      continue;
    } else if (candidate_rdcs_root_directory ==
                   basic_default_rdcs_root_directory ||
               candidate_rdcs_root_directory == default_rdcs_root_directory) {
      continue;
    }
    candidate_directories2.push_back(candidate_directory);
  }
  candidate_directories = candidate_directories2;

  std::vector<double> candidate_save_directory_available_sizes;
  for (size_t i = 0; i < candidate_directories.size(); ++i) {
    // RCLCPP_ERROR(pSharedNode_->get_logger(), "RUN HERE: " <<
    // (candidate_directories[i] +
    // "/RDCS_ROOT/");
    const std::string &candidate_directory = candidate_directories[i];
    std::string ready_save_directory_path;
    if (candidate_directory == default_rdcs_root_directory) {
      ready_save_directory_path = default_rdcs_root_directory;
    } else if (candidate_directory == basic_default_rdcs_root_directory) {
      ready_save_directory_path = basic_default_rdcs_root_directory;
    } else {
      ready_save_directory_path = candidate_directory + "/RDCS_ROOT/";
    }

    bool is_already_exist =
        RSFileSystem::isDirectoryExist(ready_save_directory_path);

    if (is_already_exist) {
      candidate_dirs.push_back(ready_save_directory_path);
      RS_INFO_STREAM(pSharedNode_,
          "1 add candidate save directory = " << ready_save_directory_path);
    } else {
      bool isSuccess =
          RSFileSystem::systemCmd("mkdir -p " + ready_save_directory_path);
      if (!isSuccess) {
        RS_WARN_STREAM(pSharedNode_,
            "make candidate save directory = "
                << ready_save_directory_path
                << "failed, not add to candidate save directory !");
      } else {
        candidate_dirs.push_back(ready_save_directory_path);
#if 0 
        isSuccess =
            RSFileSystem::systemCmd("rm -rf " + ready_save_directory_path);
        if (!isSuccess) {
          RCLCPP_WARN(pSharedNode_->get_logger(), "remove candidate save directory = "
                << ready_save_directory_path << ", but not lead to error !");
        } else {
          RCLCPP_INFO(pSharedNode_->get_logger(), "remove check directory valid candidate save directory = "
                << ready_save_directory_path << " sucessed !");
        }
#endif
        RS_INFO_STREAM(pSharedNode_,
            "2 add candidate save directory = " << ready_save_directory_path);
      }
    }

    // 对于候选的文件夹路径:
    // 如果是basic_default_rdcs_root_directory则强制加入候选
    if (candidate_dirs.size() > 0) {
      if (candidate_dirs[candidate_dirs.size() - 1] ==
          ready_save_directory_path) {
        // 检查文件系统类型
        std::string matchFileSystemType;
        bool isMatchFileSystem = RSFileSystem::checkHDiskFileSystem(
            candidate_directory, RS_DEFAULT_SUPPORT_FILESYSTEM_TYPE,
            matchFileSystemType);
        if (!isMatchFileSystem) {
          if (ready_save_directory_path == basic_default_rdcs_root_directory) {
            RS_WARN_STREAM(pSharedNode_,
                "get candidate directory: "
                    << candidate_directory
                    << " filesystem type: " << matchFileSystemType
                    << ", is not match default support filesystem type: "
                    << RS_DEFAULT_SUPPORT_FILESYSTEM_TYPE
                    << ", but force add candidate directory !");
          } else {
            RS_WARN_STREAM(pSharedNode_,
                "get candidate directory: "
                    << candidate_directory
                    << " filesystem type: " << matchFileSystemType
                    << ", is not match default support filesystem type: "
                    << RS_DEFAULT_SUPPORT_FILESYSTEM_TYPE
                    << ", and remove from candidate directory !");
            candidate_dirs.erase(candidate_dirs.begin() +
                                 candidate_dirs.size() - 1);
            continue;
          }
        } else {
          RS_INFO_STREAM(pSharedNode_,
                             "get candidate directory: "
                                 << candidate_directory << " filesystem type: "
                                 << matchFileSystemType);
        }

        // 检查文件空间
        double total_size = 0;
        double avail_size = 0;
        double free_size = 0;
        bool isSuccess = RSFileSystem::getDirectorySpace(
            candidate_directory, total_size, avail_size, free_size);
        if (!isSuccess) {
          if (ready_save_directory_path == basic_default_rdcs_root_directory) {
            RS_WARN_STREAM(pSharedNode_,
                               "get candidate directory: "
                                   << candidate_directory
                                   << " space info failed, but force "
                                      "add candidate directory !");
            candidate_save_directory_available_sizes.push_back(0.0);
          } else {
            RS_WARN_STREAM(pSharedNode_,
                               "get candidate directory: "
                                   << candidate_directory
                                   << " space info failed, and remove "
                                      "from candidate directory !");
            candidate_dirs.erase(candidate_dirs.begin() +
                                 candidate_dirs.size() - 1);
            continue;
          }
        } else {
          candidate_save_directory_available_sizes.push_back(avail_size);
          RS_INFO_STREAM(pSharedNode_,
                             "candidate directory: "
                                 << candidate_directory
                                 << "space info: total_size = " << total_size
                                 << ", avail_size = " << avail_size
                                 << ", free_size = " << free_size);
        }
      }
    }
  }
  logger_json["candidate_save_direcories"] = candidate_dirs;
  logger_json["candidate_save_directory_available_sizes"] =
      candidate_save_directory_available_sizes;

  // pSharedNode_->get_logger()json["collect_copy_raw_message"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .enable_copy_raw_message();

  // pSharedNode_->get_logger()json["collect_single_message_buffer"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .enable_single_message_buffer();

  // pSharedNode_->get_logger()json["collect_single_message_buffer_size"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .single_message_buffer_size();

  // pSharedNode_->get_logger()json["collect_disk_mount_monitor"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .enable_collect_disk_mount_monitor();

  // pSharedNode_->get_logger()json["collect_disk_freespace_monitor"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .enable_collect_freespace_monitor();

  // pSharedNode_->get_logger()json["collect_disk_freespace_min_gbyte"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .collect_freespace_min_gbyte();

  // pSharedNode_->get_logger()json["collect_record_buffer"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"]
  //         .enable_record_buffer();

  // pSharedNode_->get_logger()json["collect_no_resplit"] =
  //     pSharedGlobalConfig_->recordConfig["recorddataconfig"].enable_no_resplit();

  return 0;
}

int RSCollectManager::app2CarCmdSetCollectSetting(
    const RSApp2CarMessage::Ptr &msgPtr) {
  RSApp2CarSetCollectSetting::Ptr setCollectSettingPtr =
      std::dynamic_pointer_cast<RSApp2CarSetCollectSetting>(msgPtr);
  if (msgPtr == nullptr) {
    RS_ERROR(pSharedNode_,
                 "Set Collect Setting Message Dynamic Convert is Nullptr !");
    return -1;
  }

  const json &content = setCollectSettingPtr->content;
  for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
    const std::string &key = iterMap.key();

    if (key == "collect_save_directory" || key == "collect_save_dirctory") {
      const std::string &dir = content[key].template get<std::string>();
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"] = dir;
      pSharedGlobalConfig_->configPath =
          pSharedGlobalConfig_
              ->recordConfig["recorddataconfig"]["rdcs_root_directory_path"]
              .as<std::string>();

      RS_INFO_STREAM(pSharedNode_,
                         "Set Collect Setting: collect_save_directory = "
                             << dir << ", configPath = "
                             << pSharedGlobalConfig_->configPath);
    } else if (key == "collect_disk_mount_monitor") {
      bool enable = content[key].template get<bool>();
      pSharedGlobalConfig_->recordConfig["recorddataconfig"]
                                        ["enable_collect_disk_mount_monitor"] =
          enable;
      RS_INFO_STREAM(pSharedNode_,
          "Set Collect Setting: collect_disk_mount_monitor = " << enable);
    } else if (key == "collect_disk_freespace_monitor") {
      bool enable = content[key].template get<bool>();
      pSharedGlobalConfig_->recordConfig["recorddataconfig"]
                                        ["enable_collect_freespace_monitor"] =
          enable;
      RS_INFO_STREAM(pSharedNode_,
          "Set Collect Setting: collect_disk_freespace_monitor = " << enable);
    } else if (key == "collect_disk_freespace_min_gbyte") {
      int32_t gbyte = content[key].template get<int32_t>();
      pSharedGlobalConfig_
          ->recordConfig["recorddataconfig"]["collect_freespace_min_gbyte"] =
          gbyte;
      RS_INFO_STREAM(pSharedNode_,
          "Set Collect Setting: collect_disk_freespace_min_gbyte = " << gbyte);
    }
  }

  return 0;
}

int RSCollectManager::app2CarCmdGetCollectStatus(
    const RSApp2CarMessage::Ptr &msgPtr, json &logger_json) {
  (void)(msgPtr);

  // 采集状态
  std::string collect_status;
  if (pSharedGlobalConfig_->isRecord && isPlayControlMode_) {
    collect_status = "start_collect";
  } else if (!pSharedGlobalConfig_->isRecord && isPlayControlMode_) {
    collect_status = "sensor_open";
  } else {
    collect_status = "sensor_close";
  }

  const std::string &taskName = pSharedGlobalConfig_->taskName;
  const int clipId = pSharedGlobalConfig_->clipId;

  std::string vehicleId;
  int ret = getLocalVehicleId(vehicleId);
  if (ret != 0) {
    RS_ERROR_STREAM(pSharedNode_,
                        "Get Local Vehicle Id Failed: ret = " << ret);
    return -1;
  }

  logger_json["collect_status"] = collect_status;
  logger_json["task_name"] = taskName;
  logger_json["clip_id"] = clipId;
  logger_json["vehicle_id"] = vehicleId;
  logger_json["max_cmd_id"] = pSharedAppCarManager_->getMaxCheckRepeatCmdId();

  return 0;
}

int RSCollectManager::writeTaskNameJson() {
  const std::string &taskNameJsonFilePath =
      pSharedGlobalConfig_->getTaskNameJsonFilePath();

  std::ofstream ofstr(taskNameJsonFilePath, std::ios_base::out |
                                                std::ios_base::binary |
                                                std::ios_base::trunc);
  if (ofstr.is_open() == false) {
    RS_WARN_STREAM(pSharedNode_,
                       "Open taskNameJsonFilePath: " << taskNameJsonFilePath
                                                     << " To Write Failed !");
    return -1;
  }

  // Json::FastWriter writer;
  std::string json_string = pSharedGlobalConfig_->taskJsonValue.dump();
  ofstr << json_string << std::endl;

  return 0;
}

int RSCollectManager::updateTaskNameJsonValue(
    const RSApp2CarMessage::Ptr &msgPtr, const bool isUpdateClipId) {
  pSharedGlobalConfig_->isTaskNameValid = true;
  pSharedGlobalConfig_->taskName = msgPtr->task_name;
  pSharedGlobalConfig_->taskJsonValue = msgPtr->content;
  if (isUpdateClipId) {
    if (msgPtr->content.find("clip_id") != msgPtr->content.end()) {
      pSharedGlobalConfig_->app2CarClipId =
          msgPtr->content["clip_id"].template get<int32_t>();
    } else {
      pSharedGlobalConfig_->app2CarClipId = -1;
    }
    RS_INFO_STREAM(pSharedNode_,
                       "pSharedGlobalConfig_->app2CarClipId = "
                           << pSharedGlobalConfig_->app2CarClipId);
  }
  RS_INFO_STREAM(pSharedNode_,
                     "pSharedGlobalConfig_->app2CarClipId = "
                         << pSharedGlobalConfig_->app2CarClipId);

  // std::cout << "content = " << msgPtr->content.dump() << std::endl;
  pSharedGlobalConfig_->systemConfig.vehicleInfo =
      msgPtr->content["real_vehicle_id"].template get<std::string>();

  return 0;
}

int RSCollectManager::checkCarAppVehicleIdWithLocal() {
  if (pSharedGlobalConfig_->isTaskNameValid) {
    std::string vehicleId;
    int ret = getLocalVehicleId(vehicleId);
    if (ret != 0) {
      RS_ERROR_STREAM(pSharedNode_,
                          "Get Local Vehicle Id Failed: ret = " << ret);
      return -1;
    }

    if (pSharedGlobalConfig_->systemConfig.vehicleInfo == vehicleId) {
      return 0;
    } else {
      RS_ERROR_STREAM(pSharedNode_,
          "Local Vehicle Id != CarApp VehicleId: Local VehicleId = "
              << vehicleId << ", CarApp VehicleId = "
              << pSharedGlobalConfig_->systemConfig.vehicleInfo);
      return -2;
    }
  }

  return 0;
}

int RSCollectManager::getLocalVehicleId(std::string &vehicleId) {
  vehicleId.clear();
  if (RSFileSystem::isFileExist(RS_DEFAULT_CALIBRATION_FILE_PATH)) {
    YAML::Node localNode;
    try {
      localNode = YAML::LoadFile(RS_DEFAULT_CALIBRATION_FILE_PATH);
    } catch (const std::exception &e) {
      RS_WARN_STREAM(pSharedNode_,
                         "Local Meta YAML::Node Exist But Load Failed: "
                             << RS_DEFAULT_CALIBRATION_FILE_PATH);
      return -1;
    }
    vehicleId = localNode["vehicle_id"].as<std::string>();
    RS_INFO_STREAM(pSharedNode_,
                       "Local Meta YAML::Node Vehicle_id = " << vehicleId);
  } else {
    RS_WARN_STREAM(pSharedNode_,
                       "Local Meta YAML::Node Not Exist: "
                           << RS_DEFAULT_CALIBRATION_FILE_PATH);
    return -2;
  }

  return 0;
}

} // namespace collect
} // namespace rs_collect
} // namespace robosense
