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
#include "modules/rs_collect/rsappcarmanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

//
// RSCarAppMessage
//

RSCarAppMessage::RSCarAppMessage() { reset(); }

RSCarAppMessage::~RSCarAppMessage() {
  // NOTHING TODO...
}

void RSCarAppMessage::reset() {
  // NOTHING TODO...
}

std::string RSCarAppMessage::toJsonString(const json &content) const {
  return content.dump();
}

int RSCarAppMessage::fromJsonString(const std::string &json_content,
                                    json &jsonValue) {
  jsonValue = json::parse(json_content);

  return (jsonValue.is_null() ? -1 : 0);
}

//
// RSCar2AppMessage
//

RSCar2AppMessage::RSCar2AppMessage() { reset(); }

RSCar2AppMessage::~RSCar2AppMessage() {
  // NOTHING TODO...
}

void RSCar2AppMessage::reset() {
  RSCarAppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UNKNOWN;
  logger_id = -1;
  lasest_received_cmd_id_from_app = -1;
  logger_content.clear();
  content = json();
}

int RSCar2AppMessage::fromProtoMessage(const std_msgs_String &proto_msg) {
  reset();

  json jsonValue;
  int ret = fromJsonString(proto_msg.data, jsonValue);
  if (ret != 0) {
    return -1;
  }

  logger_type = static_cast<RS_CAR_2_APP_MESSAGE_TYPE>(
      jsonValue["cmd_type"].template get<int>());
  logger_id = jsonValue["cmd_id"].template get<uint32_t>();
  lasest_received_cmd_id_from_app =
      jsonValue["lasest_received_cmd_id_from_app"].template get<uint32_t>();
  logger_content = jsonValue["logger_content"].template get<std::string>();

  if (!logger_content.empty()) {
    ret = fromJsonString(logger_content, content);
    if (ret != 0) {
      return -2;
    }
  }

  return 0;
}

int RSCar2AppMessage::toProtoMessage(std_msgs_String &proto_msg) {
  json jsonValue;
  jsonValue["logger_type"] = static_cast<int>(logger_type);
  jsonValue["logger_id"] = logger_id;
  jsonValue["lasest_received_cmd_id_from_app"] =
      lasest_received_cmd_id_from_app;

  logger_content = toJsonString(content);
  jsonValue["logger_content"] = logger_content;

  proto_msg.data = toJsonString(jsonValue);

  return 0;
}

bool RSCar2AppMessage::isNothingTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING);
}

bool RSCar2AppMessage::isInfoTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO);
}

bool RSCar2AppMessage::isHintTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_HINT);
}

bool RSCar2AppMessage::isWarnTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_WARN);
}

bool RSCar2AppMessage::isFetalTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL);
}

bool RSCar2AppMessage::isResponseTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE);
}

bool RSCar2AppMessage::isUpdateClipIdTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID);
}

bool RSCar2AppMessage::isCollectVersionTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COLLECT_VERSION);
}

bool RSCar2AppMessage::isCollectSoftwareUpdateProgressTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::
              RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS);
}

bool RSCar2AppMessage::isCyberPostProcessTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_POSTPROCESS);
}

bool RSCar2AppMessage::isCyberCombineProgressTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE_PROGRESS);
}

bool RSCar2AppMessage::isCyberResplitProgressTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT_PROGRESS);
}

bool RSCar2AppMessage::isCyberCheckProgressTypeMessage() const {
  return (logger_type ==
          RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECK_PROGRESS);
}

bool RSCar2AppMessage::isAckTypeMessage() const {
  return (logger_type == RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK);
}

//
// RSCar2AppNothingMessage
//
RSCar2AppNothingMessage::RSCar2AppNothingMessage() { reset(); }

RSCar2AppNothingMessage::~RSCar2AppNothingMessage() {
  // NOTHING TODO...
}

void RSCar2AppNothingMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING;
}

//
// RSCar2AppNothingMessage
//
RSCar2AppInfoMessage::RSCar2AppInfoMessage() { reset(); }

RSCar2AppInfoMessage::~RSCar2AppInfoMessage() {
  // NOTHING TODO...
}

void RSCar2AppInfoMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO;
}

//
// RSCar2AppHintMessage
//
RSCar2AppHintMessage::RSCar2AppHintMessage() { reset(); }

RSCar2AppHintMessage::~RSCar2AppHintMessage() {
  // NOTHING TODO...
}

void RSCar2AppHintMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_HINT;
}

//
// RSCar2AppWarnMessage
//
RSCar2AppWarnMessage::RSCar2AppWarnMessage() { reset(); }

RSCar2AppWarnMessage::~RSCar2AppWarnMessage() {
  // NOTHING TODO...
}

void RSCar2AppWarnMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_WARN;
}

//
// RSCar2AppFetalMessage
//
RSCar2AppFetalMessage::RSCar2AppFetalMessage() { reset(); }

RSCar2AppFetalMessage::~RSCar2AppFetalMessage() {
  // NOTHING TODO...
}

void RSCar2AppFetalMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL;
}

//
// RSCar2AppResponseMessage
//
RSCar2AppResponseMessage::RSCar2AppResponseMessage() { reset(); }

RSCar2AppResponseMessage::~RSCar2AppResponseMessage() {
  // NOTHING TODO...
}

void RSCar2AppResponseMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE;
}

//
// RSCar2AppUpdateClipIdMessage
//
RSCar2AppUpdateClipIdMessage::RSCar2AppUpdateClipIdMessage() {
  RSCar2AppUpdateClipIdMessage::reset();
}

RSCar2AppUpdateClipIdMessage::~RSCar2AppUpdateClipIdMessage() {
  // NOTHING TODO...
}

void RSCar2AppUpdateClipIdMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID;
}

//
// RSCar2AppCollectVersionMessage
//
RSCar2AppCollectVersionMessage::RSCar2AppCollectVersionMessage() {
  RSCar2AppCollectVersionMessage::reset();
}

RSCar2AppCollectVersionMessage::~RSCar2AppCollectVersionMessage() {
  // NOTHING TODO...
}

void RSCar2AppCollectVersionMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COLLECT_VERSION;
}

//
// RSCar2AppCollectSoftwareUpdateProgressMessage
//
RSCar2AppCollectSoftwareUpdateProgressMessage::
    RSCar2AppCollectSoftwareUpdateProgressMessage() {
  RSCar2AppCollectSoftwareUpdateProgressMessage::reset();
}

RSCar2AppCollectSoftwareUpdateProgressMessage::
    ~RSCar2AppCollectSoftwareUpdateProgressMessage() {
  // NOTHING TODO...
}

void RSCar2AppCollectSoftwareUpdateProgressMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::
      RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS;
}

//
// RSCar2AppCyberPostProcessMessage
//
RSCar2AppCyberPostProcessMessage::RSCar2AppCyberPostProcessMessage() {
  RSCar2AppCyberPostProcessMessage::reset();
}

RSCar2AppCyberPostProcessMessage::~RSCar2AppCyberPostProcessMessage() {
  // NOTHING TODO...
}

void RSCar2AppCyberPostProcessMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_POSTPROCESS;
}

//
// RSCar2AppCyberCombineProgressMessage
//
RSCar2AppCyberCombineProgressMessage::RSCar2AppCyberCombineProgressMessage() {
  RSCar2AppCyberCombineProgressMessage::reset();
}

RSCar2AppCyberCombineProgressMessage::~RSCar2AppCyberCombineProgressMessage() {
  // NOTHING TODO...
}

void RSCar2AppCyberCombineProgressMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type =
      RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE_PROGRESS;
}

//
// RSCar2AppCyberResplitProgressMessage
//
RSCar2AppCyberResplitProgressMessage::RSCar2AppCyberResplitProgressMessage() {
  RSCar2AppCyberResplitProgressMessage::reset();
}

RSCar2AppCyberResplitProgressMessage::~RSCar2AppCyberResplitProgressMessage() {
  // NOTHING TODO...
}

void RSCar2AppCyberResplitProgressMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type =
      RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT_PROGRESS;
}

//
// RSCar2AppCyberCheckProgressMessage
//
RSCar2AppCyberCheckProgressMessage::RSCar2AppCyberCheckProgressMessage() {
  RSCar2AppCyberCheckProgressMessage::reset();
}

RSCar2AppCyberCheckProgressMessage::~RSCar2AppCyberCheckProgressMessage() {
  // NOTHING TODO...
}

void RSCar2AppCyberCheckProgressMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type =
      RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECK_PROGRESS;
}

//
// RSCar2AppAckMessage
//
RSCar2AppAckMessage::RSCar2AppAckMessage() { RSCar2AppAckMessage::reset(); }

RSCar2AppAckMessage::~RSCar2AppAckMessage() {
  // NOTHING TODO...
}

void RSCar2AppAckMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK;
}

//
// RSCar2AppCopyLogProgressMessage
//
RSCar2AppCopyLogProgressMessage::RSCar2AppCopyLogProgressMessage() {
  RSCar2AppCopyLogProgressMessage::reset();
}

RSCar2AppCopyLogProgressMessage::~RSCar2AppCopyLogProgressMessage() {
  // NOTHING TODO...
}

void RSCar2AppCopyLogProgressMessage::reset() {
  RSCar2AppMessage::reset();
  logger_type = RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COPY_LOG_PROGRESS;
}

//
// RSApp2CarMessage
//
RSApp2CarMessage::RSApp2CarMessage() : RSCarAppMessage() { reset(); }

RSApp2CarMessage::~RSApp2CarMessage() {
  // NOTHING TODO...
}

void RSApp2CarMessage::reset() {
  RSCarAppMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UNKNOWN;
  cmd_id = -1;
  task_name.clear();
  lasest_received_logger_id_from_car = -1;
  device_id.clear();
  json_content.clear();

  content = json();
}

int RSApp2CarMessage::fromProtoMessage(const std_msgs_String &proto_msg) {
  reset();

  json jsonValue;
  int ret = fromJsonString(proto_msg.data, jsonValue);
  if (ret != 0) {
    return -1;
  }

  cmd_type = static_cast<RS_APP_2_CAR_MESSAGE_TYPE>(
      jsonValue["cmd_type"].template get<uint32_t>());
  cmd_id = jsonValue["cmd_id"].template get<uint32_t>();
  task_name = jsonValue["task_name"].template get<std::string>();
  lasest_received_logger_id_from_car =
      jsonValue["lasest_received_logger_id_from_car"].template get<uint32_t>();
  json_content = jsonValue["json_content"].template get<std::string>();

  if (jsonValue.contains("device_id")) {
    device_id = jsonValue["device_id"].template get<std::string>();
  } else {
    device_id.clear();
  }

  if (!json_content.empty()) {
    ret = fromJsonString(json_content, content);
    if (ret != 0) {
      return -2;
    }
  }

  return 0;
}

int RSApp2CarMessage::toProtoMessage(std_msgs_String &proto_msg) {
  json jsonValue;
  jsonValue["cmd_type"] = static_cast<int>(cmd_type);
  jsonValue["cmd_id"] = cmd_id;
  jsonValue["task_name"] = task_name;
  jsonValue["device_id"] = device_id;
  jsonValue["lasest_received_logger_id_from_car"] =
      lasest_received_logger_id_from_car;

  json_content = toJsonString(content);
  jsonValue["json_content"] = json_content;

  proto_msg.data = toJsonString(jsonValue);

  return 0;
}

bool RSApp2CarMessage::isCmdTypeNothing() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING;
}

bool RSApp2CarMessage::isCmdTypeSelfCheck() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SELFCHECK;
}

bool RSApp2CarMessage::isCmdTypeStartCollection() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STARTCOLLECTION;
}

bool RSApp2CarMessage::isCmdTypePauseCollection() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_PAUSECOLLECTION;
}

bool RSApp2CarMessage::isCmdTypeEndCollection() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ENDCOLLECTION;
}

bool RSApp2CarMessage::isCmdTypeOnlyUpdateJson() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON;
}

bool RSApp2CarMessage::isCmdTypeUpdateTag() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_TAG;
}

bool RSApp2CarMessage::isCmdTypeUpdateCalibrationFile() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE;
}

bool RSApp2CarMessage::isCmdTypeStopCollectVersion() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STOP_COLLECT_VERSION;
}

bool RSApp2CarMessage::isCmdTypeUpdateCollectSoftware() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE;
}

bool RSApp2CarMessage::isCmdTypeTagRoadNavigation() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION;
}

bool RSApp2CarMessage::isCmdTypeTagIconType() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ICON_TYPE;
}

bool RSApp2CarMessage::isCmdTypeTagIntersectionType() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE;
}

bool RSApp2CarMessage::isCmdTypeUpdateFileTag() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG;
}

bool RSApp2CarMessage::isCmdTypeUpdateAudioTag() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG;
}

bool RSApp2CarMessage::isCmdTypeCyberChecker() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECKER;
}

bool RSApp2CarMessage::isCmdTypeCyberCombine() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE;
}

bool RSApp2CarMessage::isCmdTypeCyberResplit() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT;
}

bool RSApp2CarMessage::isCmdTypeAck() const {
  return this->cmd_type == RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK;
}

bool RSApp2CarMessage::isCmdTypeOrinKillNodes() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_NODES;
}

bool RSApp2CarMessage::isCmdTypeSetCollectSetting() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING;
}

bool RSApp2CarMessage::isCmdTypeGetCollectSetting() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING;
}

bool RSApp2CarMessage::isCmdTypeGetCollectStatus() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS;
}

bool RSApp2CarMessage::isCmdTypeOrinCopyLog() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_COPY_LOG;
}

bool RSApp2CarMessage::isCmdTypeOrinKillHmiNode() const {
  return this->cmd_type ==
         RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_HMI_NODE;
}

int RSApp2CarMessage::getTaskName(std::string &taskName) const {
  if (isCmdTypeNothing()) {
    return -1;
  }

  taskName = std::to_string(content["task_seq"].template get<int32_t>()) + "_" +
             std::to_string(content["task_pool_id"].template get<int32_t>()) +
             "_" + content["vehicle_name"].template get<std::string>() + "_" +
             content["task_proposer"].template get<std::string>() + "_" +
             content["task_executer"].template get<std::string>() + "_" +
             content["task_expected_start_time"].template get<std::string>();

  return 0;
}

///
/// ====================================== ///
///
RSApp2CarNothingMessage::RSApp2CarNothingMessage() : RSApp2CarMessage() {
  RSApp2CarNothingMessage::reset();
}

RSApp2CarNothingMessage::~RSApp2CarNothingMessage() {
  // NOTHING TODO...
}

void RSApp2CarNothingMessage::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING;
}

///
/// ====================================== ///
///
RSApp2CarSelfCheck::RSApp2CarSelfCheck() : RSApp2CarMessage() {
  RSApp2CarSelfCheck::reset();
}

RSApp2CarSelfCheck::~RSApp2CarSelfCheck() {
  // NOTHING TODO...
}

void RSApp2CarSelfCheck::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SELFCHECK;
}

///
/// ====================================== ///
///
RSApp2CarStartCollection::RSApp2CarStartCollection() : RSApp2CarMessage() {
  RSApp2CarStartCollection::reset();
}

RSApp2CarStartCollection::~RSApp2CarStartCollection() {
  // NOTHING TODO...
}

void RSApp2CarStartCollection::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STARTCOLLECTION;
}

///
/// ====================================== ///
///
RSApp2CarPauseCollection::RSApp2CarPauseCollection() : RSApp2CarMessage() {
  RSApp2CarPauseCollection::reset();
}

RSApp2CarPauseCollection::~RSApp2CarPauseCollection() {
  // NOTHING TODO...
}

void RSApp2CarPauseCollection::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_PAUSECOLLECTION;
}

///
/// ====================================== ///
///
RSApp2CarEndCollection::RSApp2CarEndCollection() : RSApp2CarMessage() {
  RSApp2CarEndCollection::reset();
}

RSApp2CarEndCollection::~RSApp2CarEndCollection() {
  // NOTHING TODO...
}

void RSApp2CarEndCollection::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ENDCOLLECTION;
}

///
/// ====================================== ///
///
RSApp2CarOnlyUpdateJson::RSApp2CarOnlyUpdateJson() : RSApp2CarMessage() {
  RSApp2CarOnlyUpdateJson::reset();
}

RSApp2CarOnlyUpdateJson::~RSApp2CarOnlyUpdateJson() {
  // NOTHING TODO...
}

void RSApp2CarOnlyUpdateJson::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON;
}

///
/// ======================================= ///
///
RSApp2CarUpdateTag::RSApp2CarUpdateTag() { RSApp2CarUpdateTag::reset(); }

RSApp2CarUpdateTag::~RSApp2CarUpdateTag() {
  // NOTHING TODO...
}

void RSApp2CarUpdateTag::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_TAG;
}

///
/// ======================================= ///
///
RSApp2CarUpdateCalibrationFile::RSApp2CarUpdateCalibrationFile() {
  RSApp2CarUpdateCalibrationFile::reset();
}

RSApp2CarUpdateCalibrationFile::~RSApp2CarUpdateCalibrationFile() {
  // NOTHING TODO...
}

void RSApp2CarUpdateCalibrationFile::reset() {
  RSApp2CarMessage::reset();
  cmd_type =
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE;
}

///
/// ======================================= ///
///
RSApp2CarStopCollectVersion::RSApp2CarStopCollectVersion() {
  RSApp2CarStopCollectVersion::reset();
}

RSApp2CarStopCollectVersion::~RSApp2CarStopCollectVersion() {
  // NOTHING TODO...
}

void RSApp2CarStopCollectVersion::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STOP_COLLECT_VERSION;
}

///
/// ======================================= ///
///
RSApp2CarUpdateCollectSoftware::RSApp2CarUpdateCollectSoftware() {
  RSApp2CarUpdateCollectSoftware::reset();
}

RSApp2CarUpdateCollectSoftware::~RSApp2CarUpdateCollectSoftware() {
  // NOTHING TODO...
}

void RSApp2CarUpdateCollectSoftware::reset() {
  RSApp2CarMessage::reset();
  cmd_type =
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE;
}

///
/// ======================================= ///
///
RSApp2CarTagRoadNavigation::RSApp2CarTagRoadNavigation() {
  RSApp2CarTagRoadNavigation::reset();
}

RSApp2CarTagRoadNavigation::~RSApp2CarTagRoadNavigation() {
  // NOTHING TODO...
}

void RSApp2CarTagRoadNavigation::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION;
}

///
/// ======================================= ///
///
RSApp2CarTagIconType::RSApp2CarTagIconType() { RSApp2CarTagIconType::reset(); }

RSApp2CarTagIconType::~RSApp2CarTagIconType() {
  // NOTHING TODO...
}

void RSApp2CarTagIconType::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ICON_TYPE;
}

///
/// ======================================= ///
///
///
RSApp2CarTagIntersectionType::RSApp2CarTagIntersectionType() {
  RSApp2CarTagIntersectionType::reset();
}

RSApp2CarTagIntersectionType::~RSApp2CarTagIntersectionType() {
  // NOTHING TODO...
}

void RSApp2CarTagIntersectionType::reset() {
  RSApp2CarMessage::reset();
  cmd_type =
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE;
}

///
/// ======================================= ///
///
RSApp2CarUpdateFileTag::RSApp2CarUpdateFileTag() {
  RSApp2CarUpdateFileTag::reset();
}

RSApp2CarUpdateFileTag::~RSApp2CarUpdateFileTag() {
  // NOTHING TODO...
}

void RSApp2CarUpdateFileTag::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG;
}

///
/// ======================================= ///
///
RSApp2CarUpdateAudioTag::RSApp2CarUpdateAudioTag() {
  RSApp2CarUpdateAudioTag::reset();
}

RSApp2CarUpdateAudioTag::~RSApp2CarUpdateAudioTag() {
  // NOTHING TODO...
}

void RSApp2CarUpdateAudioTag::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG;
}

///
/// ======================================= ///
///
RSApp2CarCyberChecker::RSApp2CarCyberChecker() {
  RSApp2CarCyberChecker::reset();
}

RSApp2CarCyberChecker::~RSApp2CarCyberChecker() {
  // NOTHING TODO...
}

void RSApp2CarCyberChecker::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECKER;
}

///
/// ======================================= ///
///
RSApp2CarCyberCombine::RSApp2CarCyberCombine() {
  RSApp2CarCyberCombine::reset();
}

RSApp2CarCyberCombine::~RSApp2CarCyberCombine() {
  // NOTHING TODO...
}

void RSApp2CarCyberCombine::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE;
}

///
/// ======================================= ///
///
RSApp2CarCyberResplit::RSApp2CarCyberResplit() {
  RSApp2CarCyberResplit::reset();
}

RSApp2CarCyberResplit::~RSApp2CarCyberResplit() {
  // NOTHING TODO...
}

void RSApp2CarCyberResplit::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT;
}

///
/// ======================================= ///
///
RSApp2CarAck::RSApp2CarAck() { RSApp2CarAck::reset(); }

RSApp2CarAck::~RSApp2CarAck() {
  // NOTHING TODO...
}

void RSApp2CarAck::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK;
}

///
/// ======================================= ///
///
RSApp2CarOrinKillNodes::RSApp2CarOrinKillNodes() {
  RSApp2CarOrinKillNodes::reset();
}

RSApp2CarOrinKillNodes::~RSApp2CarOrinKillNodes() {
  // NOTHING TODO...
}

void RSApp2CarOrinKillNodes::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_NODES;
}

///
/// ======================================= ///
///
RSApp2CarSetCollectSetting::RSApp2CarSetCollectSetting() {
  RSApp2CarSetCollectSetting::reset();
}

RSApp2CarSetCollectSetting::~RSApp2CarSetCollectSetting() {
  // NOTHING TODO...
}

void RSApp2CarSetCollectSetting::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING;
}

///
/// ==================================== ///
///
RSApp2CarGetCollectSetting::RSApp2CarGetCollectSetting() {
  RSApp2CarGetCollectSetting::reset();
}

RSApp2CarGetCollectSetting::~RSApp2CarGetCollectSetting() {
  // NOTHING TODO...
}

void RSApp2CarGetCollectSetting::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING;
}

///
/// ==================================== ///
///
RSApp2CarGetCollectStatus::RSApp2CarGetCollectStatus() {
  RSApp2CarGetCollectStatus::reset();
}

RSApp2CarGetCollectStatus::~RSApp2CarGetCollectStatus() {
  // NOTHING TODO...
}

void RSApp2CarGetCollectStatus::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS;
}

///
/// ==================================== ///
///
RSApp2CarOrinCopyLog::RSApp2CarOrinCopyLog() { RSApp2CarOrinCopyLog::reset(); }

RSApp2CarOrinCopyLog::~RSApp2CarOrinCopyLog() {
  // NOTHING TODO...
}

void RSApp2CarOrinCopyLog::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_COPY_LOG;
}

///
/// ==================================== ///
///
RSApp2CarOrinKillHmiNode::RSApp2CarOrinKillHmiNode() {
  RSApp2CarOrinKillHmiNode::reset();
}

RSApp2CarOrinKillHmiNode::~RSApp2CarOrinKillHmiNode() {
  // NOTHING TODO...
}

void RSApp2CarOrinKillHmiNode::reset() {
  RSApp2CarMessage::reset();
  cmd_type = RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_HMI_NODE;
}

///
/// ======================================= ///
///
/// ====================================== ///
///
RSAppCarManager::RSAppCarManager() {
  m_isAppIsStart = false;
  m_isSendCollectSoftwareVersion = true;
}

RSAppCarManager::~RSAppCarManager() {
  if (m_isExit == false) {
    m_isExit = true;
    m_msgRecvCond.notify_all();
    m_msgSndCond.notify_all();
  }

  if (m_msgRecvThreadPtr != nullptr) {
    if (m_msgRecvThreadPtr->joinable()) {
      m_msgRecvThreadPtr->join();
    }
  }

  if (m_msgSndThreadPtr != nullptr) {
    if (m_msgSndThreadPtr->joinable()) {
      m_msgSndThreadPtr->join();
    }
  }

  if (m_timeoutThreadPtr != nullptr) {
    if (m_timeoutThreadPtr->joinable()) {
      m_timeoutThreadPtr->join();
    }
  }

  if (m_car2AppHeartBeatThreadPtr != nullptr) {
    if (m_car2AppHeartBeatThreadPtr->joinable()) {
      m_car2AppHeartBeatThreadPtr->join();
    }
  }

  if (m_softwareVersionThreadPtr != nullptr) {
    if (m_softwareVersionThreadPtr->joinable()) {
      m_softwareVersionThreadPtr->join();
    }
  }

  if (m_sendAckWaitThreadPtr != nullptr) {
    if (m_sendAckWaitThreadPtr->joinable()) {
      m_sendAckWaitThreadPtr->join();
    }
  }
}

int RSAppCarManager::initManager(const NodeHandlePtr &pNode,
                                 const RSApp2CarConfig &app2CarConfig,
                                 // const std::string& vehicleId,
                                 const RS_RECV_ROSMESSAGE_CALLBACK &callback) {
  if (pNode == nullptr) {
    return -1;
  }
  m_pSharedNodeHandle = pNode;

  if (callback == nullptr) {
    RS_ERROR(m_pSharedNodeHandle, "callback is Nullptr !");
    return -2;
  }

  m_app2CarConfig = app2CarConfig;
  m_rosMessageCallback = callback;
  m_isAppIsStart = false;
  // m_vehicleId = vehicleId;

  return initManager();
}

int RSAppCarManager::addSendMessage(
    const RSCar2AppMessage::Ptr &appCarMessagePtr) {
  if (m_isExit == true) {
    return 0;
  } else if (appCarMessagePtr == nullptr) {
    return 0;
  }

  {
    std::lock_guard<std::mutex> lg(m_msgSndMtx);
    m_msgSndBuffers.push(appCarMessagePtr);
    m_msgSndCond.notify_all();
  }

  return 0;
}

uint32_t RSAppCarManager::getMaxCheckRepeatCmdId() { return m_maxApp2CarCmdId; }

int RSAppCarManager::initManager() {
  try {
    m_isExit = false;
    m_msgRecvThreadPtr.reset(
        new std::thread(&RSAppCarManager::recvWorkThread, this));
    m_msgSndThreadPtr.reset(
        new std::thread(&RSAppCarManager::sndWorkThread, this));
    m_timeoutThreadPtr.reset(
        new std::thread(&RSAppCarManager::timeoutCheckThread, this));
    m_sendAckWaitThreadPtr.reset(
        new std::thread(&RSAppCarManager::ackCheckThread, this));
  } catch (...) {
    m_isExit = true;
    RS_ERROR(m_pSharedNodeHandle,
                 "Create Recv/Snd/TimeoutCheck/AckCheck Thread Failed !");
    return -1;
  }

  m_msgLastestRecvId = -1;
  m_msgLastestRecvTimestampNs = -1;
  m_msgLastestSndId = -1;

#if __ROS2__
  m_cyberrtApp2CarPub =
      m_pSharedNodeHandle->create_publisher<std_msgs_String>(
          RS_CAR_2_APP_TOPICNAME, 10);
  if (m_cyberrtApp2CarPub == nullptr) {
    RS_ERROR_STREAM(
        m_pSharedNodeHandle,
        "Create Cyber Writer Failed: channelName = " << RS_CAR_2_APP_TOPICNAME);
    return -2;
  }
#elif __ROS1__
  m_cyberrtApp2CarPub =
      m_pSharedNodeHandle->advertise<std_msgs_String>(
          RS_CAR_2_APP_TOPICNAME, 10);
#endif

  {
#if __ROS2__
    m_cyberrtApp2CarSub =
        m_pSharedNodeHandle->create_subscription<std_msgs_String>(
            RS_APP_2_CAR_TOPICNAME, 20,
            std::bind(&RSAppCarManager::recvMessageCallback, this,
                      std::placeholders::_1));
    if (m_cyberrtApp2CarSub == nullptr) {
      RS_ERROR_STREAM(m_pSharedNodeHandle,
                          "Create Cyber Reader Failed: channelName = "
                              << RS_APP_2_CAR_TOPICNAME);
      return -3;
    }
#elif __ROS1__
  m_cyberrtApp2CarSub =
      m_pSharedNodeHandle->subscribe(RS_APP_2_CAR_TOPICNAME, 20,
                                     &RSAppCarManager::recvMessageCallback, this);
#endif
  }

  try {
    m_isSendCollectSoftwareVersion = true;
    m_car2AppHeartBeatThreadPtr.reset(
        new std::thread(&RSAppCarManager::car2appHeartBeatThread, this));
    m_softwareVersionThreadPtr.reset(
        new std::thread(&RSAppCarManager::softwareVersionThread, this));
  } catch (...) {
    m_isExit = true;
    m_isSendCollectSoftwareVersion = false;
    RS_ERROR(m_pSharedNodeHandle,
                 "Create HeartBeat/SoftwareVersion Thread Failed !");
    return -4;
  }

  return 0;
}

void RSAppCarManager::recvMessageCallback(
    const CALLBACK_PARAM_TYPE(std_msgs_String) &msgPtr) {
  // RS_STD_DEBUG("Receive Message Data msgPtr->data = " << msgPtr->data);
  if (msgPtr != nullptr) {
    RSApp2CarMessage::Ptr appCarMessagePtr =
        RSApp2CarMessageFactory::createApp2CarMessage(*msgPtr);
    if (appCarMessagePtr != nullptr) {
      if (m_app2CarConfig.enableDebug) {
        appCarMessagePtr->printMessage(m_pSharedNodeHandle);
      }

      if (appCarMessagePtr->isCmdTypeAck()) {
        // RCLCPP_ERROR(m_pSharedNodeHandle->get_logger(),"RUN HERE";
        RS_ACK_KEY key{
            static_cast<RS_CAR_2_APP_MESSAGE_TYPE>(
                appCarMessagePtr->content["Receive_cmd_type"]
                    .template get<int>()),
            appCarMessagePtr->content["Receive_cmd_id"].template get<int>()};

        std::lock_guard<std::mutex> lg(sendAckInfoMtx);
        auto iterMap = m_sendAckInfo.find(key);
        if (iterMap != m_sendAckInfo.end()) {
          auto &ackInfo = m_sendAckInfo[key];
          ackInfo.isAckReceive = true;

          RS_INFO_STREAM(
              m_pSharedNodeHandle,
              "CAR => APP Send Message Receive APP ACK: ack check count: "
                  << ackInfo.ack_check_count << ", check cmd type = "
                  << RSCar2AppMessage().fromCar2AppTypeToName(ackInfo.key.first)
                  << ", check cmd id = " << ackInfo.key.second);
        }
        return;
      } else if (appCarMessagePtr->isCmdTypeNothing()) {
        // NOTHING
      } else {
        // 响应ACK
        RSCar2AppAckMessage::Ptr car2AppAckMessagePtr(
            new RSCar2AppAckMessage());

        car2AppAckMessagePtr->content["Receive_cmd_type"] =
            static_cast<int>(appCarMessagePtr->cmd_type);
        car2AppAckMessagePtr->content["Receive_cmd_id"] =
            appCarMessagePtr->cmd_id;

        {
          std::lock_guard<std::mutex> lg(m_msgSndMtx);
          m_msgSndBuffers.push(car2AppAckMessagePtr);
          m_msgSndCond.notify_all();
        }
      }

      std::lock_guard<std::mutex> lg(m_msgSndMtx);
      m_msgRecvBuffers.push(appCarMessagePtr);
      m_msgRecvCond.notify_one();
    } else {
      RS_WARN(
          m_pSharedNodeHandle,
          "Not Collect Parse Json Data, Cann't Create Matched Message !");
    }
  }
}

void RSAppCarManager::recvWorkThread() {
  while (m_isExit == false) {
    RSApp2CarMessage::Ptr appCarMessagePtr;
    {
      std::unique_lock<std::mutex> lg(m_msgRecvMtx);
      m_msgRecvCond.wait(
          lg, [this] { return m_isExit || m_msgRecvBuffers.size(); });
      if (m_isExit) {
        break;
      }

      appCarMessagePtr = m_msgRecvBuffers.front();
      m_msgRecvBuffers.pop();
    }

    if (appCarMessagePtr != nullptr) {
      m_isAppIsStart = true;
      if (appCarMessagePtr->isCmdTypeSelfCheck()) {
        m_postProcessCmdId.clear();
      } else if (appCarMessagePtr->isCmdTypeEndCollection()) {
        m_isAppIsStart = false;
        m_postProcessCmdId.clear();
      } else if (appCarMessagePtr->isCmdTypeStopCollectVersion()) {
        m_isSendCollectSoftwareVersion = false;
      }
      m_msgLastestRecvId = appCarMessagePtr->cmd_id;
      m_msgLastestRecvTimestampNs = RS_TIMESTAMP_NS;
      // 更新最大App2CarCmdId
      if (m_msgLastestRecvId > m_maxApp2CarCmdId) {
        m_maxApp2CarCmdId = m_msgLastestRecvId;
      }

      if (m_rosMessageCallback != nullptr) {
        // 增加后处理保护，避免重复处理重传的后处理命令
        if (m_postProcessMessageType.find(appCarMessagePtr->cmd_type) !=
            m_postProcessMessageType.end()) {
          if (m_postProcessCmdId.find(appCarMessagePtr->cmd_id) !=
              m_postProcessCmdId.end()) {
            // 不进行重复处理
            RS_WARN_STREAM(m_pSharedNodeHandle,
                               "Receive Repeated PostProcess CmdId = "
                                   << appCarMessagePtr->cmd_id << ", cmdType = "
                                   << RSApp2CarMessage().fromApp2CarTypeToName(
                                          appCarMessagePtr->cmd_type));
            continue;
          } else {
            m_postProcessCmdId.insert(appCarMessagePtr->cmd_id);
          }
        }

        m_rosMessageCallback(appCarMessagePtr);
      }
    }
  }
}

void RSAppCarManager::timeoutCheckThread() {
  while (m_isExit == false) {
    double currentTimestampNs = RS_TIMESTAMP_NS;
    if (currentTimestampNs > m_msgLastestRecvTimestampNs &&
        (currentTimestampNs - m_msgLastestRecvTimestampNs) >
            m_app2CarConfig.app2CarTimeoutMs * 1e6 &&
        m_msgLastestRecvTimestampNs > 0 && m_isAppIsStart == true) {
      std::lock_guard<std::mutex> lg(m_msgRecvMtx);
      RSApp2CarEndCollection::Ptr appCarMessagePtr(
          new RSApp2CarEndCollection());
      appCarMessagePtr->cmd_id = (m_msgLastestRecvId + 1);
      m_msgRecvBuffers.push(appCarMessagePtr);
      m_msgRecvCond.notify_one();

      m_msgLastestRecvTimestampNs = currentTimestampNs;
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

void RSAppCarManager::car2appHeartBeatThread() {
  while (m_isExit == false) {
    {
      std::lock_guard<std::mutex> lg(m_msgSndMtx);
      RSCar2AppNothingMessage::Ptr carAppMessagePtr(
          new RSCar2AppNothingMessage());
      m_msgLastestSndId++;
      carAppMessagePtr->logger_id = m_msgLastestSndId;
      m_msgSndBuffers.push(carAppMessagePtr);
      m_msgSndCond.notify_one();
    }
    std::this_thread::sleep_for(
        std::chrono::milliseconds(m_app2CarConfig.car2AppHeartBeatMs));
  }
}

void RSAppCarManager::softwareVersionThread() {
  while (m_isExit == false && m_isSendCollectSoftwareVersion) {
    {
      std::lock_guard<std::mutex> lg(m_msgSndMtx);
      RSCar2AppCollectVersionMessage::Ptr carAppMessagePtr(
          new RSCar2AppCollectVersionMessage());
      m_msgLastestSndId++;
      carAppMessagePtr->logger_id = m_msgLastestSndId;
      carAppMessagePtr->content["version"] = RSVersionUtil::VERSIONSTRING("v");
      // if (!m_vehicleId.empty()) {
      //   carAppMessagePtr->content["vehicle_id"] = m_vehicleId;
      // }
      m_msgSndBuffers.push(carAppMessagePtr);
      m_msgSndCond.notify_one();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }
}

void RSAppCarManager::ackCheckThread() {
  while (m_isExit == false) {
    uint64_t current_timestamp_ns = RS_TIMESTAMP_NS;
    // 检查ACK是否收到或者超时
    {
      std::lock_guard<std::mutex> lg(sendAckInfoMtx);
      for (auto iterMap = m_sendAckInfo.begin();
           iterMap != m_sendAckInfo.end();) {
        const auto &ackSendInfo = iterMap->second;
        if (ackSendInfo.isAckReceive) {
          if (m_app2CarConfig.enableDebug) {
            RS_INFO_STREAM(
                m_pSharedNodeHandle,
                "CAR => APP Send Message Receive APP ACK: ack check count: "
                    << ackSendInfo.ack_check_count << ", check cmd type = "
                    << RSCar2AppMessage().fromCar2AppTypeToName(
                           ackSendInfo.key.first)
                    << ", check cmd id = " << ackSendInfo.key.second);
          }
          iterMap = m_sendAckInfo.erase(iterMap);
        } else if (ackSendInfo.ack_check_count > RS_MAX_ACK_WAIT_CNT) {
          if (m_app2CarConfig.enableDebug) {
            RS_INFO_STREAM(
                m_pSharedNodeHandle,
                "CAR => APP Send Message Wait APP ACK Timeout: ack check "
                "count: "
                    << ackSendInfo.ack_check_count << ", check cmd type = "
                    << RSCar2AppMessage().fromCar2AppTypeToName(
                           ackSendInfo.key.first)
                    << ", check cmd id = " << ackSendInfo.key.second);
          }
          iterMap = m_sendAckInfo.erase(iterMap);
        } else {
          ++iterMap;
        }
      }
    }

    // 进行状态更新
    {
      std::lock_guard<std::mutex> lg(sendAckInfoMtx);
      for (auto iterMap = m_sendAckInfo.begin(); iterMap != m_sendAckInfo.end();
           ++iterMap) {
        auto &ackSendInfo = iterMap->second;
        if (ackSendInfo.next_ack_check_timestamp_ns <= current_timestamp_ns) {
          ackSendInfo.ack_check_count++;
          ackSendInfo.next_ack_check_timestamp_ns +=
              RS_ACK_CHECK_TIMEOUT_TH_MS * 1e6;

          // 再次发送
          if (ackSendInfo.car2AppMsgPtr != nullptr) {
            std::lock_guard<std::mutex> lg(m_msgSndMtx);
            m_msgSndBuffers.push(ackSendInfo.car2AppMsgPtr);
            m_msgSndCond.notify_all();
          }
        }
      }
    }

    std::this_thread::sleep_for(
        std::chrono::milliseconds(RS_ACK_CHECK_TIMEOUT_TH_MS));
  }
}

void RSAppCarManager::sndWorkThread() {
  while (m_isExit == false) {
    RSCar2AppMessage::Ptr appCarMessagePtr;
    {
      std::unique_lock<std::mutex> lg(m_msgSndMtx);
      m_msgSndCond.wait(lg,
                        [this] { return m_isExit || m_msgSndBuffers.size(); });
      if (m_isExit) {
        break;
      }

      appCarMessagePtr = m_msgSndBuffers.front();
      m_msgSndBuffers.pop();

      // 更新Send Message Id
      if (appCarMessagePtr->logger_id == -1) {
        ++m_msgLastestSndId;
        appCarMessagePtr->logger_id = m_msgLastestSndId;
      }

      if (appCarMessagePtr->isResponseTypeMessage()) {
        appCarMessagePtr->lasest_received_cmd_id_from_app =
            appCarMessagePtr->content["Receive_cmd_id"]
                .template get<uint32_t>();
      } else {
        appCarMessagePtr->lasest_received_cmd_id_from_app = m_msgLastestRecvId;
      }
    }

    if (appCarMessagePtr != nullptr) {
      std_msgs_String car2app_protomsg;
      int ret = appCarMessagePtr->toProtoMessage(car2app_protomsg);
      if (ret != 0) {
        RS_WARN(m_pSharedNodeHandle,
                    "To Proto Message Failed !");
        appCarMessagePtr->printMessage(m_pSharedNodeHandle);
        continue;
      }

      if (m_app2CarConfig.enableDebug) {
        appCarMessagePtr->printMessage(m_pSharedNodeHandle);
      }

      // 发送消息
#if __ROS2__
      m_cyberrtApp2CarPub->publish(car2app_protomsg);
#elif __ROS1__
      m_cyberrtApp2CarPub.publish(car2app_protomsg);
#endif
      RS_INFO(m_pSharedNodeHandle, "ROS Publish Message !");
    }

    if (!appCarMessagePtr->isAckTypeMessage() &&
        !appCarMessagePtr->isNothingTypeMessage() &&
        !appCarMessagePtr->isInfoTypeMessage() &&
        !appCarMessagePtr->isHintTypeMessage() &&
        !appCarMessagePtr->isWarnTypeMessage() &&
        !appCarMessagePtr->isFetalTypeMessage() &&
        !appCarMessagePtr->isCollectSoftwareUpdateProgressTypeMessage() &&
        !appCarMessagePtr->isCyberCheckProgressTypeMessage() &&
        !appCarMessagePtr->isCyberCombineProgressTypeMessage() &&
        !appCarMessagePtr->isCyberResplitProgressTypeMessage() &&
        !appCarMessagePtr->isCollectVersionTypeMessage()) {
      RS_ACK_KEY key{appCarMessagePtr->logger_type,
                     appCarMessagePtr->logger_id};

      if (m_sendAckInfo.find(key) == m_sendAckInfo.end()) {
        RSAckInfo ackInfo;
        ackInfo.start_ack_wait_timestamp_ns = RS_TIMESTAMP_NS;
        ackInfo.next_ack_check_timestamp_ns =
            ackInfo.start_ack_wait_timestamp_ns +
            RS_ACK_CHECK_TIMEOUT_TH_MS * 1e6;
        ackInfo.isAckReceive = false;
        ackInfo.ack_check_count = 0;
        ackInfo.key = key;
        ackInfo.car2AppMsgPtr = appCarMessagePtr;
        m_sendAckInfo[key] = ackInfo;
      }
    }
  }
}

} // namespace collect
} // namespace rs_collect
} // namespace robosense