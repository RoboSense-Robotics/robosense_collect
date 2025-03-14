#ifndef RSAPP2CARMANAGER_H
#define RSAPP2CARMANAGER_H

/*
 * ModuleName: RSApp2CarManager
 * Description: 实现App 和 Car
 * Author: hsw
 * Date: 2023-09-07
 *
 */

#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

enum class RS_APP_2_CAR_MESSAGE_TYPE : unsigned char {
  RS_APP_CAR_MESSAGE_NOTHING = 0,
  RS_APP_CAR_MESSAGE_SELFCHECK,
  RS_APP_CAR_MESSAGE_STARTCOLLECTION,
  RS_APP_CAR_MESSAGE_PAUSECOLLECTION,
  RS_APP_CAR_MESSAGE_ENDCOLLECTION,
  RS_APP_CAR_MESSAGE_ONLYUPDATEJSON,
  RS_APP_CAR_MESSAGE_UPDATE_TAG,
  RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE,
  RS_APP_CAR_MESSAGE_STOP_COLLECT_VERSION,
  RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE,
  RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION,
  RS_APP_CAR_MESSAGE_TAG_ICON_TYPE,
  RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE,
  RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG,
  RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG,
  RS_APP_CAR_MESSAGE_CYBER_COMBINE,
  RS_APP_CAR_MESSAGE_CYBER_RESPLIT,
  RS_APP_CAR_MESSAGE_CYBER_CHECKER,
  RS_APP_CAR_MESSAGE_ACK,
  RS_APP_CAR_MESSAGE_ORIN_KILL_NODES,
  RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING,
  RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING,
  RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS,
  RS_APP_CAR_MESSAGE_ORIN_COPY_LOG,
  RS_APP_CAR_MESSAGE_ORIN_KILL_HMI_NODE,

  RS_APP_CAR_MESSAGE_UNKNOWN = 0xFF,
};

enum class RS_CAR_2_APP_MESSAGE_TYPE : unsigned char {
  RS_APP_CAR_MESSAGE_NOTHING = 0,
  RS_APP_CAR_MESSAGE_INFO,
  RS_APP_CAR_MESSAGE_HINT,
  RS_APP_CAR_MESSAGE_WARN,
  RS_APP_CAR_MESSAGE_FETAL,
  RS_APP_CAR_MESSAGE_RESPONSE,
  RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID,
  RS_APP_CAR_MESSAGE_COLLECT_VERSION,
  RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS,
  RS_APP_CAR_MESSAGE_CYBER_POSTPROCESS,
  RS_APP_CAR_MESSAGE_CYBER_COMBINE_PROGRESS,
  RS_APP_CAR_MESSAGE_CYBER_RESPLIT_PROGRESS,
  RS_APP_CAR_MESSAGE_CYBER_CHECK_PROGRESS,
  RS_APP_CAR_MESSAGE_ACK,

  RS_APP_CAR_MESSAGE_COPY_LOG_PROGRESS = 17,

  RS_APP_CAR_MESSAGE_UNKNOWN = 0xFF,
};

// Car <-> App
class RSCarAppMessage {
public:
  using Ptr = std::shared_ptr<RSCarAppMessage>;
  using ConstPtr = std::shared_ptr<const RSCarAppMessage>;

public:
  RSCarAppMessage();
  virtual ~RSCarAppMessage();

public:
  virtual void reset();

public:
  std::string
  fromApp2CarTypeToName(const RS_APP_2_CAR_MESSAGE_TYPE cmd_type) const {
    std::string msg;
    switch (cmd_type) {
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING: {
      msg += "NOTHING";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SELFCHECK: {
      msg += "SELFCHECK";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STARTCOLLECTION: {
      msg += "STARTCOLLECT";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_PAUSECOLLECTION: {
      msg += "PAUSECOLLECTION";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ENDCOLLECTION: {
      msg += "ENDCOLLECTION";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON: {
      msg += "ONLYUPDATEJSON";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_TAG: {
      msg += "UPDATE TAG";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE: {
      msg += "UPDATE CALIBRATION FILE";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STOP_COLLECT_VERSION: {
      msg += "STOP COLLECT VERSION";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE: {
      msg += "UPDATE COLLECT SOFTWARE";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION: {
      msg += "TAG ROAD NAVIGATION";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ICON_TYPE: {
      msg += "TAG ICON TYPE";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE: {
      msg += "TAG INTERSECTION TYPE";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG: {
      msg += "UPDATE FILE TAG";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG: {
      msg += "AUDIO TAG";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE: {
      msg += "CYBER COMBINE";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT: {
      msg += "CYBER RESPLIT";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECKER: {
      msg += "CYBER CHECKER";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK: {
      msg += "ACK";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_NODES: {
      msg += "ORIN KILL NODES";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING: {
      msg += "GET COLLECT SETTING";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING: {
      msg += "SET COLLECT SETTING";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS: {
      msg += "GET COLLECT STATUS";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_COPY_LOG: {
      msg += "ORIN COPY LOG";
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_HMI_NODE: {
      msg += "ORIN KILL HMI NODE";
      break;
    }
    default: {
      msg = "UNKNOWN";
      break;
    }
    }
    return msg;
  }

  std::string
  fromCar2AppTypeToName(const RS_CAR_2_APP_MESSAGE_TYPE logger_type) const {
    std::string msg;

    switch (logger_type) {
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING: {
      msg += "NOTHING";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO: {
      msg += "INFO";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_HINT: {
      msg += "HINT";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_WARN: {
      msg += "WARN";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL: {
      msg += "FETAL";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE: {
      msg += "RESPONSE";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID: {
      msg += "UPDATE CLIP ID";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COLLECT_VERSION: {
      msg += "COLLECT VERSION";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS: {
      msg += "COLLECT SOFTWARE UPDATE PROGRESS";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_POSTPROCESS: {
      msg += "CYBER POSTPROCESS";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE_PROGRESS: {
      msg += "CYBER COMBINE PROGRESS";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT_PROGRESS: {
      msg += "CYBER RESPLIT PROGRESS";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECK_PROGRESS: {
      msg += "CYBER CHECK PROGRESS";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK: {
      msg += "ACK";
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COPY_LOG_PROGRESS: {
      msg += "COPY LOG PROGRESS";
      break;
    }
    default: {
      msg += "UNKNOWN";
      break;
    }
    }
    return msg;
  }

protected:
  std::string toJsonString(const json &jsonValue) const;
  int fromJsonString(const std::string &json_content, json &jsonValue);
};

// Car => App
class RSCar2AppMessage : public RSCarAppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppMessage>;

public:
  RSCar2AppMessage();
  virtual ~RSCar2AppMessage();

public:
  void reset() override;
  int fromProtoMessage(const std_msgs_String &proto_msg);
  int toProtoMessage(std_msgs_String &proto_msg);
  void printMessage(const NodeHandlePtr &node) const {
    std::string msg =
        "APP <<<=== CAR " + fromCar2AppTypeToName(logger_type) + ":";

    RS_APP_2_CAR_MESSAGE_TYPE app2CarMessageType =
        RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UNKNOWN;
    for (auto iterMap = content.begin(); iterMap != content.end(); ++iterMap) {
      const std::string &key = iterMap.key();
      if (key == "Receive_cmd_type") {
        app2CarMessageType = static_cast<RS_APP_2_CAR_MESSAGE_TYPE>(
            content[key].template get<int>());
      }
    }

    RS_INFO_STREAM(node, msg << ", Receive cmd type = "
                             << fromApp2CarTypeToName(app2CarMessageType)
                             << ", logger_id = " << logger_id
                             << ", cmd_id_from_app = "
                             << lasest_received_cmd_id_from_app
                             << ", json_content = " << logger_content);
  }

public:
  bool isNothingTypeMessage() const;
  bool isInfoTypeMessage() const;
  bool isHintTypeMessage() const;
  bool isWarnTypeMessage() const;
  bool isFetalTypeMessage() const;
  bool isResponseTypeMessage() const;
  bool isUpdateClipIdTypeMessage() const;
  bool isCollectVersionTypeMessage() const;
  bool isCollectSoftwareUpdateProgressTypeMessage() const;
  bool isCyberPostProcessTypeMessage() const;
  bool isCyberCombineProgressTypeMessage() const;
  bool isCyberResplitProgressTypeMessage() const;
  bool isCyberCheckProgressTypeMessage() const;
  bool isAckTypeMessage() const;

public:
  RS_CAR_2_APP_MESSAGE_TYPE logger_type;
  unsigned int logger_id;
  unsigned int lasest_received_cmd_id_from_app;
  std::string logger_content;

public:
  json content;
};

class RSCar2AppNothingMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppMessage>;

public:
  RSCar2AppNothingMessage();
  virtual ~RSCar2AppNothingMessage();

public:
  void reset() override;
};

class RSCar2AppInfoMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppInfoMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppInfoMessage>;

public:
  RSCar2AppInfoMessage();
  virtual ~RSCar2AppInfoMessage();

public:
  void reset() override;
};

class RSCar2AppHintMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppHintMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppHintMessage>;

public:
  RSCar2AppHintMessage();
  virtual ~RSCar2AppHintMessage();

public:
  void reset() override;
};

class RSCar2AppWarnMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppWarnMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppWarnMessage>;

public:
  RSCar2AppWarnMessage();
  virtual ~RSCar2AppWarnMessage();

public:
  void reset() override;
};

class RSCar2AppFetalMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppFetalMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppFetalMessage>;

public:
  RSCar2AppFetalMessage();
  virtual ~RSCar2AppFetalMessage();

public:
  void reset() override;
};

class RSCar2AppResponseMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppResponseMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppResponseMessage>;

public:
  RSCar2AppResponseMessage();
  virtual ~RSCar2AppResponseMessage();

public:
  void reset() override;
};

class RSCar2AppUpdateClipIdMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppUpdateClipIdMessage>;
  using ConstPtr = std::shared_ptr<const RSCar2AppUpdateClipIdMessage>;

public:
  RSCar2AppUpdateClipIdMessage();
  virtual ~RSCar2AppUpdateClipIdMessage();

public:
  void reset() override;
};

class RSCar2AppCollectVersionMessage : public RSCar2AppMessage {
public:
  using Ptr = std::shared_ptr<RSCar2AppCollectVersionMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCollectVersionMessage>;

public:
  RSCar2AppCollectVersionMessage();
  virtual ~RSCar2AppCollectVersionMessage();

public:
  void reset() override;
};

class RSCar2AppCollectSoftwareUpdateProgressMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCollectSoftwareUpdateProgressMessage>;
  using ConstPtr =
      std::shared_ptr<RSCar2AppCollectSoftwareUpdateProgressMessage>;

public:
  RSCar2AppCollectSoftwareUpdateProgressMessage();
  virtual ~RSCar2AppCollectSoftwareUpdateProgressMessage();

public:
  void reset() override;
};

class RSCar2AppCyberPostProcessMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCyberPostProcessMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCyberPostProcessMessage>;

public:
  RSCar2AppCyberPostProcessMessage();
  virtual ~RSCar2AppCyberPostProcessMessage();

public:
  void reset() override;
};

class RSCar2AppCyberCombineProgressMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCyberCombineProgressMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCyberCombineProgressMessage>;

public:
  RSCar2AppCyberCombineProgressMessage();
  virtual ~RSCar2AppCyberCombineProgressMessage();

public:
  void reset() override;
};

class RSCar2AppCyberResplitProgressMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCyberResplitProgressMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCyberResplitProgressMessage>;

public:
  RSCar2AppCyberResplitProgressMessage();
  virtual ~RSCar2AppCyberResplitProgressMessage();

public:
  void reset() override;
};

class RSCar2AppCyberCheckProgressMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCyberCheckProgressMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCyberCheckProgressMessage>;

public:
  RSCar2AppCyberCheckProgressMessage();
  virtual ~RSCar2AppCyberCheckProgressMessage();

public:
  void reset() override;
};

class RSCar2AppAckMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppAckMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppAckMessage>;

public:
  RSCar2AppAckMessage();
  virtual ~RSCar2AppAckMessage();

public:
  void reset() override;
};

class RSCar2AppCopyLogProgressMessage : public RSCar2AppMessage {
public:
public:
  using Ptr = std::shared_ptr<RSCar2AppCopyLogProgressMessage>;
  using ConstPtr = std::shared_ptr<RSCar2AppCopyLogProgressMessage>;

public:
  RSCar2AppCopyLogProgressMessage();
  virtual ~RSCar2AppCopyLogProgressMessage();

public:
  void reset() override;
};

// App => Car
class RSApp2CarMessage : public RSCarAppMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarMessage>;
  using ConstPtr = std::shared_ptr<const RSApp2CarMessage>;

public:
  RSApp2CarMessage();

  virtual ~RSApp2CarMessage();

public:
  virtual void reset();
  int fromProtoMessage(const std_msgs_String &proto_msg);
  int toProtoMessage(std_msgs_String &proto_msg);

public:
  bool isCmdTypeNothing() const;
  bool isCmdTypeSelfCheck() const;
  bool isCmdTypeStartCollection() const;
  bool isCmdTypePauseCollection() const;
  bool isCmdTypeEndCollection() const;
  bool isCmdTypeOnlyUpdateJson() const;
  bool isCmdTypeUpdateTag() const;
  bool isCmdTypeUpdateCalibrationFile() const;
  bool isCmdTypeStopCollectVersion() const;
  bool isCmdTypeUpdateCollectSoftware() const;
  bool isCmdTypeTagRoadNavigation() const;
  bool isCmdTypeTagIconType() const;
  bool isCmdTypeTagIntersectionType() const;
  bool isCmdTypeUpdateFileTag() const;
  bool isCmdTypeUpdateAudioTag() const;
  bool isCmdTypeCyberChecker() const;
  bool isCmdTypeCyberCombine() const;
  bool isCmdTypeCyberResplit() const;
  bool isCmdTypeAck() const;
  bool isCmdTypeOrinKillNodes() const;
  bool isCmdTypeSetCollectSetting() const;
  bool isCmdTypeGetCollectSetting() const;
  bool isCmdTypeGetCollectStatus() const;
  bool isCmdTypeOrinCopyLog() const;
  bool isCmdTypeOrinKillHmiNode() const;
  int getTaskName(std::string &taskName) const;
  void printMessage(const NodeHandlePtr &node) const {
    std::string msg = "APP ===>>> CAR " + fromApp2CarTypeToName(cmd_type) + ":";
    RS_INFO_STREAM(
        node, msg   << "cmd_id = " << cmd_id << ", task_name = " << task_name
                    << ", cmd_id = " << cmd_id << ", device_id = " << device_id
                    << ", logger_id_from_car = "
                    << lasest_received_logger_id_from_car << ", json_content = "
                    << std::string(json_content.data(),
                                   std::min(json_content.size(),
                                            static_cast<size_t>(512))));
  }

public:
  RS_APP_2_CAR_MESSAGE_TYPE cmd_type;
  unsigned int cmd_id;
  std::string task_name;
  unsigned int lasest_received_logger_id_from_car;
  std::string device_id;
  std::string json_content;

public:
  json content;
};

class RSApp2CarNothingMessage : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarNothingMessage>;
  using ConstPtr = std::shared_ptr<const RSApp2CarNothingMessage>;

public:
  RSApp2CarNothingMessage();

  virtual ~RSApp2CarNothingMessage();

public:
  void reset() override;
};

class RSApp2CarSelfCheck : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarSelfCheck>;
  using ConstPtr = std::shared_ptr<const RSApp2CarSelfCheck>;

public:
  RSApp2CarSelfCheck();

  virtual ~RSApp2CarSelfCheck();

public:
  void reset() override;
};

class RSApp2CarStartCollection : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarStartCollection>;
  using ConstPtr = std::shared_ptr<const RSApp2CarStartCollection>;

public:
  RSApp2CarStartCollection();

  virtual ~RSApp2CarStartCollection();

public:
  void reset() override;
};

class RSApp2CarPauseCollection : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarPauseCollection>;
  using ConstPtr = std::shared_ptr<const RSApp2CarPauseCollection>;

public:
  RSApp2CarPauseCollection();

  virtual ~RSApp2CarPauseCollection();

public:
  void reset() override;
};

class RSApp2CarEndCollection : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarEndCollection>;
  using ConstPtr = std::shared_ptr<const RSApp2CarEndCollection>;

public:
  RSApp2CarEndCollection();

  virtual ~RSApp2CarEndCollection();

public:
  void reset() override;
};

class RSApp2CarOnlyUpdateJson : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarOnlyUpdateJson>;
  using ConstPtr = std::shared_ptr<RSApp2CarOnlyUpdateJson>;

public:
  RSApp2CarOnlyUpdateJson();

  virtual ~RSApp2CarOnlyUpdateJson();

public:
  void reset() override;
};

class RSApp2CarUpdateTag : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarUpdateTag>;
  using ConstPtr = std::shared_ptr<RSApp2CarUpdateTag>;

public:
  RSApp2CarUpdateTag();

  virtual ~RSApp2CarUpdateTag();

public:
  void reset() override;
};

class RSApp2CarUpdateCalibrationFile : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarUpdateCalibrationFile>;
  using ConstPtr = std::shared_ptr<RSApp2CarUpdateCalibrationFile>;

public:
  RSApp2CarUpdateCalibrationFile();

  virtual ~RSApp2CarUpdateCalibrationFile();

public:
  void reset() override;
};

class RSApp2CarStopCollectVersion : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarStopCollectVersion>;
  using ConstPtr = std::shared_ptr<const RSApp2CarStopCollectVersion>;

public:
  RSApp2CarStopCollectVersion();
  virtual ~RSApp2CarStopCollectVersion();

public:
  void reset() override;
};

class RSApp2CarUpdateCollectSoftware : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarUpdateCollectSoftware>;
  using ConstPtr = std::shared_ptr<const RSApp2CarUpdateCollectSoftware>;

public:
  RSApp2CarUpdateCollectSoftware();
  virtual ~RSApp2CarUpdateCollectSoftware();

public:
  void reset() override;
};

class RSApp2CarTagRoadNavigation : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarTagRoadNavigation>;
  using ConstPtr = std::shared_ptr<const RSApp2CarTagRoadNavigation>;

public:
  RSApp2CarTagRoadNavigation();
  virtual ~RSApp2CarTagRoadNavigation();

public:
  void reset() override;
};

class RSApp2CarTagIconType : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarTagIconType>;
  using ConstPtr = std::shared_ptr<const RSApp2CarTagIconType>;

public:
  RSApp2CarTagIconType();
  virtual ~RSApp2CarTagIconType();

public:
  void reset() override;
};

class RSApp2CarTagIntersectionType : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarTagIntersectionType>;
  using ConstPtr = std::shared_ptr<const RSApp2CarTagIntersectionType>;

public:
  RSApp2CarTagIntersectionType();
  virtual ~RSApp2CarTagIntersectionType();

public:
  void reset() override;
};

class RSApp2CarUpdateFileTag : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarUpdateFileTag>;
  using ConstPtr = std::shared_ptr<const RSApp2CarUpdateFileTag>;

public:
  RSApp2CarUpdateFileTag();
  virtual ~RSApp2CarUpdateFileTag();

public:
  void reset() override;
};

class RSApp2CarUpdateAudioTag : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarUpdateAudioTag>;
  using ConstPtr = std::shared_ptr<const RSApp2CarUpdateAudioTag>;

public:
  RSApp2CarUpdateAudioTag();
  virtual ~RSApp2CarUpdateAudioTag();

public:
  void reset() override;
};

class RSApp2CarCyberChecker : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarCyberChecker>;
  using ConstPtr = std::shared_ptr<const RSApp2CarCyberChecker>;

public:
  RSApp2CarCyberChecker();
  virtual ~RSApp2CarCyberChecker();

public:
  void reset() override;
};

class RSApp2CarCyberCombine : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarCyberCombine>;
  using ConstPtr = std::shared_ptr<const RSApp2CarCyberCombine>;

public:
  RSApp2CarCyberCombine();
  virtual ~RSApp2CarCyberCombine();

public:
  void reset() override;
};

class RSApp2CarCyberResplit : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarCyberResplit>;
  using ConstPtr = std::shared_ptr<const RSApp2CarCyberResplit>;

public:
  RSApp2CarCyberResplit();
  virtual ~RSApp2CarCyberResplit();

public:
  void reset() override;
};

class RSApp2CarAck : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarAck>;
  using ConstPtr = std::shared_ptr<const RSApp2CarAck>;

public:
  RSApp2CarAck();
  virtual ~RSApp2CarAck();

public:
  void reset() override;
};

class RSApp2CarOrinKillNodes : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarOrinKillNodes>;
  using ConstPtr = std::shared_ptr<const RSApp2CarOrinKillNodes>;

public:
  RSApp2CarOrinKillNodes();
  virtual ~RSApp2CarOrinKillNodes();

public:
  void reset() override;
};

class RSApp2CarSetCollectSetting : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarSetCollectSetting>;
  using ConstPtr = std::shared_ptr<const RSApp2CarSetCollectSetting>;

public:
  RSApp2CarSetCollectSetting();
  virtual ~RSApp2CarSetCollectSetting();

public:
  void reset() override;
};

class RSApp2CarGetCollectSetting : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarGetCollectSetting>;
  using ConstPtr = std::shared_ptr<const RSApp2CarGetCollectSetting>;

public:
  RSApp2CarGetCollectSetting();
  virtual ~RSApp2CarGetCollectSetting();

public:
  void reset() override;
};

class RSApp2CarGetCollectStatus : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarGetCollectStatus>;
  using ConstPtr = std::shared_ptr<const RSApp2CarGetCollectStatus>;

public:
  RSApp2CarGetCollectStatus();
  virtual ~RSApp2CarGetCollectStatus();

public:
  void reset() override;
};

class RSApp2CarOrinCopyLog : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarOrinCopyLog>;
  using ConstPtr = std::shared_ptr<const RSApp2CarOrinCopyLog>;

public:
  RSApp2CarOrinCopyLog();
  virtual ~RSApp2CarOrinCopyLog();

public:
  void reset() override;
};

class RSApp2CarOrinKillHmiNode : public RSApp2CarMessage {
public:
  using Ptr = std::shared_ptr<RSApp2CarOrinKillHmiNode>;
  using ConstPtr = std::shared_ptr<const RSApp2CarOrinKillHmiNode>;

public:
  RSApp2CarOrinKillHmiNode();
  virtual ~RSApp2CarOrinKillHmiNode();

public:
  void reset() override;
};

class RSCar2AppMessageFactory {
public:
  using Ptr = std::shared_ptr<RSCar2AppMessageFactory>;
  using ConstPtr = std::shared_ptr<const RSCar2AppMessageFactory>;

public:
  static RSCar2AppMessage::Ptr
  createCar2AppMessage(const RS_CAR_2_APP_MESSAGE_TYPE type) {
    RSCar2AppMessage::Ptr car2AppMessagePtr = nullptr;

    switch (type) {
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING: {
      car2AppMessagePtr.reset(new RSCar2AppNothingMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_INFO: {
      car2AppMessagePtr.reset(new RSCar2AppInfoMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_HINT: {
      car2AppMessagePtr.reset(new RSCar2AppHintMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_WARN: {
      car2AppMessagePtr.reset(new RSCar2AppWarnMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_FETAL: {
      car2AppMessagePtr.reset(new RSCar2AppFetalMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_RESPONSE: {
      car2AppMessagePtr.reset(new RSCar2AppResponseMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_CLIP_ID: {
      car2AppMessagePtr.reset(new RSCar2AppUpdateClipIdMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COLLECT_VERSION: {
      car2AppMessagePtr.reset(new RSCar2AppCollectVersionMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_COLLECT_SOFTWARE_UPDATE_PROGRESS: {
      car2AppMessagePtr.reset(
          new RSCar2AppCollectSoftwareUpdateProgressMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_POSTPROCESS: {
      car2AppMessagePtr.reset(new RSCar2AppCyberPostProcessMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE_PROGRESS: {
      car2AppMessagePtr.reset(new RSCar2AppCyberCombineProgressMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT_PROGRESS: {
      car2AppMessagePtr.reset(new RSCar2AppCyberResplitProgressMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECK_PROGRESS: {
      car2AppMessagePtr.reset(new RSCar2AppCyberCheckProgressMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK: {
      car2AppMessagePtr.reset(new RSCar2AppAckMessage());
      break;
    }
    case RS_CAR_2_APP_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_COPY_LOG_PROGRESS: {
      car2AppMessagePtr.reset(new RSCar2AppCopyLogProgressMessage());
      break;
    }
    default: {
      break;
    }
    }

    return car2AppMessagePtr;
  }
};

class RSApp2CarMessageFactory {
public:
  using Ptr = std::shared_ptr<RSApp2CarMessageFactory>;
  using ConstPtr = std::shared_ptr<const RSApp2CarMessageFactory>;

public:
  static RSApp2CarMessage::Ptr
  createApp2CarMessage(const std_msgs_String &msg) {
    RSApp2CarMessage app2CarMessage;
    int ret = app2CarMessage.fromProtoMessage(msg);
    if (ret != 0) {
      return nullptr;
    }
    auto app2CarMessagePtr = createApp2CarMessage(app2CarMessage.cmd_type);
    if (app2CarMessagePtr != nullptr) {
      ret = app2CarMessagePtr->fromProtoMessage(msg);
      if (ret != 0) {
        return nullptr;
      }
      return app2CarMessagePtr;
    }

    return nullptr;
  }

  static RSApp2CarMessage::Ptr
  createApp2CarMessage(const RS_APP_2_CAR_MESSAGE_TYPE type) {
    RSApp2CarMessage::Ptr app2CarMessagePtr = nullptr;
    switch (type) {
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_NOTHING: {
      app2CarMessagePtr.reset(new RSApp2CarNothingMessage());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SELFCHECK: {
      app2CarMessagePtr.reset(new RSApp2CarSelfCheck());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STARTCOLLECTION: {
      app2CarMessagePtr.reset(new RSApp2CarStartCollection());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_PAUSECOLLECTION: {
      app2CarMessagePtr.reset(new RSApp2CarPauseCollection());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ENDCOLLECTION: {
      app2CarMessagePtr.reset(new RSApp2CarEndCollection());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ONLYUPDATEJSON: {
      app2CarMessagePtr.reset(new RSApp2CarOnlyUpdateJson());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_TAG: {
      app2CarMessagePtr.reset(new RSApp2CarUpdateTag());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_CALIBRATION_FILE: {
      app2CarMessagePtr.reset(new RSApp2CarUpdateCalibrationFile());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_STOP_COLLECT_VERSION: {
      app2CarMessagePtr.reset(new RSApp2CarStopCollectVersion());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::
        RS_APP_CAR_MESSAGE_UPDATE_COLLECT_SOFTWARE: {
      app2CarMessagePtr.reset(new RSApp2CarUpdateCollectSoftware());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ROAD_NAVIGATION: {
      app2CarMessagePtr.reset(new RSApp2CarTagRoadNavigation());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_ICON_TYPE: {
      app2CarMessagePtr.reset(new RSApp2CarTagIconType());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_TAG_INTERSECTION_TYPE: {
      app2CarMessagePtr.reset(new RSApp2CarTagIntersectionType());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_FILE_TAG: {
      app2CarMessagePtr.reset(new RSApp2CarUpdateFileTag());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_UPDATE_AUDIO_TAG: {
      app2CarMessagePtr.reset(new RSApp2CarUpdateAudioTag());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECKER: {
      app2CarMessagePtr.reset(new RSApp2CarCyberChecker());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE: {
      app2CarMessagePtr.reset(new RSApp2CarCyberCombine());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT: {
      app2CarMessagePtr.reset(new RSApp2CarCyberResplit());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ACK: {
      app2CarMessagePtr.reset(new RSApp2CarAck());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_NODES: {
      app2CarMessagePtr.reset(new RSApp2CarOrinKillNodes());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_SETTING: {
      app2CarMessagePtr.reset(new RSApp2CarGetCollectSetting());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_SET_COLLECT_SETTING: {
      app2CarMessagePtr.reset(new RSApp2CarSetCollectSetting());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_GET_COLLECT_STATUS: {
      app2CarMessagePtr.reset(new RSApp2CarGetCollectStatus());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_COPY_LOG: {
      app2CarMessagePtr.reset(new RSApp2CarOrinCopyLog());
      break;
    }
    case RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_ORIN_KILL_HMI_NODE: {
      app2CarMessagePtr.reset(new RSApp2CarOrinKillHmiNode());
      break;
    }
    default: {
      break;
    }
    }

    return app2CarMessagePtr;
  }
};

class RSAppCarManager {
public:
  using Ptr = std::shared_ptr<RSAppCarManager>;
  using ConstPtr = std::shared_ptr<const RSAppCarManager>;

public:
  using RS_RECV_ROSMESSAGE_CALLBACK =
      std::function<void(const RSApp2CarMessage::Ptr &)>;

  using RS_ACK_KEY = std::pair<RS_CAR_2_APP_MESSAGE_TYPE, uint32_t>;

  struct RSAckInfo {
  public:
    RS_ACK_KEY key;
    uint64_t start_ack_wait_timestamp_ns = 0;
    uint64_t next_ack_check_timestamp_ns = 0;
    uint32_t ack_check_count = 0;
    bool isAckReceive = false;
    RSCar2AppMessage::Ptr car2AppMsgPtr = nullptr;
  };

public:
  RSAppCarManager();
  ~RSAppCarManager();

public:
  int initManager(const NodeHandlePtr &pNode,
                  const RSApp2CarConfig &app2CarConfig,
                  // const std::string& vehicleId,
                  const RS_RECV_ROSMESSAGE_CALLBACK &callback);

  int addSendMessage(const RSCar2AppMessage::Ptr &appCarMessagePtr);

  uint32_t getMaxCheckRepeatCmdId();

private:
  int initManager();

  void recvMessageCallback(const CALLBACK_PARAM_TYPE(std_msgs_String) &msgPtr);

  void recvWorkThread();
  void car2appHeartBeatThread();
  void timeoutCheckThread();
  void sndWorkThread();
  void softwareVersionThread();
  void ackCheckThread();

private:
  bool m_isExit;
  // 接收队列
  unsigned int m_msgLastestRecvId;
  double m_msgLastestRecvTimestampNs;
  std::mutex m_msgRecvMtx;
  std::condition_variable m_msgRecvCond;
  std::queue<RSApp2CarMessage::Ptr> m_msgRecvBuffers;
  std::shared_ptr<std::thread> m_msgRecvThreadPtr;
  // 发送队列
  unsigned int m_msgLastestSndId;
  std::mutex m_msgSndMtx;
  std::condition_variable m_msgSndCond;
  std::queue<RSCar2AppMessage::Ptr> m_msgSndBuffers;
  std::shared_ptr<std::thread> m_msgSndThreadPtr;
  //
  std::shared_ptr<std::thread> m_timeoutThreadPtr;
  //
  std::shared_ptr<std::thread> m_car2AppHeartBeatThreadPtr;
  //
  std::shared_ptr<std::thread> m_softwareVersionThreadPtr;
  //
  NodeHandlePtr m_pSharedNodeHandle;
  //
  RSApp2CarConfig m_app2CarConfig;
  //
  RS_RECV_ROSMESSAGE_CALLBACK m_rosMessageCallback;
  bool m_isAppIsStart;
  // std::string m_vehicleId;

private:
#if __ROS2__
  rclcpp::Publisher<std_msgs_String>::SharedPtr m_cyberrtApp2CarPub;
  rclcpp::Subscription<std_msgs_String>::SharedPtr m_cyberrtApp2CarSub;
#elif __ROS1__
  ros::Publisher m_cyberrtApp2CarPub;
  ros::Subscriber m_cyberrtApp2CarSub;
#endif

private:
  bool m_isSendCollectSoftwareVersion;
  std::shared_ptr<std::thread> m_sendAckWaitThreadPtr;
  std::mutex sendAckInfoMtx;
  std::map<RS_ACK_KEY, RSAckInfo> m_sendAckInfo;

private:
  // 后处理id集合
  std::set<uint32_t> m_postProcessCmdId;

  uint32_t m_maxApp2CarCmdId = 0;

  std::set<RS_APP_2_CAR_MESSAGE_TYPE> m_postProcessMessageType = {
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_CHECKER,
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_COMBINE,
      RS_APP_2_CAR_MESSAGE_TYPE::RS_APP_CAR_MESSAGE_CYBER_RESPLIT};

private:
  const std::string RS_APP_2_CAR_TOPICNAME =
      "/carapp/app2car"; // 固定的topicName
  const std::string RS_CAR_2_APP_TOPICNAME =
      "/carapp/car2app"; // 固定的topicName
private:
  const uint32_t RS_MAX_ACK_WAIT_CNT = 10;
  const uint32_t RS_ACK_CHECK_TIMEOUT_TH_MS = 1000;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSAPP2CARMANAGER_H