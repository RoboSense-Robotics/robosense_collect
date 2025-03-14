#include "modules/rs_collect/rsrecordclipmanager.h"

namespace robosense {
namespace rs_collect {
namespace collect {

// 静态变量
std::string RSRecordFileCheckInfo::RS_RECORD_NOT_COMPLETE_INFO =
    "Record File Not Complete";
std::string RSRecordFileCheckInfo::RS_RECORD_ZERO_MESSAGE_COUNT_INFO =
    "Record Is Zero Message Count";
std::string RSRecordFileCheckInfo::RS_RECOED_CHECK_PASS_INFO =
    "Record Check Pass";

// 静态变量
std::map<std::string, std::list<std::shared_ptr<RSRecordSingleMessage>>>
    RSRecordClipManager::RS_CAMERA_H265_BUFFER_MAPPER =
        std::map<std::string,
                 std::list<std::shared_ptr<RSRecordSingleMessage>>>();

} // namespace collect
} // namespace rs_collect
} // namespace robosense