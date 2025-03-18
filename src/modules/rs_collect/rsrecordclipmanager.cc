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