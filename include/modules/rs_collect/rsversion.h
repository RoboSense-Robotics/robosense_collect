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
#ifndef RSVERSION_H
#define RSVERSION_H

/*
 * ModuleName: RSVersion
 * Description: 定义软件版本信息
 * Author: hsw
 * Date: 2024-12-06
 *
 */

#include <string>

namespace robosense {
namespace rs_collect {
namespace collect {

class RSVersionUtil {
 public:
  static const std::string RS_SOFT_RELEASE_DATE;
  static const std::string RS_SOFTWARE_NAME;
  static const int RS_TOOL_MAJOR_VERSION;
  static const int RS_TOOL_MINJOR_VERSION;
  static const int RS_TOOL_MINJOR_MINJOR_VERSION;

 public:
  static std::string VERSIONSTRING(const std::string& prefix) {
    return prefix + std::to_string(RSVersionUtil::RS_TOOL_MAJOR_VERSION) + "." +
           std::to_string(RSVersionUtil::RS_TOOL_MINJOR_VERSION) + "." +
           std::to_string(RSVersionUtil::RS_TOOL_MINJOR_MINJOR_VERSION);
  }
};

}  // namespace collect
}  // namespace rs_collect
}  // namespace robosense

#endif  // RSVERSION_H
