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
