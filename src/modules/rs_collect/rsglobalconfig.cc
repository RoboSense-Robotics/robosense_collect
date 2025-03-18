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
#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

RSGlobalConfig::RSGlobalConfig() {
  configPath = "./RDCS_ROOT/";
  isTaskNameValid = false;
  taskName = "";
  clipId = -1;
  app2CarClipId = -1;

  saveMode = RS_DATA_SAVE_MODE::RS_DATA_SAVE_BY_NOSEGMENT;
  saveDataTh = 5120;
  isRecord = false;
}

RSGlobalConfig::~RSGlobalConfig() {
  // NOTHING TODO...
}

}  // namespace collect
}  // namespace rs_collect
}  // namespace robosense