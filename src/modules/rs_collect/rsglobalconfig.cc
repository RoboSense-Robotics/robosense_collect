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