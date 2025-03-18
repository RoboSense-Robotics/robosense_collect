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
#ifndef RSFILESYSTEM_H
#define RSFILESYSTEM_H

#include <dirent.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdint.h>

#include <iostream>
#include <memory.h>
#include <sstream>
#include <string>
#include <vector>

namespace robosense {
namespace filesystem {
namespace util {

class RSFileSystem {
public:
  RSFileSystem() = default;
  ~RSFileSystem() = default;

public:
  // 执行system命令
  static bool systemCmd(const std::string &cmd);

  static bool systemCmdReturnCode(const std::string &cmd, int32_t &ret_value);

  // 删除文件
  static bool removeFile(const std::string &filePath);

  // 删除文件夹
  static bool removeDirectory(const std::string &directoryPath);

  static bool mvFile(const std::string &oldFilePath,
                     const std::string &newFilePath, const bool isForce);

  // 判断文件是否存在
  static bool isFileExist(const std::string &filePath);

  // 判断文件夹是否存在
  static bool isDirectoryExist(const std::string &directoryPath);

  // 在指定文件夹目录下，创建文件
  static bool makeDirectory(const std::string &directoryPath,
                            const std::string &dirName);

  // 创建路径
  static bool makePath(const std::string &directoryPath);

  static bool searchFilesWithoutFilter(const std::string &directoryPath,
                                       const bool isRec,
                                       std::vector<std::string> &files);

  static bool searchFilesWithFilter(const std::string &directoryPath,
                                    const bool isRec,
                                    std::vector<std::string> &files,
                                    bool (*filter)(const std::string &));

  static bool
  searchDirectoryWithoutFilter(const std::string &directoryPath,
                               const bool isRec,
                               std::vector<std::string> &directories);

  static bool searchDirectoryWithFilter(const std::string &directoryPath,
                                        const bool isRec,
                                        std::vector<std::string> &directories,
                                        bool (*filter)(const std::string &));

  static std::string replaceRepeatSlash(const std::string &path);

  static bool checkHDiskFileSystem(const std::string &configPath,
                                   const std::string &fileSystemType,
                                   std::string &matchFileSystemType);

  static bool getFileSize(const std::string &filePath, uint64_t &fileSize);

public:
  static bool isFilterYamlFile(const std::string &filePath);

  static bool isFilterBagFile(const std::string &filePath);

  static bool isFilterRecordFile(const std::string &filePath);

  static bool isFilterRecordFileNoSegment(const std::string &filePath);

  static bool isRecordFileSegment(const std::string &filePath);

  static bool isRecordFileSegment00000(const std::string &filePath);

  static bool isFilterPcdFile(const std::string &filePath);

  static bool isFilterJpegFile(const std::string &filePath);

  static bool isFilterJpgFile(const std::string &filePath);

  static bool isFilterBgrFile(const std::string &filePath);

  static bool isFilterPackageDirDir(const std::string &dirPath);

  static bool isFilterPackageCameraDirDir(const std::string &dirPath);

public:
  static bool fromFilePathToTimestamp(const std::string &filePath,
                                      double &timestampS);

public:
  static bool
  fromFilePathToFileNameWithoutFormat(const std::string &filePath,
                                      std::string &fileNameWithoutFormat);

  static bool fromFilePathToFileNameWithFormat(const std::string &filePath,
                                               std::string &fileNameWithFormat);

  static bool fromFilePathToParentDirPath(const std::string &filePath,
                                          std::string &fileParentDirPath);

  static bool fromDirectoryPathToParentDirPath(const std::string &directoryPath,
                                               std::string &dirParentDirPath);

  static bool fromDirectoryPathToDirName(const std::string &directoryPath,
                                         std::string &directoryName);

public:
  static bool getDirectorySpace(const std::string &directoryPath,
                                double &total_space_gbyte,
                                double &avaiable_space_gbyte,
                                double &free_space_gbyte);

  static bool getDiskIsMount(const std::string &filePath);

  static bool mapperFilePathToMountInfo(const std::string &filePath,
                                        std::string &mountDeviceName,
                                        std::string &mountPointName);

  static std::vector<std::string> StrSplit(const std::string &content,
                                           const char spliter,
                                           const bool isSkipWhiteSpace);

  // Write
  static bool checkIsWriteWithAllRoles(const std::string &filePath);

  static bool checkIsWriteWithUsrRole(const std::string &filePath);

  static bool checkIsWriteWithGrpRole(const std::string &filePath);

  static bool checkIsWriteWithOthRole(const std::string &filePath);

  // Read
  static bool checkIsReadWithAllRoles(const std::string &filePath);

  static bool checkIsReadWithUsrRole(const std::string &filePath);

  static bool checkIsReadWithGrpRole(const std::string &filePath);

  static bool checkIsReadWithOthRole(const std::string &filePath);

  // Read & Write
  static bool checkIsReadWriteWithAllRole(const std::string &filePath);

  static bool checkIsReadWriteWithUsrRole(const std::string &filePath);

  static bool checkIsReadWriteWithGrpRole(const std::string &filePath);

  static bool checkIsReadWriteWithOthRole(const std::string &filePath);

  static bool checkFileRolesMode(const std::string &filePath,
                                 const std::vector<uint32_t> modes);

private:
  template <typename T> static T toValue(const std::string &value) {
    std::istringstream ifstr(value);
    T val;
    ifstr >> val;
    return val;
  }
};

} // namespace util
} // namespace filesystem
} // namespace robosense

#endif // RSFILESYSTEM_H