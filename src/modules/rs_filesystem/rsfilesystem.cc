#include <fstream>

#include "modules/rs_filesystem/rsfilesystem.h"

namespace robosense {
namespace filesystem {
namespace util {

bool RSFileSystem::systemCmd(const std::string &cmd) {
  int status = system(cmd.c_str());
  if (status == -1) {
    return false;
  } else {
    if (WIFEXITED(status)) {
      if (WEXITSTATUS(status) == 0) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}

bool RSFileSystem::systemCmdReturnCode(const std::string &cmd,
                                       int32_t &ret_value) {
  int status = system(cmd.c_str());
  if (status == -1) {
    return false;
  } else {
    if (WIFEXITED(status)) {
      ret_value = WEXITSTATUS(status);
      if (ret_value == 0) {
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
}

bool RSFileSystem::removeFile(const std::string &filePath) {
  if (isFileExist(filePath)) {
    const std::string &cmd = "rm -f " + filePath;
    bool isSuccess = systemCmd(cmd);
    if (isSuccess == false) {
      return false;
    }
  }
  return true;
}

bool RSFileSystem::removeDirectory(const std::string &directoryPath) {
  if (isDirectoryExist(directoryPath)) {
    const std::string &cmd = "rm -rf " + directoryPath;
    bool isSuccess = systemCmd(cmd);
    if (isSuccess == false) {
      return false;
    }
  }
  return true;
}

bool RSFileSystem::mvFile(const std::string &oldFilePath,
                          const std::string &newFilePath, const bool isForce) {
  if (!RSFileSystem::isFileExist(oldFilePath)) {
    return false;
  }
  if (RSFileSystem::isFileExist(newFilePath) && isForce) {
    bool isSuccess = RSFileSystem::removeFile(newFilePath);
    if (!isSuccess) {
      return false;
    }
  } else if (RSFileSystem::isFileExist(newFilePath)) {
    return false;
  }

  const std::string &cmd = "mv " + oldFilePath + " " + newFilePath;
  bool isSuccess = systemCmd(cmd);
  if (!isSuccess) {
    return false;
  }

  return true;
}

bool RSFileSystem::isFileExist(const std::string &filePath) {
  int ret = access(filePath.c_str(), 0);

  if (ret != 0) {
    return false;
  }

  return true;
}

bool RSFileSystem::isDirectoryExist(const std::string &directoryPath) {
  return isFileExist(directoryPath);
}

bool RSFileSystem::makeDirectory(const std::string &directoryPath,
                                 const std::string &dirName) {
  if (isDirectoryExist(directoryPath + "/" + dirName)) { // 已经存在，则不在创建
    return true;
  }

  bool isSuccess = makePath(directoryPath + "/" + dirName);
  return isSuccess;
}

bool RSFileSystem::makePath(const std::string &directoryPath) {
  if (isDirectoryExist(directoryPath)) {
    return true;
  }

  const std::string &cmd = std::string("mkdir -p ") + directoryPath;

  return systemCmd(cmd);
}

bool RSFileSystem::searchFilesWithoutFilter(const std::string &directoryPath,
                                            const bool isRec,
                                            std::vector<std::string> &files) {
  return searchFilesWithFilter(directoryPath, isRec, files, nullptr);
}

bool RSFileSystem::searchFilesWithFilter(const std::string &directoryPath,
                                         const bool isRec,
                                         std::vector<std::string> &files,
                                         bool (*filter)(const std::string &)) {
  files.clear();

  DIR *dp = opendir(directoryPath.c_str());
  if (dp == NULL) {
    return false;
  }

  struct dirent *dirp;
  while ((dirp = readdir(dp)) != NULL) {
    const std::string &d_name = std::string(dirp->d_name);
    if (d_name == std::string(".") || d_name == std::string("..")) {
      continue;
    }

    if (dirp->d_type == DT_REG) { // 普通文件
      std::string file_path = directoryPath + "/" + d_name;
      bool isFilter = false;
      if (filter != nullptr) {
        isFilter = filter(file_path);
      }
      if (!isFilter) {
        // std::cout << "filter yaml: file_path = " << file_path << std::endl;
        files.push_back(file_path);
      }
    } else if (dirp->d_type == DT_DIR && isRec) { // 文件夹
      const std::string &dir_path = directoryPath + "/" + d_name + "/";
      std::vector<std::string> subFiles;
      bool isSuccess = searchFilesWithFilter(dir_path, isRec, subFiles, filter);
      if (isSuccess) {
        for (size_t i = 0; i < subFiles.size(); ++i) {
          files.push_back(subFiles[i]);
        }
      }
    }
  }
  closedir(dp);

  return true;
}

bool RSFileSystem::searchDirectoryWithoutFilter(
    const std::string &directoryPath, const bool isRec,
    std::vector<std::string> &directories) {
  return searchDirectoryWithFilter(directoryPath, isRec, directories, nullptr);
}

bool RSFileSystem::searchDirectoryWithFilter(
    const std::string &directoryPath, const bool isRec,
    std::vector<std::string> &directories,
    bool (*filter)(const std::string &)) {
  directories.clear();

  DIR *dp = opendir(directoryPath.c_str());
  if (dp == NULL) {
    return false;
  }

  struct dirent *dirp;
  while ((dirp = readdir(dp)) != NULL) {
    const std::string &d_name = std::string(dirp->d_name);
    if (d_name == std::string(".") || d_name == std::string("..")) {
      continue;
    }

    if (dirp->d_type == DT_DIR) { // 文件夹
      const std::string &dir_path = directoryPath + "/" + d_name + "/";
      bool isFilter = false;
      if (filter != nullptr) {
        isFilter = filter(dir_path);
      }

      if (!isFilter) {
        // std::cout << "filter directory: dir_path = " << dir_path <<
        // std::endl;
        directories.push_back(dir_path);
      }

      if (isRec) {
        std::vector<std::string> subDirectories;
        bool isSuccess =
            searchDirectoryWithFilter(dir_path, isRec, subDirectories, filter);
        if (isSuccess) {
          for (size_t i = 0; i < subDirectories.size(); ++i) {
            directories.push_back(subDirectories[i]);
          }
        }
      }
    }
  }
  closedir(dp);

  return true;
}

std::string RSFileSystem::replaceRepeatSlash(const std::string &path) {
  std::string newPath;
  for (size_t i = 0; i < path.size(); ++i) {
    const char ch = path[i];
    if (ch != '/') {
      newPath.push_back(ch);
    } else {
      newPath.push_back(ch);
      for (size_t j = i + 1; j < path.size(); ++j) {
        i = j;
        if (path[j] != '/') {
          newPath.push_back(path[j]);
          break;
        }
      }
    }
  }
  // std::cout << "newPath = " << newPath << ", path = " << path << std::endl;

  return newPath;
}

bool RSFileSystem::checkHDiskFileSystem(const std::string &configPath,
                                        const std::string &fileSystemType,
                                        std::string &matchFileSystemType) {
  char buf_ps[1024];
  char ps[1024] = "df -Th";

  FILE *ptr = NULL;
  const int type_no = 1;
  const int mount_no = 6;
  std::string matched_type_info;
  if ((ptr = popen(ps, "r")) != NULL) {
    bool isFirstLine = true;
    while (fgets(buf_ps, 1024, ptr) != NULL) {
      // AINFO << std::string(buf_ps);
      if (isFirstLine) {
        isFirstLine = false;
        continue;
      }

      const std::vector<std::string> mount_infos =
          StrSplit(std::string(buf_ps), ' ', true);
      if (mount_infos.size() != 7) {
        continue;
      }
      const std::string &type_info = mount_infos[type_no];
      std::string path_info = mount_infos[mount_no];

      while (path_info.size() > 0) {
        if (path_info[path_info.size() - 1] == '\n') {
          path_info.resize(path_info.size() - 1);
        } else {
          break;
        }
      }

      if (configPath.size() >= (int)path_info.size()) {
        if (configPath.substr(0, path_info.size()) == path_info) {
          matched_type_info = type_info;
        }
      }
      memset(buf_ps, '\0', sizeof(buf_ps));
    }
    fclose(ptr);
  }
  matchFileSystemType = matched_type_info;

  return (matched_type_info == fileSystemType);
}

bool RSFileSystem::getFileSize(const std::string &filePath,
                               uint64_t &fileSize) {
  std::ifstream input(filePath, std::ios_base::in | std::ios_base::binary);
  if (!input.is_open()) {
    return false;
  }

  std::streampos data_size = 0;
  input.seekg(0, std::ios::end); // 移动到文件末尾
  data_size = input.tellg();     // 获取文件大小
  input.seekg(0, std::ios::beg); // 移动回文件开头

  fileSize = data_size;

  return true;
}

bool RSFileSystem::isFilterYamlFile(const std::string &filePath) {
  const std::string &yamlFormat = ".yaml";
  if (filePath.size() > yamlFormat.size()) {
    // std::cout << "filePath = " << filePath << std::endl;
    // std::cout << "filePath.substr(filePath.size() - yamlFormat.size()) = "
    // << filePath.substr(filePath.size() - yamlFormat.size()) << std::endl;
    if (filePath.substr(filePath.size() - yamlFormat.size()) == yamlFormat) {
      return false;
    }
  }

  return true;
}

bool RSFileSystem::isFilterBagFile(const std::string &filePath) {
  const std::string &bagFormat = ".bag";
  if (filePath.size() > bagFormat.size()) {
    if (filePath.substr(filePath.size() - bagFormat.size()) == bagFormat) {
      return false;
    }
  }

  return true;
}

bool RSFileSystem::isFilterRecordFile(const std::string &filePath) {
  // 适配采集软件
  const std::string &bagFormat = ".record";
  if (filePath.size() > bagFormat.size()) {
    if (filePath.substr(filePath.size() - bagFormat.size()) == bagFormat) {
      return false;
    }
  }

  // 适配cyber_record场景
  const std::string &bagFormat2 = ".record.";
  if (filePath.size() > bagFormat2.size()) {
    size_t lastSlashPos = filePath.find_last_of('/');
    for (size_t i = lastSlashPos + 1; i < filePath.size(); ++i) {
      if (filePath.size() - i > bagFormat2.size()) {
        if (filePath.substr(i, bagFormat2.size()) == bagFormat2) {
          return false;
        }
      }
    }
  }

  return true;
}

bool RSFileSystem::isFilterRecordFileNoSegment(const std::string &filePath) {
  // 适配采集软件
  const std::string &bagFormat = ".record";
  if (filePath.size() > bagFormat.size()) {
    if (filePath.substr(filePath.size() - bagFormat.size()) == bagFormat) {
      return false;
    }
  }

  return true;
}

bool RSFileSystem::isRecordFileSegment(const std::string &filePath) {
  const std::string &bagFormat2 = ".record.";
  if (filePath.size() > bagFormat2.size()) {
    size_t lastSlashPos = filePath.find_last_of('/');
    for (size_t i = lastSlashPos + 1; i < filePath.size(); ++i) {
      if (filePath.size() - i > bagFormat2.size()) {
        if (filePath.substr(i, bagFormat2.size()) == bagFormat2) {
          return true;
        }
      }
    }
  }

  return false;
}

bool RSFileSystem::isRecordFileSegment00000(const std::string &filePath) {
  // 适配cyber_record场景
  const std::string &bagFormat2 = ".record.00000";
  if (filePath.size() > bagFormat2.size()) {
    size_t lastSlashPos = filePath.find_last_of('/');
    for (size_t i = lastSlashPos + 1; i < filePath.size(); ++i) {
      if (filePath.size() - i >= bagFormat2.size()) {
        if (filePath.substr(i, bagFormat2.size()) == bagFormat2) {
          return true;
        }
      }
    }
  }
  return false;
}

bool RSFileSystem::isFilterPcdFile(const std::string &filePath) {
  const std::string &pcdFormat = ".pcd";
  if (filePath.size() > pcdFormat.size()) {
    if (filePath.substr(filePath.size() - pcdFormat.size()) == pcdFormat) {
      return false;
    }
  }

  return true;
}

bool RSFileSystem::isFilterJpegFile(const std::string &filePath) {
  const std::string &jpegFormat = ".jpeg";
  if (filePath.size() > jpegFormat.size()) {
    if (filePath.substr(filePath.size() - jpegFormat.size()) == jpegFormat) {
      return false;
    }
  }
  return true;
}

bool RSFileSystem::isFilterJpgFile(const std::string &filePath) {
  const std::string &jpgFormat = ".jpg";
  if (filePath.size() > jpgFormat.size()) {
    if (filePath.substr(filePath.size() - jpgFormat.size()) == jpgFormat) {
      return false;
    }
  }
  return true;
}

bool RSFileSystem::isFilterBgrFile(const std::string &filePath) {
  const std::string &bgrFormat = ".bgr";
  if (filePath.size() > bgrFormat.size()) {
    if (filePath.substr(filePath.size() - bgrFormat.size()) == bgrFormat) {
      return false;
    }
  }
  return true;
}

bool RSFileSystem::isFilterPackageDirDir(const std::string &dirPath) {
  if (RSFileSystem::isDirectoryExist(dirPath + "/lidar") &&
      RSFileSystem::isDirectoryExist(dirPath + "/camera")) {
    return false;
  }
  return true;
}

bool RSFileSystem::isFilterPackageCameraDirDir(const std::string &dirPath) {
  if (RSFileSystem::isDirectoryExist(dirPath + "/camera")) {
    return false;
  }
  return true;
}

bool RSFileSystem::fromFilePathToTimestamp(const std::string &filePath,
                                           double &timestampS) {
  size_t lastSlashPos = filePath.find_last_of('/');
  size_t lastDotPos = filePath.find_last_of('.');
  if (lastSlashPos != std::string::npos && lastDotPos != std::string::npos) {
    std::string sTimestampNs =
        filePath.substr(lastSlashPos + 1, (lastDotPos - lastSlashPos));
    uint64_t timestampNs = toValue<uint64_t>(sTimestampNs);
    timestampS = timestampNs * 1e-9;
    return true;
  }
  return false;
}

bool RSFileSystem::fromFilePathToFileNameWithoutFormat(
    const std::string &filePath, std::string &fileNameWithoutFormat) {
  size_t lastSlashPos = filePath.find_last_of('/');
  size_t lastDotPos = std::string::npos;
  if (lastSlashPos != std::string::npos) {
    lastDotPos = filePath.find_first_of('.', lastSlashPos);
  }
  if (lastSlashPos != std::string::npos && lastDotPos != std::string::npos) {
    fileNameWithoutFormat =
        filePath.substr(lastSlashPos + 1, (lastDotPos - lastSlashPos - 1));
    return true;
  }
  return false;
}

bool RSFileSystem::fromFilePathToFileNameWithFormat(
    const std::string &filePath, std::string &fileNameWithFormat) {
  size_t lastSlashPos = filePath.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    fileNameWithFormat = filePath.substr(lastSlashPos + 1);
    return true;
  }
  return false;
}

bool RSFileSystem::fromFilePathToParentDirPath(const std::string &filePath,
                                               std::string &fileParentDirPath) {
  fileParentDirPath.clear();
  size_t lastSlashPos = filePath.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    fileParentDirPath = filePath.substr(0, lastSlashPos);
    return true;
  }

  return false;
}

bool RSFileSystem::fromDirectoryPathToParentDirPath(
    const std::string &directoryPath, std::string &dirParentDirPath) {
  dirParentDirPath.clear();
  std::string directoryPath2 = directoryPath;
  while (directoryPath2.size() > 0) {
    if (directoryPath2[directoryPath2.size() - 1] != '/') {
      break;
    }
    directoryPath2.resize(directoryPath2.size() - 1);
  }

  size_t lastSlashPos = directoryPath2.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    dirParentDirPath = directoryPath2.substr(0, lastSlashPos);
    return true;
  }

  return false;
}

bool RSFileSystem::fromDirectoryPathToDirName(const std::string &directoryPath,
                                              std::string &directoryName) {
  directoryName.clear();
  // 排除最后的无效'/'
  std::string newDirectoryPath = directoryPath;
  while (newDirectoryPath.size() > 0) {
    if (newDirectoryPath[newDirectoryPath.size() - 1] != '/') {
      break;
    }
    newDirectoryPath.resize(newDirectoryPath.size() - 1);
  }

  size_t lastSlashPos = newDirectoryPath.find_last_of('/');
  if (lastSlashPos != std::string::npos) {
    directoryName = newDirectoryPath.substr(lastSlashPos + 1);
    return true;
  }

  return false;
}

bool RSFileSystem::getDirectorySpace(const std::string &directoryPath,
                                     double &total_space_gbyte,
                                     double &avaiable_space_gbyte,
                                     double &free_space_gbyte) {
  struct statvfs64 diskInfo;

  int ret = ::statvfs64(directoryPath.c_str(), &diskInfo);
  if (ret != 0) {
    return false;
  }

  unsigned long long int blocksize = diskInfo.f_frsize;
  unsigned long long int totalsize = blocksize * diskInfo.f_blocks;
  unsigned long long int availsize = blocksize * diskInfo.f_bavail;
  unsigned long long int freesize = blocksize * diskInfo.f_bfree;

  total_space_gbyte = totalsize / 1000.0 / 1000.0 / 1000.0;
  avaiable_space_gbyte = availsize / 1000.0 / 1000.0 / 1000.0;
  free_space_gbyte = freesize / 1000.0 / 1000.0 / 1000.0;

  return true;
}

bool RSFileSystem::getDiskIsMount(const std::string &filePath) {
  char buf_ps[1024];
  char ps[1024] = "df -h";

  FILE *ptr = NULL;
  bool isMount = false;
  const int filesystem_no = 0;
  const int mount_no = 5;
  if ((ptr = popen(ps, "r")) != NULL) {
    bool isFirstLine = true;
    while (fgets(buf_ps, 1024, ptr) != NULL) {
      // AINFO << std::string(buf_ps);
      if (isFirstLine) {
        isFirstLine = false;
        continue;
      }

      const std::vector<std::string> mount_infos =
          StrSplit(std::string(buf_ps), ' ', true);

      // for (size_t i = 0; i < mount_infos.size(); ++i) {
      //   std::cout << "mount_infos[" << i << "] = " << mount_infos[i]
      //             << std::endl;
      // }

      if (mount_infos.size() < mount_no) {
        continue;
      }
      // std::cout << "mount_infos[mount_no] = " << mount_infos[mount_no]
      //           << ", filePath = " << filePath
      //           << ", mount_infos[mount_no] size = "
      //           << mount_infos[mount_no].size()
      //           << ", filePath size = " << filePath.size() << std::endl;
      if (mount_infos[mount_no].size() >= filePath.size()) {
        if (mount_infos[mount_no].substr(0, filePath.size()) == filePath) {
          // std::cout << "filePath: " << filePath << " is Mount !" <<
          // std::endl;
          isMount = true;
        }
      }
      memset(buf_ps, '\0', sizeof(buf_ps));
    }
    fclose(ptr);
  }

  return isMount;
}

bool RSFileSystem::mapperFilePathToMountInfo(const std::string &filePath,
                                             std::string &mountDeviceName,
                                             std::string &mountPointName) {
  mountDeviceName.clear();
  mountPointName.clear();
  char buf_ps[1024];
  char ps[1024] = "df -h";

  FILE *ptr = NULL;
  bool isMount = false;
  const int filesystem_no = 0;
  const int mount_no = 5;
  if ((ptr = popen(ps, "r")) != NULL) {
    bool isFirstLine = true;
    while (fgets(buf_ps, 1024, ptr) != NULL) {
      // AINFO << std::string(buf_ps);
      if (isFirstLine) {
        isFirstLine = false;
        continue;
      }

      const std::vector<std::string> mount_infos =
          StrSplit(std::string(buf_ps), ' ', true);

      // for (size_t i = 0; i < mount_infos.size(); ++i) {
      //   std::cout << "mount_infos[" << i << "] = " << mount_infos[i]
      //             << std::endl;
      // }

      if (mount_infos.size() < mount_no) {
        continue;
      }
      // std::cout << "mount_infos[mount_no] = " << mount_infos[mount_no]
      //           << ", filePath = " << filePath
      //           << ", mount_infos[mount_no] size = "
      //           << mount_infos[mount_no].size()
      //           << ", filePath size = " << filePath.size() << std::endl;
      std::string mount_point_name = mount_infos[mount_no];
      do {
        const char ch = mount_point_name[mount_point_name.size() - 1];
        if (ch == '\n' || std::isblank(ch)) {
          mount_point_name.resize(mount_point_name.size() - 1);
        } else {
          break;
        }
      } while (!mount_point_name.empty());

      if (mount_point_name.size() <= filePath.size()) {
        if (filePath.substr(0, mount_point_name.size()) == mount_point_name) {
          mountDeviceName = mount_infos[filesystem_no];
          // 匹配最长的
          if (mount_point_name.size() > mountPointName.size()) {
            mountPointName = mount_point_name;
            std::cout << "filePath: " << filePath
                      << ", mountDeviceName = " << mountDeviceName
                      << ", mountPointName = " << mountPointName << std::endl;
          }
          isMount = true;
        }
      }
      memset(buf_ps, '\0', sizeof(buf_ps));
    }
    fclose(ptr);
  }

  return isMount;
}

std::vector<std::string> RSFileSystem::StrSplit(const std::string &content,
                                                const char spliter,
                                                const bool isSkipWhiteSpace) {
  std::vector<std::string> contents;
  std::string sub_content;
  for (size_t i = 0; i < content.size(); ++i) {
    const char ch = content[i];
    if (ch == spliter) {
      if (isSkipWhiteSpace) {
        if (sub_content.size()) {
          do {
            if (!std::isspace(sub_content[0])) {
              break;
            } else {
              sub_content.erase(sub_content.begin());
            }
          } while (!sub_content.empty());
        }
        if (!sub_content.empty()) {
          contents.push_back(sub_content);
        }
      } else {
        contents.push_back(sub_content);
      }
    } else {
      sub_content.push_back(ch);
    }
  }

  if (!sub_content.empty()) {
    if (isSkipWhiteSpace) {
      if (sub_content.size()) {
        do {
          if (!std::isspace(sub_content[0])) {
            break;
          } else {
            sub_content.erase(sub_content.begin());
          }
        } while (!sub_content.empty());
      }
      if (!sub_content.empty()) {
        contents.push_back(sub_content);
      }
    } else {
      contents.push_back(sub_content);
    }
  }

  return contents;
}

bool RSFileSystem::checkIsWriteWithAllRoles(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWUSR, S_IWGRP, S_IWOTH});
}

bool RSFileSystem::checkIsWriteWithUsrRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWUSR});
}

bool RSFileSystem::checkIsWriteWithGrpRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWGRP});
}

bool RSFileSystem::checkIsWriteWithOthRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWOTH});
}

bool RSFileSystem::checkIsReadWithAllRoles(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IRUSR, S_IRGRP, S_IROTH});
}

bool RSFileSystem::checkIsReadWithUsrRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IRUSR});
}

bool RSFileSystem::checkIsReadWithGrpRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IRGRP});
}

bool RSFileSystem::checkIsReadWithOthRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IROTH});
}

bool RSFileSystem::checkIsReadWriteWithAllRole(const std::string &filePath) {
  return checkFileRolesMode(
      filePath, {S_IWUSR, S_IWGRP, S_IWOTH, S_IRUSR, S_IRGRP, S_IROTH});
}

bool RSFileSystem::checkIsReadWriteWithUsrRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWUSR, S_IRUSR});
}

bool RSFileSystem::checkIsReadWriteWithGrpRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWGRP, S_IRGRP});
}

bool RSFileSystem::checkIsReadWriteWithOthRole(const std::string &filePath) {
  return checkFileRolesMode(filePath, {S_IWOTH, S_IROTH});
}

bool RSFileSystem::checkFileRolesMode(const std::string &filePath,
                                      const std::vector<uint32_t> modes) {
  struct stat file_stat;
  if (stat(filePath.c_str(), &file_stat) < 0) {
    return false;
  }
  bool isMatch = true;
  for (size_t i = 0; i < modes.size(); ++i) {
    isMatch = isMatch && (file_stat.st_mode & modes[i]);
  }

  return isMatch;
}

} // namespace util
} // namespace filesystem
} // namespace robosense