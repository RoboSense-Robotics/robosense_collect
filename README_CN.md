# robosense_collect 

[README](./README.md)|[中文文档](README_CN.md)

## 1. 简介

​	为了能够采集超级传感器以及超级传感器SDK的数据，所以开发了基于YAML文件配置的**robosense_collect**节点实现数据采集功能；**该采集节点通过在编译时检查系统中ROS环境自动编译为ROS/ROS2版本节点**，如下ROS/ROS2节点支持功能简单介绍：  

- ROS节点时支持如下功能: 
  - 基于时间自动分段/不分段两种模式
  - 支持bag格式

- ROS2节点时支持如下功能
  - 基于时间自动分段/不分段两种模式
  - 支持.db3 和.mcap格式两种格式
  - 支持zstd压缩和非压缩两种模式

关于**robosense_collect**节点的配置，详细见**第5节配置说明**



## 2. 前置依赖

### 2.1 编译ROS版本节点前置依赖

- 安装ROS noetic 或其他可支持的版本： 根据您的操作系统选择 [官方教程](http://wiki.ros.org/ROS/Installation) 中的指定内容进行执行

- 编译安装前置依赖包robosense_msgs: 

  - 从我们的开放代码仓库拉取源码

    ```bash
    git clone https://github.com/RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
    ```

  - 通过 `catkin_make` 工具进行编译安装

    ```bash
    catkin_make --only-pkt-with-deps robosense_msgs
    ```

  - 更新工作空间中的 ROS 包元信息

    ```bash
    source devel/setup.bash 或 source devel/setup.zsh 
    ```

  - 使用 `rosdep` 工具安装 `package.xml` 中定义的依赖

    ```sh
    rosdep install -y --from-paths /PATH/TO/ROBOSENSE_COLLECT --ignore-src
    ```

### 2.2 编译ROS2版本节点前置依赖

- 安装 ROS2 **humble** 或其他可支持的版本: 根据您的操作系统选择 [官方教程](https://fishros.org/doc/ros2/humble/Installation.html) 中的指定内容进行执行

- 编译安装前置依赖包 **robosense_msgs**:

  - 从我们的开放代码仓库拉取源码

  ```bash
  git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
  ```

  - 通过 `colcon` 工具进行编译安装

  ```bash
  colcon build --symlink-install --packages-select robosense_msgs
  ```

  - 更新工作空间中的 ROS 包元信息

  ```bash
  source install/setup.bash 或 source install/setup.zsh 
  ```

  - 使用 `rosdep` 工具安装 `package.xml` 中定义的依赖

  ```sh
  rosdep install -y --from-paths /PATH/TO/ROBOSENSE_COLLECT --ignore-src
  ```



## 3. 编译robosense_collect

### 3.1 编译ROS版本节点 

- 步骤1: 打开终端切换工作路径到包含**robosense_collect**包的路径

- 步骤2: 执行ROS2包编译命令: 

  ```shell
  catkin_make --only-pkt-with-deps robosense_collect
  ```

等待编译完成即可 

### 3.2 编译ROS2版本节点

- 步骤1: 打开终端切换工作路径到包含**robosense_collect**包的路径

- 步骤2: 执行ROS2包编译命令: 

  ```shell
  colcon build --symlink-install --packages-select robosense_collect
  ```

等待编译完成即可 

## 4. 运行robosense_collect

### 4.1 运行ROS版本节点

- 步骤1：对于编译好的包，执行source命令设置节点运行环境: 对于bash解释器终端和zsh解析器终端分别运行如下对应命令

  ```shell
  source devel/setup.bash 
  ```

- 步骤2： 启动节点: 

  ```shell
  roslaunch robosense_collect robosense_collect.launch 
  ```

节点启动后，即自动开始数据采集到指定的目录。 

### 4.2 运行ROS2版本节点

- 步骤1: 对编译好的包，执行source命令设置节点运行环境: 

  ```sh
  source install/setup.bash 或 source install/setup.zsh 
  ```

- 步骤2: 启动节点: 

  ```shell
  ros2 launch robosense_collect robosense_collect.py
  ```

节点启动后，即自动开始数据采集到指定的目录。 

**注意: 如果在编译节点后，需要修改配置文件，那么，在修改完成后需要再次编译包以使配置文件能够生效。** 

## 5.  配置文件字段说明 

​	 **robosense_collect**的配置文件在包的相对路径为: **DEFAULT_CONFIG/conf/collect.yaml**，该节点中包含默认的配置，使用者根据自己数据采集的需要进行配置拓展，**特别是下表中标记为黑色粗体的字段**

| 一级字段名称           | 二级字段名称                      | 三级字段名称         | 类型     | 说明                                                         |
| ---------------------- | --------------------------------- | -------------------- | -------- | ------------------------------------------------------------ |
| recordappcarconfig     | timeout_ms                        |                      | uint32_t | 在App控制模式生效: 同App通信超时阈值，默认为604800000，单位ms |
|                        | heartbeat_ms                      |                      | uint32_t | 在App控制模式生效: 同App心跳通信的周期，默认为5000，单位ms   |
|                        | enable_debug                      |                      | bool     | 在App控制模式生效: 是否开启调试模式，默认为true              |
| recordclipdirconfig    | clip_sub_directory_names          |                      | string[] | 每个clip中需要生成的子目录名称，默认包含如下子目录: "lidar", "camera",  "localization", "perception", "pnc", "prediction", "control" |
| **recordsensorconfig** | record_sensors[]                  | sensor_type          | string   | 支持两种类型:  "RS_RECORD_DATA_ANY": 表示非H265格式数据；“RS_RECORD_DATA_H265”: 表示H265格式消息数据， **当配置消息时，将会进行消息解析，并按照消息的I/P帧类型，保证在进行分段采集时，每个分段以I帧开头，但是，将增加资源消耗。** |
|                        |                                   | enable_topic         | bool     | 表示是否采集该话题                                           |
|                        |                                   | topic_name           | string   | 需要采集的话题名称                                           |
|                        |                                   | clip_rel_directory   | string   | 该话题保存到的子目录名称                                     |
|                        |                                   | record_file_name     | string   | 该话题保存到的ros2文件名称                                   |
|                        |                                   | enable_timeout_check | bool     | 表示是否启用接收超时检查                                     |
|                        |                                   | timeout_warn_th_ms   | uint32_t | 接收话题数据超时告警阈值，单位ms                             |
|                        |                                   | timeout_error_th_ms  | uint32_t | 接收话题数据超时错误阈值，单位ms                             |
| recorddataconfig       | **segment_type**                  |                      | string   | 支持两种类型: \n "RS_RECORD_SEGMENT_BY_TIME": 表示按照采集时长分段，分段阈值为segment_time_th_ms的取值; "RS_RECROD_SEGMENT_NO_SEG": 表示采集不进行分段 |
|                        | **segment_time_th_ms**            |                      | uint32_t | 采集分段时，数据分段阈值，单位ms                             |
|                        | serialization_format              |                      | string   | 固定取值为"cdr"即可                                          |
|                        | storage_id                        |                      | string   | 支持"sqlite3"和"mcap"，默认为"sqlite3"                       |
|                        | enable_compression                |                      | bool     | 是否开启压缩功能，默认为false                                |
|                        | compression_mode                  |                      | string   | 支持“MESSAGE"/"FILE", 默认为"MESSAGE", 当storage_id=“mcap”时，只能取值为"FILE" |
|                        | compression_format                |                      | string   | 支持"fake_comp"和"zstd", 默认为"zstd", **实际数据测试，设置为"fake_comp"时，数据压缩不明显，不推荐使用** |
|                        | compression_quene_size            |                      | uint32_t | 压缩队列大小，默认值为50                                     |
|                        | compression_thread_cnt            |                      | uint32_t | 压缩线程个数，默认值为1                                      |
|                        | enable_clip_erase_timeout_debug   |                      | bool     | 表示是否进行clip写超时测试，默认为false                      |
|                        | clip_erase_timeout_debug_count    |                      | uint32_t | 进行clip写超时测试时，触发超时的次数                         |
|                        | rdcs_root_directory_path          |                      | string   | 数据保存的路径，必须以"RDCS_ROOT/"结尾，例如"/media/sti/data/RDCS_ROOT/" |
|                        | enable_collect_freespace_monitor  |                      | bool     | 是否进行保存硬盘空间检查                                     |
|                        | collect_freespace_min_gbyte       |                      | uint32_t | 保存硬盘最小可用空间大小，单位GByte                          |
|                        | enable_collect_disk_mount_monitor |                      | bool     | 是否进行保存路径挂载检查，默认为true                         |
|                        | collect_disk_write_check_modes    |                      | string[] | 进行保存硬盘读写属性检查，支持“robosense_collect_DISK_WRITE_CHECK_USR”，“robosense_collect_DISK_WRITE_CHECK_OTH”, "robosense_collect_DISK_WRITE_CHECK_GRP" 默认为"robosense_collect_DISK_WRITE_CHECK_USR"，表示对当前用户检查是否有读写权限 |
|                        | enable_disk_mount_retry_debug     |                      | bool     | 是否进行硬盘重新挂载调试，默认为false                        |
|                        | enable_disk_mount_retry           |                      | bool     | 当检查到硬盘umount后，是否进行重新挂载                       |
|                        | max_disk_mount_retry_cnt          |                      | uint32_t | 当检查到硬盘umount后，进行重新挂载尝试的次数                 |
|                        | encoder_type                      |                      | string   | 相机数据保存的格式，默认为“H265”                             |
| recordmetaconfig       | **template_meta_directory_path**  |                      | string   | 表示Meta文件搜索目录                                         |
| recordsystemconfig     | vehicle_id                        |                      | string   | 在非App控制模式生效，进行META文件匹配的车辆匹配ID号          |
|                        | clip_id                           |                      | uint32_t | 在非App控制模式生效，开始采集的起始CLIP ID值，默认为0        |
|                        | task_name                         |                      | string   | 在非App控制模式生效，采集的任务名称                          |
|                        | task_name_add_timestamp           |                      | bool     | 在非App控制模式生效，采集的任务名称中，是否默认增加时间戳后缀，默认true |
| recordcollectconfig    | is_auto_collect                   |                      | bool     | 是否为非App控制模式，默认为true                              |
|                        | is_use_sim                        |                      | bool     | 是否为模拟时钟模式，默认为false                              |
|                        | is_discovery_disabled             |                      | bool     | 是否进行服务发现，以监听全部需求的话题                       |

