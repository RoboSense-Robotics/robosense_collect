# robosense_collect 

[README](./README.md)|[中文文档](README_CN.md)

## 1. Overview

​	In order to collect data from super sensors and super sensor SDKs, a **robosense_collect** node based on YAML file configuration was developed to implement data collection functionality; **This collection node is automatically compiled into ROS/ROS 2 version nodes by checking the ROS environment in the system during compilation**, The following is a brief introduction to the supported functions of ROS/ROS 2 nodes:

- The ROS Version node support the following functions:
  - Two modes based on time automatic segmentation/non segmentation
  - Supports .bag format
- The ROS2 Version node supports the following functions
  - Two modes based on time automatic segmentation/non segmentation
  - Supports two formats: .db3 and .mcap
  - Supports both zstd compression and non compression modes

Regarding the configuration of the **robosense_collect** node, please refer to Section 5 for detailed configuration instructions



## 2. Prerequisites 

### 2.1 Setup ROS version node pre dependencies

- Install ROS noetic or other supported versions: Follow the instructions in the official tutorial  [Office Document](http://wiki.ros.org/ROS/Installation)  based on your operating system

- Build And Install package robosense_msgs: 

  - Pull source code from our open code repository

    ```bash
    mkdir src 
    cd src 
    git clone https://github.com/RoboSense-Robotics/robosense_ac_ros_sdk_infra.git
    ```

  -  Using `catkin_make`  tool build 

    ```bash
    catkin_make --only-pkg-with-deps robosense_msgs
    ```

  - Update Ros package meta environment by source command  

    ```bash
    source devel/setup.bash or source devel/setup.zsh 
    ```

- Install dependencies defined in `package.xml` using the `rosdep` tool

  ```shell
  rosdep install -y --from-paths /PATH/TO/ROBOSENSE_COLLECT --ignore-src
  ```

### 2.2 Setup ROS2 version node pre dependencies

- Install ROS2 **humble** or other supported versions: Follow the instructions in the official tutorial  [Office Document](https://fishros.org/doc/ros2/humble/Installation.html)  based on your operating system

- Build And Install package robosense_msgs: 

  - Pull source code from our open code repository

    ```bash
    git clone https://github.com/RoboSense-Robotics/robosense_ac_ros2_sdk_infra.git
    ```

  -  Using `colcon`  tool build 

    ```bash
    colcon build --symlink-install --packages-select robosense_msgs
    ```

  - Update Ros package meta environment by source command  

    ```bash
    source install/setup.bash or source install/setup.zsh 
    ```

- Install dependencies defined in `package.xml` using the `rosdep` tool

  ```sh
  rosdep install -y --from-paths /PATH/TO/ROBOSENSE_COLLECT --ignore-src
  ```

## 3. Build 

- ### 3.1 Compile ROS version node 

  - Open a new terminal and change the work directory path which include the **robosense_collect** directory 

  - Using `catkin_make`  tool to build this node 

    ```shell
    catkin_make --only-pkg-with-deps robosense_collect
    ```

  waiting for build finish!

  ### 3.2 Compile ROS2 version node

  - Open a new terminal and change the work directory path which include the **robosense_collect** directory 

  - Using `colcon`  tool to build this node  

    ```shell
    colcon build --symlink-install --packages-select robosense_collect
    ```

  waiting for build finish!

## 4. Run 

### 4.1 Run Ros Version node 

- Setup the node's environment by source command: 

  - `source devel/setup.bash`(For Bash Terminal) 
  - `source devel/setup.zsh`(For Zsh Termial) 

- Run robosense_collect node: 

  ```shell
  roslaunch robosense_collect robosense_collect.launch 
  ```

### 4.2 Run Ros2 Version node 

- Setup the node's environment by source command: 

  - `source install/setup.bash`(For Bash Terminal) 
  - `source install/setup.zsh`(For Zsh Termial) 

- Run robosense_collect node: 

  ```shell
  ros2 launch robosense_collect robosense_collect.py
  ```

  **Notice: If you modified the setting yaml file before run the node, you should build this package again** 

## 5.  Node's Setting 

​	The node's setting file reletive to package at **DEFAULT_CONFIG/conf/collect.yaml**, the user can modify the setting if need, and the blow table is the description of the setting items, the black bold items in the table which need set firstly. 	

| The First Level Item   | The Second Level Item             | The Third Level Item | Type     | Discription                                                  |
| ---------------------- | --------------------------------- | -------------------- | -------- | ------------------------------------------------------------ |
| recordappcarconfig     | timeout_ms                        |                      | uint32_t | Effective in App Control Mode: communication timeout threshold with App, default:  604800000，unit: ms |
|                        | heartbeat_ms                      |                      | uint32_t | Effective in App Control Mode: heartbeat timeout threshold with App, default: 5000，unit: ms |
|                        | enable_debug                      |                      | bool     | Effective in App Control Mode: whether enable debug information output, default: true |
| recordclipdirconfig    | clip_sub_directory_names          |                      | string[] | sub-directory name in clip                                   |
| **recordsensorconfig** | record_sensors[]                  | sensor_type          | string   | support: <br />"RS_RECORD_DATA_ANY": means this topic data is not h265 format data<br />“RS_RECORD_DATA_H265”: means this topic data is h265 format data，<br /> |
|                        |                                   | enable_topic         | bool     | whether enable this topic to collect, default: false         |
|                        |                                   | topic_name           | string   | topic name which need collect                                |
|                        |                                   | clip_rel_directory   | string   | topic message data save sub-directory name                   |
|                        |                                   | record_file_name     | string   | topic message save file name                                 |
|                        |                                   | enable_timeout_check | bool     | whether enable timeout check, default: false                 |
|                        |                                   | timeout_warn_th_ms   | uint32_t | warning level timeout check threshold，unit: ms              |
|                        |                                   | timeout_error_th_ms  | uint32_t | error level timeout check threshold，unit: ms                |
| recorddataconfig       | **segment_type**                  |                      | string   | support two types: (a) "RS_RECORD_SEGMENT_BY_TIME": means data collection will auto segment by time, the time threshold is "segment_time_th_ms" "RS_RECROD_SEGMENT_NO_SEG":  means data collection don't segment, default |
|                        | **segment_time_th_ms**            |                      | uint32_t | auto segment by time threshold，unit: ms                     |
|                        | serialization_format              |                      | string   | Fixed: "cdr"                                                 |
|                        | storage_id                        |                      | string   | support two types:  <br />(a)"sqlite3", default <br />(b)"mcap" |
|                        | enable_compression                |                      | bool     | whether enable compression, default: false                   |
|                        | compression_mode                  |                      | string   | support two types: <br />(a)  “MESSAGE", default<br />(b) "FILE"<br />**Notice:** If **storage_id** is"mcap", the **compress_mode** must "FILE" |
|                        | compression_format                |                      | string   | support two types: <br />(a) "fake_comp"<br />(b) "zstd",  default<br />**Notice:** We are not suggest to use "fake_comp", Because of this compress format seem don't compress data |
|                        | compression_quene_size            |                      | uint32_t | compression queue size, default: 50                          |
|                        | compression_thread_cnt            |                      | uint32_t | compression thread count, default: 1                         |
|                        | enable_clip_erase_timeout_debug   |                      | bool     | whether enable clip write timeout debug mode，default: false |
|                        | clip_erase_timeout_debug_count    |                      | uint32_t | clip write timeout debug mode trigger count, default: 2      |
|                        | rdcs_root_directory_path          |                      | string   | data save directory，the setting must end with "RDCS_ROOT/"，<br />For example: "/media/sti/data/RDCS_ROOT/" |
|                        | enable_collect_freespace_monitor  |                      | bool     | whether enable collection disk freespace check, default:  true |
|                        | collect_freespace_min_gbyte       |                      | uint32_t | the collection disk's minimal freespace size，default: 32, unit: GByte |
|                        | enable_collect_disk_mount_monitor |                      | bool     | whether enable collection disk mount check，default: true    |
|                        | collect_disk_write_check_modes    |                      | string[] | collection disk's user/other/group write property check, support three types: <br />(a) “RS_COLLECT_DISK_WRITE_CHECK_USR”, default<br />(b) “RS_COLLECT_DISK_WRITE_CHECK_OTH”<br /> (c) "RS_COLLECT_DISK_WRITE_CHECK_GRP" |
|                        | enable_disk_mount_retry_debug     |                      | bool     | whether enable remount disk debug mode，default: false       |
|                        | enable_disk_mount_retry           |                      | bool     | whether enable remount disk, default: false                  |
|                        | max_disk_mount_retry_cnt          |                      | uint32_t | remount disk retry count, default: 2                         |
|                        | encoder_type                      |                      | string   | camera data encode type，default: “H265”                     |
| recordmetaconfig       | template_meta_directory_path      |                      | string   | meta file search directory path                              |
| recordsystemconfig     | vehicle_id                        |                      | string   | vehicle id                                                   |
|                        | clip_id                           |                      | uint32_t | clip start number, default: 0                                |
|                        | task_name                         |                      | string   | collection task name, default: ""                            |
|                        | task_name_add_timestamp           |                      | bool     | whether use timestamp string as a part of task name, default: true |
| recordcollectconfig    | is_auto_collect                   |                      | bool     | whether is Non-App mode，default: true                       |
|                        | is_use_sim                        |                      | bool     | whether use simulated clock, default: false                  |
|                        | is_discovery_disabled             |                      | bool     | whether use disable topic discovery, default: false          |

