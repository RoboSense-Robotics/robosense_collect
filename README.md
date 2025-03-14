# ros2_collect 

[README](./README.md)|[中文文档](README_CN.md)

## 1. Overview

​	In order to collect the super sensor's original data and application data, we offer this node to collect data,  this node is base on YAML setting file. This Package support blow functions: 

-  auto segment by time / no segment 
- .db3 / .mcap  format 
- zstd compression / no compression 

For more detail about setting, You Reference To **5. Node's Setting**



## 2. Prerequisites 

- Install Ros2  **humble** or others Version: Please refer to the official [ROS 2 installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) guidance

- Build pre-requisites package **robosense_msgs**

  - Fetch code from our open repository to your workspace

    ```bash
    # https
    git clone http://gitlab.robosense.cn/super_sensor_sdk/ros2_sdk/sdk_infra.git
    # OR ssh
    git clone git@gitlab.robosense.cn:super_sensor_sdk/ros2_sdk/sdk_infra.git
    ```

  - Build through `colcon`

    ```bash
    colcon build --symlink-install --packages-select robosense_msgs
    ```

  - Update the metadata of ros packages

    ```bash
    source install/setup.bash
    ```

- Install dependency defined in `package.xml`

  ```shell
  rosdep install -y --from-paths /PATH/TO/RS_COLLECT --ignore-src
  ```

  

## 3. Build 

- Step1: Open a teminal And change the work path to this node root directory 

- Step2: Run ros2 build command: 

  ```shell
  colcon build --symlink-install --packages-select ros2_collect
  ```

  and waiting for finish. 

  

## 4. Run 

- Step1: Setup the node's environment by source command: 

  - `source install/setup.bash`(For Bash Terminal) 
  - `source install/setup.zsh`(For Zsh Termial) 

- Step2: Run ros2_collect node: 

  ```shell
  ros2 launch ros2_collect ros2_collect.launch.py
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
|                        | **rdcs_root_directory_path**      |                      | string   | data save directory，the setting must end with "RDCS_ROOT/"，<br />For example: "/media/sti/data/RDCS_ROOT/" |
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

