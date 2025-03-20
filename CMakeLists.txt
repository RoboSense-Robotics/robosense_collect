cmake_minimum_required(VERSION 3.5)
project(robosense_collect)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows supplies macros for min and max by default. We should only use min and max from stl
if(WIN32)
  add_definitions(-DNOMINMAX)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif()

# Detect ros version
if(DEFINED ENV{ROS_VERSION})
  set(ROS_VERSION $ENV{ROS_VERSION})
else()
  find_package(catkin QUIET)
  find_package(ament_cmake QUIET)

  if(catkin_FOUND)
    set(ROS_VERSION 1)
  elseif(ament_cmake_FOUND)
    set(ROS_VERSION 2)
  else()
    message(FATAL_ERROR "Can not detect ROS version automatically")
  endif()
endif()

message(STATUS "Building for ROS${ROS_VERSION}")

# Build for ROS1
if(ROS_VERSION EQUAL 1)
  add_compile_definitions(__ROS1__=1)
  add_compile_definitions(__ROS2__=0)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosbag
    std_msgs
    robosense_msgs
    tf2
    tf2_ros
  )
  find_package(yaml-cpp REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
      roscpp rosbag std_msgs robosense_msgs tf2 tf2_ros
  )

  include_directories(${catkin_INCLUDE_DIRS})

# Build for ROS2
elseif(ROS_VERSION EQUAL 2)
  add_compile_definitions(__ROS1__=0)
  add_compile_definitions(__ROS2__=1)

  find_package(ament_cmake REQUIRED)
  find_package(ament_cmake_ros REQUIRED)

  find_package(rcl REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rmw REQUIRED)
  find_package(rosbag2_compression REQUIRED)
  find_package(rosbag2_cpp REQUIRED)
  find_package(rosbag2_interfaces REQUIRED)
  find_package(rosbag2_storage REQUIRED)
  find_package(rosbag2_transport REQUIRED)
  find_package(rmw_implementation_cmake REQUIRED)
  find_package(keyboard_handler REQUIRED)
  find_package(shared_queues_vendor REQUIRED)
  find_package(yaml_cpp_vendor REQUIRED)

  find_package(std_msgs REQUIRED)
  find_package(robosense_msgs REQUIRED)

  # add macros of ros2 version
  message(STATUS "Using rclcpp version: [${rclcpp_VERSION}]")

  # parse version and add macros
  string(REPLACE "." ";" RCLCPP_VERSION_LIST ${rclcpp_VERSION})
  list(GET RCLCPP_VERSION_LIST 0 RCLCPP_VERSION_MAJOR)
  list(GET RCLCPP_VERSION_LIST 1 RCLCPP_VERSION_MINOR)
  list(GET RCLCPP_VERSION_LIST 2 RCLCPP_VERSION_PATCH)

  add_definitions(-DRCLCPP_VERSION_MAJOR=${RCLCPP_VERSION_MAJOR})
  add_definitions(-DRCLCPP_VERSION_MINOR=${RCLCPP_VERSION_MINOR})
  add_definitions(-DRCLCPP_VERSION_PATCH=${RCLCPP_VERSION_PATCH})

  # add distribution macros
  if(${RCLCPP_VERSION_MAJOR} LESS 9)
    add_definitions(-DROS2_FOXY)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 16)
    add_definitions(-DROS2_GALACTIC)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 20)
    add_definitions(-DROS2_HUMBLE)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 24)
    add_definitions(-DROS2_IRON)
  elseif(${RCLCPP_VERSION_MAJOR} LESS 29)
    add_definitions(-DROS2_JAZZY)
  else()
    add_definitions(-DROS2_ROLLING)
  endif()

endif()

# ---------------------------- Build Target ----------------------------
find_package(nlohmann_json REQUIRED)

set(PROJECT_SOURCES
  src/modules/rs_filesystem/rsfilesystem.cc
  src/modules/rs_collect/rsglobalconfig.cc
  src/modules/rs_collect/rsappcarmanager.cc
  src/modules/rs_collect/rsbase64.cc
  src/modules/rs_collect/rsversion.cc
  src/modules/rs_collect/rsmessagemonitor.cc
  src/modules/rs_collect/rscollectinfomanager.cc
  src/modules/rs_collect/rscollectmanager.cc
  src/modules/rs_collect/rsrecordiomanager.cc
  src/modules/rs_collect/rsrecordclipmanager.cc
  src/modules/rs_collect/rsrecordclipchannel.cc
  src/modules/rscollectnode.cc)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# ----------------------------------------------------------------------

# Alternatives
if(ROS_VERSION EQUAL 1)
  target_include_directories(${PROJECT_NAME} PUBLIC ${YAML_CPP_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
    nlohmann_json::nlohmann_json
  )

  install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
  install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  )
  install(DIRECTORY DEFAULT_CONFIG/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/DEFAULT_CONFIG
  )

elseif(ROS_VERSION EQUAL 2)
  ament_target_dependencies(${PROJECT_NAME}
    keyboard_handler
    rcl
    rclcpp
    rcutils
    rmw
    rosbag2_compression
    rosbag2_cpp
    rosbag2_interfaces
    rosbag2_storage
    rosbag2_transport
    shared_queues_vendor
    yaml_cpp_vendor
    std_msgs
    robosense_msgs
  )
  target_link_libraries(${PROJECT_NAME}
    nlohmann_json::nlohmann_json
  )

  install(TARGETS ${PROJECT_NAME} DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
  install(DIRECTORY DEFAULT_CONFIG DESTINATION share/${PROJECT_NAME}/)

  ament_package()
endif()
