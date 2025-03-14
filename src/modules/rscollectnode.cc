#include "modules/rs_collect/rscollectmanager.h"

#if __ROS2__
class RSCollectNode : public rclcpp::Node {
#elif __ROS1__
class RSCollectNode {
#endif
public:
  using Ptr = std::shared_ptr<RSCollectNode>;
  using ConstPtr = std::shared_ptr<const RSCollectNode>;

public:
#if __ROS2__
  RSCollectNode(const std::string &nodeName) : rclcpp::Node(nodeName) {
    this->declare_parameter<std::string>("collect_config_path", "");
    configFilePath_ =
        this->get_parameter("collect_config_path").get_value<std::string>();
  }
#elif __ROS1__
  RSCollectNode(const std::string &nodeName) : nodeHandle_(nodeName) {
    ros::NodeHandle privateHandle("~");
    privateHandle.param<std::string>("collect_config_path", configFilePath_, "");
  }
#endif

public:
  int init() {
    YAML::Node configNode;
#if __ROS2__
    auto ref = dynamic_cast<rclcpp::Node*>(this);
#elif __ROS1__
    auto& ref = nodeHandle_;
#endif

    try {
      configNode = YAML::LoadFile(configFilePath_);
    } catch (...) {
      // std::cerr << "Load Config File Path Failed !" << std::endl;
      RS_ERROR_STREAM(ref,
                          "Load Config File = " << configFilePath_
                                                << " Failed !");
      return -1;
    }

    // std::cerr << "Load Config File Path Successed !" << std::endl;
    RS_ERROR_STREAM(ref,
                       "Load Config File = " << configFilePath_
                                             << " Successed !");

    try {
#if __ROS2__
      collectManagerPtr_.reset(
          new robosense::rs_collect::collect::RSCollectManager(
              configNode, shared_from_this()));
#elif __ROS1__
      collectManagerPtr_.reset(
          new robosense::rs_collect::collect::RSCollectManager(
              configNode, std::make_shared<ros::NodeHandle>(nodeHandle_)));
#endif
    } catch (...) {
      // std::cerr << "Malloc CollectManager Failed !" << std::endl;
      RS_ERROR_STREAM(ref,
                       "Malloc RSCollectManager Instance Failed !");
      return -2;
    }

    // std::cerr << "Malloc CollectManager Successed !" << std::endl;
    RS_INFO_STREAM(ref,
                "Malloc RSCollectManager Instance Successed !");

    bool isSuccess = collectManagerPtr_->Init();
    if (!isSuccess) {
      // std::cerr << "Collect Manager Initial Failed !" << std::endl;
      RS_ERROR_STREAM(ref,
                       "RSCollectManager Instance Initial Failed !");
      return -3;
    }

    // std::cerr << "Collect Manager Initial Successed !" << std::endl;
    RS_INFO_STREAM(ref,
                "RSCollectManager Instance Initial Successed !");

    isSuccess = collectManagerPtr_->Start();
    if (!isSuccess) {
      // std::cerr << "Collect Manager Start Failed !" << std::endl;
      RS_ERROR_STREAM(ref,
                       "RSCollectManager Instance Start Failed !");
      return -4;
    }

    // std::cerr << "Collect Manager Start Successed !" << std::endl;
    RS_INFO_STREAM(ref,
                "RSCollectManager Instance Start Successed !");

    return 0;
  }

  int stop() {
    if (is_stop_ == false) {
      if (collectManagerPtr_ != nullptr) {
        collectManagerPtr_->Stop();
      }
      is_stop_ = true;
    }
    return 0;
  }

private:
  std::shared_ptr<robosense::rs_collect::collect::RSCollectManager>
      collectManagerPtr_;
  bool is_stop_ = false;
  std::string configFilePath_;
#if __ROS1__
  ros::NodeHandle nodeHandle_;
#endif
};

static std::shared_ptr<RSCollectNode> collect_node_ptr = nullptr;

void signalHandler(int signum) {
  (void)(signum);
#if __ROS2__
  rclcpp::shutdown();
#elif __ROS1__
  ros::shutdown();
#endif
  if (collect_node_ptr != nullptr) {
    collect_node_ptr->stop();
  }
}

int main(int argc, char **argv) {
  // 测试参数
  // for (int i = 0; i < argc; ++i) {
  //   std::cout << "i = " << std::string(argv[i]) << std::endl;
  // }
  std::signal(SIGINT, signalHandler);
  uint64_t timestampNs = RS_TIMESTAMP_NS;
  const std::string &nodeName =
      "robosense_collect_" + std::to_string(timestampNs);

  // 创建节点
#if __ROS2__
  rclcpp::init(argc, argv);
#elif __ROS1__
  ros::init(argc, argv, nodeName);
#endif

  collect_node_ptr = std::make_shared<RSCollectNode>(nodeName);
  int ret = collect_node_ptr->init();
  if (ret != 0) {
    std::cerr << "RoboSense Collect Node Initial Failed: ret = " << ret
              << std::endl;
    return -1;
  }
  // std::cerr << "run here +++" << std::endl;

#if __ROS2__
  rclcpp::spin(collect_node_ptr);
  rclcpp::shutdown();
#elif __ROS1__
  ros::spin();
  ros::shutdown();
#endif

  return 0;
}
