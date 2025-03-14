#ifndef RSRECORDCLIPCHANNEL_H
#define RSRECORDCLIPCHANNEL_H

#include "modules/rs_collect/rsglobalconfig.h"

namespace robosense {
namespace rs_collect {
namespace collect {

enum class RS_RECORD_DATA_TYPE : int {
  RS_RECORD_DATA_H265 = 0,
  RS_RECORD_DATA_ANY = 1,
};

inline RS_RECORD_DATA_TYPE
fromStringToRecordDataType(const std::string &dataType) {
  if (dataType == "RS_RECORD_DATA_H265") {
    return RS_RECORD_DATA_TYPE::RS_RECORD_DATA_H265;
  } else {
    return RS_RECORD_DATA_TYPE::RS_RECORD_DATA_ANY;
  }
}

class RSRecordClipChannel {
public:
  using Ptr = std::shared_ptr<RSRecordClipChannel>;
  using ConstPtr = std::shared_ptr<const RSRecordClipChannel>;

public:
  RSRecordClipChannel() { reset(); }

  ~RSRecordClipChannel() {
    if (checkIsNeedWriterClose()) {
      closeWriter();
    }
  }

  void reset() {
    record_data_type_ = RS_RECORD_DATA_TYPE::RS_RECORD_DATA_ANY;
    channel_name_ = "";
    output_file_path_ = "";
    writer_ = nullptr;
    start_time_ = 0;
    end_time_ = 0;
    write_size_ = 0;
    is_writer_close_ = true;
    is_h265IFrame_ = false;
  }

  void init(const RS_RECORD_DATA_TYPE record_data_type,
            const std::string &topic_name,
            const std::string &output_file_path) {
    record_data_type_ = record_data_type;
    channel_name_ = topic_name;
    output_file_path_ = output_file_path;
    start_time_ = 0;
    end_time_ = 0;
    write_size_ = 0;
    record_statistical_ = 0;

    writer_ = nullptr;
    is_writer_close_ = false;
    is_h265IFrame_ = false;
  }

  bool checkIsNeedWriterClose() const { return !is_writer_close_; }

  void updateWriterClose(const bool isWriterClose) {
    is_writer_close_ = isWriterClose;
  }

#if __ROS1__
  void updateWriter(std::shared_ptr<rosbag::Bag> &writer) {
#elif __ROS2__
  void updateWriter(std::shared_ptr<rosbag2_cpp::Writer> &writer) {
#endif
    writer_ = writer;
  }

  void closeWriter() {
    if (writer_ != nullptr) {
      writer_->close();
      std::cout << "Clip Channel File: " << output_file_path_ << " Closed !"
                << std::endl;
    }
    writer_.reset();
  }

  std::string getOutputFilePath() const { return output_file_path_; }

  uint64_t getOutputFileDurationNs() const { return (end_time_ - start_time_); }

  uint64_t getOutputFileByteSize() const { return write_size_; }

  RS_RECORD_DATA_TYPE getRecordDataType() const { return record_data_type_; }

  size_t getOutputFileMessageCnt() const { return record_statistical_; }

  uint64_t getOutputFileBeginNs() const { return start_time_; }

  uint64_t getOutputFileEndNs() const { return end_time_; }

  // other message(s)
  bool writeMessage(const std::string &topic_name,
                    const std::string &topic_type,
                    const ros_time_t  &message_time,
                    const SerializedMessagePtr &message) {
    if (start_time_ == 0) {
      start_time_ = message_time;
    }
    end_time_ = message_time;
    write_size_ += message->size();
    ++record_statistical_;

#if __ROS1__
    ros::Time time;
    time.fromNSec(message_time);
    writer_->write(topic_name, time, *message);
#elif __ROS2__
    writer_->write(message, topic_name, topic_type, rclcpp::Time(message_time));
#endif

    return true;
  }

  // only for h265 message
  bool writeMessage(const std::string &topic_name,
                    const std::string &topic_type,
                    const ros_time_t  &message_time,
                    const SerializedMessagePtr &message,
                    const bool isH265IFrame) {
    // 非I帧开头不保存
    if (is_h265IFrame_ == false && isH265IFrame == false) {
      std::cerr << "camera h265 first frame is not i-frame, not write is to "
                   "file: topic_name = "
                << topic_name << std::endl;
      return true;
    }
    is_h265IFrame_ = true;

    if (start_time_ == 0) {
      start_time_ = message_time;
    }
    end_time_ = message_time;
    write_size_ += message->size();
    ++record_statistical_;

#if __ROS1__
    ros::Time time;
    time.fromNSec(message_time);
    writer_->write(topic_name, time, message);
#elif __ROS2__
    writer_->write(message, topic_name, topic_type, rclcpp::Time(message_time));
#endif

    return true;
  }

private:
  bool is_h265IFrame_ = false;
  std::string channel_name_;
  std::string output_file_path_;
  RS_RECORD_DATA_TYPE record_data_type_;

#if __ROS1__
  std::shared_ptr<rosbag::Bag> writer_;
#elif __ROS2__
  std::shared_ptr<rosbag2_cpp::Writer> writer_;
#endif

  uint64_t start_time_;
  uint64_t end_time_;
  uint64_t write_size_;
  bool is_writer_close_;
  size_t record_statistical_;
};

} // namespace collect
} // namespace rs_collect
} // namespace robosense

#endif // RSRECORDCLIPCHANNEL_H