#include "astra_rgb_camera/device_discovery.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace astra_rgb_camera
{

class AstraRgbCameraNode : public rclcpp::Node
{
public:
  AstraRgbCameraNode()
  : Node("astra_rgb_camera")
  {
    device_ = declare_parameter<std::string>("device", "");
    vendor_id_ = declare_parameter<std::string>("vendor_id", kAstraRgbVendorId);
    product_id_ = declare_parameter<std::string>("product_id", kAstraRgbProductId);
    image_topic_ = declare_parameter<std::string>("image_topic", "/front_camera/image_raw");
    frame_id_ = declare_parameter<std::string>(
      "frame_id", "front_camera_color_optical_frame");
    width_ = declare_parameter<int>("width", 640);
    height_ = declare_parameter<int>("height", 480);
    fps_ = declare_parameter<double>("fps", 30.0);
    pixel_format_ = declare_parameter<std::string>("pixel_format", "MJPG");
    flip_horizontal_ = declare_parameter<bool>("flip_horizontal", false);
    flip_vertical_ = declare_parameter<bool>("flip_vertical", false);
    reconnect_interval_ = declare_parameter<double>("reconnect_interval", 1.0);
    qos_reliability_ = declare_parameter<std::string>("qos_reliability", "reliable");

    if (width_ <= 0 || height_ <= 0 || fps_ <= 0.0 || reconnect_interval_ <= 0.0) {
      throw std::invalid_argument("width, height, fps, and reconnect_interval must be positive");
    }
    if (pixel_format_.size() != 4) {
      throw std::invalid_argument("pixel_format must be a four-character V4L2 code");
    }

    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    if (qos_reliability_ == "reliable") {
      qos.reliable();
    } else if (qos_reliability_ == "best_effort") {
      qos.best_effort();
    } else {
      throw std::invalid_argument("qos_reliability must be 'reliable' or 'best_effort'");
    }
    publisher_ = create_publisher<sensor_msgs::msg::Image>(image_topic_, qos);

    RCLCPP_INFO(
      get_logger(), "Publishing Astra RGB images on %s at up to %.1f Hz; USB ID %s:%s; QoS %s",
      image_topic_.c_str(), fps_, vendor_id_.c_str(), product_id_.c_str(),
      qos_reliability_.c_str());

    running_.store(true);
    capture_thread_ = std::thread(&AstraRgbCameraNode::capture_loop, this);
    publish_thread_ = std::thread(&AstraRgbCameraNode::publish_loop, this);
  }

  ~AstraRgbCameraNode() override
  {
    running_.store(false);
    stop_condition_.notify_all();
    frame_condition_.notify_all();
    if (capture_thread_.joinable()) {
      capture_thread_.join();
    }
    if (publish_thread_.joinable()) {
      publish_thread_.join();
    }
  }

private:
  std::vector<std::string> candidate_devices() const
  {
    if (!device_.empty()) {
      return {device_};
    }
    return find_video_devices(vendor_id_, product_id_);
  }

  void configure_capture(cv::VideoCapture & capture) const
  {
    const int fourcc = cv::VideoWriter::fourcc(
      pixel_format_[0], pixel_format_[1], pixel_format_[2], pixel_format_[3]);
    capture.set(cv::CAP_PROP_FOURCC, fourcc);
    capture.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    capture.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    capture.set(cv::CAP_PROP_FPS, fps_);
    capture.set(cv::CAP_PROP_BUFFERSIZE, 1);
  }

  void warn_throttled(const std::string & message)
  {
    const auto now = std::chrono::steady_clock::now();
    if (now >= next_warning_time_) {
      RCLCPP_WARN(get_logger(), "%s", message.c_str());
      next_warning_time_ = now + std::chrono::seconds(5);
    }
  }

  bool open_camera(cv::VideoCapture & capture)
  {
    const auto candidates = candidate_devices();
    if (candidates.empty()) {
      warn_throttled(
        "No V4L2 node found for USB ID " + vendor_id_ + ":" + product_id_);
      return false;
    }

    for (const auto & candidate : candidates) {
      if (!running_.load()) {
        return false;
      }
      capture.open(candidate, cv::CAP_V4L2);
      if (!capture.isOpened()) {
        capture.release();
        continue;
      }

      configure_capture(capture);
      cv::Mat initial_frame;
      bool received_frame = false;
      for (int attempt = 0; attempt < 10 && running_.load(); ++attempt) {
        if (capture.read(initial_frame) && !initial_frame.empty()) {
          received_frame = true;
          break;
        }
      }
      if (!received_frame) {
        capture.release();
        continue;
      }

      capture_device_ = candidate;
      store_frame(std::move(initial_frame));
      const int actual_width = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_WIDTH));
      const int actual_height = static_cast<int>(capture.get(cv::CAP_PROP_FRAME_HEIGHT));
      const double actual_fps = capture.get(cv::CAP_PROP_FPS);
      const int actual_fourcc = static_cast<int>(capture.get(cv::CAP_PROP_FOURCC));
      const std::string fourcc{
        static_cast<char>(actual_fourcc & 0xff),
        static_cast<char>((actual_fourcc >> 8) & 0xff),
        static_cast<char>((actual_fourcc >> 16) & 0xff),
        static_cast<char>((actual_fourcc >> 24) & 0xff)};
      RCLCPP_INFO(
        get_logger(), "Opened %s: %dx%d@%.1f, format %s", candidate.c_str(), actual_width,
        actual_height, actual_fps, fourcc.c_str());
      return true;
    }

    warn_throttled(
      "Astra RGB V4L2 nodes were found but none produced a frame; the camera may be in use");
    return false;
  }

  bool wait_for_reconnect()
  {
    std::unique_lock<std::mutex> lock(stop_mutex_);
    return stop_condition_.wait_for(
      lock, std::chrono::duration<double>(reconnect_interval_),
      [this]() {return !running_.load();});
  }

  void capture_loop()
  {
    cv::VideoCapture capture;
    int failed_reads = 0;

    while (running_.load()) {
      if (!capture.isOpened()) {
        if (!open_camera(capture)) {
          if (wait_for_reconnect()) {
            break;
          }
          continue;
        }
        failed_reads = 0;
      }

      cv::Mat frame;
      if (!capture.read(frame) || frame.empty()) {
        ++failed_reads;
        if (failed_reads >= 5) {
          RCLCPP_WARN(
            get_logger(), "Lost RGB frames from %s; reconnecting", capture_device_.c_str());
          capture.release();
          capture_device_.clear();
          if (wait_for_reconnect()) {
            break;
          }
        }
        continue;
      }

      failed_reads = 0;
      store_frame(std::move(frame));
    }

    capture.release();
  }

  void store_frame(cv::Mat frame)
  {
    if (frame.channels() == 1) {
      cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
    } else if (frame.channels() == 4) {
      cv::cvtColor(frame, frame, cv::COLOR_BGRA2BGR);
    } else if (frame.channels() != 3) {
      warn_throttled("Ignoring a camera frame with an unsupported channel count");
      return;
    }

    if (flip_horizontal_ && flip_vertical_) {
      cv::flip(frame, frame, -1);
    } else if (flip_horizontal_) {
      cv::flip(frame, frame, 1);
    } else if (flip_vertical_) {
      cv::flip(frame, frame, 0);
    }

    cv::Mat owned_frame = frame.clone();
    const builtin_interfaces::msg::Time stamp = get_clock()->now();
    std::lock_guard<std::mutex> lock(frame_mutex_);
    latest_frame_ = std::move(owned_frame);
    latest_stamp_ = stamp;
    ++latest_sequence_;
    frame_condition_.notify_one();
  }

  void publish_loop()
  {
    using SteadyClock = std::chrono::steady_clock;
    const auto publish_period = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / fps_));
    auto next_publish_time = SteadyClock::now();
    bool first_frame = true;

    while (running_.load()) {
      cv::Mat frame;
      builtin_interfaces::msg::Time stamp;
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_condition_.wait(
          lock, [this]() {
            return !running_.load() ||
            (!latest_frame_.empty() && latest_sequence_ != published_sequence_);
          });
        if (!running_.load()) {
          break;
        }

        const auto now = SteadyClock::now();
        if (now > next_publish_time + publish_period) {
          next_publish_time = now;
        } else if (now < next_publish_time) {
          frame_condition_.wait_until(
            lock, next_publish_time, [this]() {return !running_.load();});
          if (!running_.load()) {
            break;
          }
        }

        frame = latest_frame_;
        stamp = latest_stamp_;
        published_sequence_ = latest_sequence_;
      }

      if (!frame.isContinuous()) {
        frame = frame.clone();
      }

      sensor_msgs::msg::Image message;
      message.header.stamp = stamp;
      message.header.frame_id = frame_id_;
      message.height = static_cast<std::uint32_t>(frame.rows);
      message.width = static_cast<std::uint32_t>(frame.cols);
      message.encoding = "bgr8";
      message.is_bigendian = false;
      message.step = message.width * 3U;
      const std::size_t data_size = static_cast<std::size_t>(message.step) * message.height;
      message.data.resize(data_size);
      std::memcpy(message.data.data(), frame.data, data_size);

      if (!rclcpp::ok()) {
        break;
      }
      try {
        publisher_->publish(std::move(message));
        if (first_frame) {
          RCLCPP_INFO(get_logger(), "Published first RGB frame");
          first_frame = false;
        }
      } catch (const std::exception & exception) {
        if (rclcpp::ok()) {
          RCLCPP_ERROR(get_logger(), "Failed to publish RGB frame: %s", exception.what());
        }
        break;
      }
      next_publish_time += publish_period;
    }
  }

  std::string device_;
  std::string vendor_id_;
  std::string product_id_;
  std::string image_topic_;
  std::string frame_id_;
  std::string pixel_format_;
  std::string qos_reliability_;
  int width_;
  int height_;
  double fps_;
  double reconnect_interval_;
  bool flip_horizontal_;
  bool flip_vertical_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  std::thread capture_thread_;
  std::thread publish_thread_;
  std::atomic<bool> running_{false};
  std::mutex stop_mutex_;
  std::condition_variable stop_condition_;

  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  cv::Mat latest_frame_;
  builtin_interfaces::msg::Time latest_stamp_;
  std::uint64_t latest_sequence_{0};
  std::uint64_t published_sequence_{0};
  std::string capture_device_;
  std::chrono::steady_clock::time_point next_warning_time_{};
};

}  // namespace astra_rgb_camera

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<astra_rgb_camera::AstraRgbCameraNode>());
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("astra_rgb_camera"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
