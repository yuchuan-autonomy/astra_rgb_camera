#include "astra_rgb_camera/device_discovery.hpp"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <unistd.h>

namespace fs = std::filesystem;

namespace
{

class TemporaryTree
{
public:
  TemporaryTree()
  : root_(fs::temp_directory_path() / ("astra_rgb_camera_test_" + std::to_string(getpid())))
  {
    fs::remove_all(root_);
    fs::create_directories(root_);
  }

  ~TemporaryTree()
  {
    std::error_code error;
    fs::remove_all(root_, error);
  }

  const fs::path & root() const
  {
    return root_;
  }

private:
  fs::path root_;
};

void make_video_device(
  const fs::path & root, const std::string & usb_name, const std::string & video_name,
  const std::string & vendor_id, const std::string & product_id)
{
  const auto usb_device = root / "sys/devices/usb1" / usb_name;
  const auto video_device =
    usb_device / (usb_name + ":1.0") / "video4linux" / video_name;
  fs::create_directories(video_device);
  std::ofstream(usb_device / "idVendor") << vendor_id;
  std::ofstream(usb_device / "idProduct") << product_id;

  const auto class_root = root / "sys/class/video4linux";
  fs::create_directories(class_root);
  fs::create_directory_symlink(video_device, class_root / video_name);

  const auto dev_root = root / "dev";
  fs::create_directories(dev_root);
  std::ofstream(dev_root / video_name);
}

TEST(DeviceDiscovery, FindsAndSortsAstraRgbDevices)
{
  TemporaryTree tree;
  make_video_device(tree.root(), "1-2", "video10", "2bc5", "050f");
  make_video_device(tree.root(), "1-3", "video2", "2BC5", "050F");

  const auto devices = astra_rgb_camera::find_video_devices(
    "2bc5", "050f", tree.root() / "sys/class/video4linux", tree.root() / "dev");

  EXPECT_EQ(
    devices,
    (std::vector<std::string>{
      (tree.root() / "dev/video2").string(),
      (tree.root() / "dev/video10").string()}));
}

TEST(DeviceDiscovery, IgnoresOtherUsbProducts)
{
  TemporaryTree tree;
  make_video_device(tree.root(), "1-2", "video0", "2bc5", "060f");

  const auto devices = astra_rgb_camera::find_video_devices(
    "2bc5", "050f", tree.root() / "sys/class/video4linux", tree.root() / "dev");

  EXPECT_TRUE(devices.empty());
}

}  // namespace
