#pragma once

#include <filesystem>
#include <string>
#include <vector>

namespace astra_rgb_camera
{

inline constexpr char kAstraRgbVendorId[] = "2bc5";
inline constexpr char kAstraRgbProductId[] = "050f";

std::vector<std::string> find_video_devices(
  const std::string & vendor_id = kAstraRgbVendorId,
  const std::string & product_id = kAstraRgbProductId,
  const std::filesystem::path & sys_class_root = "/sys/class/video4linux",
  const std::filesystem::path & dev_root = "/dev");

}  // namespace astra_rgb_camera
