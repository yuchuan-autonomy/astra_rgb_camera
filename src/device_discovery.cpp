#include "astra_rgb_camera/device_discovery.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <optional>
#include <system_error>
#include <utility>

namespace astra_rgb_camera
{
namespace
{

std::string normalize_id(std::string value)
{
  value.erase(
    std::remove_if(
      value.begin(), value.end(), [](unsigned char character) {
        return std::isspace(character) != 0;
      }),
    value.end());
  std::transform(
    value.begin(), value.end(), value.begin(), [](unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  return value;
}

std::optional<std::string> read_id(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  std::string value;
  if (!stream || !std::getline(stream, value)) {
    return std::nullopt;
  }
  return normalize_id(value);
}

std::optional<std::pair<std::string, std::string>> usb_ids_for_video_class(
  const std::filesystem::path & video_class)
{
  std::error_code error;
  auto current = std::filesystem::canonical(video_class, error);
  if (error) {
    return std::nullopt;
  }

  while (!current.empty()) {
    const auto vendor = read_id(current / "idVendor");
    const auto product = read_id(current / "idProduct");
    if (vendor && product) {
      return std::make_pair(*vendor, *product);
    }

    const auto parent = current.parent_path();
    if (parent == current) {
      break;
    }
    current = parent;
  }
  return std::nullopt;
}

int video_device_index(const std::filesystem::path & path)
{
  const auto name = path.filename().string();
  constexpr char prefix[] = "video";
  if (name.rfind(prefix, 0) != 0 || name.size() <= sizeof(prefix) - 1) {
    return -1;
  }

  try {
    return std::stoi(name.substr(sizeof(prefix) - 1));
  } catch (const std::exception &) {
    return -1;
  }
}

}  // namespace

std::vector<std::string> find_video_devices(
  const std::string & vendor_id,
  const std::string & product_id,
  const std::filesystem::path & sys_class_root,
  const std::filesystem::path & dev_root)
{
  std::vector<std::filesystem::path> matches;
  std::error_code error;
  if (!std::filesystem::is_directory(sys_class_root, error)) {
    return {};
  }

  for (const auto & entry : std::filesystem::directory_iterator(sys_class_root, error)) {
    if (error) {
      break;
    }
    const int index = video_device_index(entry.path());
    if (index < 0) {
      continue;
    }

    const auto ids = usb_ids_for_video_class(entry.path());
    if (!ids || ids->first != normalize_id(vendor_id) || ids->second != normalize_id(product_id)) {
      continue;
    }

    const auto device_node = dev_root / entry.path().filename();
    if (std::filesystem::exists(device_node, error)) {
      matches.push_back(device_node);
    }
    error.clear();
  }

  std::sort(
    matches.begin(), matches.end(), [](const auto & left, const auto & right) {
      const int left_index = video_device_index(left);
      const int right_index = video_device_index(right);
      if (left_index != right_index) {
        return left_index < right_index;
      }
      return left.string() < right.string();
    });

  std::vector<std::string> result;
  result.reserve(matches.size());
  for (const auto & match : matches) {
    result.push_back(match.string());
  }
  return result;
}

}  // namespace astra_rgb_camera
