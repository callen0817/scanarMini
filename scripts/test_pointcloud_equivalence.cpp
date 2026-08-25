#include <iostream>
#include <vector>
#include <cmath>
#include <cstring>
#include <cassert>
#include <iomanip>

// ROS 2 sensor_msgs
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

// Define RS point types
#define POINT_TYPE_XYZIRT

namespace robosense {
namespace lidar {

struct PointXYZIRT {
  float x;
  float y;
  float z;
  uint8_t intensity;
  uint16_t ring;
  double timestamp;
};

struct LidarPointCloudMsg {
  std::vector<PointXYZIRT> points;
  uint32_t width = 0;
  uint32_t height = 0;
  bool is_dense = false;
  double timestamp = 0.0;
  uint32_t seq = 0;
};

#pragma pack(push, 1)
struct DirectPointXYZIRT {
  float x;
  float y;
  float z;
  float intensity;
  uint16_t ring;
  double timestamp;
};
#pragma pack(pop)

inline int addPointField(sensor_msgs::msg::PointCloud2& msg, const std::string& name, 
                         uint32_t count, uint8_t datatype, int offset) {
  sensor_msgs::msg::PointField field;
  field.name = name;
  field.count = count;
  field.datatype = datatype;
  field.offset = offset;
  msg.fields.push_back(field);
  switch (datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8:
      return offset + 1 * count;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16:
      return offset + 2 * count;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32:
      return offset + 4 * count;
    case sensor_msgs::msg::PointField::FLOAT64:
      return offset + 8 * count;
    default:
      return offset;
  }
}

// Method 1: Original Iterator Path
sensor_msgs::msg::PointCloud2 toRosMsgIterator(const LidarPointCloudMsg& rs_msg, bool send_by_rows) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows) {
    ros_msg.width = rs_msg.width; 
    ros_msg.height = rs_msg.height; 
  } else {
    ros_msg.width = rs_msg.height; 
    ros_msg.height = rs_msg.width; 
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

  sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
  sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
  sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
  sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");

  for (size_t i = 0; i < rs_msg.height; i++) {
    for (size_t j = 0; j < rs_msg.width; j++) {
      const PointXYZIRT& point = rs_msg.points[i + j * rs_msg.height];
      *iter_x_ = point.x;
      *iter_y_ = point.y;
      *iter_z_ = point.z;
      *iter_intensity_ = point.intensity;
      *iter_ring_ = point.ring;
      *iter_timestamp_ = point.timestamp;

      ++iter_x_;
      ++iter_y_;
      ++iter_z_;
      ++iter_intensity_;
      ++iter_ring_;
      ++iter_timestamp_;
    }
  }

  return ros_msg;
}

// Method 2: Direct Memory Copy Path
sensor_msgs::msg::PointCloud2 toRosMsgDirectCopy(const LidarPointCloudMsg& rs_msg, bool send_by_rows) {
  sensor_msgs::msg::PointCloud2 ros_msg;
  int fields = 6;
  ros_msg.fields.clear();
  ros_msg.fields.reserve(fields);

  if (send_by_rows) {
    ros_msg.width = rs_msg.width; 
    ros_msg.height = rs_msg.height; 
  } else {
    ros_msg.width = rs_msg.height; 
    ros_msg.height = rs_msg.width; 
  }

  int offset = 0;
  offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
  offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
  offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);

  ros_msg.point_step = offset;
  ros_msg.row_step = ros_msg.width * ros_msg.point_step;
  ros_msg.is_dense = rs_msg.is_dense;
  ros_msg.data.resize(ros_msg.point_step * ros_msg.width * ros_msg.height);

  if (offset == sizeof(DirectPointXYZIRT)) {
    DirectPointXYZIRT* dst = reinterpret_cast<DirectPointXYZIRT*>(ros_msg.data.data());
    size_t out_idx = 0;
    for (size_t i = 0; i < rs_msg.height; ++i) {
      for (size_t j = 0; j < rs_msg.width; ++j) {
        const auto& src = rs_msg.points[i + j * rs_msg.height];
        dst[out_idx].x = src.x;
        dst[out_idx].y = src.y;
        dst[out_idx].z = src.z;
        dst[out_idx].intensity = static_cast<float>(src.intensity);
        dst[out_idx].ring = src.ring;
        dst[out_idx].timestamp = src.timestamp;
        ++out_idx;
      }
    }
  }

  return ros_msg;
}

} // namespace lidar
} // namespace robosense

int main() {
  std::cout << "================================================================================" << std::endl;
  std::cout << " POINTCLOUD2 DIRECT-COPY VS ITERATOR BITWISE & SEMANTIC EQUIVALENCE TEST" << std::endl;
  std::cout << "================================================================================" << std::endl;

  // 1. Check struct layout and offsets
  std::cout << "1. Struct Layout & Memory Alignment:" << std::endl;
  std::cout << "   - sizeof(DirectPointXYZIRT): " << sizeof(robosense::lidar::DirectPointXYZIRT) << " bytes (Expected: 26)" << std::endl;
  std::cout << "   - offsetof(x):         " << offsetof(robosense::lidar::DirectPointXYZIRT, x) << " (Expected: 0)" << std::endl;
  std::cout << "   - offsetof(y):         " << offsetof(robosense::lidar::DirectPointXYZIRT, y) << " (Expected: 4)" << std::endl;
  std::cout << "   - offsetof(z):         " << offsetof(robosense::lidar::DirectPointXYZIRT, z) << " (Expected: 8)" << std::endl;
  std::cout << "   - offsetof(intensity): " << offsetof(robosense::lidar::DirectPointXYZIRT, intensity) << " (Expected: 12)" << std::endl;
  std::cout << "   - offsetof(ring):      " << offsetof(robosense::lidar::DirectPointXYZIRT, ring) << " (Expected: 16)" << std::endl;
  std::cout << "   - offsetof(timestamp): " << offsetof(robosense::lidar::DirectPointXYZIRT, timestamp) << " (Expected: 18)" << std::endl;

  assert(sizeof(robosense::lidar::DirectPointXYZIRT) == 26);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, x) == 0);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, y) == 4);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, z) == 8);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, intensity) == 12);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, ring) == 16);
  assert(offsetof(robosense::lidar::DirectPointXYZIRT, timestamp) == 18);
  std::cout << "   [PASS] Struct layout matches advertised PointCloud2 field offsets exactly." << std::endl;

  // 2. Generate synthetic sweep of 72,000 points (Airy full revolution: 1800 blocks * 40 channels)
  const size_t NUM_POINTS = 72000;
  robosense::lidar::LidarPointCloudMsg raw_msg;
  raw_msg.width = 1800; // 1800 blocks
  raw_msg.height = 40;  // 40 channels
  raw_msg.points.resize(NUM_POINTS);
  raw_msg.is_dense = true;

  double base_time = 1787533600.123456789;
  for (size_t i = 0; i < NUM_POINTS; ++i) {
    float angle = (i * 2.0 * M_PI) / NUM_POINTS;
    float dist = 2.5f + 1.5f * std::sin(angle * 4.0f);
    raw_msg.points[i].x = dist * std::cos(angle);
    raw_msg.points[i].y = dist * std::sin(angle);
    raw_msg.points[i].z = -0.5f + (i % 40) * 0.025f;
    raw_msg.points[i].intensity = static_cast<uint8_t>((i * 7) % 256);
    raw_msg.points[i].ring = static_cast<uint16_t>(i % 40);
    raw_msg.points[i].timestamp = base_time + (i * 0.00000137);
  }

  // 3. Run both conversion paths
  std::cout << "\n2. Converting sweep of " << NUM_POINTS << " points via both paths..." << std::endl;
  auto cloud_A = robosense::lidar::toRosMsgIterator(raw_msg, false);
  auto cloud_B = robosense::lidar::toRosMsgDirectCopy(raw_msg, false);

  // 4. Validate metadata equivalence
  std::cout << "\n3. Validating Metadata Equivalence:" << std::endl;
  std::cout << "   - cloud_A.width=" << cloud_A.width << ", cloud_B.width=" << cloud_B.width << std::endl;
  std::cout << "   - cloud_A.height=" << cloud_A.height << ", cloud_B.height=" << cloud_B.height << std::endl;
  std::cout << "   - cloud_A.point_step=" << cloud_A.point_step << ", cloud_B.point_step=" << cloud_B.point_step << std::endl;
  std::cout << "   - cloud_A.row_step=" << cloud_A.row_step << ", cloud_B.row_step=" << cloud_B.row_step << std::endl;
  std::cout << "   - cloud_A.is_dense=" << (int)cloud_A.is_dense << ", cloud_B.is_dense=" << (int)cloud_B.is_dense << std::endl;
  std::cout << "   - cloud_A.data.size()=" << cloud_A.data.size() << " bytes, cloud_B.data.size()=" << cloud_B.data.size() << " bytes" << std::endl;

  assert(cloud_A.width == cloud_B.width);
  assert(cloud_A.height == cloud_B.height);
  assert(cloud_A.point_step == cloud_B.point_step);
  assert(cloud_A.row_step == cloud_B.row_step);
  assert(cloud_A.is_dense == cloud_B.is_dense);
  assert(cloud_A.data.size() == cloud_B.data.size());
  assert(cloud_A.fields.size() == cloud_B.fields.size());

  for (size_t f = 0; f < cloud_A.fields.size(); ++f) {
    assert(cloud_A.fields[f].name == cloud_B.fields[f].name);
    assert(cloud_A.fields[f].offset == cloud_B.fields[f].offset);
    assert(cloud_A.fields[f].datatype == cloud_B.fields[f].datatype);
    assert(cloud_A.fields[f].count == cloud_B.fields[f].count);
    std::cout << "   - Field " << f << " [" << cloud_A.fields[f].name << "]: offset=" 
              << cloud_A.fields[f].offset << ", datatype=" << (int)cloud_A.fields[f].datatype << " [OK]" << std::endl;
  }
  std::cout << "   [PASS] All metadata and field definitions are 100% identical." << std::endl;

  // 5. Bitwise memcmp comparison across entire buffer
  std::cout << "\n4. Performing Full Buffer Bitwise Memcmp Comparison (" << cloud_A.data.size() << " bytes):" << std::endl;
  int mem_res = std::memcmp(cloud_A.data.data(), cloud_B.data.data(), cloud_A.data.size());
  if (mem_res == 0) {
    std::cout << "   [PASS] Exact Bitwise Match across all " << cloud_A.data.size() << " bytes (0 mismatched bytes)!" << std::endl;
  } else {
    std::cerr << "   [FAIL] Bitwise mismatch detected!" << std::endl;
    return 1;
  }

  // 6. Semantic Per-Point Inspection
  std::cout << "\n5. Semantic Per-Point Equivalence Sample Inspection:" << std::endl;
  const robosense::lidar::DirectPointXYZIRT* pts_A = reinterpret_cast<const robosense::lidar::DirectPointXYZIRT*>(cloud_A.data.data());
  const robosense::lidar::DirectPointXYZIRT* pts_B = reinterpret_cast<const robosense::lidar::DirectPointXYZIRT*>(cloud_B.data.data());

  std::cout << std::fixed << std::setprecision(6);
  size_t check_indices[] = {0, 100, 1000, 10000, 40000, NUM_POINTS - 1};
  for (size_t idx : check_indices) {
    std::cout << "   Point [" << idx << "]:" << std::endl;
    std::cout << "     A: x=" << pts_A[idx].x << " y=" << pts_A[idx].y << " z=" << pts_A[idx].z 
              << " intensity=" << pts_A[idx].intensity << " ring=" << pts_A[idx].ring 
              << " ts=" << pts_A[idx].timestamp << std::endl;
    std::cout << "     B: x=" << pts_B[idx].x << " y=" << pts_B[idx].y << " z=" << pts_B[idx].z 
              << " intensity=" << pts_B[idx].intensity << " ring=" << pts_B[idx].ring 
              << " ts=" << pts_B[idx].timestamp << std::endl;
    assert(pts_A[idx].x == pts_B[idx].x);
    assert(pts_A[idx].y == pts_B[idx].y);
    assert(pts_A[idx].z == pts_B[idx].z);
    assert(pts_A[idx].intensity == pts_B[idx].intensity);
    assert(pts_A[idx].ring == pts_B[idx].ring);
    assert(pts_A[idx].timestamp == pts_B[idx].timestamp);
  }
  std::cout << "   [PASS] Point-by-point values are 100% equivalent across all fields." << std::endl;

  std::cout << "\n================================================================================" << std::endl;
  std::cout << " FINAL RESULT: BITWISE & SEMANTIC EQUIVALENCE FULLY PROVEN (PASS)" << std::endl;
  std::cout << "================================================================================" << std::endl;

  return 0;
}
