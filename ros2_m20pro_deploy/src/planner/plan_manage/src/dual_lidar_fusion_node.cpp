/**
 * @file dual_lidar_fusion_node.cpp
 * @brief 前后双激光雷达点云时间同步、TF 统一坐标变换、拼接融合与按需 PCD 导出节点。
 *        Dual-LiDAR (front + rear) point cloud synchronization, TF transform to robot
 *        base frame, fusion, filtering, XYZIRT timestamp/ring preservation (for Super-LIO /
 *        RoboSense RSAIRY deskewing), and on-demand single-PCD export node.
 *
 * 功能概述 / Overview:
 *   1. 订阅前、后两个激光雷达的 sensor_msgs/PointCloud2 点云话题。
 *   2. 可选发布基于 YAML 外参配置的静态 TF (target_frame -> front/rear lidar frame)，
 *      同时支持查询外部 URDF / 动态 TF 树中的变换。
 *   3. 支持两种融合策略:
 *      - "sync": 基于 message_filters ApproximateTime 的双雷达时间同步拼接（默认）。
 *      - "async_latest": 异步缓存融合（任意雷达到达时与另一路最新有效点云拼接，单雷达降级可用）。
 *   4. 完整保留点云逐点属性 (x, y, z, intensity, ring[uint16], timestamp[float64])：
 *      - 自动识别 RoboSense `timestamp` (float64)、Velodyne `time` (float32)、
 *        Ouster/Livox `t`/`offset_time` (uint32 ns) 与通道号 `ring` (uint16/uint8)。
 *      - 输出标准 32 字节对齐 `PointXYZIRT` PointCloud2，同时兼容 `Super-LIO` 运动去畸变、
 *        `SCAN-Planner` (`GridMap`) 与 `RViz2`。
 *      - 体素降采样保留每体素内首个真实点，绝不破坏 `ring` 与逐点 `timestamp`。
 *   5. 提供 `~/save_pcd` (std_srvs/srv/Trigger) 服务：支持保存最新一帧融合点云或
 *      Step 0 累积的全局三维点云地图至单个二进制 `.pcd` 文件。
 *
 * 输入话题 / Subscribed Topics:
 *   - front_topic (默认 `/lidar_front/points`, sensor_msgs/msg/PointCloud2)
 *   - rear_topic  (默认 `/lidar_rear/points`,  sensor_msgs/msg/PointCloud2)
 *   - `/tf`, `/tf_static` (用于将各雷达点云变换至 target_frame 及可选 map_frame 累积)
 *
 * 输出话题 / Published Topics:
 *   - output_topic (默认 `/fused_lidar/points`, sensor_msgs/msg/PointCloud2, frame_id=target_frame)
 *
 * 提供服务 / Services:
 *   - `~/save_pcd`              (std_srvs/srv/Trigger): 按需导出单个二进制 `.pcd` 文件
 *   - `~/clear_accumulated_map` (std_srvs/srv/Trigger): 清空内部累积点云缓存
 *
 * 运行方式 / Usage:
 *   ros2 run scan_planner dual_lidar_fusion_node --ros-args \
 *     --params-file src/planner/plan_manage/config/dual_lidar_fusion.yaml
 */

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace scan_planner
{

/// 标准 32 字节对齐 XYZIRT 点结构，兼容 RoboSense rslidar_sdk / Super-LIO 与 PCL
struct FusedPointXYZIRT
{
  float x{0.0f};
  float y{0.0f};
  float z{0.0f};
  float intensity{0.0f};
  std::uint16_t ring{0};
  std::uint16_t padding0{0};
  std::uint32_t padding1{0};
  double timestamp{0.0};
};
static_assert(sizeof(FusedPointXYZIRT) == 32, "FusedPointXYZIRT must be 32 bytes aligned");

/// 体素哈希键：用于保留真实逐点 ring 与 timestamp 的无损代表点体素降采样
struct VoxelKey
{
  std::int32_t ix;
  std::int32_t iy;
  std::int32_t iz;
  bool operator==(const VoxelKey &other) const noexcept
  {
    return ix == other.ix && iy == other.iy && iz == other.iz;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey &k) const noexcept
  {
    const std::size_t h1 = std::hash<std::int32_t>{}(k.ix);
    const std::size_t h2 = std::hash<std::int32_t>{}(k.iy);
    const std::size_t h3 = std::hash<std::int32_t>{}(k.iz);
    return h1 ^ (h2 * 2654435761u) ^ (h3 * 2246822519u);
  }
};

class DualLidarFusionNode : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZI;
  using CloudT = pcl::PointCloud<PointT>;
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::msg::PointCloud2, sensor_msgs::msg::PointCloud2>;

  DualLidarFusionNode() : Node("dual_lidar_fusion")
  {
    // ------------------------------------------------------------------------
    // 1. 读取 YAML 硬件与算法参数 / Declare & read configurable parameters
    // ------------------------------------------------------------------------
    front_topic_ = declare_parameter<std::string>("front_topic", "/lidar_front/points");
    rear_topic_ = declare_parameter<std::string>("rear_topic", "/lidar_rear/points");
    output_topic_ = declare_parameter<std::string>("output_topic", "/fused_lidar/points");

    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    front_frame_id_ = declare_parameter<std::string>("front_frame_id", "lidar_front_link");
    rear_frame_id_ = declare_parameter<std::string>("rear_frame_id", "lidar_rear_link");
    map_frame_ = declare_parameter<std::string>("map_frame", "world");

    publish_static_tf_ = declare_parameter<bool>("publish_static_tf", true);
    front_extrinsic_ = declare_parameter<std::vector<double>>(
        "front_extrinsic", {0.32028, 0.0, -0.013, 0.0, 0.0, 0.0});
    rear_extrinsic_ = declare_parameter<std::vector<double>>(
        "rear_extrinsic", {-0.32028, 0.0, -0.013, 0.0, 0.0, 3.141592653589793});

    fusion_mode_ = declare_parameter<std::string>("fusion_mode", "sync");
    sync_queue_size_ = declare_parameter<int>("sync_queue_size", 20);
    sync_slop_sec_ = declare_parameter<double>("sync_slop_sec", 0.10);
    max_cloud_age_sec_ = declare_parameter<double>("max_cloud_age_sec", 0.25);
    tf_timeout_sec_ = declare_parameter<double>("tf_timeout_sec", 0.05);

    input_qos_best_effort_ = declare_parameter<bool>("input_qos_best_effort", true);
    output_qos_reliable_ = declare_parameter<bool>("output_qos_reliable", true);
    preserve_xyzirt_ = declare_parameter<bool>("preserve_xyzirt", true);

    min_range_ = declare_parameter<double>("min_range", 0.25);
    max_range_ = declare_parameter<double>("max_range", 30.0);
    enable_self_filter_ = declare_parameter<bool>("enable_self_filter", true);
    self_filter_min_ = declare_parameter<std::vector<double>>(
        "self_filter_min", {-0.45, -0.30, -0.35});
    self_filter_max_ = declare_parameter<std::vector<double>>(
        "self_filter_max", {0.45, 0.30, 0.25});
    voxel_leaf_size_ = declare_parameter<double>("voxel_leaf_size", 0.05);

    pcd_save_path_ = declare_parameter<std::string>(
        "pcd_save_path", "/tmp/scan_planner_maps/building_1f_2f.pcd");
    enable_map_accumulation_ = declare_parameter<bool>("enable_map_accumulation", false);
    accumulate_voxel_size_ = declare_parameter<double>("accumulate_voxel_size", 0.08);
    max_accumulated_points_ = declare_parameter<int>("max_accumulated_points", 2000000);

    accumulated_cloud_ = std::make_shared<CloudT>();
    latest_fused_cloud_ = std::make_shared<CloudT>();

    // ------------------------------------------------------------------------
    // 2. 初始化 TF 缓冲、监听器与可选静态外参广播 / Setup TF2 listener & static TF
    // ------------------------------------------------------------------------
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    if (publish_static_tf_)
    {
      static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
      publishStaticExtrinsics();
    }

    // ------------------------------------------------------------------------
    // 3. 创建融合点云发布器 (默认 Reliable，兼容 SensorDataQoS 与 Super-LIO/FAST-LIO)
    // ------------------------------------------------------------------------
    rclcpp::QoS pub_qos(10);
    if (output_qos_reliable_)
    {
      pub_qos.reliable();
    }
    else
    {
      pub_qos.best_effort();
    }
    pub_qos.durability_volatile();
    fused_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);

    // ------------------------------------------------------------------------
    // 4. 按配置创建双雷达同步或异步订阅链路 / Setup dual-LiDAR subscriptions
    // ------------------------------------------------------------------------
    if (fusion_mode_ == "async_latest")
    {
      rclcpp::QoS sub_qos = input_qos_best_effort_
                                ? rclcpp::SensorDataQoS()
                                : rclcpp::QoS(10).reliable();
      front_async_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          front_topic_, sub_qos,
          std::bind(&DualLidarFusionNode::frontAsyncCallback, this, std::placeholders::_1));
      rear_async_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
          rear_topic_, sub_qos,
          std::bind(&DualLidarFusionNode::rearAsyncCallback, this, std::placeholders::_1));
    }
    else
    {
      const rmw_qos_profile_t sub_rmw_qos =
          input_qos_best_effort_ ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
      front_sync_sub_ =
          std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
      rear_sync_sub_ =
          std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>>();
      front_sync_sub_->subscribe(this, front_topic_, sub_rmw_qos);
      rear_sync_sub_->subscribe(this, rear_topic_, sub_rmw_qos);

      sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
          SyncPolicy(std::max(1, sync_queue_size_)), *front_sync_sub_, *rear_sync_sub_);
      if (sync_slop_sec_ > 0.0)
      {
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(sync_slop_sec_));
      }
      sync_->registerCallback(std::bind(
          &DualLidarFusionNode::syncedCallback, this,
          std::placeholders::_1, std::placeholders::_2));
    }

    // ------------------------------------------------------------------------
    // 5. 注册按需保存 PCD 与重置累积缓存服务 / Register PCD export services
    // ------------------------------------------------------------------------
    save_pcd_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/save_pcd",
        std::bind(&DualLidarFusionNode::handleSavePcd, this,
                  std::placeholders::_1, std::placeholders::_2));
    clear_map_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/clear_accumulated_map",
        std::bind(&DualLidarFusionNode::handleClearMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(
        get_logger(),
        "DualLidarFusion ready [mode=%s, xyzirt=%s]: front='%s' + rear='%s' -> '%s' "
        "(target_frame='%s', static_tf=%s, map_accum=%s)",
        fusion_mode_.c_str(), preserve_xyzirt_ ? "true" : "false",
        front_topic_.c_str(), rear_topic_.c_str(),
        output_topic_.c_str(), target_frame_.c_str(),
        publish_static_tf_ ? "true" : "false",
        enable_map_accumulation_ ? "true" : "false");
  }

private:
  /// 根据 6 自由度外参 [x, y, z, roll, pitch, yaw] 构造静态 TF 消息
  geometry_msgs::msg::TransformStamped makeStaticTransform(
      const std::string &parent_frame,
      const std::string &child_frame,
      const std::vector<double> &extrinsic) const
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = parent_frame;
    tf_msg.child_frame_id = child_frame;

    const double x = extrinsic.size() > 0 ? extrinsic[0] : 0.0;
    const double y = extrinsic.size() > 1 ? extrinsic[1] : 0.0;
    const double z = extrinsic.size() > 2 ? extrinsic[2] : 0.0;
    const double roll = extrinsic.size() > 3 ? extrinsic[3] : 0.0;
    const double pitch = extrinsic.size() > 4 ? extrinsic[4] : 0.0;
    const double yaw = extrinsic.size() > 5 ? extrinsic[5] : 0.0;

    tf_msg.transform.translation.x = x;
    tf_msg.transform.translation.y = y;
    tf_msg.transform.translation.z = z;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q.normalize();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();
    return tf_msg;
  }

  /// 广播前/后激光雷达相对 target_frame 的静态外参 TF
  void publishStaticExtrinsics()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    if (front_frame_id_ != target_frame_)
    {
      transforms.push_back(makeStaticTransform(target_frame_, front_frame_id_, front_extrinsic_));
    }
    if (rear_frame_id_ != target_frame_)
    {
      transforms.push_back(makeStaticTransform(target_frame_, rear_frame_id_, rear_extrinsic_));
    }
    if (!transforms.empty())
    {
      static_tf_broadcaster_->sendTransform(transforms);
      RCLCPP_INFO(get_logger(), "Published %zu static LiDAR extrinsic TF(s) under '%s'",
                  transforms.size(), target_frame_.c_str());
    }
  }

  /// 在 PointCloud2 字段表中查找指定字段指针 / Find field descriptor by name
  static const sensor_msgs::msg::PointField *findField(
      const sensor_msgs::msg::PointCloud2 &msg, const std::string &name)
  {
    for (const auto &field : msg.fields)
    {
      if (field.name == name)
      {
        return &field;
      }
    }
    return nullptr;
  }

  /// 从字节缓冲安全读取浮点值 (支持 FLOAT32/FLOAT64/UINT8/UINT16)
  static double readNumericField(
      const std::uint8_t *base_ptr, const sensor_msgs::msg::PointField &field)
  {
    const std::uint8_t *ptr = base_ptr + field.offset;
    switch (field.datatype)
    {
      case sensor_msgs::msg::PointField::FLOAT32:
      {
        float v = 0.0f;
        std::memcpy(&v, ptr, sizeof(float));
        return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::FLOAT64:
      {
        double v = 0.0;
        std::memcpy(&v, ptr, sizeof(double));
        return v;
      }
      case sensor_msgs::msg::PointField::UINT8:
        return static_cast<double>(*ptr);
      case sensor_msgs::msg::PointField::UINT16:
      {
        std::uint16_t v = 0;
        std::memcpy(&v, ptr, sizeof(std::uint16_t));
        return static_cast<double>(v);
      }
      case sensor_msgs::msg::PointField::UINT32:
      {
        std::uint32_t v = 0;
        std::memcpy(&v, ptr, sizeof(std::uint32_t));
        return static_cast<double>(v);
      }
      default:
        return 0.0;
    }
  }

  /// 将任意格式 PointCloud2 解码为保留 XYZIRT (含逐点 timestamp 与 ring) 的点数组
  static std::vector<FusedPointXYZIRT> decodeXYZIRT(const sensor_msgs::msg::PointCloud2 &msg)
  {
    std::vector<FusedPointXYZIRT> points;
    const std::size_t total_points =
        static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
    if (total_points == 0 || msg.point_step == 0)
    {
      return points;
    }

    const auto *fx = findField(msg, "x");
    const auto *fy = findField(msg, "y");
    const auto *fz = findField(msg, "z");
    if (!fx || !fy || !fz)
    {
      return points;
    }

    const auto *fi = findField(msg, "intensity");
    const auto *fring = findField(msg, "ring");
    if (!fring)
    {
      fring = findField(msg, "line");
    }

    // 兼容主流雷达逐点时间戳命名: RoboSense(timestamp), Velodyne(time), Ouster/Livox(t/offset_time)
    const auto *fts_abs = findField(msg, "timestamp");
    const auto *fts_vel = findField(msg, "time");
    const auto *fts_ns = findField(msg, "offset_time");
    if (!fts_ns)
    {
      fts_ns = findField(msg, "t");
    }

    const double header_sec =
        static_cast<double>(msg.header.stamp.sec) +
        static_cast<double>(msg.header.stamp.nanosec) * 1e-9;

    const std::size_t max_safe_points = msg.data.size() / msg.point_step;
    const std::size_t n = std::min(total_points, max_safe_points);
    points.reserve(n);

    for (std::size_t i = 0; i < n; ++i)
    {
      const std::uint8_t *base = msg.data.data() + i * msg.point_step;
      const double x = readNumericField(base, *fx);
      const double y = readNumericField(base, *fy);
      const double z = readNumericField(base, *fz);
      if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
      {
        continue;
      }

      FusedPointXYZIRT pt;
      pt.x = static_cast<float>(x);
      pt.y = static_cast<float>(y);
      pt.z = static_cast<float>(z);
      if (fi)
      {
        const double intensity = readNumericField(base, *fi);
        pt.intensity = std::isfinite(intensity) ? static_cast<float>(intensity) : 0.0f;
      }
      if (fring)
      {
        const double ring = readNumericField(base, *fring);
        pt.ring = (std::isfinite(ring) && ring >= 0.0 && ring <= 65535.0)
                      ? static_cast<std::uint16_t>(ring)
                      : 0u;
      }
      if (fts_abs)
      {
        const double ts = readNumericField(base, *fts_abs);
        pt.timestamp = std::isfinite(ts) ? ts : header_sec;
      }
      else if (fts_vel)
      {
        const double rel_s = readNumericField(base, *fts_vel);
        pt.timestamp = std::isfinite(rel_s) ? (header_sec + rel_s) : header_sec;
      }
      else if (fts_ns)
      {
        const double rel_ns = readNumericField(base, *fts_ns);
        pt.timestamp = std::isfinite(rel_ns) ? (header_sec + rel_ns * 1e-9) : header_sec;
      }
      else
      {
        pt.timestamp = header_sec;
      }
      points.push_back(pt);
    }
    return points;
  }

  /// 将融合后的 XYZIRT 点集打包为标准 PointCloud2 (兼容 Super-LIO / RoboSense / SCAN-Planner)
  sensor_msgs::msg::PointCloud2 encodeOutputCloud(
      const std::vector<FusedPointXYZIRT> &points,
      const builtin_interfaces::msg::Time &stamp) const
  {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = target_frame_;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.is_bigendian = false;
    msg.is_dense = true;

    auto make_field = [](const std::string &name, std::uint32_t offset, std::uint8_t dtype) {
      sensor_msgs::msg::PointField f;
      f.name = name;
      f.offset = offset;
      f.datatype = dtype;
      f.count = 1;
      return f;
    };

    if (preserve_xyzirt_)
    {
      // 32 字节对齐标准 RoboSense PointXYZIRT 布局
      msg.point_step = sizeof(FusedPointXYZIRT);
      msg.row_step = msg.point_step * msg.width;
      msg.fields = {
          make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
          make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
          make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
          make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
          make_field("ring", 16, sensor_msgs::msg::PointField::UINT16),
          make_field("timestamp", 24, sensor_msgs::msg::PointField::FLOAT64),
      };
      msg.data.resize(static_cast<std::size_t>(msg.row_step));
      if (!points.empty())
      {
        std::memcpy(msg.data.data(), points.data(), msg.data.size());
      }
    }
    else
    {
      // 紧凑 16 字节 XYZI 布局
      msg.point_step = 16;
      msg.row_step = msg.point_step * msg.width;
      msg.fields = {
          make_field("x", 0, sensor_msgs::msg::PointField::FLOAT32),
          make_field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
          make_field("z", 8, sensor_msgs::msg::PointField::FLOAT32),
          make_field("intensity", 12, sensor_msgs::msg::PointField::FLOAT32),
      };
      msg.data.resize(static_cast<std::size_t>(msg.row_step));
      for (std::size_t i = 0; i < points.size(); ++i)
      {
        std::uint8_t *dst = msg.data.data() + i * 16;
        std::memcpy(dst + 0, &points[i].x, 4);
        std::memcpy(dst + 4, &points[i].y, 4);
        std::memcpy(dst + 8, &points[i].z, 4);
        std::memcpy(dst + 12, &points[i].intensity, 4);
      }
    }
    return msg;
  }

  /// 查询 TF 并将单路点云变换至目标坐标系 (保持 intensity, ring, timestamp 逐点无损)
  bool transformAndAppend(
      const sensor_msgs::msg::PointCloud2 &msg,
      const std::string &fallback_frame,
      const std::string &dest_frame,
      std::vector<FusedPointXYZIRT> &out_points)
  {
    const std::string src_frame = msg.header.frame_id.empty() ? fallback_frame : msg.header.frame_id;
    auto decoded = decodeXYZIRT(msg);
    if (decoded.empty())
    {
      return true;
    }

    if (src_frame == dest_frame)
    {
      out_points.insert(out_points.end(), decoded.begin(), decoded.end());
      return true;
    }

    geometry_msgs::msg::TransformStamped tf_stamped;
    try
    {
      tf_stamped = tf_buffer_->lookupTransform(
          dest_frame, src_frame, msg.header.stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
    }
    catch (const tf2::TransformException &)
    {
      // 当雷达时间戳略超前于动态 TF 时，回退查询最新可用变换以保证实时性
      try
      {
        tf_stamped = tf_buffer_->lookupTransform(dest_frame, src_frame, tf2::TimePointZero);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "TF lookup failed (%s -> %s): %s",
            src_frame.c_str(), dest_frame.c_str(), ex.what());
        return false;
      }
    }

    const Eigen::Isometry3d iso = tf2::transformToEigen(tf_stamped.transform);
    const Eigen::Matrix3f rot = iso.rotation().cast<float>();
    const Eigen::Vector3f trans = iso.translation().cast<float>();

    out_points.reserve(out_points.size() + decoded.size());
    for (auto &pt : decoded)
    {
      const Eigen::Vector3f p_dst = rot * Eigen::Vector3f(pt.x, pt.y, pt.z) + trans;
      pt.x = p_dst.x();
      pt.y = p_dst.y();
      pt.z = p_dst.z();
      out_points.push_back(pt);
    }
    return true;
  }

  /// 测距过滤 + 机器狗本体自遮挡盒裁剪 + 保留逐点属性的代表点体素降采样
  std::vector<FusedPointXYZIRT> filterFusedPoints(
      const std::vector<FusedPointXYZIRT> &raw_points) const
  {
    std::vector<FusedPointXYZIRT> filtered;
    filtered.reserve(raw_points.size());

    const double min_r2 = min_range_ * min_range_;
    const double max_r2 = max_range_ * max_range_;
    const bool valid_box = enable_self_filter_ &&
                           self_filter_min_.size() == 3 &&
                           self_filter_max_.size() == 3;

    for (const auto &pt : raw_points)
    {
      if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
      {
        continue;
      }
      const double r2 = static_cast<double>(pt.x) * pt.x +
                        static_cast<double>(pt.y) * pt.y +
                        static_cast<double>(pt.z) * pt.z;
      if (r2 < min_r2 || r2 > max_r2)
      {
        continue;
      }
      if (valid_box &&
          pt.x >= self_filter_min_[0] && pt.x <= self_filter_max_[0] &&
          pt.y >= self_filter_min_[1] && pt.y <= self_filter_max_[1] &&
          pt.z >= self_filter_min_[2] && pt.z <= self_filter_max_[2])
      {
        continue;
      }
      filtered.push_back(pt);
    }

    // 若启用体素降采样，每体素仅保留一个真实采样点，确保 ring 与逐点 timestamp 不被平均破坏
    if (voxel_leaf_size_ > 0.0 && !filtered.empty())
    {
      const float inv_leaf = static_cast<float>(1.0 / voxel_leaf_size_);
      std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> occupied;
      occupied.reserve(filtered.size());
      std::vector<FusedPointXYZIRT> downsampled;
      downsampled.reserve(filtered.size());
      for (const auto &pt : filtered)
      {
        const VoxelKey key{
            static_cast<std::int32_t>(std::floor(pt.x * inv_leaf)),
            static_cast<std::int32_t>(std::floor(pt.y * inv_leaf)),
            static_cast<std::int32_t>(std::floor(pt.z * inv_leaf)),
        };
        if (occupied.emplace(key, downsampled.size()).second)
        {
          downsampled.push_back(pt);
        }
      }
      return downsampled;
    }

    return filtered;
  }

  /// 将内部点集转换为 PCL PointXYZI 供 PCD 导出与地图累积使用
  static CloudT::Ptr toPclCloud(const std::vector<FusedPointXYZIRT> &points)
  {
    auto cloud = std::make_shared<CloudT>();
    cloud->points.reserve(points.size());
    for (const auto &pt : points)
    {
      PointT p;
      p.x = pt.x;
      p.y = pt.y;
      p.z = pt.z;
      p.intensity = pt.intensity;
      cloud->points.push_back(p);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }

  /// 若启用 Step 0 地图累积模式，则查询 map_frame -> target_frame 并增量累积至全局点云
  void maybeAccumulateMap(const CloudT::Ptr &fused_body_cloud, const builtin_interfaces::msg::Time &stamp)
  {
    if (!enable_map_accumulation_ || fused_body_cloud->empty())
    {
      return;
    }

    geometry_msgs::msg::TransformStamped map_tf;
    try
    {
      map_tf = tf_buffer_->lookupTransform(
          map_frame_, target_frame_, stamp,
          rclcpp::Duration::from_seconds(tf_timeout_sec_));
    }
    catch (const tf2::TransformException &)
    {
      try
      {
        map_tf = tf_buffer_->lookupTransform(map_frame_, target_frame_, tf2::TimePointZero);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 3000,
            "Map accumulation skipped: TF '%s' -> '%s' unavailable (%s)",
            map_frame_.c_str(), target_frame_.c_str(), ex.what());
        return;
      }
    }

    CloudT world_scan;
    const Eigen::Matrix4f mat = tf2::transformToEigen(map_tf.transform).matrix().cast<float>();
    pcl::transformPointCloud(*fused_body_cloud, world_scan, mat);

    std::lock_guard<std::mutex> lock(cloud_mutex_);
    *accumulated_cloud_ += world_scan;
    ++accumulated_frames_;

    // 每累积 10 帧或点数超限时执行一次体素去重，控制内存占用
    if (accumulate_voxel_size_ > 0.0 &&
        (accumulated_frames_ % 10 == 0 ||
         static_cast<int>(accumulated_cloud_->size()) > max_accumulated_points_))
    {
      auto compact = std::make_shared<CloudT>();
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(accumulated_cloud_);
      const float leaf = static_cast<float>(accumulate_voxel_size_);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter(*compact);
      accumulated_cloud_ = compact;
    }
  }

  /// 执行前后点云变换、拼接、滤波、发布与缓存的统一核心流程
  void fuseAndPublish(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &front_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &rear_msg)
  {
    std::vector<FusedPointXYZIRT> merged_raw;
    builtin_interfaces::msg::Time output_stamp = now();
    bool has_valid_input = false;

    if (front_msg)
    {
      if (transformAndAppend(*front_msg, front_frame_id_, target_frame_, merged_raw))
      {
        output_stamp = front_msg->header.stamp;
        has_valid_input = true;
      }
    }

    if (rear_msg)
    {
      if (transformAndAppend(*rear_msg, rear_frame_id_, target_frame_, merged_raw))
      {
        if (!front_msg ||
            rclcpp::Time(rear_msg->header.stamp) > rclcpp::Time(output_stamp))
        {
          output_stamp = rear_msg->header.stamp;
        }
        has_valid_input = true;
      }
    }

    if (!has_valid_input)
    {
      return;
    }

    const auto fused_points = filterFusedPoints(merged_raw);
    auto pcl_cloud = toPclCloud(fused_points);

    // 缓存最新一帧融合点云供 ~/save_pcd 按需导出
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      *latest_fused_cloud_ = *pcl_cloud;
    }

    maybeAccumulateMap(pcl_cloud, output_stamp);

    const auto output_msg = encodeOutputCloud(fused_points, output_stamp);
    fused_pub_->publish(output_msg);

    ++fused_frame_count_;
    if (fused_frame_count_ == 1 || fused_frame_count_ % 100 == 0)
    {
      RCLCPP_INFO(
          get_logger(),
          "Published fused cloud #%zu: %zu points in '%s' (format=%s)",
          fused_frame_count_, fused_points.size(), target_frame_.c_str(),
          preserve_xyzirt_ ? "XYZIRT" : "XYZI");
    }
  }

  /// ApproximateTime 时间同步回调 (fusion_mode == "sync")
  void syncedCallback(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &front_msg,
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &rear_msg)
  {
    fuseAndPublish(front_msg, rear_msg);
  }

  /// 异步模式前雷达回调 (fusion_mode == "async_latest")
  void frontAsyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr rear_copy;
    {
      std::lock_guard<std::mutex> lock(async_mutex_);
      latest_front_msg_ = msg;
      latest_front_rx_time_ = now();
      if (latest_rear_msg_ &&
          (now() - latest_rear_rx_time_).seconds() <= max_cloud_age_sec_)
      {
        rear_copy = latest_rear_msg_;
      }
    }
    fuseAndPublish(msg, rear_copy);
  }

  /// 异步模式后雷达回调 (fusion_mode == "async_latest")
  void rearAsyncCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2::ConstSharedPtr front_copy;
    {
      std::lock_guard<std::mutex> lock(async_mutex_);
      latest_rear_msg_ = msg;
      latest_rear_rx_time_ = now();
      if (latest_front_msg_ &&
          (now() - latest_front_rx_time_).seconds() <= max_cloud_age_sec_)
      {
        front_copy = latest_front_msg_;
      }
    }
    if (!front_copy)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "Front LiDAR stale/missing (>%.2fs); publishing rear LiDAR only", max_cloud_age_sec_);
    }
    fuseAndPublish(front_copy, msg);
  }

  /// ~/save_pcd 服务处理：将累积地图（或单帧融合点云）保存为二进制 PCD 文件
  void handleSavePcd(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    CloudT snapshot;
    std::string source_desc;
    {
      std::lock_guard<std::mutex> lock(cloud_mutex_);
      if (enable_map_accumulation_ && !accumulated_cloud_->empty())
      {
        snapshot = *accumulated_cloud_;
        source_desc = "accumulated map (" + map_frame_ + ")";
      }
      else if (!latest_fused_cloud_->empty())
      {
        snapshot = *latest_fused_cloud_;
        source_desc = "latest fused frame (" + target_frame_ + ")";
      }
    }

    if (snapshot.empty())
    {
      response->success = false;
      response->message = "No point cloud data available yet to save";
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
      return;
    }

    if (enable_map_accumulation_ && accumulate_voxel_size_ > 0.0)
    {
      CloudT compact;
      pcl::VoxelGrid<PointT> vg;
      vg.setInputCloud(snapshot.makeShared());
      const float leaf = static_cast<float>(accumulate_voxel_size_);
      vg.setLeafSize(leaf, leaf, leaf);
      vg.filter(compact);
      snapshot.swap(compact);
    }

    try
    {
      std::filesystem::path save_path(pcd_save_path_);
      if (save_path.has_parent_path())
      {
        std::filesystem::create_directories(save_path.parent_path());
      }
      const int ret = pcl::io::savePCDFileBinary(pcd_save_path_, snapshot);
      if (ret != 0)
      {
        response->success = false;
        response->message = "pcl::io::savePCDFileBinary failed for " + pcd_save_path_;
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }

      response->success = true;
      response->message = "Saved " + std::to_string(snapshot.size()) + " points from " +
                          source_desc + " to " + pcd_save_path_;
      RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
    }
    catch (const std::exception &ex)
    {
      response->success = false;
      response->message = std::string("Exception while saving PCD: ") + ex.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  /// ~/clear_accumulated_map 服务处理：清空内部累积地图缓存
  void handleClearMap(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(cloud_mutex_);
    accumulated_cloud_->clear();
    accumulated_frames_ = 0;
    response->success = true;
    response->message = "Cleared accumulated point cloud map buffer";
    RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
  }

  // 配置参数成员 / Parameters
  std::string front_topic_, rear_topic_, output_topic_;
  std::string target_frame_, front_frame_id_, rear_frame_id_, map_frame_;
  std::string fusion_mode_, pcd_save_path_;
  bool publish_static_tf_{true};
  bool input_qos_best_effort_{true};
  bool output_qos_reliable_{true};
  bool preserve_xyzirt_{true};
  bool enable_self_filter_{true};
  bool enable_map_accumulation_{false};
  int sync_queue_size_{20};
  int max_accumulated_points_{2000000};
  double sync_slop_sec_{0.10};
  double max_cloud_age_sec_{0.25};
  double tf_timeout_sec_{0.05};
  double min_range_{0.25};
  double max_range_{30.0};
  double voxel_leaf_size_{0.05};
  double accumulate_voxel_size_{0.08};
  std::vector<double> front_extrinsic_, rear_extrinsic_;
  std::vector<double> self_filter_min_, self_filter_max_;

  // ROS 2 通信与 TF 对象 / ROS 2 handles & TF2
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fused_pub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> front_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2>> rear_sync_sub_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr front_async_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr rear_async_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pcd_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_map_srv_;

  // 运行时缓存 / Runtime caches
  std::mutex async_mutex_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_front_msg_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_rear_msg_;
  rclcpp::Time latest_front_rx_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time latest_rear_rx_time_{0, 0, RCL_ROS_TIME};

  std::mutex cloud_mutex_;
  CloudT::Ptr latest_fused_cloud_;
  CloudT::Ptr accumulated_cloud_;
  size_t accumulated_frames_{0};
  size_t fused_frame_count_{0};
};

}  // namespace scan_planner

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<scan_planner::DualLidarFusionNode>());
  rclcpp::shutdown();
  return 0;
}
