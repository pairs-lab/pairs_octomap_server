/* include //{ */

#include <octomap/OcTreeNode.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <laser_geometry/laser_geometry.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <std_srvs/srv/empty.hpp>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.hpp>

#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

#include <pairs_lib/node.h>
#include <pairs_lib/param_loader.h>
#include <pairs_lib/transformer.h>
#include <pairs_lib/subscriber_handler.h>
#include <pairs_lib/mutex.h>
#include <pairs_lib/scope_timer.h>
#include <pairs_lib/publisher_handler.h>

#include <pairs_modules_msgs/msg/pose_with_size.hpp>

#include <pairs_msgs/msg/control_manager_diagnostics.hpp>
#include <pairs_msgs/msg/float64_stamped.hpp>
#include <pairs_msgs/srv/string.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cmath>

//}

/* defines //{ */

namespace pairs_octomap_server
{

// helper: convert a geometry_msgs Vector3 -> octomap::point3d
inline octomap::point3d vector3ToOctomap(const geometry_msgs::msg::Vector3& v) {
  return octomap::point3d(v.x, v.y, v.z);
}

#if USE_ROS_TIMER == 1
typedef pairs_lib::ROSTimer TimerType;
#else
typedef pairs_lib::ThreadTimer TimerType;
#endif

using vec3s_t          = Eigen::Matrix<float, 3, -1>;
using vec3_t           = Eigen::Vector3f;
using PCLPointCloud    = pcl::PointCloud<pcl::PointXYZ>;
using PCLPointCloudPtr = PCLPointCloud::Ptr;

struct xyz_lut_t
{
  vec3s_t directions;  // a matrix of normalized direction column vectors
  vec3s_t offsets;     // a matrix of offset vectors
};

typedef struct
{
  double max_range;
  int    horizontal_rays;
} SensorParams2DLidar_t;

typedef struct
{
  double max_range;
  double free_ray_distance;
  double vertical_fov;
  int    vertical_rays;
  int    horizontal_rays;
  bool   update_free_space;
  bool   clear_occupied;
  double free_ray_distance_unknown;
  bool   decimation_enabled;
  double decimation_voxel_size;
  bool   crop_body_enabled;
  double crop_body_box_size;
} SensorParams3DLidar_t;

typedef struct
{
  double max_range;
  double free_ray_distance;
  double vertical_fov;
  double horizontal_fov;
  int    vertical_rays;
  int    horizontal_rays;
  bool   update_free_space;
  bool   clear_occupied;
  double free_ray_distance_unknown;
  bool   decimation_enabled;
  double decimation_voxel_size;
  bool   crop_body_enabled;
  double crop_body_box_size;
} SensorParamsDepthCam_t;

#ifdef COLOR_OCTOMAP_SERVER
using PCLPoint      = pcl::PointXYZRGB;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::ColorOcTree;
#else
using PCLPoint      = pcl::PointXYZ;
using PCLPointCloud = pcl::PointCloud<PCLPoint>;
using OcTree_t      = octomap::OcTree;
#endif

typedef enum
{
  LIDAR_3D,
  LIDAR_2D,
  LIDAR_1D,
  DEPTH_CAMERA,
  ULTRASOUND,
} SensorType_t;

const std::string _sensor_names_[] = {"LIDAR_3D", "LIDAR_2D", "LIDAR_1D", "DEPTH_CAMERA", "ULTRASOUND"};
//}

/* class OctomapServer //{ */

class OctomapServer : public pairs_lib::Node {
public:
  OctomapServer(const rclcpp::NodeOptions& options);

private:
  void initialize();

  std::atomic<bool> is_initialized_ = false;

  rclcpp::Node::SharedPtr  node_;
  rclcpp::Clock::SharedPtr clock_;

  // | --------------------- callback groups -------------------- |

  rclcpp::CallbackGroup::SharedPtr cbkgrp_subs_;
  rclcpp::CallbackGroup::SharedPtr cbkgrp_timers_;

  // | ------------------------ callbacks ----------------------- |

  bool callbackResetMap([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                        [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response>      resp);

  void callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const SensorType_t sensor_type, const int sensor_id,
                             const std::string topic, const bool free_rays);

  void callbackCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, const int sensor_id);

  // | -------------------- topic subscribers ------------------- |

  pairs_lib::SubscriberHandler<pairs_msgs::msg::ControlManagerDiagnostics> sh_control_manager_diag_;
  pairs_lib::SubscriberHandler<pairs_msgs::msg::Float64Stamped>            sh_height_;
  pairs_lib::SubscriberHandler<pairs_modules_msgs::msg::PoseWithSize>      sh_clear_box_;

  std::vector<pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_3dlaser_pc2_;
  std::vector<pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>> sh_depth_cam_pc2_;
  std::vector<pairs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>>  sh_depth_cam_info_;
  std::vector<pairs_lib::SubscriberHandler<sensor_msgs::msg::LaserScan>>   sh_laser_scan_;

  // | ----------------------- publishers ----------------------- |

  pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_global_full_;
  pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_global_binary_;
  pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_local_full_;
  pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap> pub_map_local_binary_;

  // | --------------------- service servers -------------------- |

  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr  ss_reset_map_;
  rclcpp::Service<pairs_msgs::srv::String>::SharedPtr ss_save_map_;
  rclcpp::Service<pairs_msgs::srv::String>::SharedPtr ss_load_map_;

  // | ------------------------- timers ------------------------- |

  std::shared_ptr<TimerType> timer_global_map_publisher_;
  double                     _global_map_publisher_rate_;
  void                       timerGlobalMapPublisher();

  std::shared_ptr<TimerType> timer_global_map_creator_;
  double                     _global_map_creator_rate_;
  void                       timerGlobalMapCreator();

  std::shared_ptr<TimerType> timer_local_map_publisher_;
  void                       timerLocalMapPublisher();

  std::shared_ptr<TimerType> timer_local_map_resizer_;
  void                       timerLocalMapResizer();

  // | ----------------------- parameters ----------------------- |

  std::string _uav_name_;

  bool _scope_timer_enabled_;

  bool   _global_map_publish_full_;
  bool   _global_map_publish_binary_;
  bool   _global_map_enabled_;
  double _global_map_size_;

  bool _map_while_grounded_;

  bool _local_map_publish_full_;
  bool _local_map_publish_binary_;

  std::unique_ptr<pairs_lib::Transformer> transformer_;

  std::shared_ptr<OcTree_t> octree_global_;
  std::shared_ptr<OcTree_t> octree_global_0_;
  std::shared_ptr<OcTree_t> octree_global_1_;
  int                       octree_global_idx_ = 0;
  std::mutex                mutex_octree_global_;

  std::shared_ptr<OcTree_t> octree_local_;
  std::shared_ptr<OcTree_t> octree_local_0_;
  std::shared_ptr<OcTree_t> octree_local_1_;
  int                       octree_local_idx_ = 0;
  std::mutex                mutex_octree_local_;

  std::atomic<bool> octrees_initialized_ = false;

  double     avg_time_cloud_insertion_ = 0;
  std::mutex mutex_avg_time_cloud_insertion_;

  std::string _world_frame_;
  std::string _robot_frame_;
  double      octree_resolution_;
  bool        _global_map_compress_;
  std::string _map_path_;

  float      _local_map_width_max_;
  float      _local_map_width_min_;
  float      _local_map_height_max_;
  float      _local_map_height_min_;
  float      local_map_width_;
  float      local_map_height_;
  std::mutex mutex_local_map_dimensions_;
  double     _local_map_publisher_rate_;

  double     local_map_duty_                 = 0;
  double     _local_map_duty_high_threshold_ = 0;
  double     _local_map_duty_low_threshold_  = 0;
  std::mutex mutex_local_map_duty_;

  float box_size = 2.0f;

  laser_geometry::LaserProjection projector_;

  bool copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min, const octomap::point3d& p_max);

  bool copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to);

  octomap::OcTreeNode* touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key, unsigned int depth,
                                       unsigned int max_depth);

  octomap::OcTreeNode* touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth);

  std::optional<double> getGroundZ(std::shared_ptr<OcTree_t>& octree, const double& x, const double& y);

  bool createLocalMap(const std::string frame_id, const double horizontal_distance, const double vertical_distance, std::shared_ptr<OcTree_t>& octree);

  virtual void insertPointCloud(const geometry_msgs::msg::Vector3& sensorOrigin, const PCLPointCloud::ConstPtr& cloud,
                                const PCLPointCloud::ConstPtr& free_cloud, double free_ray_distance, bool unknown_clear_occupied = false);

  void initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params);
  void initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params);

  bool                                       scope_timer_enabled_ = false;
  std::shared_ptr<pairs_lib::ScopeTimerLogger> scope_timer_logger_;

  int n_sensors_2d_lidar_;
  int n_sensors_3d_lidar_;
  int n_sensors_depth_cam_;

  std::vector<xyz_lut_t> sensor_2d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_3d_lidar_xyz_lut_;

  std::vector<xyz_lut_t> sensor_depth_camera_xyz_lut_;

  std::vector<SensorParams2DLidar_t> sensor_params_2d_lidar_;

  std::vector<SensorParams3DLidar_t> sensor_params_3d_lidar_;

  std::vector<SensorParamsDepthCam_t> sensor_params_depth_cam_;

  std::mutex mutex_lut_;

  std::vector<bool> vec_camera_info_processed_;

  // sensor model
  double _probHit_;
  double _probMiss_;
  double _thresMin_;
  double _thresMax_;
};

//}

/* OctomapServer() //{ */

OctomapServer::OctomapServer(const rclcpp::NodeOptions& options) : pairs_lib::Node("octomap_server", options) {
  initialize();
}

//}

/* initialize() //{ */

void OctomapServer::initialize() {

  node_  = this->this_node_ptr();
  clock_ = node_->get_clock();

  cbkgrp_subs_   = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cbkgrp_timers_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // | ----------------------- load files ----------------------- |

  pairs_lib::ParamLoader param_loader(node_);

  // load custom config
  std::string custom_config_path;
  param_loader.loadParam("custom_config", custom_config_path);

  if (custom_config_path != "") {

    RCLCPP_INFO(node_->get_logger(), "loading custom config '%s", custom_config_path.c_str());

    bool succ = param_loader.addYamlFile(custom_config_path);

    if (!succ) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load custom config.");
      rclcpp::shutdown();
      exit(1);
    }
  }

  {
    bool succ = param_loader.addYamlFileFromParam("public_config");

    if (!succ) {
      RCLCPP_ERROR(node_->get_logger(), "Failed to load public config.");
      rclcpp::shutdown();
      exit(1);
    }
  }

  param_loader.loadParam("uav_name", _uav_name_);
  param_loader.loadParam("world_frame_id", _world_frame_);
  param_loader.loadParam("robot_frame_id", _robot_frame_);
  param_loader.loadParam("map_path", _map_path_);

  param_loader.setPrefix("pairs_octomap_server/octomap_server/");

  param_loader.loadParam("scope_timer/enabled", _scope_timer_enabled_);

  param_loader.loadParam("map_while_grounded", _map_while_grounded_);

  param_loader.loadParam("global_map/size", _global_map_size_);
  param_loader.loadParam("global_map/enabled", _global_map_enabled_);
  param_loader.loadParam("global_map/publisher_rate", _global_map_publisher_rate_);
  param_loader.loadParam("global_map/creation_rate", _global_map_creator_rate_);
  param_loader.loadParam("global_map/compress", _global_map_compress_);
  param_loader.loadParam("global_map/publish_full", _global_map_publish_full_);
  param_loader.loadParam("global_map/publish_binary", _global_map_publish_binary_);

  param_loader.loadParam("local_map/size/max_width", _local_map_width_max_);
  param_loader.loadParam("local_map/size/max_height", _local_map_height_max_);
  param_loader.loadParam("local_map/size/min_width", _local_map_width_min_);
  param_loader.loadParam("local_map/size/min_height", _local_map_height_min_);
  param_loader.loadParam("local_map/size/duty_high_threshold", _local_map_duty_high_threshold_);
  param_loader.loadParam("local_map/size/duty_low_threshold", _local_map_duty_low_threshold_);
  param_loader.loadParam("local_map/publisher_rate", _local_map_publisher_rate_);
  param_loader.loadParam("local_map/publish_full", _local_map_publish_full_);
  param_loader.loadParam("local_map/publish_binary", _local_map_publish_binary_);

  local_map_width_  = _local_map_width_max_;
  local_map_height_ = _local_map_height_max_;

  param_loader.loadParam("map_resolution", octree_resolution_);

  param_loader.loadParam("sensor_params/2d_lidar/n_sensors", n_sensors_2d_lidar_);
  param_loader.loadParam("sensor_params/3d_lidar/n_sensors", n_sensors_3d_lidar_);
  param_loader.loadParam("sensor_params/depth_camera/n_sensors", n_sensors_depth_cam_);

  for (int i = 0; i < n_sensors_2d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/2d_lidar/sensor_" << i << "/max_range";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/2d_lidar/sensor_" << i << "/horizontal_rays";

    SensorParams2DLidar_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);

    sensor_params_2d_lidar_.push_back(params);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/depth_camera/sensor_" << i << "/max_range";

    std::stringstream free_ray_distance_param_name;
    free_ray_distance_param_name << "sensor_params/depth_camera/sensor_" << i << "/free_ray_distance";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/depth_camera/sensor_" << i << "/horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params/depth_camera/sensor_" << i << "/vertical_rays";

    std::stringstream hfov_param_name;
    hfov_param_name << "sensor_params/depth_camera/sensor_" << i << "/horizontal_fov_angle";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params/depth_camera/sensor_" << i << "/vertical_fov_angle";

    std::stringstream update_free_space_param_name;
    update_free_space_param_name << "sensor_params/depth_camera/sensor_" << i << "/unknown_rays/update_free_space";

    std::stringstream clear_occupied_param_name;
    clear_occupied_param_name << "sensor_params/depth_camera/sensor_" << i << "/unknown_rays/clear_occupied";

    std::stringstream free_ray_distance_unknown_param_name;
    free_ray_distance_unknown_param_name << "sensor_params/depth_camera/sensor_" << i << "/unknown_rays/free_ray_distance_unknown";

    std::stringstream decimation_enabled_param_name;
    decimation_enabled_param_name << "sensor_params/depth_camera/sensor_" << i << "/decimation/enabled";

    std::stringstream decimation_size_param_name;
    decimation_size_param_name << "sensor_params/depth_camera/sensor_" << i << "/decimation/voxel_size";

    std::stringstream crop_body_enabled_param_name;
    crop_body_enabled_param_name << "sensor_params/depth_camera/sensor_" << i << "/crop_robot_body/enabled";

    std::stringstream crop_body_box_size_param_name;
    crop_body_box_size_param_name << "sensor_params/depth_camera/sensor_" << i << "/crop_robot_body/box_size";

    SensorParamsDepthCam_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(free_ray_distance_param_name.str(), params.free_ray_distance);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);
    param_loader.loadParam(vertical_rays_param_name.str(), params.vertical_rays);
    param_loader.loadParam(hfov_param_name.str(), params.horizontal_fov);
    param_loader.loadParam(vfov_param_name.str(), params.vertical_fov);
    param_loader.loadParam(update_free_space_param_name.str(), params.update_free_space);
    param_loader.loadParam(clear_occupied_param_name.str(), params.clear_occupied);
    param_loader.loadParam(free_ray_distance_unknown_param_name.str(), params.free_ray_distance_unknown);
    param_loader.loadParam(decimation_enabled_param_name.str(), params.decimation_enabled);
    param_loader.loadParam(decimation_size_param_name.str(), params.decimation_voxel_size);
    param_loader.loadParam(crop_body_enabled_param_name.str(), params.crop_body_enabled);
    param_loader.loadParam(crop_body_box_size_param_name.str(), params.crop_body_box_size);

    sensor_params_depth_cam_.push_back(params);
  }

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    std::stringstream max_range_param_name;
    max_range_param_name << "sensor_params/3d_lidar/sensor_" << i << "/max_range";

    std::stringstream free_ray_distance_param_name;
    free_ray_distance_param_name << "sensor_params/3d_lidar/sensor_" << i << "/free_ray_distance";

    std::stringstream horizontal_rays_param_name;
    horizontal_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/horizontal_rays";

    std::stringstream vertical_rays_param_name;
    vertical_rays_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_rays";

    std::stringstream vfov_param_name;
    vfov_param_name << "sensor_params/3d_lidar/sensor_" << i << "/vertical_fov_angle";

    std::stringstream update_free_space_param_name;
    update_free_space_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/update_free_space";

    std::stringstream clear_occupied_param_name;
    clear_occupied_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/clear_occupied";

    std::stringstream free_ray_distance_unknown_param_name;
    free_ray_distance_unknown_param_name << "sensor_params/3d_lidar/sensor_" << i << "/unknown_rays/free_ray_distance_unknown";

    std::stringstream decimation_enabled_param_name;
    decimation_enabled_param_name << "sensor_params/3d_lidar/sensor_" << i << "/decimation/enabled";

    std::stringstream decimation_size_param_name;
    decimation_size_param_name << "sensor_params/3d_lidar/sensor_" << i << "/decimation/voxel_size";

    std::stringstream crop_body_enabled_param_name;
    crop_body_enabled_param_name << "sensor_params/3d_lidar/sensor_" << i << "/crop_robot_body/enabled";

    std::stringstream crop_body_box_size_param_name;
    crop_body_box_size_param_name << "sensor_params/3d_lidar/sensor_" << i << "/crop_robot_body/box_size";

    SensorParams3DLidar_t params;

    param_loader.loadParam(max_range_param_name.str(), params.max_range);
    param_loader.loadParam(free_ray_distance_param_name.str(), params.free_ray_distance);
    param_loader.loadParam(horizontal_rays_param_name.str(), params.horizontal_rays);
    param_loader.loadParam(vertical_rays_param_name.str(), params.vertical_rays);
    param_loader.loadParam(vfov_param_name.str(), params.vertical_fov);
    param_loader.loadParam(update_free_space_param_name.str(), params.update_free_space);
    param_loader.loadParam(clear_occupied_param_name.str(), params.clear_occupied);
    param_loader.loadParam(free_ray_distance_unknown_param_name.str(), params.free_ray_distance_unknown);
    param_loader.loadParam(decimation_enabled_param_name.str(), params.decimation_enabled);
    param_loader.loadParam(decimation_size_param_name.str(), params.decimation_voxel_size);
    param_loader.loadParam(crop_body_enabled_param_name.str(), params.crop_body_enabled);
    param_loader.loadParam(crop_body_box_size_param_name.str(), params.crop_body_box_size);

    sensor_params_3d_lidar_.push_back(params);
  }

  param_loader.loadParam("sensor_model/hit", _probHit_);
  param_loader.loadParam("sensor_model/miss", _probMiss_);
  param_loader.loadParam("sensor_model/min", _thresMin_);
  param_loader.loadParam("sensor_model/max", _thresMax_);

  if (!param_loader.loadedSuccessfully()) {
    RCLCPP_ERROR(node_->get_logger(), "Could not load all non-optional parameters. Shutting down.");
    rclcpp::shutdown();
    exit(1);
  }

  /* initialize sensor LUT model //{ */

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    xyz_lut_t lut_table;

    sensor_3d_lidar_xyz_lut_.push_back(lut_table);

    initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[i], sensor_params_3d_lidar_[i]);
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    xyz_lut_t lut_table;

    sensor_depth_camera_xyz_lut_.push_back(lut_table);

    initializeDepthCamLUT(sensor_depth_camera_xyz_lut_[i], sensor_params_depth_cam_[i]);

    vec_camera_info_processed_.push_back(false);
  }

  //}

  /* initialize octomap object & params //{ */

  octree_global_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_->setProbHit(_probHit_);
  octree_global_->setProbMiss(_probMiss_);
  octree_global_->setClampingThresMin(_thresMin_);
  octree_global_->setClampingThresMax(_thresMax_);

  octree_local_0_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_0_->setProbHit(_probHit_);
  octree_local_0_->setProbMiss(_probMiss_);
  octree_local_0_->setClampingThresMin(_thresMin_);
  octree_local_0_->setClampingThresMax(_thresMax_);

  octree_local_1_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_local_1_->setProbHit(_probHit_);
  octree_local_1_->setProbMiss(_probMiss_);
  octree_local_1_->setClampingThresMin(_thresMin_);
  octree_local_1_->setClampingThresMax(_thresMax_);

  octree_global_0_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_0_->setProbHit(_probHit_);
  octree_global_0_->setProbMiss(_probMiss_);
  octree_global_0_->setClampingThresMin(_thresMin_);
  octree_global_0_->setClampingThresMax(_thresMax_);

  octree_global_1_ = std::make_shared<OcTree_t>(octree_resolution_);
  octree_global_1_->setProbHit(_probHit_);
  octree_global_1_->setProbMiss(_probMiss_);
  octree_global_1_->setClampingThresMin(_thresMin_);
  octree_global_1_->setClampingThresMax(_thresMax_);

  octree_local_  = octree_local_0_;
  octree_global_ = octree_global_0_;

  octrees_initialized_ = true;

  //}

  /* transformer //{ */

  transformer_ = std::make_unique<pairs_lib::Transformer>(node_);
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->setLookupTimeout(std::chrono::duration<double>(1.0));
  transformer_->retryLookupNewest(true);

  //}

  /* publisher handler //{ */

  pairs_lib::PublisherHandlerOptions phopts;
  phopts.node = node_;

  pub_map_global_full_   = pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap>(phopts, "~/octomap_global_full_out");
  pub_map_global_binary_ = pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap>(phopts, "~/octomap_global_binary_out");
  pub_map_local_full_    = pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap>(phopts, "~/octomap_local_full_out");
  pub_map_local_binary_  = pairs_lib::PublisherHandler<octomap_msgs::msg::Octomap>(phopts, "~/octomap_local_binary_out");

  //}

  /* subscribers //{ */

  rclcpp::QoS qos_profile = rclcpp::SensorDataQoS();

  pairs_lib::SubscriberHandlerOptions shopts;
  shopts.no_message_timeout = pairs_lib::no_timeout;
  shopts.threadsafe         = true;
  shopts.autostart          = true;
  shopts.node               = node_;
  shopts.qos                = qos_profile;

  rclcpp::SubscriptionOptions subscription_options = rclcpp::SubscriptionOptions();
  subscription_options.callback_group              = cbkgrp_subs_;
  shopts.subscription_options                      = subscription_options;

  sh_control_manager_diag_ = pairs_lib::SubscriberHandler<pairs_msgs::msg::ControlManagerDiagnostics>(shopts, "~/control_manager_diagnostics_in");
  sh_height_               = pairs_lib::SubscriberHandler<pairs_msgs::msg::Float64Stamped>(shopts, "~/height_in");
  sh_clear_box_            = pairs_lib::SubscriberHandler<pairs_modules_msgs::msg::PoseWithSize>(shopts, "~/clear_box_in");

  // | ------------------------ 3d LiDARs ----------------------- |

  for (int i = 0; i < n_sensors_3d_lidar_; i++) {

    {
      std::stringstream ss;
      ss << "~/lidar_3d_" << i << "/points_in";
      std::string topic_name = ss.str();

      auto callback = [this, i, topic = topic_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->callback3dLidarCloud2(msg, LIDAR_3D, i, topic, false);
      };

      sh_3dlaser_pc2_.push_back(pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, topic_name, callback));
    }

    {
      std::stringstream ss;
      ss << "~/lidar_3d_" << i << "/free_points_in";
      std::string topic_name = ss.str();

      auto callback = [this, i, topic = topic_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->callback3dLidarCloud2(msg, LIDAR_3D, i, topic, true);
      };

      sh_3dlaser_pc2_.push_back(pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, topic_name, callback));
    }
  }

  // | ---------------------- depth cameras --------------------- |

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    {
      std::stringstream ss;
      ss << "~/depth_camera_" << i << "/points_in";
      std::string topic_name = ss.str();

      auto callback = [this, i, topic = topic_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->callback3dLidarCloud2(msg, DEPTH_CAMERA, i, topic, false);
      };

      sh_depth_cam_pc2_.push_back(pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, topic_name, callback));
    }

    {
      std::stringstream ss;
      ss << "~/depth_camera_" << i << "/free_points_in";
      const std::string topic_name = ss.str();

      auto callback = [this, i, topic = topic_name](const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
        this->callback3dLidarCloud2(msg, DEPTH_CAMERA, i, topic, true);
      };

      sh_depth_cam_pc2_.push_back(pairs_lib::SubscriberHandler<sensor_msgs::msg::PointCloud2>(shopts, topic_name, callback));
    }
  }

  for (int i = 0; i < n_sensors_depth_cam_; i++) {

    std::stringstream ss;
    ss << "~/depth_camera_" << i << "/camera_info_in";
    const std::string topic_name = ss.str();

    auto callback = [this, i](const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) { this->callbackCameraInfo(msg, i); };
    sh_depth_cam_info_.push_back(pairs_lib::SubscriberHandler<sensor_msgs::msg::CameraInfo>(shopts, topic_name, callback));
  }

  //}

  /* service servers //{ */

  ss_reset_map_ = node_->create_service<std_srvs::srv::Empty>("~/reset_map_in",
                                                              std::bind(&OctomapServer::callbackResetMap, this, std::placeholders::_1, std::placeholders::_2));

  //}

  /* timers //{ */

  pairs_lib::TimerHandlerOptions timer_opts_autostart;

  timer_opts_autostart.node           = node_;
  timer_opts_autostart.callback_group = cbkgrp_timers_;
  timer_opts_autostart.autostart      = true;

  if (_global_map_enabled_) {

    {
      std::function<void()> callback_global_map_publisher = std::bind(&OctomapServer::timerGlobalMapPublisher, this);

      timer_global_map_publisher_ =
          std::make_shared<TimerType>(timer_opts_autostart, rclcpp::Rate(_global_map_publisher_rate_, clock_), callback_global_map_publisher);
    }

    {
      std::function<void()> callback_global_map_creator = std::bind(&OctomapServer::timerGlobalMapCreator, this);

      timer_global_map_creator_ =
          std::make_shared<TimerType>(timer_opts_autostart, rclcpp::Rate(_global_map_creator_rate_, clock_), callback_global_map_creator);
    }
  }

  {
    std::function<void()> callback_local_map = std::bind(&OctomapServer::timerLocalMapPublisher, this);

    timer_local_map_publisher_ = std::make_shared<TimerType>(timer_opts_autostart, rclcpp::Rate(_local_map_publisher_rate_, clock_), callback_local_map);
  }

  {
    std::function<void()> callback_local_map_resizer = std::bind(&OctomapServer::timerLocalMapResizer, this);

    timer_local_map_resizer_ = std::make_shared<TimerType>(timer_opts_autostart, rclcpp::Rate(1.0), callback_local_map_resizer);
  }

  //}

  /* scope timer logger //{ */

  const std::string scope_timer_log_filename = param_loader.loadParam2("scope_timer/log_filename", std::string(""));
  scope_timer_logger_                        = std::make_shared<pairs_lib::ScopeTimerLogger>(node_, scope_timer_log_filename, _scope_timer_enabled_);

  //}

  is_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "initialized");
}

//}

// | --------------------- topic callbacks -------------------- |

/* callbackCameraInfo() //{ */

void OctomapServer::callbackCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg, const int sensor_id) {

  if (!is_initialized_) {
    return;
  }

  // bounds check to avoid out_of_range exception
  if (sensor_id < 0 || sensor_id >= static_cast<int>(vec_camera_info_processed_.size())) {
    RCLCPP_WARN(node_->get_logger(), "Received camera_info for invalid sensor_id %d", sensor_id);
    return;
  }

  if (vec_camera_info_processed_.at(sensor_id)) {
    return;
  }

  std::scoped_lock lock(mutex_lut_);
  sensor_params_depth_cam_[sensor_id].horizontal_fov = 2 * atan(msg->width / (2 * msg->k[0]));
  sensor_params_depth_cam_[sensor_id].vertical_fov   = 2 * atan(msg->height / (2 * msg->k[4]));

  RCLCPP_INFO_ONCE(node_->get_logger(),
                   "Changing sensor params based on camera_info for depth camera %d to %d horizontal rays, %d vertical rays, %.3f horizontal FOV, %.3f "
                   "vertical FOV.",
                   (int)sensor_id, sensor_params_depth_cam_[sensor_id].horizontal_rays, sensor_params_depth_cam_[sensor_id].vertical_rays,
                   sensor_params_depth_cam_[sensor_id].horizontal_fov * (180 / M_PI), sensor_params_depth_cam_[sensor_id].vertical_fov * (180 / M_PI));

  initializeDepthCamLUT(sensor_depth_camera_xyz_lut_[sensor_id], sensor_params_depth_cam_[sensor_id]);
  vec_camera_info_processed_.at(sensor_id) = true;
}

//}

/* callback3dLidarCloud2() //{ */

void OctomapServer::callback3dLidarCloud2(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg, const SensorType_t sensor_type, const int sensor_id,
                                          const std::string topic, const bool free_rays) {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  if (sensor_type == DEPTH_CAMERA) {
    if (sensor_id < 0 || sensor_id >= static_cast<int>(vec_camera_info_processed_.size())) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "received data for depth camera with invalid sensor_id %d", sensor_id);
      return;
    }
    if (!vec_camera_info_processed_.at(sensor_id)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "received data for depth camera %d but no camera info received yet.", sensor_id);
      return;
    }
  }

  if (!_map_while_grounded_) {

    if (!sh_control_manager_diag_.hasMsg()) {

      RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "missing control manager diagnostics, can not integrate data!");
      return;

    } else {

      rclcpp::Time last_time = sh_control_manager_diag_.lastMsgTime();

      if ((clock_->now() - last_time).seconds() > 1.0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "control manager diagnostics too old, can not integrate data!");
        return;
      }

      if (!sh_control_manager_diag_.getMsg()->flying_normally) {
        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "not flying normally, therefore, not integrating data");
        return;
      }
    }
  }

  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud = msg;

  rclcpp::Time time_start = clock_->now();

  PCLPointCloud::Ptr pc              = pcl::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr free_vectors_pc = pcl::make_shared<PCLPointCloud>();
  PCLPointCloud::Ptr hit_pc          = pcl::make_shared<PCLPointCloud>();

  try {
    pcl::fromROSMsg(*cloud, *pc);
  }
  catch (const std::exception& e) {
    RCLCPP_INFO_ONCE(node_->get_logger(), "pcl::fromROSMsg threw exception: %s", e.what());
    return;
  }
  RCLCPP_INFO_ONCE(node_->get_logger(), "pcl::fromROSMsg returned, pc->size=%zu", pc->points.size());

  // Debug: before TF lookup
  RCLCPP_INFO_ONCE(node_->get_logger(), "calling transformer_->getTransform(from=%s, to=%s, stamp=%u.%u)", cloud->header.frame_id.c_str(),
                   _world_frame_.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);

  auto res = transformer_->getTransform(cloud->header.frame_id, _world_frame_, rclcpp::Time(msg->header.stamp));

  if (!res) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "could not find tf from %s to %s (stamp %u.%u). Consider checking /tf and using latest transform.",
                         cloud->header.frame_id.c_str(), _world_frame_.c_str(), cloud->header.stamp.sec, cloud->header.stamp.nanosec);
    return;
  }

  Eigen::Matrix4f                      sensorToWorld;
  geometry_msgs::msg::TransformStamped sensorToWorldTf = res.value();
  pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);
  double max_range = 0.0;  // a voir pour debug (init to avoid build error message)

  if (!free_rays) {

    // generate sensor lookup table for free space raycasting based on pointcloud dimensions
    if (cloud->height == 1 || cloud->width == 1) {
      RCLCPP_WARN_ONCE(node_->get_logger(),
                       "Incoming pointcloud from %s #%d on topic %s is organized as a list! Free space raycasting of unknown rays won't work properly!",
                       _sensor_names_[sensor_type].c_str(), sensor_id, topic.c_str());
    }

    switch (sensor_type) {

      case LIDAR_3D: {

        std::scoped_lock lock(mutex_lut_);

        // change number of rays if it differs from the pointcloud dimensions
        if (sensor_params_3d_lidar_[sensor_id].horizontal_rays != static_cast<int>(cloud->width) ||
            sensor_params_3d_lidar_[sensor_id].vertical_rays != static_cast<int>(cloud->height)) {

          sensor_params_3d_lidar_[sensor_id].horizontal_rays = static_cast<int>(cloud->width);
          sensor_params_3d_lidar_[sensor_id].vertical_rays   = static_cast<int>(cloud->height);

          RCLCPP_INFO_ONCE(node_->get_logger(), "changing sensor params for lidar %d to %d horizontal rays, %d vertical rays, %.2f degs fov.", sensor_id,
                           sensor_params_3d_lidar_[sensor_id].horizontal_rays, sensor_params_3d_lidar_[sensor_id].vertical_rays,
                           (sensor_params_3d_lidar_[sensor_id].vertical_fov / M_PI) * 180.0);

          initialize3DLidarLUT(sensor_3d_lidar_xyz_lut_[sensor_id], sensor_params_3d_lidar_[sensor_id]);
        }

        max_range = sensor_params_3d_lidar_[sensor_id].max_range;

        break;
      }

      case DEPTH_CAMERA: {

        std::scoped_lock lock(mutex_lut_);

        // change number of rays if it differs from the pointcloud dimensions
        if (sensor_params_depth_cam_[sensor_id].horizontal_rays != static_cast<int>(cloud->width) ||
            sensor_params_depth_cam_[sensor_id].vertical_rays != static_cast<int>(cloud->height)) {

          sensor_params_depth_cam_[sensor_id].horizontal_rays = static_cast<int>(cloud->width);
          sensor_params_depth_cam_[sensor_id].vertical_rays   = static_cast<int>(cloud->height);

          RCLCPP_INFO_ONCE(node_->get_logger(),
                           "changing sensor params for depth camera %d to %d horizontal rays, %d vertical rays, %.3f horizontal FOV, %.3f vertical FOV.",
                           sensor_id, sensor_params_depth_cam_[sensor_id].horizontal_rays, sensor_params_depth_cam_[sensor_id].vertical_rays,
                           sensor_params_depth_cam_[sensor_id].horizontal_fov * (180 / M_PI), sensor_params_depth_cam_[sensor_id].vertical_fov * (180 / M_PI));

          initializeDepthCamLUT(sensor_depth_camera_xyz_lut_[sensor_id], sensor_params_depth_cam_[sensor_id]);
        }

        max_range = sensor_params_depth_cam_[sensor_id].max_range;

        break;
      }

      default: {

        break;
      }
    }
  }

  // get raycasting parameters
  double free_ray_distance      = 0;
  bool   unknown_clear_occupied = false;

  switch (sensor_type) {

    case LIDAR_3D: {

      std::scoped_lock lock(mutex_lut_);

      free_ray_distance      = sensor_params_3d_lidar_[sensor_id].free_ray_distance;
      unknown_clear_occupied = sensor_params_3d_lidar_[sensor_id].clear_occupied;

      break;
    }

    case DEPTH_CAMERA: {

      std::scoped_lock lock(mutex_lut_);

      free_ray_distance      = sensor_params_depth_cam_[sensor_id].free_ray_distance;
      unknown_clear_occupied = sensor_params_depth_cam_[sensor_id].clear_occupied;

      break;
    }

    default: {
      break;
    }
  }

  // | ------------------ pointcloud decimation ----------------- |

  bool   decimation_enabled    = false;
  double decimation_voxel_size = 0.0;

  switch (sensor_type) {

    case LIDAR_3D: {

      decimation_enabled    = sensor_params_3d_lidar_[sensor_id].decimation_enabled;
      decimation_voxel_size = sensor_params_3d_lidar_[sensor_id].decimation_voxel_size;

      break;
    }

    case DEPTH_CAMERA: {

      decimation_enabled    = sensor_params_depth_cam_[sensor_id].decimation_enabled;
      decimation_voxel_size = sensor_params_depth_cam_[sensor_id].decimation_voxel_size;

      break;
    }

    default: {
      break;
    }
  }

  // | ---------------------- robot removal --------------------- |

  bool   crop_robot_enabled = false;
  double crop_box_size      = 0.0;

  switch (sensor_type) {

    case LIDAR_3D: {

      crop_robot_enabled = sensor_params_3d_lidar_[sensor_id].crop_body_enabled;
      crop_box_size      = sensor_params_3d_lidar_[sensor_id].crop_body_box_size;

      break;
    }

    case DEPTH_CAMERA: {

      crop_robot_enabled = sensor_params_depth_cam_[sensor_id].crop_body_enabled;
      crop_box_size      = sensor_params_depth_cam_[sensor_id].crop_body_box_size;

      break;
    }

    default: {
      break;
    }
  }

  // | ---------------- splitting the pointcloud ---------------- |

  double raycasting_distance       = 0;
  bool   unknown_update_free_space = false;

  switch (sensor_type) {

    case LIDAR_3D: {

      std::scoped_lock lock(mutex_lut_);
      raycasting_distance       = sensor_params_3d_lidar_[sensor_id].free_ray_distance_unknown;
      unknown_update_free_space = sensor_params_3d_lidar_[sensor_id].update_free_space;

      break;
    }

    case DEPTH_CAMERA: {

      std::scoped_lock lock(mutex_lut_);
      raycasting_distance       = sensor_params_depth_cam_[sensor_id].free_ray_distance_unknown;
      unknown_update_free_space = sensor_params_depth_cam_[sensor_id].update_free_space;

      break;
    }

    default: {
      break;
    }
  }

  // * when the poincloud we received contains only free rays (points in free directions)
  if (free_rays) {

    RCLCPP_INFO_ONCE(node_->get_logger(), "fusing free points for sensor %d:%d", sensor_type, sensor_id);

    for (int i = 0; i < static_cast<int>(pc->size()); i++) {

      pcl::PointXYZ pt = pc->at(i);

      double orig_dist = std::hypot(pt.x, pt.y, pt.z);

      pcl::PointXYZ new_pt;

      new_pt.x = (pt.x / orig_dist) * raycasting_distance;
      new_pt.y = (pt.y / orig_dist) * raycasting_distance;
      new_pt.z = (pt.z / orig_dist) * raycasting_distance;

      free_vectors_pc->push_back(new_pt);
    }

  } else {

    std::scoped_lock lock(mutex_lut_);

    // * when receiving normal pointcloud, check for invalid points
    // * the direction of the invalid points can be reconstructed using the lookup table
    // * than the free space can be raycasted into this direction
    for (int i = 0; i < static_cast<int>(pc->size()); i++) {

      pcl::PointXYZ pt = pc->at(i);

      /* if ((!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) || (pt.x < 1e-3 && pt.y < 1e-3 && pt.z < 1e-3)) { */
      if ((!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))) {

        RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "XXX: the pc contains nonfinite or 0 points");

        // datapoint is missing, update only free space, if desired
        vec3_t ray_vec;

        switch (sensor_type) {

          case LIDAR_3D: {
            ray_vec = sensor_3d_lidar_xyz_lut_[sensor_id].directions.col(i);
            break;
          }

          case DEPTH_CAMERA: {
            ray_vec = sensor_depth_camera_xyz_lut_[sensor_id].directions.col(i);
            break;
          }

          default: {
            break;
          }
        }

        if (unknown_update_free_space) {

          pcl::PointXYZ temp_pt;

          temp_pt.x = ray_vec(0) * float(raycasting_distance);
          temp_pt.y = ray_vec(1) * float(raycasting_distance);
          temp_pt.z = ray_vec(2) * float(raycasting_distance);

          free_vectors_pc->push_back(temp_pt);
        }

      } else if ((pow(pt.x, 2) + pow(pt.y, 2) + pow(pt.z, 2)) > pow(max_range, 2)) {

        // point is over the max range, update only free space
        free_vectors_pc->push_back(pt);

      } else {

        // point is ok
        hit_pc->push_back(pt);
      }
    }
  }

  free_vectors_pc->header = pc->header;

  // transform to the map frame

  if (crop_robot_enabled && !free_rays) {

    RCLCPP_INFO_ONCE(node_->get_logger(), "cropping robot's body");

    // Remove points around the drone (apply crop to downsampled result)
    pcl::CropBox<pcl::PointXYZ> box_filter;
    Eigen::Vector4f             min_point(-crop_box_size / 2.0, -crop_box_size / 2.0, -crop_box_size / 2.0, 1.0);
    Eigen::Vector4f             max_point(crop_box_size / 2.0, crop_box_size / 2.0, crop_box_size / 2.0, 1.0);
    box_filter.setMin(min_point);
    box_filter.setMax(max_point);
    box_filter.setInputCloud(hit_pc);  // use downsampled_cloud as input
    box_filter.setNegative(true);
    box_filter.filter(*hit_pc);
  }

  if (decimation_enabled) {

    RCLCPP_INFO_ONCE(node_->get_logger(), "decimating pointcloud");

    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

      voxel_filter.setInputCloud(hit_pc);

      voxel_filter.setLeafSize(decimation_voxel_size, decimation_voxel_size, decimation_voxel_size);

      voxel_filter.filter(*hit_pc);
    }

    {
      pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;

      voxel_filter.setInputCloud(free_vectors_pc);

      voxel_filter.setLeafSize(decimation_voxel_size, decimation_voxel_size, decimation_voxel_size);

      voxel_filter.filter(*free_vectors_pc);
    }
  }

  pcl::transformPointCloud(*hit_pc, *hit_pc, sensorToWorld);
  pcl::transformPointCloud(*free_vectors_pc, *free_vectors_pc, sensorToWorld);

  hit_pc->header.frame_id          = _world_frame_;
  free_vectors_pc->header.frame_id = _world_frame_;

  insertPointCloud(sensorToWorldTf.transform.translation, hit_pc, free_vectors_pc, free_ray_distance, unknown_clear_occupied);

  [[maybe_unused]] const octomap::point3d sensor_origin = vector3ToOctomap(sensorToWorldTf.transform.translation);

  {
    std::scoped_lock lock(mutex_avg_time_cloud_insertion_);

    rclcpp::Time time_end = clock_->now();

    double exec_duration = (time_end - time_start).seconds();

    double coef               = 0.5;
    avg_time_cloud_insertion_ = coef * avg_time_cloud_insertion_ + (1.0 - coef) * exec_duration;

    RCLCPP_INFO_THROTTLE(node_->get_logger(), *clock_, 1000, "avg cloud insertion time = %.3f sec", avg_time_cloud_insertion_);
  }
}

//}

// | -------------------- service callbacks ------------------- |

/* callbackResetMap() //{ */

bool OctomapServer::callbackResetMap([[maybe_unused]] const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                                     [[maybe_unused]] std::shared_ptr<std_srvs::srv::Empty::Response>      resp) {

  {
    std::scoped_lock lock(mutex_octree_global_, mutex_octree_local_);

    octree_global_->clear();
    octree_local_->clear();
  }

  octrees_initialized_ = true;

  RCLCPP_INFO(node_->get_logger(), "Octomap cleared");

  return true;
}

//}

// | ------------------------- timers ------------------------- |

/* timerGlobalMapPublisher() //{ */

void OctomapServer::timerGlobalMapPublisher() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "full map publisher timer spinning");

  size_t octomap_size;

  {
    std::scoped_lock lock(mutex_octree_global_);

    octomap_size = octree_global_->size();
  }

  if (octomap_size <= 1) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "nothing to publish, octree is empty");
    return;
  }

  /* if (_global_map_compress_) { */
  /*   octree_global_->prune(); */
  /* } */

  if (_global_map_publish_full_) {

    octomap_msgs::msg::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = clock_->now();

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_global_);

      pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::globalMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::fullMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_full_.publish(map);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "error serializing global octomap to full representation");
    }
  }

  if (_global_map_publish_binary_) {

    octomap_msgs::msg::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = clock_->now();

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_global_);

      pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::globalMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::binaryMapToMsg(*octree_global_, map);
    }

    if (success) {
      pub_map_global_binary_.publish(map);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "error serializing global octomap to binary representation");
    }
  }
}

//}

/* timerGlobalMapCreator() //{ */

void OctomapServer::timerGlobalMapCreator() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::timerGlobalMapCreator", scope_timer_logger_, _scope_timer_enabled_);

  RCLCPP_INFO_ONCE(node_->get_logger(), "global map creator timer spinning");

  // copy the local map into a buffer

  std::shared_ptr<OcTree_t> local_map_tmp_;
  {
    std::scoped_lock lock(mutex_octree_local_);

    local_map_tmp_ = std::make_shared<OcTree_t>(*octree_local_);
  }

  local_map_tmp_->expand();

  {
    std::scoped_lock lock(mutex_octree_global_);

    copyLocalMap(local_map_tmp_, octree_global_);
  }

  geometry_msgs::msg::PoseStamped uav;
  uav.header.frame_id = _robot_frame_;

  auto res = transformer_->transformSingle(uav, _world_frame_);

  if (res) {

    std::scoped_lock lock(mutex_octree_global_);

    octomap::point3d roi_min(res->pose.position.x - _global_map_size_ / 2.0, res->pose.position.y - _global_map_size_ / 2.0,
                             res->pose.position.z - _global_map_size_ / 2.0);
    octomap::point3d roi_max(res->pose.position.x + _global_map_size_ / 2.0, res->pose.position.y + _global_map_size_ / 2.0,
                             res->pose.position.z + _global_map_size_ / 2.0);

    std::shared_ptr<OcTree_t> from;

    if (octree_global_idx_ == 0) {
      from               = octree_global_0_;
      octree_global_     = octree_global_1_;
      octree_global_idx_ = 1;
    } else {
      from               = octree_global_1_;
      octree_global_     = octree_global_0_;
      octree_global_idx_ = 0;
    }

    octree_global_->clear();

    copyInsideBBX2(from, octree_global_, roi_min, roi_max);
  }
}

//}

/* timerLocalMapPublisher() //{ */

void OctomapServer::timerLocalMapPublisher() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "local map publisher timer spinning");

  size_t octomap_size = octree_local_->size();

  if (octomap_size <= 1) {
    RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "nothing to publish, octree_local_, octree is empty");
    return;
  }

  if (_local_map_publish_full_) {

    octomap_msgs::msg::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = clock_->now();

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_local_);

      pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::localMapFullPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::fullMapToMsg(*octree_local_, map);
    }

    if (success) {
      pub_map_local_full_.publish(map);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "error serializing local octomap to full representation");
    }
  }

  if (_local_map_publish_binary_) {

    octomap_msgs::msg::Octomap map;
    map.header.frame_id = _world_frame_;
    map.header.stamp    = clock_->now();

    bool success = false;

    {
      std::scoped_lock lock(mutex_octree_local_);

      pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::localMapBinaryPublish", scope_timer_logger_, _scope_timer_enabled_);

      success = octomap_msgs::binaryMapToMsg(*octree_local_, map);
    }

    if (success) {
      pub_map_local_binary_.publish(map);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "error serializing local octomap to binary representation");
    }
  }
}

//}

/* timerLocalMapResizer() //{ */

void OctomapServer::timerLocalMapResizer() {

  if (!is_initialized_) {
    return;
  }

  if (!octrees_initialized_) {
    return;
  }

  RCLCPP_INFO_ONCE(node_->get_logger(), "local map resizer timer spinning");

  auto local_map_duty = pairs_lib::get_mutexed(mutex_local_map_duty_, local_map_duty_);

  {
    std::scoped_lock lock(mutex_local_map_dimensions_);

    if (local_map_duty > _local_map_duty_high_threshold_) {
      local_map_width_ -= ceil(10.0 * (local_map_duty - _local_map_duty_high_threshold_));
      local_map_height_ -= ceil(10.0 * (local_map_duty - _local_map_duty_high_threshold_));
    } else if (local_map_duty < _local_map_duty_low_threshold_) {
      local_map_width_  = local_map_width_ + 1.0f;
      local_map_height_ = local_map_height_ + 1.0f;
    }

    if (local_map_width_ < _local_map_width_min_) {
      local_map_width_ = _local_map_width_min_;
    } else if (local_map_width_ > _local_map_width_max_) {
      local_map_width_ = _local_map_width_max_;
    }

    if (local_map_height_ < _local_map_height_min_) {
      local_map_height_ = _local_map_height_min_;
    } else if (local_map_height_ > _local_map_height_max_) {
      local_map_height_ = _local_map_height_max_;
    }

    RCLCPP_INFO(node_->get_logger(), "local map - duty time: %.3f s; size: width %.3f m, height %.3f m", local_map_duty, local_map_width_, local_map_height_);

    local_map_duty = 0;
  }

  pairs_lib::set_mutexed(mutex_local_map_duty_, local_map_duty, local_map_duty_);
}

//}

// | ------------------------ routines ------------------------ |

/* insertPointCloud() //{ */

void OctomapServer::insertPointCloud(const geometry_msgs::msg::Vector3& sensorOriginTf, const PCLPointCloud::ConstPtr& cloud,
                                     const PCLPointCloud::ConstPtr& free_vectors_cloud, double free_ray_distance, bool unknown_clear_occupied) {

  pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::timerInsertPointCloud", scope_timer_logger_, _scope_timer_enabled_);

  rclcpp::Time time_start = clock_->now();

  std::scoped_lock lock(mutex_octree_local_);

  auto [local_map_width, local_map_height] = pairs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

  // const octomap::point3d sensor_origin = octomap::pointTfToOctomap(sensorOriginTf);
  const octomap::point3d sensor_origin = vector3ToOctomap(sensorOriginTf);


  // Use the filtered cloud for the rest of the function
  const float     free_space_ray_len = std::min(float(free_ray_distance), float(sqrt(2 * pow(local_map_width / 2.0, 2) + pow(local_map_height / 2.0, 2))));
  octomap::KeySet occupied_cells;
  octomap::KeySet free_cells;
  octomap::KeySet free_ends;

  // All measured points: make it free on ray, occupied on endpoint
  for (PCLPointCloud::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());

    octomap::OcTreeKey key;
    if (octree_local_->coordToKeyChecked(measured_point, key)) {
      occupied_cells.insert(key);
    }

    // Move end point to distance min(free space ray len, current distance)
    measured_point                  = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);
    octomap::OcTreeKey measured_key = octree_local_->coordToKey(measured_point);
    free_ends.insert(measured_key);
  }

  // FREE VECTORS
  for (PCLPointCloud::const_iterator it = free_vectors_cloud->begin(); it != free_vectors_cloud->end(); ++it) {

    if (!(std::isfinite(it->x) && std::isfinite(it->y) && std::isfinite(it->z))) {
      continue;
    }

    octomap::point3d measured_point(it->x, it->y, it->z);
    const float      point_distance = float((measured_point - sensor_origin).norm());
    octomap::KeyRay  keyRay;

    // Move end point to distance min(free space ray len, current distance)
    measured_point = sensor_origin + (measured_point - sensor_origin).normalize() * std::min(free_space_ray_len, point_distance);

    // Check if the ray intersects a cell in the occupied list
    if (octree_local_->computeRayKeys(sensor_origin, measured_point, keyRay)) {

      octomap::KeyRay::iterator alterantive_ray_end = keyRay.end();

      if (!unknown_clear_occupied) {

        for (octomap::KeyRay::iterator it2 = keyRay.begin(), end = keyRay.end(); it2 != end; ++it2) {

          // Check if the cell is occupied in the map
          auto node = octree_local_->search(*it2);

          if (node && octree_local_->isNodeOccupied(node)) {

            if (it2 == keyRay.begin()) {
              alterantive_ray_end = keyRay.begin();  // Special case
            } else {
              alterantive_ray_end = it2 - 1;
            }
            break;
          }
        }
      }

      free_cells.insert(keyRay.begin(), alterantive_ray_end);
    }
  }

  // For FREE RAY ENDS
  for (octomap::KeySet::iterator it = free_ends.begin(), end = free_ends.end(); it != end; ++it) {

    octomap::point3d coords = octree_local_->keyToCoord(*it);
    octomap::KeyRay  key_ray;

    if (octree_local_->computeRayKeys(sensor_origin, coords, key_ray)) {

      octomap::KeyRay::iterator alterantive_ray_end = key_ray.end();

      for (octomap::KeyRay::iterator it2 = key_ray.begin(), end = key_ray.end(); it2 != end; ++it2) {

        if (occupied_cells.count(*it2)) {
          if (it2 == key_ray.begin()) {
            alterantive_ray_end = key_ray.begin();  // Special case
          } else {
            alterantive_ray_end = it2 - 1;
          }
          break;
        }
      }

      free_cells.insert(key_ray.begin(), alterantive_ray_end);
    }
  }

  // FREE CELLS
  for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
    octree_local_->updateNode(*it, octree_local_->getProbMissLog());
  }

  // OCCUPIED CELLS
  for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; ++it) {
    octree_local_->updateNode(*it, octree_local_->getProbHitLog());
  }

  // CROP THE MAP AROUND THE ROBOT
  {
    pairs_lib::ScopeTimer timer = pairs_lib::ScopeTimer(node_, "OctomapServer::localMapCopy", scope_timer_logger_, _scope_timer_enabled_);

    auto [local_map_width, local_map_height] = pairs_lib::get_mutexed(mutex_local_map_dimensions_, local_map_width_, local_map_height_);

    float x        = sensor_origin.x();
    float y        = sensor_origin.y();
    float z        = sensor_origin.z();
    float width_2  = local_map_width / 2.0;
    float height_2 = local_map_height / 2.0;

    octomap::point3d roi_min(x - width_2, y - width_2, z - height_2);
    octomap::point3d roi_max(x + width_2, y + width_2, z + height_2);

    std::shared_ptr<OcTree_t> from;

    if (octree_local_idx_ == 0) {
      from              = octree_local_0_;
      octree_local_     = octree_local_1_;
      octree_local_idx_ = 1;
    } else {
      from              = octree_local_1_;
      octree_local_     = octree_local_0_;
      octree_local_idx_ = 0;
    }

    octree_local_->clear();

    copyInsideBBX2(from, octree_local_, roi_min, roi_max);
  }

  // Set free space in the bounding box specified by clear_box topic
  /* if (sh_clear_box_.hasMsg()) { */

  /*   pairs_modules_msgs::msg::PoseWithSize pws = *sh_clear_box_.getMsg(); */

  /*   if ((clock_->now() - pws.header.stamp).seconds() < 1.0) { */

  /*     geometry_msgs::msg::PoseStamped pose_stamped; */
  /*     pose_stamped.header = pws.header; */
  /*     pose_stamped.pose   = pws.pose; */
  /*     auto res            = transformer_->transformSingle(pose_stamped, _world_frame_); */

  /*     if (res) { */

  /*       auto pose = res.value(); */
  /*       // calculate bounding box around the odometry */
  /*       double resolution = octree_local_->getResolution(); */
  /*       double min_x      = pose.pose.position.x - pws.width / 2 - resolution; */
  /*       double max_x      = pose.pose.position.x + pws.width / 2 + resolution; */
  /*       double min_y      = pose.pose.position.y - pws.width / 2 - resolution; */
  /*       double max_y      = pose.pose.position.y + pws.width / 2 + resolution; */
  /*       double min_z      = pose.pose.position.z - pws.height / 2 - resolution; */
  /*       double max_z      = pose.pose.position.z + pws.height / 2 + resolution; */
  /*       double step       = resolution / 2; */

  /*       for (double x = min_x; x < max_x; x += step) { */
  /*         for (double y = min_y; y < max_y; y += step) { */
  /*           for (double z = min_z; z < max_z; z += step) { */
  /*             octree_local_->setNodeValue(x, y, z, octomap::logodds(0.0)); */
  /*           } */
  /*         } */
  /*       } */

  /*     } else { */
  /*       RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Unable to transform the pose to be cleared from frame %s to frame %s.", */
  /*                            pws.header.frame_id.c_str(), _world_frame_.c_str()); */
  /*     } */
  /*   } else { */
  /*     RCLCPP_WARN_THROTTLE(node_->get_logger(), *clock_, 1000, "Latest pose from clear_box is too old - diff from now: %.3f", */
  /*                          (clock_->now() - pws.header.stamp).seconds()); */
  /*   } */
  /* } */

  octree_local_->setNodeValue(sensor_origin.x(), sensor_origin.y(), sensor_origin.z(), octomap::logodds(0.0));
  rclcpp::Time time_end = clock_->now();

  {
    std::scoped_lock lock(mutex_local_map_duty_);
    local_map_duty_ += (time_end - time_start).seconds();
  }
}

//}

/* initializeLidarLUT() //{ */

void OctomapServer::initialize3DLidarLUT(xyz_lut_t& lut, const SensorParams3DLidar_t sensor_params) {

  const int rangeCount         = sensor_params.horizontal_rays;
  const int verticalRangeCount = sensor_params.vertical_rays;

  std::vector<std::tuple<double, double, double>> coord_coeffs;

  const double minAngle = 0.0;
  const double maxAngle = 2.0 * M_PI;

  const double verticalMinAngle = -sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = sensor_params.vertical_fov / 2.0;

  const double yDiff = maxAngle - minAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  double yAngle_step = yDiff / (rangeCount - 1);

  double pAngle_step;

  if (verticalRangeCount > 1) {
    pAngle_step = pDiff / (verticalRangeCount - 1);
  } else {
    pAngle_step = 0;
  }

  coord_coeffs.reserve(rangeCount * verticalRangeCount);

  for (int i = 0; i < rangeCount; i++) {
    for (int j = 0; j < verticalRangeCount; j++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + minAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);

      coord_coeffs.push_back({x_coeff, y_coeff, z_coeff});
    }
  }

  int it = 0;
  lut.directions.resize(3, rangeCount * verticalRangeCount);
  lut.offsets.resize(3, rangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < rangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col * verticalRangeCount + row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* initializeDepthCamLUT() //{ */

void OctomapServer::initializeDepthCamLUT(xyz_lut_t& lut, const SensorParamsDepthCam_t sensor_params) {

  const int horizontalRangeCount = sensor_params.horizontal_rays;
  const int verticalRangeCount   = sensor_params.vertical_rays;

  RCLCPP_INFO(node_->get_logger(), "initializing depth camera lut, res %d x %d = %d points", horizontalRangeCount, verticalRangeCount,
              horizontalRangeCount * verticalRangeCount);

  std::vector<std::tuple<double, double, double>> coord_coeffs;

  // yes it is flipped, pixel [0, 0] is top-left
  const double horizontalMinAngle = sensor_params.horizontal_fov / 2.0;
  const double horizontalMaxAngle = -sensor_params.horizontal_fov / 2.0;

  const double verticalMinAngle = sensor_params.vertical_fov / 2.0;
  const double verticalMaxAngle = -sensor_params.vertical_fov / 2.0;

  const double yDiff = horizontalMaxAngle - horizontalMinAngle;
  const double pDiff = verticalMaxAngle - verticalMinAngle;

  Eigen::Quaterniond rot = Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(0.5 * M_PI, Eigen::Vector3d::UnitZ());

  double yAngle_step = yDiff / (horizontalRangeCount - 1);

  double pAngle_step;
  if (verticalRangeCount > 1) {
    pAngle_step = pDiff / (verticalRangeCount - 1);
  } else {
    pAngle_step = 0;
  }

  coord_coeffs.reserve(horizontalRangeCount * verticalRangeCount);

  for (int j = 0; j < verticalRangeCount; j++) {
    for (int i = 0; i < horizontalRangeCount; i++) {

      // Get angles of ray to get xyz for point
      const double yAngle = i * yAngle_step + horizontalMinAngle;
      const double pAngle = j * pAngle_step + verticalMinAngle;

      const double x_coeff = cos(pAngle) * cos(yAngle);
      const double y_coeff = cos(pAngle) * sin(yAngle);
      const double z_coeff = sin(pAngle);

      Eigen::Vector3d p(x_coeff, y_coeff, z_coeff);

      p = rot * p;

      [[maybe_unused]] double r = (double)(i) / horizontalRangeCount;
      [[maybe_unused]] double g = (double)(j) / horizontalRangeCount;

      coord_coeffs.push_back({p.x(), p.y(), p.z()});
    }
  }

  int it = 0;
  lut.directions.resize(3, horizontalRangeCount * verticalRangeCount);
  lut.offsets.resize(3, horizontalRangeCount * verticalRangeCount);

  for (int row = 0; row < verticalRangeCount; row++) {
    for (int col = 0; col < horizontalRangeCount; col++) {
      const auto [x_coeff, y_coeff, z_coeff] = coord_coeffs.at(col + horizontalRangeCount * row);
      lut.directions.col(it)                 = vec3_t(x_coeff, y_coeff, z_coeff);
      lut.offsets.col(it)                    = vec3_t(0, 0, 0);
      it++;
    }
  }
}

//}

/* copyInsideBBX2() //{ */

bool OctomapServer::copyInsideBBX2(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to, const octomap::point3d& p_min,
                                   const octomap::point3d& p_max) {

  octomap::OcTreeKey minKey, maxKey;

  if (!from->coordToKeyChecked(p_min, minKey) || !from->coordToKeyChecked(p_max, maxKey)) {
    return false;
  }

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, octomap::logodds(0.0));
  }

  for (OcTree_t::leaf_bbx_iterator it = from->begin_leafs_bbx(p_min, p_max), end = from->end_leafs_bbx(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  return true;
}

//}

/* copyLocalMap() //{ */

bool OctomapServer::copyLocalMap(std::shared_ptr<OcTree_t>& from, std::shared_ptr<OcTree_t>& to) {

  octomap::OcTreeKey minKey, maxKey;

  octomap::OcTreeNode* root = to->getRoot();

  bool got_root = root ? true : false;

  if (!got_root) {
    octomap::OcTreeKey key = to->coordToKey(0, 0, 0, to->getTreeDepth());
    to->setNodeValue(key, octomap::logodds(0.0));
  }

  for (OcTree_t::leaf_iterator it = from->begin_leafs(), end = from->end_leafs(); it != end; ++it) {

    octomap::OcTreeKey   k    = it.getKey();
    octomap::OcTreeNode* node = touchNode(to, k, it.getDepth());
    node->setValue(it->getValue());
  }

  return true;
}

//}

/* touchNode() //{ */

octomap::OcTreeNode* OctomapServer::touchNode(std::shared_ptr<OcTree_t>& octree, const octomap::OcTreeKey& key, unsigned int target_depth = 0) {

  return touchNodeRecurs(octree, octree->getRoot(), key, 0, target_depth);
}

//}

/* touchNodeRecurs() //{ */

octomap::OcTreeNode* OctomapServer::touchNodeRecurs(std::shared_ptr<OcTree_t>& octree, octomap::OcTreeNode* node, const octomap::OcTreeKey& key,
                                                    unsigned int depth, unsigned int max_depth = 0) {

  assert(node);

  // follow down to last level
  if (depth < octree->getTreeDepth() && (max_depth == 0 || depth < max_depth)) {

    unsigned int pos = octomap::computeChildIdx(key, int(octree->getTreeDepth() - depth - 1));

    /* ROS_INFO("pos: %d", pos); */
    if (!octree->nodeChildExists(node, pos)) {

      // not a pruned node, create requested child
      octree->createNodeChild(node, pos);
    }

    return touchNodeRecurs(octree, octree->getNodeChild(node, pos), key, depth + 1, max_depth);
  }

  // at last level, update node, end of recursion
  else {
    return node;
  }
}

//}

//}

}  // namespace pairs_octomap_server

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pairs_octomap_server::OctomapServer)
