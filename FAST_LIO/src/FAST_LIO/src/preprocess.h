#ifndef PREPROCESS_H
#define PREPROCESS_H


#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;
/*激光雷达点云预处理，包含特征提取等*/
#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE{AVIA = 1, VELO16, OUST64, MARSIM}; //{1, 2, 3}
enum TIME_UNIT{SEC = 0, MS = 1, US = 2, NS = 3};
enum Feature{Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint};
enum Surround{Prev, Next};
enum E_jump{Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind};

//用于存储点的特征的结构体
struct orgtype
{
  double range;//距离
  double dista; //
  double angle[2];
  double intersect;
  E_jump edj[2];//边缘跳变信息
  Feature ftype;//特征类别
  orgtype()
  {
    range = 0;
    edj[Prev] = Nr_nor;
    edj[Next] = Nr_nor;
    ftype = Nor;
    intersect = 2;
  }
};

namespace velodyne_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (uint16_t, ring, ring)
)

namespace ouster_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      uint32_t t;
      uint16_t reflectivity;
      uint8_t  ring;
      uint16_t ambient;
      uint32_t range;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    // use std::uint32_t to avoid conflicting with pcl::uint32_t
    (std::uint32_t, t, t)
    (std::uint16_t, reflectivity, reflectivity)
    (std::uint8_t, ring, ring)
    (std::uint16_t, ambient, ambient)
    (std::uint32_t, range, range)
)

class Preprocess
{
  public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  Preprocess();
  ~Preprocess();
  
  void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//livox雷达数据处理
  void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//velodyne雷达数据处理
  void set(bool feat_en, int lid_type, double bld, int pfilt_num);//设置参数

  // sensor_msgs::PointCloud2::ConstPtr pointcloud;
  PointCloudXYZI pl_full, pl_corn, pl_surf;
  PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
  vector<orgtype> typess[128]; //maximum 128 line lidar 存储点特征的向量
  float time_unit_scale;
  int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;
  double blind;
  bool feature_enabled, given_offset_time;
  ros::Publisher pub_full, pub_surf, pub_corn;
    

  private:
  void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);//处理Livox雷达数据
  void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);//处理Ouster雷达数据
  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);//处理Velodyne雷达数据
  void sim_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);//处理模拟数据
  
  void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);//给点云添加特征
  void pub_func(PointCloudXYZI &pl, const ros::Time &ct);//发布点云
  int  plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex, Eigen::Vector3d &curr_direct);
  bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);
  
  int group_size;
  double disA, disB, inf_bound;
  double limit_maxmid, limit_midmin, limit_maxmin;
  double p2l_ratio;
  double jump_up_limit, jump_down_limit;
  double cos160;
  double edgea, edgeb;
  double smallp_intersect, smallp_ratio;
  double vx, vy, vz;
};
#endif