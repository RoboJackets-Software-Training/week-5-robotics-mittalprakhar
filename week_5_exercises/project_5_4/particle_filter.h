//
// Created by jason on 10/17/20.
//
#pragma once

#include <ros/ros.h>
#include <random>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl_ros/point_cloud.h>

struct Particle {
  double x = 0;
  double y = 0;
  double yaw = 0;
  double vx = 0;
  double vy = 0;
};

class ParticleFilter {
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;

  ros::Publisher state_pub_;
  ros::Publisher particle_pub_;
  ros::Publisher pc_pub_;
  int num_particles_;
  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  std::vector<Particle> particles_;
  std::vector<double> weights_;
  std::vector<double> search_weights_;
  std::vector<int> sorted_indexes_;

  pcl::PointCloud<pcl::PointXYZRGB> point_cloud_;

  double sigma_x_, sigma_y_, sigma_yaw_, sigma_vx_, sigma_vy_;
  double gps_cov_;

  bool use_gps_ = true;

  int resample_frequency_ = 10;
  double max_weight_ = 0;
  double min_weight_ = 1;

  std::default_random_engine generator_;
  std::normal_distribution<double> distribution_;
  std::uniform_real_distribution<double> uniform_dist_ = std::uniform_real_distribution<double>(0.0,1.0);

  ros::Time last_imu_time_ = ros::Time(0);
  nav_msgs::Odometry state_;


public:
  ParticleFilter() {
    // TODO grab params
    pNh = ros::NodeHandle("~");

    gps_sub_ = nh.subscribe("/oswin/fix", 1, &ParticleFilter::gpsCallback, this);
    imu_sub_ = nh.subscribe("/oswin/imu", 100, &ParticleFilter::imuCallback, this);

    state_pub_ = pNh.advertise<nav_msgs::Odometry>("pose_estimate", 1);
    particle_pub_ = pNh.advertise<geometry_msgs::PoseArray>("particles", 1);
    pc_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>> ("points", 1);
    distribution_ = std::normal_distribution<double>(0, 1.0);

    pNh.param("resampling_frequency", resample_frequency_, 1);

    pNh.param("use_gps", use_gps_, true);

    srand (100);

    initParticles();
  }

  void initParticles();

  void gpsCallback(const geometry_msgs::PointStampedConstPtr& msg);
  void gpsSensorUpdate(const geometry_msgs::Point& msg);

  void imuCallback(const sensor_msgs::ImuConstPtr& msg);
  void imuMotionUpdate(const geometry_msgs::Vector3 linear_acc,
                       const geometry_msgs::Vector3& angular_vel, const double dt);
  void propagateParticle(Particle& p, const geometry_msgs::Vector3& linear_acc,
                         const geometry_msgs::Vector3& angular_vel, const double dt);

  void resampleParticles();
  void calculateState();
  void noramlizeWeights();
  std::vector<int> sortIndexes(const std::vector<double>& v);
  void publishParticles();


  //1-2-3 Euler Angle to Quaternion Conversion
  void EulerToQuat(double r, double p, double y, double& q0, double& q1, double& q2, double& q3)
  {
    double phi = r;
    double theta = p;
    double psi = y;
    q0 = cos(phi/2)*cos(theta/2)*cos(psi/2) + sin(phi/2)*sin(theta/2)*sin(psi/2);
    q1 = -cos(phi/2)*sin(theta/2)*sin(psi/2) + cos(theta/2)*cos(psi/2)*sin(phi/2);
    q2 = cos(phi/2)*cos(psi/2)*sin(theta/2) + sin(phi/2)*cos(theta/2)*sin(psi/2);
    q3 = cos(phi/2)*cos(theta/2)*sin(psi/2) - sin(phi/2)*cos(psi/2)*sin(theta/2);
  }

};

