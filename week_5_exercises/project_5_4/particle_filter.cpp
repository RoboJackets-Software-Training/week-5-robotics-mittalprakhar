//
// Created by jason on 10/17/20.
//

#include "particle_filter.h"

void ParticleFilter::initParticles() {
  // TODO have multiple based off of swtich

  pNh.param("num_particles", num_particles_, 100);
  weights_ = std::vector<double>(num_particles_);
  for(int i = 0; i < num_particles_; i++) {
    weights_[i] = 1.0/num_particles_;
  }
  particles_ = std::vector<Particle>(num_particles_);
  search_weights_ = std::vector<double>(num_particles_);

  int init_type;
  pNh.param("init_type", init_type, 0);
  double x_mean, x_var, y_mean, y_var, yaw_mean, yaw_var;
  pNh.param("init_x_mean", x_mean, 0.0);
  pNh.param("init_x_var", x_var, 1.0);
  pNh.param("init_y_mean", y_mean, 0.0);
  pNh.param("init_y_var", y_var, 1.0);
  pNh.param("init_yaw_mean", yaw_mean, 0.0);
  pNh.param("init_yaw_var", yaw_var, 0.5);

  pNh.param("imu_sigma_x", sigma_x_, 0.2);
  pNh.param("imu_sigma_y", sigma_y_, 0.2);
  pNh.param("imu_sigma_yaw", sigma_yaw_, 0.1);
  pNh.param("imu_sigma_vx", sigma_vx_, 0.1);
  pNh.param("imu_sigma_vy", sigma_vy_, 0.1);

  pNh.param("gps_cov", gps_cov_, 0.1);

  switch (init_type) {
    case 0:
      // TODO standard init
      for(int i = 0; i < num_particles_; i++) {
        Particle particle;
        particle.x = x_mean + distribution_(generator_)*x_var;
        particle.y = y_mean + distribution_(generator_)*y_var;
        particle.yaw = yaw_mean + distribution_(generator_)*yaw_var;
        particles_[i] = particle;
      }
      break;
    case 1:
      // TODO all facing one direction
      break;
    case 2:
      // TODO
      break;
  }
}


void ParticleFilter::gpsCallback(const geometry_msgs::PointStampedConstPtr& msg) {
  gpsSensorUpdate(msg->point);
  // TODO publish state with GPS measurement
  if(msg->header.seq % resample_frequency_ == 0) {
    resampleParticles();
  }
  calculateState();
}

void ParticleFilter::gpsSensorUpdate(const geometry_msgs::Point& point) {
  double log_normalizer = log(sqrt(pow(2*M_PI, 2))) + log(sqrt(gps_cov_)) + log(sqrt(gps_cov_));
  //std::cout << "gps: " << point.x << ", " << point.y << std::endl;
  //std::cout << "log norm: " << log_normalizer << std::endl;

  for(int i = 0; i < this->particles_.size(); i++) {
    double logProb = 0;
    Particle p = this->particles_[i];
    //std::cout << "\nparticle: " << p.x << ", " << p.y << std::endl;
    logProb += pow(point.x - p.x, 2)/gps_cov_;
    logProb += pow(point.y - p.y, 2)/gps_cov_;
    //std::cout << "logProb: " << logProb << std::endl;
    logProb *= -0.5;
    logProb -= log_normalizer;
    weights_[i] = exp(logProb);
    //std::cout << "weights[" << i << "]: " << weights_[i] << std::endl;
  }
  noramlizeWeights();
  publishParticles();
}

void ParticleFilter::imuCallback(const sensor_msgs::ImuConstPtr& msg) {
  // if it is the first message use the initial time
  if(last_imu_time_.toSec() == 0) {
    last_imu_time_ = msg->header.stamp;
    return;
  }

  double dt = msg->header.stamp.toSec() - last_imu_time_.toSec();
  last_imu_time_ = msg->header.stamp;

  imuMotionUpdate(msg->linear_acceleration, msg->angular_velocity, dt);
}

void ParticleFilter::imuMotionUpdate(const geometry_msgs::Vector3 linear_acc,
                                     const geometry_msgs::Vector3& angular_vel, const double dt) {
  for(int i = 0; i < num_particles_; i++) {
    propagateParticle(particles_[i], linear_acc, angular_vel, dt);
  }
}

void ParticleFilter::propagateParticle(Particle& p, const geometry_msgs::Vector3& linear_acc,
                                       const geometry_msgs::Vector3& angular_vel, const double dt) {
  p.x = p.x + p.vx*dt + sigma_x_*distribution_(generator_)*sqrt(dt);
  p.y = p.y + p.vy*dt + sigma_y_*distribution_(generator_)*sqrt(dt);

  //std::cout << "yaw: " << p.yaw << ", " << sin(p.yaw) << std::endl;
  double ux = linear_acc.x*dt + sigma_vx_*distribution_(generator_)*sqrt(dt);
  double uy = linear_acc.y*dt + sigma_vy_*distribution_(generator_)*sqrt(dt);
  //std::cout << linear_acc.x << ", " << linear_acc.y << std::endl;
  //std::cout << "x = " << cos(p.yaw)*ux + sin(p.yaw)*uy << std::endl;
  //std::cout << "y = " << sin(p.yaw)*ux + cos(p.yaw)*uy << std::endl;
  p.vx = p.vx + cos(p.yaw)*ux + sin(p.yaw)*uy;
  //std::cout << "before: " << p.vy << std::endl;
  p.vy = p.vy + sin(p.yaw)*ux + cos(p.yaw)*uy;
  //std::cout << "after: " << p.vy << std::endl;
  //std::cout << "yaw = " << angular_vel.z*dt << std::endl;
  p.yaw = p.yaw + angular_vel.z*dt + sigma_yaw_*distribution_(generator_)*sqrt(dt);
}

void ParticleFilter::resampleParticles() {
  std::vector<Particle> new_particles;
  for(int i = 0; i < num_particles_; i++) {
    // here do a binary search to find what particle we want to sample
    double target_val = uniform_dist_(generator_);
    int left_idx = 0;
    int right_idx = num_particles_-1;
    int cur_index = (left_idx+right_idx)/2;

    bool found_sol = false;
    //std::cout << "\nsearching for " << target_val << std::endl;
    while(left_idx <= right_idx && !found_sol) {
      cur_index = (left_idx+right_idx)/2;
      double cur_val = search_weights_[cur_index];
      //std::cout << "cur_index: " << cur_index << " (" << left_idx << ", " << right_idx << ")" << std::endl;
      //std::cout << "cur_val: " << cur_val << std::endl;

      if(cur_index >= num_particles_-1 || cur_index <= 0) {
        // we have reached either end of the search
        //std::cout << "we have reached the bounds" << std::endl;
        found_sol = true;
      } else if(target_val >= search_weights_[cur_index-1] && target_val <= cur_val) {
        // we are at the correct value
        //std::cout << search_weights_[cur_index-1] << ", "
        // << ": We are in the middle, use current index" << std::endl;
        found_sol = true;
      } else if(cur_val < target_val) {
        //std::cout << "search to the right" << std::endl;
        // search to the right
        left_idx = cur_index + 1;
      } else if(cur_val > target_val) {
        //std::cout << "search left" << std::endl;
        // search to the left
        right_idx = cur_index - 1;
      }
    }
    int index = sorted_indexes_[cur_index];
    new_particles.push_back(particles_[index]);
    //std::cout << "sampling particle " << index << ": " << new_particles[i].x << ", " << new_particles[i].y << std::endl;
  }
  particles_ = new_particles;
}

void ParticleFilter::calculateState() {
  state_.header.stamp = last_imu_time_;
  state_.header.frame_id = "odom";

  // calculate the estimated state
  state_.pose.pose.position.x = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.pose.pose.position.x += particles_[i].x*weights_[i];
  }
  state_.pose.pose.position.y = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.pose.pose.position.y += particles_[i].y*weights_[i];
  }
  double yaw = 0;
  for(int i = 0; i < particles_.size(); i++) {
    yaw += particles_[i].yaw*weights_[i];
  }
  double q0, q1, q2, q3;
  EulerToQuat(0, 0, yaw, q0, q1, q2, q3);
  state_.pose.pose.orientation.w = q0;
  state_.pose.pose.orientation.x = q1;
  state_.pose.pose.orientation.y = q2;
  state_.pose.pose.orientation.z = q3;

  state_.twist.twist.linear.x = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.twist.twist.linear.x += particles_[i].vx*weights_[i];
  }
  state_.twist.twist.linear.y = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.twist.twist.linear.y += particles_[i].vy*weights_[i];
  }

  // calculate the estimated covaraince
  state_.pose.covariance[0] = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.pose.covariance[0] += pow(state_.pose.pose.position.x - particles_[i].x, 2)*weights_[i];
  }
  state_.pose.covariance[7] = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.pose.covariance[7] += pow(state_.pose.pose.position.y - particles_[i].y, 2)*weights_[i];
  }
  state_.twist.covariance[0] = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.twist.covariance[0] += pow(state_.twist.twist.linear.x - particles_[i].vx, 2)*weights_[i];
  }
  state_.twist.covariance[7] = 0;
  for(int i = 0; i < particles_.size(); i++) {
    state_.twist.covariance[7] += pow(state_.twist.twist.linear.y - particles_[i].vy, 2)*weights_[i];
  }

  state_pub_.publish(state_);
}

void ParticleFilter::noramlizeWeights() {
  // sort the indexes of the particles
  sorted_indexes_ = sortIndexes(weights_);
  max_weight_ = 0;
  min_weight_ = 1;

  double sum = 0;
  for(int i = 0; i < weights_.size(); i++) {
    sum += weights_[i];
  }
  for(int i = 0; i < weights_.size(); i++) {
    weights_[i] /= sum;
    max_weight_ = std::max(max_weight_, weights_[i]);
    min_weight_ = std::min(min_weight_, weights_[i]);
    //std::cout << "weights[" << i << "]: " << weights_[i] << std::endl;
  }
  double search_weights_sum = 0;
  for(int i = 0; i < weights_.size(); i++) {
    search_weights_sum += weights_[sorted_indexes_[i]];
    search_weights_[i] = search_weights_sum;
  }
}

std::vector<int> ParticleFilter::sortIndexes(const std::vector<double> &v) {
  // initialize original index locations
  std::vector<int> idx(v.size());
  iota(idx.begin(), idx.end(), 0);
  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](int i1, int i2) {return v[i1] < v[i2];});
  return idx;
}

void ParticleFilter::publishParticles() {
  geometry_msgs::PoseArray array;
  array.header.frame_id = "odom";
  array.header.stamp = last_imu_time_;

  point_cloud_.clear();
  point_cloud_.header.frame_id = "odom";
  //std::cout << max_weight_ << std::endl;
  for(int i = 0; i < particles_.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position.x = particles_[i].x;
    pose.position.y = particles_[i].y;

    double yaw = particles_[i].yaw;
    double q0, q1, q2, q3;
    EulerToQuat(0, 0, yaw, q0, q1, q2, q3);
    pose.orientation.w = q0;
    pose.orientation.x = q1;
    pose.orientation.y = q2;
    pose.orientation.z = q3;

    array.poses.push_back(pose);

    int color = (weights_[i]-min_weight_)*(255/(max_weight_-min_weight_));
    pcl::PointXYZRGB point(color, 0, 0);
    point.x = particles_[i].x;
    point.y = particles_[i].y;
    point_cloud_.push_back(point);
  }
  pc_pub_.publish(point_cloud_);
  particle_pub_.publish(array);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "particle_filter");
  ParticleFilter pf;

  ros::spin();
}
