/*
 * node.hpp
 *
 *  This file is part of imu_covariance.
 *
 *	Created on: 31/08/2014
 */

#ifndef GALT_IMU_COVARIANCE_NODE_HPP_
#define GALT_IMU_COVARIANCE_NODE_HPP_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/FluidPressure.h>

namespace imu_covariance {

class Node {
 public:
  Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh);

 private:
  //  number of items in the input queues
  static constexpr int kROSQueueSize = 10;

  ros::NodeHandle nh_, pnh_;
  ros::Subscriber subImu_, subMagneticField_, subPressure_;
  ros::Publisher pubImu_, pubMagneticField_, pubPressure_;

  //  standard deviations on noise for each sensor
  double accelStd_[3];
  double gyroStd_[3];
  double fieldStd_[3];
  double pressureStd_;

  void imuCallback(const sensor_msgs::ImuConstPtr &imuMsg);
  void magneticFieldCallback(const sensor_msgs::MagneticFieldConstPtr &magMsg);
  void fluidPressureCallback(
      const sensor_msgs::FluidPressureConstPtr &pressureMsg);
};

}  //  imu_covariance

#endif  //  GALT_IMU_COVARIANCE_NODE_HPP_
