/*
 * node.cpp
 *
 *  This file is part of imu_covariance.
 *
 *	Created on: 31/08/2014
 */

#include <imu_covariance/node.hpp>
#include <sstream>

namespace imu_covariance {

Node::Node(const ros::NodeHandle &nh, const ros::NodeHandle &pnh)
    : nh_(nh), pnh_(pnh) {
  if (!pnh.hasParam("noise_std")) {
    ROS_ERROR("You must specify the covariance");
    throw std::invalid_argument("No config specified");
  }

  ros::NodeHandle stdNh(pnh, "noise_std");
  stdNh.param<double>("linear_acceleration/x", accelStd_[0], 0.2);
  stdNh.param<double>("linear_acceleration/y", accelStd_[1], 0.2);
  stdNh.param<double>("linear_acceleration/z", accelStd_[2], 0.2);

  stdNh.param<double>("angular_velocity/x", gyroStd_[0], 0.1);
  stdNh.param<double>("angular_velocity/y", gyroStd_[1], 0.1);
  stdNh.param<double>("angular_velocity/z", gyroStd_[2], 0.1);

  stdNh.param<double>("magnetic_field/x", fieldStd_[0], 0.1);
  stdNh.param<double>("magnetic_field/y", fieldStd_[1], 0.1);
  stdNh.param<double>("magnetic_field/z", fieldStd_[2], 0.1);

  stdNh.param<double>("fluid_pressure", pressureStd_, 1.0);
  ROS_INFO("linear acceleration: %f, %f, %f", accelStd_[0], accelStd_[1],
           accelStd_[2]);
  ROS_INFO("angular velocity:    %f, %f, %f", gyroStd_[0], gyroStd_[1],
           gyroStd_[2]);
  ROS_INFO("magnetic field:      %f, %f, %f", fieldStd_[0], fieldStd_[1],
           fieldStd_[2]);
  ROS_INFO("fluid pressure:      %f", pressureStd_);

  // inputs, subscribe to topics under imu namespace
  subImu_ = pnh_.subscribe("imu", kROSQueueSize, &Node::imuCallback, this);
  subMagneticField_ = pnh_.subscribe("magnetic_field", kROSQueueSize,
                                     &Node::magneticFieldCallback, this);
  subPressure_ = pnh_.subscribe("pressure", kROSQueueSize,
                                &Node::fluidPressureCallback, this);
  // outputs, advertice topics with same names under imu/imu_covariance
  // namespace
  pubImu_ = pnh_.advertise<sensor_msgs::Imu>("imu_cov", 1);
  pubMagneticField_ =
      pnh_.advertise<sensor_msgs::MagneticField>("magnetic_field_cov", 1);
  pubPressure_ = pnh_.advertise<sensor_msgs::FluidPressure>("pressure_cov", 1);
}

void Node::imuCallback(const sensor_msgs::ImuConstPtr &imuMsg) {
  sensor_msgs::Imu imu = *imuMsg;
  imu.linear_acceleration_covariance[0] = accelStd_[0] * accelStd_[0];
  imu.linear_acceleration_covariance[4] = accelStd_[1] * accelStd_[1];
  imu.linear_acceleration_covariance[8] = accelStd_[2] * accelStd_[2];
  imu.angular_velocity_covariance[0] = gyroStd_[0] * gyroStd_[0];
  imu.angular_velocity_covariance[4] = gyroStd_[1] * gyroStd_[1];
  imu.angular_velocity_covariance[8] = gyroStd_[2] * gyroStd_[2];
  pubImu_.publish(imu);
}

void Node::magneticFieldCallback(
    const sensor_msgs::MagneticFieldConstPtr &magMsg) {
  sensor_msgs::MagneticField field = *magMsg;
  field.magnetic_field_covariance[0] = fieldStd_[0] * fieldStd_[0];
  field.magnetic_field_covariance[4] = fieldStd_[1] * fieldStd_[1];
  field.magnetic_field_covariance[8] = fieldStd_[2] * fieldStd_[2];
  pubMagneticField_.publish(field);
}

void Node::fluidPressureCallback(
    const sensor_msgs::FluidPressureConstPtr &pressureMsg) {
  sensor_msgs::FluidPressure pressure = *pressureMsg;
  pressure.variance = pressureStd_ * pressureStd_;
  pubPressure_.publish(pressure);
}

}  //  imu_covariance
