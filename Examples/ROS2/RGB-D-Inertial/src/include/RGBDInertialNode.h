/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef RGBD_INERTIAL_ROS2_H
#define RGBD_INERTIAL_ROS2_H

#include <cv_bridge/cv_bridge.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <opencv2/core/core.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <thread>
#include <vector>

#include "ImuTypes.h"
#include "System.h"

double stamp2Sec(const builtin_interfaces::msg::Time &stamp);
rclcpp::Time sec2Stamp(double timestamp);

class ImageGrabber : public rclcpp::Node {
 public:
  ImageGrabber(ORB_SLAM3::System *pSLAM, const bool bRect, const bool bClahe);

  void GrabImageRgb(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImageDepth(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
  void SyncWithImu();

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  std::queue<sensor_msgs::msg::Image::SharedPtr> imgRgbBuf;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgDepthBuf;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<nav_msgs::msg::Odometry::SharedPtr> odomBuf;

  std::mutex mBufMutexRgb;
  std::mutex mBufMutexDepth;
  std::mutex mBufMutex;
  std::mutex mBufMutexOdom;
  std::mutex mTrackMutex;

  ORB_SLAM3::System *mpSLAM;

  const bool mbClahe = false;
  const bool do_rectify = false;

  cv::Mat M1l, M2l, M1r, M2r;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif  // RGBD_INERTIAL_ROS2_H
