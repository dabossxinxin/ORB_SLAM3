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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <fstream>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <iostream>
#include <mutex>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <opencv2/core/core.hpp>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <thread>
#include <vector>

#include "ImuTypes.h"
#include "System.h"

double stamp2Sec(const builtin_interfaces::msg::Time& stamp);
rclcpp::Time sec2Stamp(double timestamp);

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudRGB;

class ImageGrabber : public rclcpp::Node {
public:
  ImageGrabber(ORB_SLAM3::System* pSLAM, const bool bRect, const bool bClahe);
  ~ImageGrabber();

  void GrabImageRgb(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImageDepth(const sensor_msgs::msg::Image::SharedPtr msg);
  void GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
  void GrabDenseCloud(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
  cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg);
  void SyncWithImu();

  void PublishWorkLoop();
  void publishOdometryAndPath();
  void publishSparseCloud();
  void publishDenseCloud();

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr
      dense_cloud_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dense_cloud_pub_;

  // --- Publishing thread synchronization ---
  Sophus::SE3f mCurrentPose;
  double mCurrentTimestamp = 0.0;
  bool mNewPoseAvailable = false;
  bool mPubThreadRunning = true;
  std::mutex mPubInfoMutex;
  std::condition_variable mPubInfoCv;
  nav_msgs::msg::Path mTrajectory;
  std::thread mPubThread;
  PointCloudRGB::Ptr mpCurrentDenseCloud;
  const float mfMaxDepthThres = 3.0f;
  const float mfMinDepthThres = 0.3f;

  // --- Subscribed data buffers ---
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgRgbBuf;
  std::queue<sensor_msgs::msg::Image::SharedPtr> imgDepthBuf;
  std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  std::queue<PointCloudRGB::Ptr> denseCloudBuf;
  std::queue<double> denseCloudTimeBuf;

  // --- Mutexes ---
  std::mutex mBufMutexRgb;
  std::mutex mBufMutexDepth;
  std::mutex mBufMutexImu;
  std::mutex mBufMutexDenseCloud;
  std::mutex mTrackMutex;

  ORB_SLAM3::System* mpSLAM;

  const bool mbClahe = false;
  const bool mbRectify = false;
  int mCloudPubCounter = 0;

  cv::Mat M1l, M2l, M1r, M2r;
  cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

#endif  // RGBD_INERTIAL_ROS2_H
