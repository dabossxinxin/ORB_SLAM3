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

#include "StereoInertialNode.h"

#include <pcl_conversions/pcl_conversions.h>

double stamp2Sec(const builtin_interfaces::msg::Time& stamp) {
  return rclcpp::Time(stamp).seconds();
}

rclcpp::Time sec2Stamp(double timestamp) {
  int32_t sec = std::floor(timestamp);
  auto nanosec_d = (timestamp - std::floor(timestamp)) * 1e9;
  uint32_t nanosec = nanosec_d;
  return rclcpp::Time(sec, nanosec);
}

ImageGrabber::ImageGrabber(ORB_SLAM3::System* pSLAM, const bool bRect,
                           const bool bClahe)
    : rclcpp::Node("image_grabber")
    , mpSLAM(pSLAM)
    , mbClahe(bClahe)
    , mbRectify(bRect) {
  rclcpp::QoS qos_profile_img(100);
  rclcpp::QoS qos_profile_imu(1000);
  qos_profile_img.reliability(rclcpp::ReliabilityPolicy::Reliable);
  qos_profile_imu.reliability(rclcpp::ReliabilityPolicy::BestEffort);

  left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra1/image_rect_raw", qos_profile_img,
      std::bind(&ImageGrabber::GrabImageLeft, this, std::placeholders::_1));
  right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/infra2/image_rect_raw", qos_profile_img,
      std::bind(&ImageGrabber::GrabImageRight, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/camera/camera/imu", qos_profile_imu,
      std::bind(&ImageGrabber::GrabImu, this, std::placeholders::_1));
  dense_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/camera/depth/color/points", qos_profile_img,
      std::bind(&ImageGrabber::GrabDenseCloud, this, std::placeholders::_1));

  pos_pub_ =
      this->create_publisher<nav_msgs::msg::Odometry>("/orb_slam3/odom", 10);
  path_pub_ =
      this->create_publisher<nav_msgs::msg::Path>("/orb_slam3/path", 10);
  cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/orb_slam3/cloud", 10);
  dense_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/orb_slam3/dense_cloud", 10);

  // Start the publishing thread
  mPubThread = std::thread(&ImageGrabber::PublishWorkLoop, this);

  // Get from RealSenseD435i configuration
  mRotColor2Infra1 << 0.999989, -0.00137899, -0.00448667, 0.00137676, 0.999999,
      -0.000500959, 0.00448736, 0.000494776, 0.99999;
  mTranColor2Infra1 << -0.0148087656125426, -9.96115413727239e-05,
      -0.00037196927587501;

  mpCurrentDenseCloud = std::make_shared<PointCloudRGB>();
}

ImageGrabber::~ImageGrabber() {
  // end publishing thread
  {
    std::lock_guard<std::mutex> lock(mPubInfoMutex);
    mPubThreadRunning = false;
    mNewPoseAvailable = true;
  }
  mPubInfoCv.notify_one();
  if (mPubThread.joinable()) {
    mPubThread.join();
  }
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutexLeft);
  if (!imgLeftBuf.empty()) {
    imgLeftBuf.pop();
  }

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "Left image received, timestamp: %.3f, size: %dx%d, delay: %.3f",
              stamp2Sec(msg->header.stamp), msg->width, msg->height, delay);
  imgLeftBuf.push(msg);
}

void ImageGrabber::GrabImageRight(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutexRight);
  if (!imgRightBuf.empty()) {
    imgRightBuf.pop();
  }

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "Right image received, timestamp: %.3f, size: %dx%d, delay: %.3f",
              stamp2Sec(msg->header.stamp), msg->width, msg->height, delay);
  imgRightBuf.push(msg);
}

void ImageGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  std::lock_guard<std::mutex> lock(mBufMutexImu);

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(imu_msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "IMU data received, timestamp: %.3f, delay: %.3f",
              stamp2Sec(imu_msg->header.stamp), delay);
  imuBuf.push(imu_msg);
}

void ImageGrabber::GrabDenseCloud(
    const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg) {
  std::lock_guard<std::mutex> lock(mBufMutexDenseCloud);

  PointCloudRGB::Ptr cloud(new PointCloudRGB);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  double msg_timestamp = stamp2Sec(cloud_msg->header.stamp);
  double delay = this->get_clock()->now().seconds() - msg_timestamp;
  RCLCPP_INFO(this->get_logger(),
              "Dense cloud received, timestamp: %.3f, size: %zu, delay: %.3f",
              stamp2Sec(cloud_msg->header.stamp), cloud->points.size(), delay);

  denseCloudBuf.push(cloud);
  denseCloudTimeBuf.push(msg_timestamp);
}

cv::Mat ImageGrabber::GetImage(
    const sensor_msgs::msg::Image::SharedPtr img_msg) {
  try {
    return cv_bridge::toCvShare(img_msg)->image.clone();
  } catch (cv_bridge::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    return cv::Mat();
  }
}

void ImageGrabber::SyncWithImu() {
  const double maxTimeDiff = 0.03333;
  while (rclcpp::ok()) {
    cv::Mat imLeft, imRight;
    PointCloudRGB::Ptr denseCloud = std::make_shared<PointCloudRGB>();
    double tImLeft = 0, tImRight = 0, tDenseCloud = 0;
    {
      std::lock_guard<std::mutex> lockLeft(mBufMutexLeft);
      std::lock_guard<std::mutex> lockRight(mBufMutexRight);
      std::lock_guard<std::mutex> lockImu(mBufMutexImu);
      if (imgLeftBuf.empty() || imgRightBuf.empty() || imuBuf.empty()) {
        continue;
      }

      tImLeft = stamp2Sec(imgLeftBuf.front()->header.stamp);
      tImRight = stamp2Sec(imgRightBuf.front()->header.stamp);
      if (!imuBuf.empty() && tImLeft > stamp2Sec(imuBuf.back()->header.stamp)) {
        continue;
      }
    }

    {
      std::lock_guard<std::mutex> lockRight(mBufMutexRight);
      while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
        imgRightBuf.pop();
        tImRight = stamp2Sec(imgRightBuf.front()->header.stamp);
      }
    }

    {
      std::lock_guard<std::mutex> lockLeft(mBufMutexLeft);
      while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
        imgLeftBuf.pop();
        tImLeft = stamp2Sec(imgLeftBuf.front()->header.stamp);
      }
    }

    if (std::abs(tImLeft - tImRight) > maxTimeDiff)
      continue;

    {
      std::lock_guard<std::mutex> lockLeft(mBufMutexLeft);
      imLeft = GetImage(imgLeftBuf.front());
      imgLeftBuf.pop();
    }

    {
      std::lock_guard<std::mutex> lockRight(mBufMutexRight);
      imRight = GetImage(imgRightBuf.front());
      imgRightBuf.pop();
    }

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
      std::lock_guard<std::mutex> lockImu(mBufMutexImu);
      while (!imuBuf.empty() &&
             stamp2Sec(imuBuf.front()->header.stamp) <= tImLeft) {
        double t = stamp2Sec(imuBuf.front()->header.stamp);
        cv::Point3f acc(imuBuf.front()->linear_acceleration.x,
                        imuBuf.front()->linear_acceleration.y,
                        imuBuf.front()->linear_acceleration.z);
        cv::Point3f gyr(imuBuf.front()->angular_velocity.x,
                        imuBuf.front()->angular_velocity.y,
                        imuBuf.front()->angular_velocity.z);
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));
        imuBuf.pop();
      }
    }

    {
      std::lock_guard<std::mutex> lockDenseCloud(mBufMutexDenseCloud);
      while (!denseCloudBuf.empty() &&
             std::abs(denseCloudTimeBuf.front() - tImLeft) > maxTimeDiff) {
        denseCloudBuf.pop();
        denseCloudTimeBuf.pop();
      }
      if (!denseCloudBuf.empty() &&
          mpSLAM->GetAtlas()->GetCurrentMap()->GetIniertialBA2()) {
        denseCloud = denseCloudBuf.front();
        tDenseCloud = denseCloudTimeBuf.front();
        denseCloudBuf.pop();
        denseCloudTimeBuf.pop();
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Processing Left ts: %.3f, Right ts: %.3f, Imu ts: %.3f~%.3f, "
        "size: %zu, dense cloud ts: %.3f, size: %zu",
        tImLeft, tImRight, vImuMeas.size() > 0 ? vImuMeas.front().t : 0.0,
        vImuMeas.size() > 0 ? vImuMeas.back().t : 0.0, vImuMeas.size(),
        tDenseCloud, denseCloud != nullptr ? denseCloud->points.size() : 0);

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 =
        std::chrono::monotonic_clock::now();
#endif
    {
      std::lock_guard<std::mutex> lock(mTrackMutex);
      std::stringstream ss_filename;
      ss_filename << std::fixed << std::setprecision(6) << tImLeft;
      Sophus::SE3f currentPose = mpSLAM->TrackStereo(
          imLeft, imRight, tImLeft, vImuMeas, ss_filename.str());

      // Store the latest pose and notify the publishing thread
      {
        std::unique_lock<std::mutex> lockPubInfo(mPubInfoMutex,
                                                 std::try_to_lock);
        if (lockPubInfo) {
          mCurrentPose = currentPose;
          mCurrentTimestamp = tImLeft;
          mNewPoseAvailable = true;
          mpCurrentDenseCloud = denseCloud;
          mPubInfoCv.notify_one();
        }
      }
    }
#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t2 =
        std::chrono::monotonic_clock::now();
#endif
    double ttrack =
        std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1)
            .count();

    double T = 0;
    if (!imgLeftBuf.empty()) {
      double tframe = stamp2Sec(imgLeftBuf.front()->header.stamp);
      T = tframe - tImLeft;
    }

    if (ttrack < T) {
      std::this_thread::sleep_for(std::chrono::duration<double>(T - ttrack));
    } else {
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}

void ImageGrabber::PublishWorkLoop() {
  while (rclcpp::ok() && mPubThreadRunning) {
    std::unique_lock<std::mutex> lock(mPubInfoMutex);
    mPubInfoCv.wait(lock,
                    [this] { return mNewPoseAvailable || !mPubThreadRunning; });

    if (!mPubThreadRunning) {
      break;
    }
    mNewPoseAvailable = false;

    publishOdometryAndPath();
    publishSparseCloud();
    publishDenseCloud();
  }
}

void ImageGrabber::publishOdometryAndPath() {
  // --- Odometry ---
  nav_msgs::msg::Odometry odom_msg;
  double publishTime = stamp2Sec(this->get_clock()->now());
  odom_msg.header.stamp = sec2Stamp(publishTime);
  odom_msg.header.frame_id = "map";
  odom_msg.child_frame_id = "camera";

  const Eigen::Matrix3f Rwc = mCurrentPose.rotationMatrix().transpose();
  const Eigen::Vector3f twc = -Rwc * mCurrentPose.translation();
  const Eigen::Quaternionf quat = Eigen::Quaternionf(Rwc);

  odom_msg.pose.pose.position.x = twc.x();
  odom_msg.pose.pose.position.y = twc.y();
  odom_msg.pose.pose.position.z = twc.z();
  odom_msg.pose.pose.orientation.x = quat.x();
  odom_msg.pose.pose.orientation.y = quat.y();
  odom_msg.pose.pose.orientation.z = quat.z();
  odom_msg.pose.pose.orientation.w = quat.w();

  pos_pub_->publish(odom_msg);
  double delay = publishTime - mCurrentTimestamp;
  RCLCPP_INFO(this->get_logger(),
              "Published odometry at timestamp: %.3f, delay: %.3f", publishTime,
              delay);

  // --- Path ---
  geometry_msgs::msg::PoseStamped pose_stamped;
  pose_stamped.header = odom_msg.header;
  pose_stamped.pose = odom_msg.pose.pose;
  mTrajectory.poses.push_back(pose_stamped);

  constexpr size_t kMaxPathPoses = 10000;
  if (mTrajectory.poses.size() > kMaxPathPoses) {
    mTrajectory.poses.erase(
        mTrajectory.poses.begin(),
        mTrajectory.poses.begin() +
            static_cast<long>(mTrajectory.poses.size() - kMaxPathPoses));
  }

  mTrajectory.header.frame_id = "map";
  mTrajectory.header.stamp = odom_msg.header.stamp;
  path_pub_->publish(mTrajectory);
}

void ImageGrabber::publishSparseCloud() {
  // Get all map points from the current map
  const auto& vpMPs = mpSLAM->GetAtlas()->GetAllMapPoints();
  if (vpMPs.empty()) {
    return;
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  double publishTime = stamp2Sec(this->get_clock()->now());
  cloud_msg.header.stamp = sec2Stamp(publishTime);
  cloud_msg.header.frame_id = "map";
  cloud_msg.height = 1;
  cloud_msg.width = static_cast<uint32_t>(vpMPs.size());
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;
  cloud_msg.point_step = 16;  // 3 float (xyz) + 1 float (padding)
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;

  // Define fields: x, y, z
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud_msg.fields.push_back(field);

  field.name = "y";
  field.offset = 4;
  cloud_msg.fields.push_back(field);

  field.name = "z";
  field.offset = 8;
  cloud_msg.fields.push_back(field);

  // Pack point data
  cloud_msg.data.resize(cloud_msg.row_step);
  size_t idx = 0;
  for (const auto& pMP : vpMPs) {
    if (pMP->isBad()) {
      continue;
    }

    Eigen::Vector3f pos = pMP->GetWorldPos();
    float* data_ptr = reinterpret_cast<float*>(&cloud_msg.data[idx]);
    data_ptr[0] = pos.x();
    data_ptr[1] = pos.y();
    data_ptr[2] = pos.z();
    idx += cloud_msg.point_step;
  }

  // Update width to actual number of written points
  cloud_msg.width = static_cast<uint32_t>(idx / cloud_msg.point_step);
  cloud_msg.row_step = idx;
  cloud_msg.data.resize(idx);

  cloud_pub_->publish(cloud_msg);
  double delay = publishTime - mCurrentTimestamp;
  RCLCPP_INFO(
      this->get_logger(),
      "Published sparse cloud with %u points, timestamp: %.3f, delay: %.3f",
      cloud_msg.width, publishTime, delay);
}

void ImageGrabber::publishDenseCloud() {
  if (!mpCurrentDenseCloud || mpCurrentDenseCloud->empty()) {
    return;
  }

  const Eigen::Matrix3f Rwc = mCurrentPose.rotationMatrix().transpose();
  const Eigen::Vector3f twc = -Rwc * mCurrentPose.translation();
  Eigen::Matrix3f Rwi = Rwc * mRotColor2Infra1;         // rgb camera to world
  Eigen::Vector3f twi = Rwc * mTranColor2Infra1 + twc;  // rgb camera to world

  PointCloudRGB::Ptr denseCloudWorld = std::make_shared<PointCloudRGB>();
  denseCloudWorld->points.reserve(mpCurrentDenseCloud->points.size());
  for (auto& p : mpCurrentDenseCloud->points) {
    Eigen::Vector3f ptBody = Eigen::Vector3f(p.x, p.y, p.z);
    if (ptBody.norm() > mfDepthThreshold) {
      continue;
    }
    Eigen::Vector3f ptWorld = Rwi * ptBody + twi;

    pcl::PointXYZRGB pt = p;
    pt.x = ptWorld.x();
    pt.y = ptWorld.y();
    pt.z = ptWorld.z();
    denseCloudWorld->points.emplace_back(pt);
  }

  sensor_msgs::msg::PointCloud2 cloud_msg;
  pcl::toROSMsg(*denseCloudWorld, cloud_msg);
  double publishTime = stamp2Sec(this->get_clock()->now());
  cloud_msg.header.stamp = sec2Stamp(publishTime);
  cloud_msg.header.frame_id = "map";
  dense_cloud_pub_->publish(cloud_msg);
  double delay = publishTime - mCurrentTimestamp;
  RCLCPP_INFO(
      this->get_logger(),
      "Published dense cloud with %u points, timestamp: %.3f, delay: %.3f",
      cloud_msg.width, publishTime, delay);
}
