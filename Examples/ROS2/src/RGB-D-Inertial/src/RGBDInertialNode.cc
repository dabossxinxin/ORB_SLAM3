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

#include "RGBDInertialNode.h"

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

  rgb_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", qos_profile_img,
      std::bind(&ImageGrabber::GrabImageRgb, this, std::placeholders::_1));
  depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/aligned_depth_to_color/image_raw", qos_profile_img,
      std::bind(&ImageGrabber::GrabImageDepth, this, std::placeholders::_1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      "/camera/camera/imu", qos_profile_imu,
      std::bind(&ImageGrabber::GrabImu, this, std::placeholders::_1));
}

void ImageGrabber::GrabImageRgb(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutexRgb);
  if (!imgRgbBuf.empty())
    imgRgbBuf.pop();

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "RGB image received, timestamp: %.3f, size: %dx%d, delay: %.3f",
              stamp2Sec(msg->header.stamp), msg->width, msg->height, delay);
  imgRgbBuf.push(msg);
}

void ImageGrabber::GrabImageDepth(
    const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutexDepth);
  if (!imgDepthBuf.empty())
    imgDepthBuf.pop();

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "Depth image received, timestamp: %.3f, size: %dx%d, delay: %.3f",
              stamp2Sec(msg->header.stamp), msg->width, msg->height, delay);
  imgDepthBuf.push(msg);
}

void ImageGrabber::GrabImu(const sensor_msgs::msg::Imu::SharedPtr imu_msg) {
  std::lock_guard<std::mutex> lock(mBufMutex);

  double delay =
      this->get_clock()->now().seconds() - stamp2Sec(imu_msg->header.stamp);
  RCLCPP_INFO(this->get_logger(),
              "IMU data received, timestamp: %.3f, delay: %.3f",
              stamp2Sec(imu_msg->header.stamp), delay);
  imuBuf.push(imu_msg);
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
    cv::Mat imRgb, imDepth;
    double tImRgb = 0, tImDepth = 0;
    {
      std::lock_guard<std::mutex> lockRgb(mBufMutexRgb);
      std::lock_guard<std::mutex> lockDepth(mBufMutexDepth);
      std::lock_guard<std::mutex> lockImu(mBufMutex);
      if (imgRgbBuf.empty() || imgDepthBuf.empty() || imuBuf.empty()) {
        continue;
      }

      tImRgb = stamp2Sec(imgRgbBuf.front()->header.stamp);
      tImDepth = stamp2Sec(imgDepthBuf.front()->header.stamp);
      if (!imuBuf.empty() && tImRgb > stamp2Sec(imuBuf.back()->header.stamp)) {
        continue;
      }
    }

    {
      std::lock_guard<std::mutex> lock(mBufMutexDepth);
      while ((tImRgb - tImDepth) > maxTimeDiff && imgDepthBuf.size() > 1) {
        imgDepthBuf.pop();
        tImDepth = stamp2Sec(imgDepthBuf.front()->header.stamp);
      }
    }

    {
      std::lock_guard<std::mutex> lock(mBufMutexRgb);
      while ((tImDepth - tImRgb) > maxTimeDiff && imgRgbBuf.size() > 1) {
        imgRgbBuf.pop();
        tImRgb = stamp2Sec(imgRgbBuf.front()->header.stamp);
      }
    }

    if (std::abs(tImRgb - tImDepth) > maxTimeDiff)
      continue;

    {
      std::lock_guard<std::mutex> lock(mBufMutexRgb);
      imRgb = GetImage(imgRgbBuf.front());
      imgRgbBuf.pop();
    }

    {
      std::lock_guard<std::mutex> lock(mBufMutexDepth);
      imDepth = GetImage(imgDepthBuf.front());
      imgDepthBuf.pop();
    }

    std::vector<ORB_SLAM3::IMU::Point> vImuMeas;
    {
      std::lock_guard<std::mutex> lock(mBufMutex);
      while (!imuBuf.empty() &&
             stamp2Sec(imuBuf.front()->header.stamp) <= tImRgb) {
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

    RCLCPP_INFO(
        this->get_logger(),
        "Processing Depth ts: %.3f, RGB ts: %.3f, Imu ts: %.3f~%.3f, size: %zu",
        tImDepth, tImRgb, vImuMeas.size() > 0 ? vImuMeas.front().t : 0.0,
        vImuMeas.size() > 0 ? vImuMeas.back().t : 0.0, vImuMeas.size());

#ifdef COMPILEDWITHC11
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
    std::chrono::monotonic_clock::time_point t1 =
        std::chrono::monotonic_clock::now();
#endif
    {
      std::lock_guard<std::mutex> lock(mTrackMutex);
      std::stringstream ss_filename;
      ss_filename << std::fixed << std::setprecision(6) << tImRgb;
      mpSLAM->TrackRGBD(imRgb, imDepth, tImRgb, vImuMeas, ss_filename.str());
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
    if (!imgRgbBuf.empty()) {
      double tframe = stamp2Sec(imgRgbBuf.front()->header.stamp);
      T = tframe - tImRgb;
    }

    if (ttrack < T) {
      std::this_thread::sleep_for(std::chrono::duration<double>(T - ttrack));
    } else {
      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
    }
  }
}
