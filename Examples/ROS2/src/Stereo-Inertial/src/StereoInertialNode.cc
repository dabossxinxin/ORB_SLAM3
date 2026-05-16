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
}

void ImageGrabber::GrabImageLeft(const sensor_msgs::msg::Image::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mBufMutexLeft);
  if (!imgLeftBuf.empty())
    imgLeftBuf.pop();

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
  if (!imgRightBuf.empty())
    imgRightBuf.pop();

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
    double tImLeft = 0, tImRight = 0;
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
      mpSLAM->TrackStereo(imLeft, imRight, tImLeft, vImuMeas,
                          ss_filename.str());
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
