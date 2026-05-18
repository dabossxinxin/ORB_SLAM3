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

#include <iostream>
#include <opencv2/core/core.hpp>

#include "StereoInertialNode.h"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  if (argc < 3) {
    std::cerr << "Usage: ros2 run ORB_SLAM3 Stereo_inertial path_to_vocabulary "
                 "path_to_settings path_to_save_dir"
              << std::endl;
    return 1;
  }

  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::IMU_STEREO, true);
  auto image_grabber = std::make_shared<ImageGrabber>(&SLAM, false, false);

  cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
  if (!fsSettings.isOpened()) {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
    return -1;
  }

  std::thread sync_thread(&ImageGrabber::SyncWithImu, image_grabber);

  rclcpp::spin(image_grabber);
  rclcpp::shutdown();
  SLAM.Shutdown();

  if (sync_thread.joinable()) {
    sync_thread.join();
  }

  return 0;
}
