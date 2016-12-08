/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "marker_detection.h"
#include <ros/console.h>

namespace aruco_marker_recognition {

MarkerDetection::MarkerDetection(double marker_size) : marker_size_(marker_size) {

}

std::vector<aruco::Marker> MarkerDetection::detect(const cv::Mat &image, CameraId id) {
    aruco::MarkerDetector mDetector;
    std::vector<aruco::Marker> markers;
    if (id == CameraId::cam_left) {
        if (cam_params_left_.isValid()) {
            mDetector.detect(image, markers, cam_params_left_, marker_size_);
            ROS_DEBUG_STREAM("Found " << markers.size() << " marker(s) in left camera image");
        } else {
            ROS_ERROR_STREAM("Left camera parameters (aruco) are not valid");
        }
    } else {
        if (cam_params_right_.isValid()) {
            mDetector.detect(image, markers, cam_params_right_, marker_size_);
            ROS_DEBUG_STREAM("Found " << markers.size() << " marker(s) in right camera image");
        } else {
            ROS_ERROR_STREAM("Right camera parameters (aruco) are not valid");
        }
    }

    return markers;
}

void MarkerDetection::setCameraParameters(const sensor_msgs::CameraInfo &cam_params_left, const sensor_msgs::CameraInfo &cam_params_right) {
    cv::Mat cameraMatrix(3, 3, CV_32FC1);
    cv::Mat distorsionCoeff(4, 1, CV_32FC1);
    //transform camera paramters from ros message to aruco representation (left)
    cv::Size size(cam_params_left.height, cam_params_left.width);
    for (int i=0; i<9; ++i) {
        cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_params_left.K[i];
    }
    if (cam_params_left.D.size() >= 4) {
        for (int i=0; i<4; ++i) {
            distorsionCoeff.at<float>(i, 0) = cam_params_left.D[i];
        }
    }
    cam_params_left_ = aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);

    //transform camera paramters from ros message to aruco representation (right)
    size = cv::Size(cam_params_right.height, cam_params_right.width);
    for (int i=0; i<9; ++i) {
        cameraMatrix.at<float>(i%3, i-(i%3)*3) = cam_params_right.K[i];
    }
    if (cam_params_right.D.size() >= 4) {
        for (int i=0; i<4; ++i) {
            distorsionCoeff.at<float>(i, 0) = cam_params_right.D[i];
        }
    }
    cam_params_right_ = aruco::CameraParameters(cameraMatrix, distorsionCoeff, size);
}

}

