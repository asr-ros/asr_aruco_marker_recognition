/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#include "marker_creator.h"
#include <ros_uri/ros_uri.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/highgui/highgui.hpp>
#include "aruco/arucofidmarkers.h"

namespace aruco_marker_recognition {

MarkerCreator::MarkerCreator() : nh_(NODE_NAME) {
    ROS_DEBUG_STREAM("Initialize parameters");

    nh_.getParam("marker_ids", marker_ids_);
    nh_.getParam("marker_pixel_size", marker_pixel_size_);
    nh_.getParam("output_rel_path", output_rel_path_);

    ROS_DEBUG_STREAM("Initializing done");

    createMarkers();
}

void MarkerCreator::createMarkers() {
    ROS_DEBUG_STREAM("Creating markers");
    std::string output_path = ros::package::getPath(PACKAGE_NAME) + output_rel_path_;

    for (int id : marker_ids_) {
        if (id >= 0 && id <= 1023) {
            std::ofstream marker_file;
            std::string marker_path = output_path + "marker_" + std::to_string(id) + ".png";
            marker_file.open (marker_path);
            if (marker_file.is_open()) {
                marker_file.close();
                cv::Mat marker = aruco::FiducidalMarkers::createMarkerImage(id, marker_pixel_size_, true, false);
                cv::imwrite(marker_path, marker);
            } else {
                ROS_ERROR_STREAM("Could not find the given output directory");
                break;
            }
        } else {
            ROS_ERROR_STREAM("Can't create a marker with id " << id << " (only ids from 0-1023 are allowed");
        }
    }

    ROS_DEBUG_STREAM("Marker creation completed");

}

}

int main(int argc, char** argv) {

    ros::init(argc, argv, "marker_creator");

    aruco_marker_recognition::MarkerCreator marker_creator;

    return 0;
}
