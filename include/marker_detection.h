/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MARKER_DETECTION_H
#define MARKER_DETECTION_H

#include "aruco/aruco.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/CameraInfo.h>
#include <asr_aruco_marker_recognition/ArucoMarkerRecognitionConfig.h>

namespace aruco_marker_recognition {

using namespace asr_aruco_marker_recognition;

/**
 * @brief This class is used to detect markers in an image using the aruco-library
 */
class MarkerDetection {


private:

    /** The default marker size in meters **/
    double DEFAULT_MARKER_SIZE = 0.1;

    /** The custom marker size in meters **/
    double marker_size_;

    /** The camera parameters of the left camera **/
    aruco::CameraParameters cam_params_left_;

    /** The camera parameters of the right camera **/
    aruco::CameraParameters cam_params_right_;


public:
    /** An enum describing the ids of the two cameras **/
    enum CameraId { cam_left, cam_right };

    /**
     * @brief The default constructor of this class
     */
    MarkerDetection() : MarkerDetection(DEFAULT_MARKER_SIZE) {}

    /**
     * @brief The constructor of this class
     * @param marker_size   The size of the used markers in meters
     */
    MarkerDetection(double marker_size);

    /**
     * @brief Detects markers in the given image with the camera parameters of the camera with the given id
     * @param image     The image the markers are detected in
     * @param id        The id of the camera the given image was fetched from (specified in the enum CameraId)
     * @return          A list containing the detected markers
     */
    std::vector<aruco::Marker> detect(const cv::Mat &image, CameraId id, const ArucoMarkerRecognitionConfig &config);

    /**
     * @brief Set the camera parameters of the two cameras
     * @param cam_params_left       The parameters of the left camera
     * @param cam_params_right      The parameters of the right camera
     */
    void setCameraParameters(const sensor_msgs::CameraInfo &cam_params_left, const sensor_msgs::CameraInfo &cam_params_right);
};

}


#endif


