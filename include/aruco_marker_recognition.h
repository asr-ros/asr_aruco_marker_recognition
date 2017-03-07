/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/


#ifndef ARUCO_MARKER_RECOGNITION_H
#define ARUCO_MARKER_RECOGNITION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/highgui/highgui.hpp>
#include <ivt_bridge/ivt_calibration.h>
#include <visualization_msgs/MarkerArray.h>
#include <asr_aruco_marker_recognition/GetRecognizer.h>
#include <asr_aruco_marker_recognition/ReleaseRecognizer.h>
#include <asr_object_database/ObjectMetaData.h>
#include <asr_msgs/AsrObject.h>
#include <dynamic_reconfigure/server.h>
#include <asr_aruco_marker_recognition/ArucoMarkerRecognitionConfig.h>

#include "marker_detection.h"

namespace aruco_marker_recognition {

using namespace asr_aruco_marker_recognition;

/** The name of this ros node **/
const static std::string NODE_NAME("asr_aruco_marker_recognition");

/** The default marker size in meters **/
const static double DEFAULT_MARKER_SIZE(0.05);

/** The names of the provided services **/
const static std::string GET_RECOGNIZER_SERVICE_NAME("get_recognizer");
const static std::string RELEASE_RECOGNIZER_SERVICE_NAME("release_recognizer");

/** The name of the service offered by the object_database to fetch information about stored objects **/
const static std::string OBJECT_DB_SERVICE_META_DATA("/asr_object_database/object_meta_data");

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> ApproximatePolicy;
typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

/**
 * @brief This is the base class of the marker recognition system used for creating the ros environment and controlling the recognition processes
 */
class ArucoMarkerRecognition {


private:

    bool use_stereo_;

    /** The topic of the left camera image **/
    std::string image_left_topic_;

    /** The topic of the right camera image **/
    std::string image_right_topic_;

    /** The topic of the left camera parameters **/
    std::string image_left_cam_info_topic_;

    /** The topic of the right camera parameters **/
    std::string image_right_cam_info_topic_;

    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh_;

    /** The size of the used markers in meters **/
    double marker_size_;

    /** The ros subscribers **/
    message_filters::Subscriber<sensor_msgs::Image> image_left_sub_;
    message_filters::Subscriber<sensor_msgs::Image> image_right_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> image_left_param_sub_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> image_right_param_sub_;

    /** The servers of the provided services **/
    ros::ServiceServer get_recognizer_service_;
    ros::ServiceServer release_recognizer_service_;

    /** Dynamic reconfigure server which keeps track of the callback function */
    dynamic_reconfigure::Server<ArucoMarkerRecognitionConfig> reconfigure_server_;

    /** The configuration file containing the dynamic parameters of this system **/
    ArucoMarkerRecognitionConfig config_;

    /** The client used to contact the object database service **/
    ros::ServiceClient object_mesh_service_client_;

    /** The policy the subscribers are synced with */
    boost::shared_ptr<ApproximateSync> sync_policy_;

    /** The detector used to recognize the markers in an image **/
    MarkerDetection detector_;

    /** The ros publishers **/
    ros::Publisher left_markers_img_pub_;
    ros::Publisher right_markers_img_pub_;
    ros::Publisher vis_markers_pub_;
    ros::Publisher asr_objects_pub_;

    /** The name of this recognizer **/
    std::string recognizer_name_;

    /** A bool value indicating whether the recognition is paused or running **/
    bool recognition_paused_;


    /**
     * Processes the request to recognize markers
     * \param req   The request message
     * \param res   The correlated response message
     * \return      True, to indicate that the request was successful
     */
    bool processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res);

    /**
     * Processes the request to stop the marker recognition
     * \param req   The request message
     * \param res   The correlated response message
     * \return      True, to indicate that the request was successful
     */
    bool processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res);

    /**
     * @brief The ros callback which is called when new images are received
     * @param input_img_left        The image of the left camera
     * @param input_img_right       The image of the right camera
     * @param cam_info_left         The camera parameters of the left camera
     * @param cam_info_right        The camera parameters of the right camera
     */
    void imageCallback(const sensor_msgs::ImageConstPtr& input_img_left, const sensor_msgs::ImageConstPtr& input_img_right, const sensor_msgs::CameraInfoConstPtr& cam_info_left, const sensor_msgs::CameraInfoConstPtr& cam_info_right);

    /**
    * @brief  The callback function which is called when the configuration file has changed
    *
    * @param config         The updated configuration
    * @param level          The level which is the result of ORing together all of level values of the parameters that have changed
    */
    void configCallback(ArucoMarkerRecognitionConfig &config_, uint32_t level);

    /**
     * @brief Draws recognized markers and their id into an image
     * @param image     The image to draw the markers into
     * @param markers   The markers to draw
     * @return          The input image with the drawn markers
     */
    cv::Mat drawMarkers(const cv::Mat &image, const std::vector<aruco::Marker> &markers);

    std::map<int, geometry_msgs::Pose> getMarkerPoses(const std::vector<aruco::Marker> &left_markers);

    /**
     * @brief Calculates poses from corresponding found markers of the right and left image
     * @param left_markers      The markers found in the left image
     * @param right_markers     The markers found in the right image
     * @param ivtCalib          The calibration information of the used stereo camera system
     * @return                  A map containing pairs of marker ids and their found poses
     */
    std::map<int, geometry_msgs::Pose> getMarkerPoses(const std::vector<aruco::Marker> &left_markers, const std::vector<aruco::Marker> &right_markers, const ivt_bridge::IvtStereoCalibration &ivtCalib);


    /**
     * @brief Create a ros marker from a given pose of a found marker
     * @param id            The id of the found marker
     * @param pose          The pose of the found marker
     * @param frame_id      The id of the frame the marker pose is relative to
     * @return              The created ros marker
     */
    visualization_msgs::Marker createSquareMarker(int id, const geometry_msgs::Pose &pose, const string &frame_id);

    /**
     * @brief Create a ros marker from a given pose of a found marker
     * @param id            The id of the found marker
     * @param pose          The pose of the found marker
     * @param frame_id      The id of the frame the marker pose is relative to
     * @param isNsX         Determine if the ns is a "X" ns or not
     * @return              The created ros marker
     */
    visualization_msgs::Marker createArrowMarker(int id, const geometry_msgs::Pose &pose, const string &frame_id, bool isNsX = false);

    /**
     * @brief Create a ros marker from a given pose of a found marker
     * @param id            The id of the found marker
     * @param pose          The pose of the found marker
     * @param mesh_res      The markers' corresponding mesh resource provided by the object_database
     * @param frame_id      The id of the frame the marker pose is relative to
     * @return              The created ros marker
     */
    visualization_msgs::Marker createMeshMarker(int id, const geometry_msgs::Pose &pose, const std::string &mesh_res, const string &frame_id);

    /**
     * @brief Create a asr_object from a given marker pose
     * @param pose              The pose of the marker
     * @param object_type       The type of the object corresponding to the found marker provided by the object_database
     * @param frame_id          The id of the frame the marker pose is relative to
     * @param mesh_res          The markers' corresponding mesh resource provided by the object_database
     * @return                  The created asr_object
     */
    asr_msgs::AsrObject createAsrObject(const geometry_msgs::Pose &pose, const std::string &object_type, const string &frame_id, const std::string &mesh_res = "");



public:


    /**
    * \brief  The constructor of this class
    */
    ArucoMarkerRecognition();
};

}


#endif

