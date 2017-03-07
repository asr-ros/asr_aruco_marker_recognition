/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "aruco_marker_recognition.h"
#include <cv_bridge/cv_bridge.h>
#include <camera_calibration_parsers/parse.h>
#include <ros_uri/ros_uri.hpp>
#include <Tracking/ICP.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"


namespace aruco_marker_recognition {

typedef boost::array< ::geometry_msgs::Point_<std::allocator<void> > , 8> BoundingBox;

ArucoMarkerRecognition::ArucoMarkerRecognition() : nh_(NODE_NAME), marker_size_(DEFAULT_MARKER_SIZE), recognition_paused_(true)
{
    ROS_DEBUG("Initializing recognition system");
    nh_.getParam("use_stereo", use_stereo_);

    nh_.getParam("image_left_topic", image_left_topic_);
    nh_.getParam("image_right_topic", image_right_topic_);
    nh_.getParam("image_left_cam_info_topic", image_left_cam_info_topic_);
    nh_.getParam("image_right_cam_info_topic", image_right_cam_info_topic_);

    // Set up dynamic reconfigure
    reconfigure_server_.setCallback(boost::bind(&ArucoMarkerRecognition::configCallback, this, _1, _2));

    nh_.getParam("marker_size", marker_size_);
    detector_ = MarkerDetection(marker_size_);

    ROS_DEBUG("Subscribe to image topics");
    image_left_sub_.subscribe(nh_, image_left_topic_, 1);
    image_left_param_sub_.subscribe(nh_, image_left_cam_info_topic_, 1);

    if (use_stereo_) {
        image_right_sub_.subscribe(nh_, image_right_topic_, 1);
        image_right_param_sub_.subscribe(nh_, image_right_cam_info_topic_, 1);
    } else {
        image_right_sub_.subscribe(nh_, image_left_topic_, 1);
        image_right_param_sub_.subscribe(nh_, image_left_cam_info_topic_, 1);
    }

    sync_policy_.reset(new ApproximateSync(ApproximatePolicy(20), image_left_sub_, image_right_sub_, image_left_param_sub_, image_right_param_sub_) );
    sync_policy_->registerCallback(boost::bind(&ArucoMarkerRecognition::imageCallback, this, _1, _2, _3, _4));

    ROS_DEBUG("Advertise services");
    get_recognizer_service_ = nh_.advertiseService(GET_RECOGNIZER_SERVICE_NAME, &ArucoMarkerRecognition::processGetRecognizerRequest, this);
    release_recognizer_service_ = nh_.advertiseService(RELEASE_RECOGNIZER_SERVICE_NAME, &ArucoMarkerRecognition::processReleaseRecognizerRequest, this);

    object_mesh_service_client_ = nh_.serviceClient<asr_object_database::ObjectMetaData>(OBJECT_DB_SERVICE_META_DATA);

    std::string left_markers_img_topic;
    std::string right_markers_img_topic;
    std::string asr_objects_topic;
    nh_.getParam("left_markers_img_topic", left_markers_img_topic);
    nh_.getParam("right_markers_img_topic", right_markers_img_topic);
    nh_.getParam("asr_objects_topic", asr_objects_topic);
    left_markers_img_pub_ = nh_.advertise<sensor_msgs::Image> (left_markers_img_topic, 10);
    if (use_stereo_) {
        right_markers_img_pub_ = nh_.advertise<sensor_msgs::Image> (right_markers_img_topic, 10);
    }
    asr_objects_pub_ = nh_.advertise<asr_msgs::AsrObject> (asr_objects_topic, 10);

    std::string vis_markers_topic;
    nh_.getParam("vis_markers_topic", vis_markers_topic);
    vis_markers_pub_ = nh_.advertise<visualization_msgs::Marker> (vis_markers_topic, 10);

    nh_.getParam("recognizer_name", recognizer_name_);

    ROS_DEBUG("Initializing completed");

    ROS_DEBUG("Recognition is paused");

}

bool ArucoMarkerRecognition::processGetRecognizerRequest(GetRecognizer::Request &req, GetRecognizer::Response &res) {
    ROS_DEBUG("Process get recognizer request");
    recognition_paused_ = false;
    ROS_DEBUG("Recognition is running");
    return true;
}

bool ArucoMarkerRecognition::processReleaseRecognizerRequest(ReleaseRecognizer::Request &req, ReleaseRecognizer::Response &res) {
    ROS_DEBUG("Process release recognizer request");
    recognition_paused_ = true;
    ROS_DEBUG("Recognition is paused");
    return true;
}

void ArucoMarkerRecognition::imageCallback(const sensor_msgs::ImageConstPtr& input_img_left, const sensor_msgs::ImageConstPtr& input_img_right, const sensor_msgs::CameraInfoConstPtr& cam_info_left, const sensor_msgs::CameraInfoConstPtr& cam_info_right) {
    if (!recognition_paused_) {
        ROS_DEBUG("Images received");
        cv_bridge::CvImagePtr cv_left;
        cv_bridge::CvImagePtr cv_right;
        try {
            cv_left = cv_bridge::toCvCopy(input_img_left, sensor_msgs::image_encodings::BGR8);
            cv_right = cv_bridge::toCvCopy(input_img_right, sensor_msgs::image_encodings::BGR8);
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("Cv_bridge exception: %s", e.what());
            return;
        }
        cv::Mat img;
        cv::GaussianBlur(cv_left->image, img, cv::Size(0, 0), config_.gaussianBlurSigma);
        cv::addWeighted(cv_left->image, config_.sharpenWeightImage, img, config_.sharpenWeightBlur, 0, cv_left->image);

        cv::GaussianBlur(cv_right->image, img, cv::Size(0, 0), config_.gaussianBlurSigma);
        cv::addWeighted(cv_right->image, config_.sharpenWeightImage, img, config_.sharpenWeightBlur, 0, cv_right->image);

        detector_.setCameraParameters(*cam_info_left, *cam_info_right);

        std::vector<aruco::Marker> left_markers = detector_.detect(cv_left->image, MarkerDetection::CameraId::cam_left, config_);
        std::vector<aruco::Marker> right_markers;
        std::map<int, geometry_msgs::Pose> idPoseMap;
        if (use_stereo_) {
            right_markers = detector_.detect(cv_right->image, MarkerDetection::CameraId::cam_right, config_);

            ivt_bridge::IvtStereoCalibration ivtCalib;
            ivtCalib.fromCameraInfo(cam_info_left, cam_info_right);

            idPoseMap = getMarkerPoses(left_markers, right_markers, ivtCalib);
        } else {
            idPoseMap = getMarkerPoses(left_markers);
        }
        std::string cam_left_frame_id = "/" + cam_info_left->header.frame_id;

        //publish marker visualizations
        int marker_id = 1;
        for (auto it = idPoseMap.begin(); it != idPoseMap.end(); ++it) {
            vis_markers_pub_.publish(createSquareMarker(marker_id, it->second, cam_left_frame_id));
            vis_markers_pub_.publish(createArrowMarker(marker_id, it->second, cam_left_frame_id));
            vis_markers_pub_.publish(createArrowMarker(marker_id, it->second, cam_left_frame_id, true));
            ++marker_id;
        }

        //publish marker mesh visualizations & asr-objects
        int id = 0;
        for (auto idPosePair : idPoseMap) {
            std::string object_type = "marker_" + std::to_string(idPosePair.first);
            asr_object_database::ObjectMetaData objectMetaData;
            objectMetaData.request.object_type = object_type;
            objectMetaData.request.recognizer = recognizer_name_;
            object_mesh_service_client_.call(objectMetaData);

            if (objectMetaData.response.is_valid) {
                vis_markers_pub_.publish(createMeshMarker(id, idPosePair.second, objectMetaData.response.object_mesh_resource, cam_left_frame_id));
                asr_objects_pub_.publish(createAsrObject(idPosePair.second, object_type, cam_left_frame_id, objectMetaData.response.object_mesh_resource));
                id++;
            } else {
                asr_objects_pub_.publish(createAsrObject(idPosePair.second, object_type, cam_left_frame_id));
            }
        }


        //publish images with marker overlay
        cv_left->image = drawMarkers(cv_left->image, left_markers);
        left_markers_img_pub_.publish(cv_left->toImageMsg());

        if (use_stereo_) {
            cv_right->image = drawMarkers(cv_right->image, right_markers);
            right_markers_img_pub_.publish(cv_right->toImageMsg());
        }
        ROS_DEBUG("--------------\n");
    }
}

void ArucoMarkerRecognition::configCallback(ArucoMarkerRecognitionConfig &config, uint32_t level) {
    this->config_ = config;
}

cv::Mat ArucoMarkerRecognition::drawMarkers(const cv::Mat &image, const std::vector<aruco::Marker> &markers) {
    cv::Mat output = image;
    for (unsigned int i = 0; i < markers.size(); i++) {
        markers[i].draw(output, cv::Scalar(0, 0, 255), 2);
    }
    return output;
}

std::map<int, geometry_msgs::Pose> ArucoMarkerRecognition::getMarkerPoses(const std::vector<aruco::Marker> &left_markers) {
    std::map<int, geometry_msgs::Pose> idPoseMap;
    for (aruco::Marker marker : left_markers) {
        geometry_msgs::Pose pose;
        pose.position.x = marker.Tvec.at<float>(0,0);
        pose.position.y = marker.Tvec.at<float>(1,0);
        pose.position.z = marker.Tvec.at<float>(2,0);

        cv::Mat rot_cv(3, 3, CV_32FC1);
        cv::Rodrigues(marker.Rvec, rot_cv);

        cv::Mat rotate_to_ros(3, 3, CV_32FC1);
        //  0 1  0
        //  0 0 -1
        // -1 0  0
        rotate_to_ros.at<float>(0,0) = 0.0;
        rotate_to_ros.at<float>(0,1) = 1.0;
        rotate_to_ros.at<float>(0,2) = 0.0;
        rotate_to_ros.at<float>(1,0) = 0.0;
        rotate_to_ros.at<float>(1,1) = 0.0;
        rotate_to_ros.at<float>(1,2) = -1.0;
        rotate_to_ros.at<float>(2,0) = -1.0;
        rotate_to_ros.at<float>(2,1) = 0.0;
        rotate_to_ros.at<float>(2,2) = 0.0;
        rot_cv = rot_cv*rotate_to_ros.t();

        float q4 = 0.5f * sqrt(1.0f + rot_cv.at<float>(0,0) + rot_cv.at<float>(1,1) + rot_cv.at<float>(2,2));
        float t = (1.0f / (4.0f * q4));
        float q1 = t * (rot_cv.at<float>(2,1) - rot_cv.at<float>(1,2));
        float q2 = t * (rot_cv.at<float>(0,2) - rot_cv.at<float>(2,0));
        float q3 = t * (rot_cv.at<float>(1,0) - rot_cv.at<float>(0,1));

        pose.orientation.x = q1;
        pose.orientation.y = q2;
        pose.orientation.z = q3;
        pose.orientation.w = q4;

        idPoseMap.insert(std::make_pair(marker.id, pose));
    }
    return idPoseMap;
}

std::map<int, geometry_msgs::Pose> ArucoMarkerRecognition::getMarkerPoses(const std::vector<aruco::Marker> &left_markers, const std::vector<aruco::Marker> &right_markers, const ivt_bridge::IvtStereoCalibration &ivtCalib) {
    std::map<int, geometry_msgs::Pose> idPoseMap;
    boost::shared_ptr<CStereoCalibration> stereoCalibPtr = ivtCalib.getStereoCalibration();

    for (int i = 0; i < left_markers.size(); i++) {
        int left_id = left_markers.at(i).id;
        for (int j = 0; j < right_markers.size(); j++) {
            if (right_markers.at(j).id == left_id) {
                CVec3dArray world_marker_corners(4);
                CVec2dArray image_marker_corners_left(4);
                CVec3dArray camera_marker_corners_left(4);

                for (int k = 0; k < 4; ++k) {
                    Vec2d img_point_left;
                    Vec3d world_point = {0.0f};
                    Vec2d left_point = { left_markers.at(i)[k].x, left_markers.at(i)[k].y };
                    Vec2d right_point = { right_markers.at(j)[k].x, right_markers.at(j)[k].y };
                    stereoCalibPtr->Calculate3DPoint(left_point, right_point, world_point, false, false);
                    world_marker_corners.AddElement(world_point);

                    stereoCalibPtr->GetLeftCalibration()->WorldToImageCoordinates(world_point, img_point_left);
                    image_marker_corners_left.AddElement(img_point_left);
                }

                Vec3d m0 = {0.0, 0.0, 0.0};
                Vec3d m1 = {static_cast<float>(marker_size_ * 1000), 0.0, 0.0};
                Vec3d m2 = {static_cast<float>(marker_size_ * 1000),static_cast<float>(marker_size_ * 1000), 0.0};
                Vec3d m3 = {0.0, static_cast<float>(marker_size_ * 1000), 0.0};
                for (int k = 0; k < 4; ++k) {
                    stereoCalibPtr->GetLeftCalibration()->WorldToCameraCoordinates(world_marker_corners[k], camera_marker_corners_left[k]);

                }
                Vec3d sourcePoints[4] = {m0, m1, m2, m3};
                Vec3d targetPointsLeft[4] = {camera_marker_corners_left[0],camera_marker_corners_left[1],camera_marker_corners_left[2],camera_marker_corners_left[3]};

                Mat3d rotation_left;
                Vec3d translation_left;
                CICP::CalculateOptimalTransformation(sourcePoints, targetPointsLeft, 4, rotation_left, translation_left);

                //translate marker position from corner to center
                const Vec3d x = { static_cast<float>(marker_size_ * 1000) / 2.0f, 0.0f, 0.0f };
                Vec3d x2;
                const Vec3d y = { 0.0f, static_cast<float>(marker_size_ * 1000) / 2.0f, 0.0f };
                Vec3d y2;

                Math3d::MulMatVec(rotation_left, x, x2);
                Math3d::MulMatVec(rotation_left, y, y2);

                Math3d::AddToVec(translation_left, x2);
                Math3d::AddToVec(translation_left, y2);

                //rotate marker in xz-plane, normal direction == -y
                Mat3d B;
                Vec3d axis;
                axis.x = 1.0;
                axis.y = 0.0;
                axis.z = 0.0;
                float theta = 0.5 * M_PI;
                Math3d::SetRotationMat(B, axis, theta);

                Mat3d C;
                Math3d::MulMatMat(rotation_left, B, C);

                geometry_msgs::Pose pose;
                pose.position.x = translation_left.x / 1000;
                pose.position.y = translation_left.y / 1000;
                pose.position.z = translation_left.z / 1000;

                float q4 = 0.5f * sqrt(1.0f + C.r1 + C.r5 + C.r9);
                float t = (1.0f / (4.0f * q4));
                float q1 = t * (C.r8 - C.r6);
                float q2 = t * (C.r3 - C.r7);
                float q3 = t * (C.r4 - C.r2);

                pose.orientation.x = q1;
                pose.orientation.y = q2;
                pose.orientation.z = q3;
                pose.orientation.w = q4;

                idPoseMap.insert(std::make_pair(left_id, pose));
                break;
            }
        }
    }

    return idPoseMap;
}

visualization_msgs::Marker ArucoMarkerRecognition::createSquareMarker(int id, const geometry_msgs::Pose &pose, const string &frame_id) {
    visualization_msgs::Marker square_marker;
    square_marker.header.frame_id = frame_id;
    square_marker.ns = "Found markers";
    square_marker.id = id;
    square_marker.action = visualization_msgs::Marker::ADD;
    square_marker.pose.position = pose.position;
    square_marker.pose.orientation = pose.orientation;
    square_marker.color.a = 1;
    square_marker.color.r = 1;
    square_marker.color.g = 0;
    square_marker.color.b = 0;
    square_marker.scale.x = marker_size_;
    square_marker.scale.y = 0.001;
    square_marker.scale.z = marker_size_;
    square_marker.type = visualization_msgs::Marker::CUBE;
    square_marker.lifetime = ros::Duration(2);

    return square_marker;
}

visualization_msgs::Marker ArucoMarkerRecognition::createArrowMarker(int id, const geometry_msgs::Pose &pose, const string &frame_id, bool isNsX) {
    visualization_msgs::Marker arrow_marker;
    arrow_marker.header.frame_id = frame_id;
    arrow_marker.ns = isNsX ? "Found markers (orientation x)" : "Found markers (orientation)";
    arrow_marker.id = id;
    arrow_marker.action = visualization_msgs::Marker::ADD;
    arrow_marker.pose.position = pose.position;
    arrow_marker.pose.orientation = pose.orientation;
    geometry_msgs::Point p0;
    p0.x = 0;
    p0.y = 0;
    p0.z = 0;
    geometry_msgs::Point p1;
    p1.x = isNsX ? 0.05 : 0;
    p1.y = isNsX ? 0 : -0.05;
    p1.z = 0;
    arrow_marker.points.push_back(p0);
    arrow_marker.points.push_back(p1);
    arrow_marker.color.a = 1;
    arrow_marker.color.r = isNsX ? 1 : 0; // marker color is red if ns = x
    arrow_marker.color.g = 0;
    arrow_marker.color.b = isNsX ? 0 : 1; // marker color is blue if ns != x
    arrow_marker.scale.x = 0.01;
    arrow_marker.scale.y = 0.02;
    arrow_marker.scale.z = 0.02;
    arrow_marker.type = visualization_msgs::Marker::ARROW;
    arrow_marker.lifetime = ros::Duration(2);

    return arrow_marker;
}

visualization_msgs::Marker ArucoMarkerRecognition::createMeshMarker(int id, const geometry_msgs::Pose &pose, const std::string &mesh_res, const string &frame_id) {

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.ns = "Found markers (meshes)";
    marker.id = id;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position = pose.position;
    marker.pose.orientation = pose.orientation;

    marker.scale.x = marker.scale.y = marker.scale.z = 0.001;
    marker.color.r = marker.color.g = marker.color.b = 130.0f / 255.0f;
    marker.color.a = 1.0;

    marker.mesh_resource = mesh_res;
    marker.mesh_use_embedded_materials = false;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;
    marker.lifetime = ros::Duration(2);

    return marker;
}

asr_msgs::AsrObject ArucoMarkerRecognition::createAsrObject(const geometry_msgs::Pose &pose, const std::string &object_type, const string &frame_id, const std::string &mesh_res) {
    asr_msgs::AsrObject o;

    o.header.frame_id = frame_id;
    o.header.stamp = ros::Time::now();
    o.providedBy = recognizer_name_;
    geometry_msgs::PoseWithCovariance current_pose_with_c;
    current_pose_with_c.pose = pose;
    o.sampledPoses.push_back(current_pose_with_c);

    //Set bounding box as planar rectangle
    BoundingBox bounding_box;

    Vec3d minPoint;
    minPoint.x = minPoint.y = minPoint.z = 0.0;
    Vec3d maxPoint;
    maxPoint.x = maxPoint.z = marker_size_;
    maxPoint.y = 0.0;

    for (unsigned int z = 0; z < 2; z++) {
        for (unsigned int y = 0; y < 2; y++) {
            for (unsigned int x = 0; x < 2; x++) {
                bounding_box[4*z+2*y+x].x = (1 - x) * minPoint.x + x * maxPoint.x;
                bounding_box[4*z+2*y+x].y = (1 - y) * minPoint.y + y * maxPoint.y;
                bounding_box[4*z+2*y+x].z = (1 - z) * minPoint.z + z * maxPoint.z;
            }
        }
    }

    o.boundingBox = bounding_box;

    o.color.r = o.color.g = o.color.b = 130.0f / 255.0f;
    o.color.a = 1.0;
    o.type = object_type;
    o.typeConfidence = 1.0f;
    o.identifier = "052052051100";
    o.meshResourcePath = mesh_res;

    return o;
}


}

int main(int argc, char** argv) {
    ros::init(argc, argv, "asr_aruco_marker_recognition");
    ROS_INFO("Starting up ArucoMarkerRecognition.\n");
    aruco_marker_recognition::ArucoMarkerRecognition rec;
    ROS_DEBUG("ArucoMarkerRecognition is started up.\n");
    //Here we set the frame rate with which marker recognition shall work.
    unsigned int hertz = 5;
    ROS_DEBUG_STREAM("ArucoMarkerRecognition runs at " << hertz << " hertz.\n");
    ros::Rate r(hertz);

    while (ros::ok()) {
      ros::spinOnce();
      r.sleep();
    }
    return 0;
}
