/**

Copyright (c) 2016, Allgeyer Tobias, Meißner Pascal, Qattan Mohamad
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef MARKER_CREATOR_H
#define MARKER_CREATOR_H

#include "aruco/aruco.h"
#include <ros/ros.h>


namespace aruco_marker_recognition {

/** The name of this ros node **/
const static std::string NODE_NAME("marker_creator");

/** The name of this package **/
const static std::string PACKAGE_NAME("asr_aruco_marker_recognition");

/**
 * @brief This class is used to create marker images which can be recognized with this package
 */
class MarkerCreator {

private:

    /** Ros' interface for creating subscribers, publishers, etc. */
    ros::NodeHandle nh_;

    /** A list containing the ids of the markers which shall be created */
    std::vector<int> marker_ids_;

    /** The size of the markers in meters */
    int marker_pixel_size_;

    /** The relative path to the output directory from the root directory of this package **/
    std::string output_rel_path_;

public:

    /**
     * @brief The constructor of this class
     */
    MarkerCreator();

    /**
     * @brief Creates the markers with the parameters set in this class' members
     */
    void createMarkers();

};

}

#endif



