/**
* Copyright 2015-2016 Social Robotics Lab, University of Freiburg
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*    # Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*    # Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*    # Neither the name of the University of Freiburg nor the names of its
*       contributors may be used to endorse or promote products derived from
*       this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* \author Billy Okal <okal@cs.uni-freiburg.de>
*/

#ifndef OVERTAKING_LAYER_H
#define OVERTAKING_LAYER_H

#include <tf/transform_listener.h> // must come first due to conflict with Boost signals
#include <memory>

/// ros
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

/// data
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <behavior_utils/entities.h>
#include <behavior_utils/geometry.h>
#include <behavior_utils/helpers.h>

// #include <behavior_functions/overtaking_cost.h>

#include <overtaking_layer/OvertakingLayerConfig.h>

namespace overtaking_layer {

using TPersons = spencer_tracking_msgs::TrackedPersons;
using LayerConfig = overtaking_layer::OvertakingLayerConfig;


/**
 * @class OvertakingLayer
 * @details A costmap2D layer for providing capability of overtaking agents or
 * people.
 */
class OvertakingLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
public:
    OvertakingLayer();
    ~OvertakingLayer();

    virtual void onInitialize();

    virtual void updateBounds(double robot_x,
        double robot_y,
        double robot_yaw,
        double* min_x,
        double* min_y,
        double* max_x,
        double* max_y);

    virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
        int min_i,
        int min_j,
        int max_i,
        int max_j);

    bool isDiscretized() { return true; }

    virtual void matchSize();

    void callbackTrackedPersons(const TPersons::ConstPtr& msg);

protected:

    void updateRobotPose();
    behavior_utils::Person getPersonInFront();

private:
    void reconfigureCB(LayerConfig& config, uint32_t level);
    dynamic_reconfigure::Server<LayerConfig>* dsrv_;

    ros::Subscriber sub_persons_;

    ros::Publisher pub_next_goal_;

    std::vector<behavior_utils::Person> persons_;
    behavior_utils::Person robot_position_;

    std::string costmap_frame_;
    std::string robot_base_frame_;

};



} // end namespace

#endif
