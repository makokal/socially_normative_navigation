/**
* Copyright 2015 Social Robotics Lab, University of Freiburg
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

#ifndef FLOW_BEHAVIOR_LAYER_H
#define FLOW_BEHAVIOR_LAYER_H

/// ros
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

/// data
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <geometry_msgs/PoseStamped.h>

#include <flow_behavior_layer/commons.h>
#include <flow_behavior_layer/cost_function.h>
#include <flow_behavior_layer/FlowBehaviorLayerConfig.h>

using TPersons = spencer_tracking_msgs::TrackedPersons;
using FlowConfig = flow_behavior_layer::FlowBehaviorLayerConfig;

namespace flow_behavior_layer {

class FlowBehaviorLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
public:
    FlowBehaviorLayer();
    ~FlowBehaviorLayer();

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

    /// subcriber callbacks
    void callbackTrackedPersons(const TPersons::ConstPtr& msg);

    void callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void computeLayerCostmap(const double& min_i,
        const double& min_j,
        const double& max_i,
        const double& max_j);

protected:
    void updateRobotPose();

private:
    void reconfigureCB(FlowConfig& config, uint32_t level);
    dynamic_reconfigure::Server<FlowConfig>* dsrv_;

    ros::Subscriber sub_persons_;
    ros::Subscriber sub_goal_;

    // local data containers
    std::vector<Person> persons_;

    std::vector<double> weights_;
    Tpoint goal_;

    // local cached costmap
    std::map<std::pair<int, int>, double> flow_map_;
    // std::vector<std::pair<int, int> > zero_cells_;
    double min_flow_cost_, max_flow_cost_;

    // navigation cost function
    std::unique_ptr<FlowCostFunction> cost_function_;

    std::string costmap_frame_;
    std::string robot_base_frame_;

    // current robot pose
    std::vector<double> robot_position_;

    // dynamic params from dynamic reconfigure
    std::string behavior_name_;
    double radius_, update_range_, min_range_;
};

} // end namespace

#endif
