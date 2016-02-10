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

#ifndef SOCIAL_COMPLIANCE_LAYER_H
#define SOCIAL_COMPLIANCE_LAYER_H

#include <tf/transform_listener.h> // must come first due to conflict with Boost signals
#include <boost/shared_ptr.hpp>

/// ros
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <dynamic_reconfigure/server.h>

/// data
#include <spencer_tracking_msgs/TrackedPerson.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <spencer_tracking_msgs/TrackedGroup.h>
#include <spencer_tracking_msgs/TrackedGroups.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <social_compliance_layer/cost_function.h>

#include <social_compliance_layer/SocialComplianceLayerConfig.h>

namespace social_compliance_layer {

using TPersons = spencer_tracking_msgs::TrackedPersons;
using TGroups = spencer_tracking_msgs::TrackedGroups;
using LayerConfig = social_compliance_layer::SocialComplianceLayerConfig;

/// ---------------------------------------------------------------------------
/// \class SocialComplianceLayer
/// \brief Add a layer for simple behavior based on inverse reinforement
/// learning reward functions (cost objective functions)
/// ---------------------------------------------------------------------------
class SocialComplianceLayer : public costmap_2d::Layer, public costmap_2d::Costmap2D {
public:
    SocialComplianceLayer();
    ~SocialComplianceLayer();

    /// overriding costmap layer methods
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

    void callbackTrackedGroups(const TGroups::ConstPtr& msg);

    void callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg);

    void callbackGlobalPath(const nav_msgs::Path::ConstPtr& path);

protected:
    /// Internal methods

    void computeLayerCostmap(const double& min_i,
        const double& min_j,
        const double& max_i,
        const double& max_j);

    void updateRobotPose();

    point_t nextMilestone();

    std::vector<double> selectWeights(std::string behavior_name);

private:
    void reconfigureCB(LayerConfig& config, uint32_t level);
    dynamic_reconfigure::Server<LayerConfig>* dsrv_;

    ros::Subscriber sub_persons_;
    ros::Subscriber sub_groups_;
    ros::Subscriber sub_goal_;
    ros::Subscriber sub_global_path_;

    // local data stores
    std::map<int, Person> persons_map_;
    std::vector<Person> persons_;
    std::vector<PairWiseRelation> relations_;
    boost::shared_ptr<nav_msgs::Path> global_path_;

    // params
    std::string behavior_name_;
    std::vector<double> polite_;
    std::vector<double> rude_;
    std::vector<double> sociable_;
    Tpoint goal_;

    // local cached costmap
    std::map<std::pair<int, int>, double> social_cost_map_;
    double min_social_cost_, max_social_cost_;

    // module for social navigation cost
    std::unique_ptr<SocialNavigationCost> cost_function_;

    std::string costmap_frame_;
    std::string robot_base_frame_;

    // current robot pose
    std::vector<double> robot_position_;

    // dynamic reconfigure params
    std::shared_ptr<CFParams> cfparams_;
    double update_range_, min_range_;
};

} // end namespace

#endif
