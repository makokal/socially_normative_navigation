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

#include <flow_behavior_layer/flow_behavior_layer.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(flow_behavior_layer::FlowBehaviorLayer,
    costmap_2d::Layer)

namespace flow_behavior_layer {

FlowBehaviorLayer::FlowBehaviorLayer() {}

FlowBehaviorLayer::~FlowBehaviorLayer() {
    sub_persons_.shutdown();
    sub_goal_.shutdown();

    delete dsrv_;
    cost_function_.reset();
}

/// ---------------------------------------------------------------------------
/// \function onInitialize
/// \brief onInitialize is called just after setting tf_
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;
    matchSize();

    std::string people_topic, goal_topic;
    nh.param("people_topic", people_topic, std::string(""));
    nh.param("goal_topic", goal_topic, std::string(""));

    costmap_frame_ = layered_costmap_->getGlobalFrameID();
    ros::param::param<std::string>("/move_base_node/global_costmap/robot_base_frame",
        robot_base_frame_, std::string(""));

    ROS_INFO("Flow behavior layer using the following data sources:");
    ROS_INFO("Persons: [%s], Goal: [%s]", people_topic.c_str(),
        goal_topic.c_str());

    sub_persons_ = nh.subscribe(people_topic, 10,
        &FlowBehaviorLayer::callbackTrackedPersons, this);
    sub_goal_ = nh.subscribe(goal_topic, 1, &FlowBehaviorLayer::callbackSetGoal, this);

    // setup dynamic reconfigure callback
    dsrv_ = new dynamic_reconfigure::Server<FlowConfig>(nh);
    dynamic_reconfigure::Server<FlowConfig>::CallbackType cb = boost::bind(&FlowBehaviorLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    // behavior parameters
    std::vector<double> merge_weights, nomerge_weights;
    nh.getParam("/flow_behavior_layer/behavior_params_merge", merge_weights);
    nh.getParam("/flow_behavior_layer/behavior_params_nomerge", nomerge_weights);
    if (behavior_name_ == "merge")
        weights_ = merge_weights;
    else
        weights_ = nomerge_weights;

    goal_ = Tpoint{ 5, 5 };
    robot_position_ = { 0., 0., 0., 0. };

    // initialize cost function
    cost_function_.reset(new FlowCostFunction(weights_, 0.99, radius_));
}

/// ---------------------------------------------------------------------------
/// \function computeLayerCostmap
/// \brief Compute the layer costmap, by computing the cost of every cell in the
/// global costmap. This maybe slow but we dont need it at fast rates anyway
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::computeLayerCostmap(const double& min_i,
    const double& min_j,
    const double& max_i,
    const double& max_j)
{
    flow_map_.clear();
    std::vector<double> all_costs; all_costs.clear();

    double dist_to_robot = 0.0;
    double cell_cost = 0.0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            std::pair<int, int> key = { i, j };

            double wx, wy;
            mapToWorld(i, j, wx, wy);
            dist_to_robot = std::hypot(robot_position_[0] - wx, robot_position_[1] - wy);

            cell_cost = 0.0;
            if ((min_range_ < dist_to_robot) && (dist_to_robot < update_range_)) {
                Tpoint p = { wx, wy, 0.0, 0.0 };
                cell_cost = cost_function_->cost(p, persons_, goal_, radius_);

                if (fabs(cell_cost) > 1e-05) {
                    flow_map_.insert(std::make_pair(key, cell_cost));
                    all_costs.push_back(cell_cost);
                }
            }
        }
    }

    if (all_costs.size() < 1) {
        min_flow_cost_ = 0.0;
        max_flow_cost_ = 0.0;
    }
    else {
        min_flow_cost_ = *std::min_element(all_costs.begin(), all_costs.end());
        max_flow_cost_ = *std::max_element(all_costs.begin(), all_costs.end());
    }

    // ROS_WARN(" MIN Flow Cost [%f], MAX Flow Cost [%f] ", min_flow_cost_, max_flow_cost_);

}

void FlowBehaviorLayer::updateRobotPose()
{
    tf::StampedTransform transform;
    try {
        tf_->waitForTransform(costmap_frame_,
            robot_base_frame_,
            ros::Time::now(),
            ros::Duration(0.05));
        tf_->lookupTransform(costmap_frame_,
            robot_base_frame_,
            ros::Time(0), transform);
    }
    catch (tf::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from robot_base_frame to global_frame failed. Reason: " << e.what());
        return;
    }

    tf::Quaternion orientation(transform.getRotation().getX(),
        transform.getRotation().getY(),
        transform.getRotation().getZ(),
        transform.getRotation().getW());

    double yaw = tf::getYaw(orientation);

    robot_position_[0] = transform.getOrigin().x();
    robot_position_[1] = transform.getOrigin().y();
    robot_position_[2] = cos(yaw);
    robot_position_[3] = sin(yaw);
}

void FlowBehaviorLayer::callbackSetGoal(
    const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_ = Tpoint{ msg->pose.position.x, msg->pose.position.y };
}

/// ---------------------------------------------------------------------------
/// \function updateCosts
/// \brief Update the cost by overwriting the previous costmap in the area
/// defined by the bounds
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i,
    int max_j)
{
    if (!enabled_)
        return;

    updateRobotPose();

    flow_map_.clear();
    computeLayerCostmap(min_i, min_j, max_i, max_j);
    if (flow_map_.size() < 1)
        return;

    unsigned int current_cell_cost = 0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            std::pair<int, int> cell = { i, j };
            // if (flow_map_.find(cell) == flow_map_.end())
            //     continue;

            auto search = flow_map_.find(cell);
            if (search != flow_map_.end()) {
                double real_cost = mapRange(search->second, min_flow_cost_, max_flow_cost_, 0, 254);
                current_cell_cost = master_grid.getCost(i, j);
                master_grid.setCost(i, j, std::max(current_cell_cost, static_cast<unsigned int>(real_cost)));
            }

        }
    }
}

/// ---------------------------------------------------------------------------
/// \function matchSize
/// \brief Match size of the costmap with the bounds
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(), master->getSizeInCellsY(),
        master->getResolution(), master->getOriginX(),
        master->getOriginY());
}

/// ---------------------------------------------------------------------------
/// \function reconfigureCB
/// \brief For automatic enable/disable of this layer
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::reconfigureCB(FlowConfig& config, uint32_t level)
{
    behavior_name_ = config.behavior_name;
    radius_ = config.radius;
    update_range_ = config.update_range;
    min_range_ = config.min_range;

    if (enabled_ != config.enabled) {
        enabled_ = config.enabled;
    }
}

/// ---------------------------------------------------------------------------
/// \function updateBounds
/// \brief Defines the part/area of the costmap that will be updated
/// ---------------------------------------------------------------------------
void FlowBehaviorLayer::updateBounds(double robot_x, double robot_y,
    double robot_yaw, double* min_x,
    double* min_y, double* max_x,
    double* max_y)
{
    if (!enabled_)
        return;

    if (layered_costmap_->isRolling())
        updateOrigin(robot_x - getSizeInMetersX() / 2.0,
            robot_y - getSizeInMetersY() / 2.0);

    *min_x = *min_x;
    *min_y = *min_y;
    *max_x = *max_x;
    *max_y = *max_y;
}

/// -----------------------------------------------------------
/// \function callbackTrackedPersons
/// \brief Receives tracked persons messages and saves them
/// -----------------------------------------------------------
void FlowBehaviorLayer::callbackTrackedPersons(const TPersons::ConstPtr& msg)
{
    tf::StampedTransform costmap_transform;
    try {
        tf_->lookupTransform(costmap_frame_,
            msg->header.frame_id,
            ros::Time(), costmap_transform);
    }
    catch (tf::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from tracking frame to costmap frame failed. Reason: " << e.what());
        return;
    }

    persons_.clear();
    for (const spencer_tracking_msgs::TrackedPerson& p : msg->tracks) {
        tf::Pose source;
        double curr_or = tf::getYaw(p.pose.pose.orientation);
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, curr_or);
        tf::Matrix3x3 base(q);
        source.setOrigin(tf::Vector3(p.pose.pose.position.x, p.pose.pose.position.y, 0));
        source.setBasis(base);

        /// Apply the proper transform
        tf::Pose result = costmap_transform * source;
        tf::Quaternion new_orientation = result.getRotation();
        double theta = tf::getYaw(new_orientation);
        double speed = std::hypot(p.twist.twist.linear.x, p.twist.twist.linear.y);
        Person pd = { result.getOrigin().x(), result.getOrigin().y(), speed*cos(theta), speed*sin(theta) };

        persons_.emplace_back(pd);
    }
}


} // end namespace
