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

#include <social_compliance_layer/social_compliance_layer.h>
#include <pluginlib/class_list_macros.h>

#include <memory>

PLUGINLIB_EXPORT_CLASS(social_compliance_layer::SocialComplianceLayer,
    costmap_2d::Layer)

namespace social_compliance_layer {

SocialComplianceLayer::SocialComplianceLayer() {}

SocialComplianceLayer::~SocialComplianceLayer()
{
    sub_persons_.shutdown();
    sub_groups_.shutdown();
    sub_goal_.shutdown();
    sub_global_path_.shutdown();

    delete dsrv_;
    cost_function_.reset();
    cfparams_.reset();

}

/// ---------------------------------------------------------------------------
/// \function onInitialize
/// \brief onInitialize is called just after setting tf_
/// ---------------------------------------------------------------------------
void SocialComplianceLayer::onInitialize()
{
    // clear as the first thing
    social_cost_map_.clear();

    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;
    matchSize();

    // read topics for data input
    std::string people_topic, groups_topic, goal_topic, path_topic;
    nh.param("people_topic", people_topic, std::string(""));
    nh.param("groups_topic", groups_topic, std::string(""));
    nh.param("goal_topic", goal_topic, std::string(""));
    nh.param("path_topic", path_topic, std::string(""));

    costmap_frame_ = layered_costmap_->getGlobalFrameID();
    // robot_base_frame_ = layered_costmap_->getBaseFrameID();
    ros::param::param<std::string>("/move_base_node/global_costmap/robot_base_frame",
        robot_base_frame_, std::string(""));

    ROS_INFO("Social compliance layer using the following data sources:");
    ROS_INFO("Persons: [%s], Groups: [%s], Goal: [%s], Costmap-Frame: [%s], Robot-Frame: [%s]",
        people_topic.c_str(), groups_topic.c_str(), goal_topic.c_str(), costmap_frame_.c_str(), robot_base_frame_.c_str());

    // setup subscribers for data input
    sub_persons_ = nh.subscribe(people_topic, 10, &SocialComplianceLayer::callbackTrackedPersons, this);
    sub_groups_ = nh.subscribe(groups_topic, 10, &SocialComplianceLayer::callbackTrackedGroups, this);
    sub_goal_ = nh.subscribe(goal_topic, 1, &SocialComplianceLayer::callbackSetGoal, this);
    sub_global_path_ = nh.subscribe(path_topic, 1, &SocialComplianceLayer::callbackGlobalPath, this);

    robot_position_ = { 0., 0., 0., 0. };
    global_path_ = boost::make_shared<nav_msgs::Path>();

    // read cost function parameters
    ros::param::param<std::string>("/social_compliance_layer/behavior_name", behavior_name_, "polite");
    nh.getParam("/social_compliance_layer/behavior_params_polite", polite_);
    nh.getParam("/social_compliance_layer/behavior_params_rude", rude_);
    nh.getParam("/social_compliance_layer/behavior_params_sociable", sociable_);
    goal_ = Tpoint{ 0., 0. };

    min_social_cost_ = 0.0;
    max_social_cost_ = 0.0;
    prediction_horizon_ = 2;

    cfparams_ = std::make_shared<CFParams>();
    cost_function_.reset(new SocialComplianceCost(0.99, behavior_name_, cfparams_));

    dsrv_ = new dynamic_reconfigure::Server<social_compliance_layer::SocialComplianceLayerConfig>(nh);
    dynamic_reconfigure::Server<social_compliance_layer::SocialComplianceLayerConfig>::CallbackType cb = boost::bind(
        &SocialComplianceLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

/// ---------------------------------------------------------------------------
/// \function computeIRLCostmap
/// \brief Compute the irl costmap, by computing the cost of every cell in the
/// global costmap. This maybe slow but we dont need it at fast rates anyway
/// ---------------------------------------------------------------------------
void SocialComplianceLayer::computeLayerCostmap(const double& min_i,
    const double& min_j,
    const double& max_i,
    const double& max_j)
{
    social_cost_map_.clear();
    std::vector<double> all_costs;
    all_costs.clear();

    double dist_to_robot = 0.0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            double wx, wy;
            mapToWorld(i, j, wx, wy); // get the world coordinates
            dist_to_robot = std::hypot(robot_position_.x - wx,
                robot_position_.y - wy);

            double cell_cost = 0.0;

            if (min_range_ < dist_to_robot && dist_to_robot < update_range_) {
                Tpoint p = { wx, wy, 0.0, 0.0 };
                // location, people, relations, goal, params, discount, horizon
                cell_cost = cost_function_->cost(p, persons_, relations_, goal_, cfparams_, robot_position_, 1.0, prediction_horizon_);
            }

            // store the cost of the cell with its coordinates
            std::pair<int, int> key = { i, j };
            social_cost_map_.insert(std::make_pair(key, cell_cost));
            all_costs.push_back(cell_cost);
        }
    }

    if (all_costs.size() < 1) {
        min_social_cost_ = 0.0;
        max_social_cost_ = 0.0;
    }
    else {
        min_social_cost_ = *std::min_element(all_costs.begin(), all_costs.end());
        max_social_cost_ = *std::max_element(all_costs.begin(), all_costs.end());
    }
}

/// ---------------------------------------------------------------------------

std::vector<double> SocialComplianceLayer::selectWeights(std::string behavior_name)
{
    std::vector<double> weights = { 1.0, 1.0, 1.0 }; // NOTE - too brittle
    if (behavior_name == "polite")
        weights = polite_;
    else if (behavior_name == "rude")
        weights = rude_;
    else if (behavior_name == "sociable")
        weights = sociable_;
    else
        ROS_ERROR("Invalid behavior name [%s]", behavior_name.c_str());

    return weights;
}

/// ---------------------------------------------------------------------------

void SocialComplianceLayer::updateRobotPose()
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

    robot_position_.x = transform.getOrigin().x();
    robot_position_.y = transform.getOrigin().y();
    robot_position_.vx = cos(yaw);
    robot_position_.vy = sin(yaw);
}

/// ---------------------------------------------------------------------------
/// \function updateCosts
/// \brief Update the cost by overwriting the previous costmap in the area
/// defined by the bounds
/// ---------------------------------------------------------------------------
void SocialComplianceLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j)
{
    if (!enabled_)
        return;

    updateRobotPose();

    // compute the social costs
    social_cost_map_.clear();
    computeLayerCostmap(min_i, min_j, max_i, max_j);
    if (social_cost_map_.size() < 1)
        return;

    unsigned int current_cell_cost = 0;
    for (int j = min_j; j < max_j; j++) {
        for (int i = min_i; i < max_i; i++) {
            std::pair<int, int> cell = { i, j };

            auto search = social_cost_map_.find(cell);
            if (search != social_cost_map_.end()) {
                double real_cost = mapRange(search->second, min_social_cost_, max_social_cost_, 0, 254);
                current_cell_cost = master_grid.getCost(i, j);
                master_grid.setCost(i, j, std::max(current_cell_cost, static_cast<unsigned int>(real_cost)));
            }

        }
    }
}

/// Goal callback
void SocialComplianceLayer::callbackSetGoal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    goal_ = Tpoint{ msg->pose.position.x, msg->pose.position.y };
}

/// ---------------------------------------------------------------------------
/// \function matchSize
/// \brief Match size of the costmap with the bounds
/// ---------------------------------------------------------------------------
void SocialComplianceLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(),
        master->getSizeInCellsY(),
        master->getResolution(),
        master->getOriginX(),
        master->getOriginY());
}

/// ---------------------------------------------------------------------------
/// \function reconfigureCB
/// \brief For automatic enable/disable of this layer
/// ---------------------------------------------------------------------------
void SocialComplianceLayer::reconfigureCB(LayerConfig& config, uint32_t level)
{
    cfparams_->thresh_link = config.thresh_link;
    cfparams_->thresh_person = config.thresh_person;
    cfparams_->scaled = config.scaled;
    cfparams_->region_type = config.region_type;
    cfparams_->weights = selectWeights(config.behavior_name);
    cfparams_->thresh_static = config.thresh_static;
    // cfparams_->front_horizon = config.front_horizon;

    prediction_horizon_ = config.prediction_horizon;

    // layer update range limit (how far drom the robot the layer should be updated)
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
void SocialComplianceLayer::updateBounds(double robot_x,
    double robot_y,
    double robot_yaw,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y)
{
    if (!enabled_)
        return;

    if (layered_costmap_->isRolling())
        updateOrigin(robot_x - getSizeInMetersX() / 2.0,
            robot_y - getSizeInMetersY() / 2.0);

    *max_y = *max_y;
    *min_y = *min_y;
    *max_x = *max_x;
    *max_y = *max_y;
}


/// -----------------------------------------------------------
/// \function callbackTrackedPersons
/// \brief Receives tracked persons messages and saves them
/// -----------------------------------------------------------
void SocialComplianceLayer::callbackTrackedPersons(const TPersons::ConstPtr& msg)
{
    tf::StampedTransform costmap_transform;
    try {
        tf_->waitForTransform(costmap_frame_,
            msg->header.frame_id,
            ros::Time::now(),
            ros::Duration(0.05));
        tf_->lookupTransform(costmap_frame_,
            msg->header.frame_id,
            ros::Time(0), costmap_transform);
    }
    catch (tf::TransformException& e) {
        ROS_WARN_STREAM_THROTTLE(5.0, "TF lookup from tracking frame to costmap frame failed. Reason: " << e.what());
        return;
    }

    persons_.clear();
    persons_map_.clear();

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

        Covariance cov = { p.pose.covariance[0], p.pose.covariance[7], p.pose.covariance[1] };
        pd.pos_cov = cov;

        persons_map_.insert(std::make_pair(p.track_id, pd));
        persons_.emplace_back(pd);
    }
}

/// -----------------------------------------------------------
/// \function callbackTrackedGroups
/// \brief Receives tracked groups messages and saves them,
/// also constructs relations and publishes markers for
/// visualizing the relations in Rviz
/// -----------------------------------------------------------
void SocialComplianceLayer::callbackTrackedGroups(const TGroups::ConstPtr& msg)
{
    relations_.clear();
    for (const spencer_tracking_msgs::TrackedGroup& group : msg->groups) {
        int n = group.track_ids.size();
        std::vector<std::vector<int> > combinations = generateCombinations(n, 2);
        for (std::vector<int> v : combinations) {
            Person p1 = persons_map_[group.track_ids[v[0]]];
            Person p2 = persons_map_[group.track_ids[v[1]]];
            relations_.emplace_back(PairWiseRelation(p1.x, p1.y, p2.x, p2.y));
        }
    }
}

/// -----------------------------------------------------------

void SocialComplianceLayer::callbackGlobalPath(const nav_msgs::Path::ConstPtr& path)
{
    for (const auto& pose : path->poses) {
        global_path_->poses.push_back(pose);
    }
}

/// -----------------------------------------------------------
/// Find a the next milestone along the planned path
///
point_t SocialComplianceLayer::nextMilestone()
{
    // If no path is available, use the the global goal
    if (global_path_->poses.size() < 1)
        return point_t(goal_[0], goal_[1]);

    // find a point 'update_range' distance away along the global path
    point_t milestone = { global_path_->poses[0].pose.position.x, global_path_->poses[0].pose.position.y };
    double milestone_distance = edist(robot_position_, milestone);

    unsigned int i = 1;
    while (milestone_distance < update_range_) {
        point_t m = { global_path_->poses[i].pose.position.x, global_path_->poses[i].pose.position.y };
        milestone_distance = edist(robot_position_, m);
        if (milestone_distance < update_range_) {
            milestone = m;
        }
        else
            break;

        i++;
    }

    return milestone;
}

} // end namespace
