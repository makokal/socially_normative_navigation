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

#include <overtaking_layer/overtaking_layer.h>
#include <pluginlib/class_list_macros.h>

#include <memory>


PLUGINLIB_EXPORT_CLASS(overtaking_layer::OvertakingLayer, costmap_2d::Layer)

namespace overtaking_layer {

using namespace behavior_utils;


OvertakingLayer::OvertakingLayer() {}
OvertakingLayer::~OvertakingLayer()
{
    // cleanup to avoid mem leaks
    sub_persons_.shutdown();
}

void OvertakingLayer::onInitialize()
{
    ros::NodeHandle nh("~/" + name_);
    current_ = true;
    default_value_ = costmap_2d::NO_INFORMATION;
    matchSize();

    // read topics for data input
    std::string people_topic;
    nh.param("people_topic", people_topic, std::string(""));

    // setup subscribers and publishers for data exchange
    sub_persons_ = nh.subscribe(people_topic, 10, &OvertakingLayer::callbackTrackedPersons, this);
    pub_next_goal_ = nh.advertise<geometry_msgs::PoseStamped>("/overtaking_layer/next_goal", 1);

    // setting up TF frames for costmap and robot
    costmap_frame_ = layered_costmap_->getGlobalFrameID();
    ros::param::param<std::string>("/move_base_node/global_costmap/robot_base_frame",
        robot_base_frame_, std::string(""));


    // setup dynamic reconfigure server
    dsrv_ = new dynamic_reconfigure::Server<overtaking_layer::OvertakingLayerConfig>(nh);
    dynamic_reconfigure::Server<overtaking_layer::OvertakingLayerConfig>::CallbackType cb = boost::bind(
        &OvertakingLayer::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
}

void OvertakingLayer::updateBounds(double robot_x,
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

void OvertakingLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j)
{
    if (!enabled_)
        return;

    updateRobotPose();

    // TODO - add the ovetaking costmap
    //
}

void OvertakingLayer::matchSize()
{
    Costmap2D* master = layered_costmap_->getCostmap();
    resizeMap(master->getSizeInCellsX(),
        master->getSizeInCellsY(),
        master->getResolution(),
        master->getOriginX(),
        master->getOriginY());
}


void OvertakingLayer::reconfigureCB(LayerConfig& config, uint32_t level)
{
    if (enabled_ != config.enabled) {
        enabled_ = config.enabled;
    }
}


void OvertakingLayer::updateRobotPose()
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

void OvertakingLayer::callbackTrackedPersons(const TPersons::ConstPtr& msg)
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
    for (const auto& p : msg->tracks) {
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

behavior_utils::Person OvertakingLayer::getPersonInFront()
{
    if (persons_.size() < 1) {
        ROS_ERROR_STREAM("No people around to overtake!!");
    }

    return persons_[0];
}


}  // end of namespace
