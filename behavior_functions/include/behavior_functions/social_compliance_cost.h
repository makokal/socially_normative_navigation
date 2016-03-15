/**
* Copyright 2016 Social Robotics Lab, University of Freiburg
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

#ifndef SOCIAL_COMPLIANCE_COST_H
#define SOCIAL_COMPLIANCE_COST_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cassert>
#include <cmath>

#include <behavior_utils/entities.h>
#include <behavior_utils/geometry.h>
#include <behavior_utils/helpers.h>
#include <behavior_utils/motion_prediction.h>

#include <nav_msgs/Path.h>

using namespace behavior_utils;


namespace behavior_functions {

/// Cost function parameters
struct CFParams {
    double thresh_link, thresh_person, thresh_static;
    std::string region_type;
    bool scaled;
    std::vector<double> weights;
};

/// ------------------------------------------------------------------------
/**
 * @brief SocialComplianceCost
 * @details Social compliance cost function that uses linear basis functions
 * compute distances to persons or affordances (links). The semantics are
 * therefore such that the shorter distances are penalized more.
 *
 * @note Because of interference from the basis features, it is necessary to add
 * cuttoff to mark the region of influence of the basis features or alternatively
 * order everything semantic identity in the environment which is harder in practise.
 */
class SocialComplianceCost {
public:
    SocialComplianceCost(const double& discount,
        const std::string& behavior_name,
        std::shared_ptr<CFParams> params);

    ~SocialComplianceCost() = default;

    double cost(const Tpoint& location,
        const VPersons& people,
        const VRelations& relations,
        const Tpoint& goal,
        std::shared_ptr<CFParams> params,
        const Person& robot,
        const double& t=1.0,
        const int& horizon=4,
        const bool& dynamic_scene=true);

    double cost(const Trajectory& traj,
        const VPersons& people,
        const VRelations& relations,
        const Tpoint& goal,
        std::shared_ptr<CFParams> params,
        const Person& robot,
        const int& horizon=4,
        const bool& dynamic_scene=true);

protected:

    double goalDeviationAngle(const Trajectory& traj, const Tpoint& goal);

    double relationDisturbance(const Tpoint& traj, const VRelations& relations, const double& t=1.0);

    double socialDisturbance(const Tpoint& traj, const VPersons& people, const double& t=1.0);

    Person findClosestPerson(const VPersons& people, const Tpoint& location);


private:
    std::vector<double> weights_;
    // std::vector<double> robot_;
    double gamma_;
    double cutoff_relation_;
    double cutoff_personal_;
    double thresh_static_;
    double socio_zone_;
    std::string region_type_;
    bool scaled_;
    std::string behavior_name_;
};

/// ======================================================================
///
/// ======================================================================

SocialComplianceCost::SocialComplianceCost(const double& discount,
    const std::string& behavior_name,
    std::shared_ptr<CFParams> params)
    : gamma_{ discount }
    , behavior_name_{ behavior_name }
    , region_type_{ params->region_type }
    , weights_{ params->weights }
{
    cutoff_relation_ = params->thresh_link;
    cutoff_personal_ = params->thresh_person;
    thresh_static_ = params->thresh_static;
    scaled_ = params->scaled;

    socio_zone_ = 0.5;
}


double SocialComplianceCost::cost(const Tpoint& location,
        const VPersons& people,
        const VRelations& relations,
        const Tpoint& goal,
        std::shared_ptr<CFParams> params,
        const Person& robot,
        const double& t,
        const int& horizon,
        const bool& dynamic_scene)
{
    // load parameters
    cutoff_relation_ = params->thresh_link;
    cutoff_personal_ = params->thresh_person;
    thresh_static_ = params->thresh_static;
    region_type_ = params->region_type;
    scaled_ = params->scaled;
    weights_ = params->weights;

    if (behavior_name_ == "rude") {
        return 0.0;
    }

    double full_cost = 0.0;
    std::vector<double> features(3, 0.0);

    // In case of static scenes or when no prediction is desired, do not skip
    // the current time step
    if (!dynamic_scene || horizon < 1) {
        features[0] = relationDisturbance(location, relations, t);
        features[1] = socialDisturbance(location, people, t);
        features[2] = 0.0;
        full_cost = behavior_utils::vdot(features, weights_);
        return full_cost;
    }


    for (int i = 1; i < horizon; i++) {
        // make people predictions for the horizon
        VPersons people_h_i;
        for (const auto& p : people)
            // check if in front (make prediction, else just skip)
            if (inFrontOfRobot(robot, p))
                people_h_i.push_back( costantVelocityPrediction(p, float(i)) );

        features[0] = relationDisturbance(location, relations, t);
        features[1] = socialDisturbance(location, people_h_i, t);
        features[2] = 0.0;
        double c = behavior_utils::vdot(features, weights_);

        full_cost += costDecay(c, horizon, float(i));
    }

    return full_cost;
}

double SocialComplianceCost::cost(const Trajectory& traj,
    const VPersons& people,
    const VRelations& relations,
    const Tpoint& goal,
    std::shared_ptr<CFParams> params,
    const Person& robot,
    const int& horizon,
    const bool& dynamic_scene)
{
    double c = 0.0;
    for (unsigned int t = 0; t < traj.size(); t++)
        c += cost(traj[t], people, relations, goal, params, robot, float(t), horizon, dynamic_scene);

    c += (goalDeviationAngle(traj, goal) * weights_[2]);

    return c;
}



/// ========================================================================


double SocialComplianceCost::goalDeviationAngle(const Trajectory& traj, const Tpoint& goal)
{
    if (traj.size() < 2)
        return 0.0;

    double gd = 0.0;
    for (unsigned int i = 0; i < traj.size() - 1; i++) {
        gd += angleBetween(traj[i], traj[i + 1], traj[i], goal) * std::pow(gamma_, float(i));
    }

    return std::move(gd);
}

double SocialComplianceCost::relationDisturbance(const Tpoint& location, const VRelations& relations, const double& t)
{
    double feature = 0.0F;
    if (relations.size() < 1)
        return feature;

    for (const auto& relation : relations) {
        point_t line_start = { relation.xs, relation.ys };
        point_t line_end = { relation.xe, relation.ye };

        auto result = distance2Segment(location, line_start, line_end);
        if (result.second) {
            // feature += 4.0 * gaussianPdf(result.first, 0.0, 0.36) * std::pow(gamma_, float(t));

            if (result.first < cutoff_relation_) {
                feature += (cutoff_relation_ - result.first) * std::pow(gamma_, float(t));
                ;
            }
        }
    }

    return std::move(feature);
}

double SocialComplianceCost::socialDisturbance(const Tpoint& location, const VPersons& people, const double& t)
{
    double feature = 0.0F;
    if (people.size() < 1)
        return feature;

    Person closest_person = findClosestPerson(people, location);
    double cdist = edist(closest_person, location);
    double speed = std::hypot(closest_person.vx, closest_person.vy);

    double discount = std::pow(gamma_, float(t));

    // double sigma = 1.0;
    // if (behavior_name_ == "sociable") {
    //     sigma = 0.1;
    //     double social_boundary = socio_zone_ + 0.8;
    //     if (cdist > socio_zone_ && cdist < social_boundary)
    //         feature += (socio_zone_ - cdist) * discount;
    // }
    // else {

    //     double aniso_boundary = speed * 3.0;
    //     if (region_type_ == "anisotropic") {
    //         double ad = anisotropicDistance(closest_person, location, aniso_boundary);
    //         if (cdist < ad)
    //             feature += gaussianPdf(cdist, 0.0, sigma) * discount;
    //     }
    //     else {
    //         feature += gaussianPdf(cdist * speed, 0.0, sigma) * discount;
    //     }
    // }

    // return feature;



    double boundary = cutoff_personal_;

    // check if person is static or moving
    // if (speed > thresh_static_)
    //     region_type_ = "anisotropic";
    // else
    //     region_type_ = "uniform";

    // scaling influence region by person speed
    if (scaled_ == true) {
        boundary = speed * cutoff_personal_;
        socio_zone_ *= speed;
    }

    if (behavior_name_ == "sociable") {
        if (region_type_ == "anisotropic") {
            double ad = anisotropicDistance(closest_person, location, speed * 3.0);
            if ((cdist < ad) && (cdist < socio_zone_) && (cdist < boundary))
                feature += (boundary - cdist) * discount;
            ;

            if ((cdist < ad) && (cdist > socio_zone_) && (cdist < boundary))
                feature += (cdist - boundary) * discount;
            ;
        }
        else if (region_type_ == "uniform") {
            if ((cdist < socio_zone_) && (cdist < boundary))
                feature += (boundary - cdist) * discount;
            ;

            if ((cdist > socio_zone_) && (cdist < boundary))
                feature += (cdist - boundary) * discount;
            ;
        }
    }
    else {
        if (region_type_ == "anisotropic") {
            double ad = anisotropicDistance(closest_person, location, speed * 3.0);
            if ((cdist < ad) && (cdist < boundary))
                feature += (boundary - cdist) * discount;
            ;
        }
        else if (region_type_ == "uniform") {
            if (cdist < boundary)
                feature += (boundary - cdist) * discount;
            ;
        }
    }

    return feature;
}


/**
 * @brief find the closest person to a location in 2D
 * @details Get the person closest in euclidean sense to a locaton in 2D
 *
 * @param people Iterable container of peoples' two locations with at least 1
 * @param location 2D location of interest
 *
 * @return The person
 */
Person SocialComplianceCost::findClosestPerson(const VPersons& people, const Tpoint& location)
{
    if (people.size() == 1)
        return people[0];

    Person closest_person = people[0];
    double closest_distance = edist(closest_person, location);

    for (unsigned int j = 1; j < people.size(); j++) {
        double distance = edist(people[j], location);
        if (distance < closest_distance) {
            closest_person = people[j];
            closest_distance = distance;
        }
    }

    return closest_person;
}



} // end namespace

#endif
