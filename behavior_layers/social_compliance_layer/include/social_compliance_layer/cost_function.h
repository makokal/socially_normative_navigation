/**
* Copyright 2014 Social Robotics Lab, University of Freiburg
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

#ifndef COST_FUNCTION_H
#define COST_FUNCTION_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cassert>
#include <cmath>

#include <social_compliance_layer/utils.h>
#include <nav_msgs/Path.h>


namespace social_compliance_layer {

/// Cost function parameters
struct CFParams {
    double thresh_link, thresh_person, thresh_static, front_horizon;
    std::string region_type;
    bool scaled;
    std::vector<double> weights;
};

/// ------------------------------------------------------------------------
/**
 * @brief SocialNavigationCost
 * @details Social compliance cost function that uses linear basis functions
 * compute distances to persons or affordances (links). The semantics are
 * therefore such that the shorter distances are penalized more.
 *
 * @note Because of interference from the basis features, it is necessary to add
 * cuttoff to mark the region of influence of the basis features or alternatively
 * order everything semantic identity in the environment which is harder in practise.
 */
class SocialNavigationCost {
public:
    SocialNavigationCost(const double& discount,
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
        front_horizon_ = params->front_horizon;

        socio_zone_ = 0.5;
    }

    ~SocialNavigationCost() = default;

    double cost(const Trajectory& traj,
        const VPersons& people,
        const VRelations& relations,
        const Tpoint& goal,
        std::shared_ptr<CFParams> params,
        const std::vector<double>& robot_state,
        const point_t& milestone)
    {
        // load parameters
        cutoff_relation_ = params->thresh_link;
        cutoff_personal_ = params->thresh_person;
        thresh_static_ = params->thresh_static;
        region_type_ = params->region_type;
        scaled_ = params->scaled;
        weights_ = params->weights;
        robot_ = robot_state;
        front_horizon_ = params->front_horizon;

        std::vector<double> features(3, 0.0);
        features[0] = relationDisturbance(traj, relations);
        features[1] = socialDisturbance(traj, people);
        features[2] = goalDeviationAngle(traj, goal);

        double cost = vdot(features, weights_);

        return std::move(cost);
    }

protected:
    double goalDeviationAngle(const Trajectory& traj, const Tpoint& goal)
    {
        if (traj.size() < 2)
            return 0.0;

        double gd = 0.0;
        for (unsigned int i = 0; i < traj.size() - 1; i++) {
            gd += angleBetween(traj[i], traj[i + 1], traj[i], goal) * std::pow(gamma_, float(i));
        }

        return std::move(gd);
    }

    double relationDisturbance(const Trajectory& traj, const VRelations& relations)
    {
        double feature = 0.0F;
        if (relations.size() < 1)
            return feature;

        for (unsigned int t = 0; t < traj.size(); t++) {
            // for every waypoints, compute distance to the closest link
            for (const auto& relation : relations) {
                point_t line_start = { relation.xs, relation.ys };
                point_t line_end = { relation.xe, relation.ye };

                auto result = distance2Segment(traj[t], line_start, line_end);
                if (result.second) {
                    if (result.first < cutoff_relation_) {
                        feature += (cutoff_relation_ - result.first) * std::pow(gamma_, float(t));
                        ;
                    }
                }
            }
        }

        return std::move(feature);
    }

    double socialDisturbance(const Trajectory& traj, const VPersons& people)
    {
        double feature = 0.0F;
        if (people.size() < 1)
            return feature;

        for (unsigned int t = 0; t < traj.size(); t++) {
            Tpoint waypoint = traj[t];
            Person closest_person = people[0];
            double cdist = edist(closest_person, waypoint);

            if (people.size() >= 2) {
                for (unsigned int j = 1; j < people.size(); j++) {
                    double dist = edist(people[j], waypoint);
                    if (dist < cdist) {
                        closest_person = people[j];
                        cdist = dist;
                    }
                }
            }

            double boundary = cutoff_personal_;

            // check if person is static or moving
            double speed = std::hypot(closest_person.vx, closest_person.vy);
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
                    double ad = anisotropicDistance(closest_person, waypoint, speed*3.0);
                    if ((cdist < ad) && (cdist < socio_zone_) && (cdist < boundary))
                        feature += (boundary - cdist) * std::pow(gamma_, float(t));
                    ;

                    if ((cdist < ad) && (cdist > socio_zone_) && (cdist < boundary))
                        feature += (cdist - boundary) * std::pow(gamma_, float(t));
                    ;
                }
                else if (region_type_ == "uniform") {
                    if ((cdist < socio_zone_) && (cdist < boundary))
                        feature += (boundary - cdist) * std::pow(gamma_, float(t));
                    ;

                    if ((cdist > socio_zone_) && (cdist < boundary))
                        feature += (cdist - boundary) * std::pow(gamma_, float(t));
                    ;
                }
            }
            else {
                if (region_type_ == "anisotropic") {
                    double ad = anisotropicDistance(closest_person, waypoint, speed*3.0);
                    if ((cdist < ad) && (cdist < boundary))
                        feature += (boundary - cdist) * std::pow(gamma_, float(t));
                    ;
                }
                else if (region_type_ == "uniform") {
                    if (cdist < boundary)
                        feature += (boundary - cdist) * std::pow(gamma_, float(t));
                    ;
                }
            }
        }

        return feature;
    }

    double frontHeading(const Trajectory& traj, const VPersons& people, const Tpoint& goal, const point_t& milestone)
    {
        double phi = 0.0;
        for (unsigned int t = 0; t < traj.size(); t++)
        {
            if (edist(traj[t], robot_) < front_horizon_) {
                if (inFrontOfRobot(robot_[0], robot_[1], robot_[2], robot_[3], traj[t][0], traj[t][1])) {
                    double density = 0;
                    double goal_pull = 0.0;
                    for (const Person& p : people) {
                        if (edist(traj[t], p) <= front_horizon_) {
                            density++;
                            goal_pull += goalOrientation(p, milestone);
                        }
                    }

                    if (density > 0)
                        goal_pull /= density;

                    phi += goal_pull * std::pow(gamma_, float(t));
                }
            }
        }

        return std::move(phi);
    }

private:
    std::vector<double> weights_;
    std::vector<double> robot_;
    double gamma_;
    double cutoff_relation_;
    double cutoff_personal_;
    double thresh_static_;
    double socio_zone_;
    double front_horizon_;
    std::string region_type_;
    bool scaled_;
    std::string behavior_name_;
};

} // end namespace

#endif
