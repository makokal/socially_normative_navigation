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

#ifndef FLOW_COST_FUNCTION_H
#define FLOW_COST_FUNCTION_H

#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cassert>

#include <behavior_utils/entities.h>
#include <behavior_utils/geometry.h>
#include <behavior_utils/helpers.h>

using namespace behavior_utils;

namespace behavior_functions {

class FlowCostFunction {
public:
    FlowCostFunction(const std::vector<double>& weights,
        const double& discount,
        const double& radius = 1.2);
    ~FlowCostFunction() = default;

    double cost(const Trajectory& traj,
        const VPersons& people,
        const Tpoint& goal,
        const double& radius = 1.2);

    double cost(const Tpoint& location,
        const VPersons& people,
        const Tpoint& goal,
        const double& radius = 1.2,
        const double& t=1.0);

    double evaluateCost(const std::vector<double>& features);

protected:
    double goalDistance(const Tpoint& location, const Tpoint& goal, const double& t=1.0);

    double density(const Tpoint& location, const VPersons& people, const double& t=1.0);

    double relativeHeading(const Tpoint& location,
        const VPersons& people,
        const Tpoint& goal,
        const double& t=1.0);

private:
    double gamma_;
    std::vector<double> weights_;
    double zone_radius_;
};



/// ======================================================================
///
/// ======================================================================


FlowCostFunction::FlowCostFunction(const std::vector<double>& weights,
    const double& discount,
    const double& radius)
    : gamma_{ discount }
    , weights_{ weights }
    , zone_radius_{ radius }
{
}

double FlowCostFunction::cost(const Tpoint& location,
        const VPersons& people,
        const Tpoint& goal,
        const double& radius,
        const double& t)
{
    zone_radius_ = radius;

    std::vector<double> features(3, 0.0);
    // NOTE - demo hacks
    // features[0] = density(location, people, t);
    features[1] = relativeHeading(location, people, goal, t);
    // features[2] = goalDistance(location, goal, t);

    return vdot(features, weights_);
}

double FlowCostFunction::cost(const Trajectory& traj,
    const VPersons& people,
    const Tpoint& goal,
    const double& radius)
{
    double c = 0.0;
    for (unsigned int t = 0; t < traj.size(); t++)
        c += cost(traj[t], people, goal, radius, float(t));

    return c;
}


double FlowCostFunction::goalDistance(const Tpoint& location, const Tpoint& goal, const double& t)
{
    return edist(location, goal) * std::pow(gamma_, float(t));
}

double FlowCostFunction::density(const Tpoint& location, const VPersons& people, const double& t)
{
    if (people.size() < 1)
        return 0.0;

    double density = 0.0;
    for (const Person& p : people) {
        double d = edist(location, p);
        density += gaussianPdf(d, 0.0, 1.0);
        // if (d < zone_radius_) {
        //     density += gaussianPdf(d, 0.0, 1.0);
        //     // density += 1.0;
        // }
    }

    return density * std::pow(gamma_, float(t));
}

double FlowCostFunction::relativeHeading(const Tpoint& location,
    const VPersons& people,
    const Tpoint& goal,
    const double& t)
{
    if (people.size() < 1)
        return 0.0;

    double goal_pull = 0.0;
    double density = 0.0;
    for (const Person& p : people) {
        double d = edist(location, p);
        // goal_pull += gaussianPdf(d, 0.0, 5.0) * tanh(goalOrientation(p, goal));
        if (d < zone_radius_) {
            // - heading to goal, + away from goal
            // double x = tanh(goalOrientation(p, goal));

            // if (x > 0.0)
            //     goal_pull += gaussianPdf(d, 0.0, 1.0) * x;
            // else
            //     goal_pull += x;


            goal_pull += gaussianPdf(d, 0.0, 1.0) * tanh(goalOrientation(p, goal));
            density += gaussianPdf(d, 0.0, 1.0) * 1.0;
        }
    }

    if (density > 0.0)
        goal_pull /= density;

    return goal_pull * std::pow(gamma_, float(t));
}



} // end of namespace

#endif // FLOW_COST_FUNCTION_H
