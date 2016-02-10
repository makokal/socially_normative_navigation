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


#include <flow_behavior_layer/commons.h>

namespace flow_behavior_layer
{



class FlowCostFunction
{
public:
    FlowCostFunction(const std::vector<double>& weights,
                     const double& discount,
                     const double& radius=1.2,
                     const double& cr=1.2,
                     const double& cp=2.4)
    : gamma_{discount},
      weights_{weights},
      zone_radius_{radius}
    {}
    ~FlowCostFunction() = default;

    double cost(const Trajectory& traj,
                const VPersons& people,
                const Tpoint& goal,
                const double& radius=1.2)
    {
        zone_radius_ = radius;

        std::vector<double> features(3, 0.0);

        features[0] = density(traj, people);
        features[1] = relativeHeading(traj, people, goal);
        features[2] = goalDistance(traj, goal);

        double cost = vdot(features, weights_);
        // double cost = evaluateCost(features);

        return std::move(cost);
    }


    double vdot(const std::vector<double>& features,
                const std::vector<double>& weights)
    {
        assert(features.size() == weights.size());
        double value = 0.0;
        for (unsigned int i = 0; i < features.size(); i++)
            value += features[i] * weights[i];
        return value;
    }

    double evaluateCost(const std::vector<double>& features)
    {
        double cost = 0.0;
        cost += features[1] * weights_[1];

        if (features[2] < 0.5)
            cost = 0.0;
        else
            cost += features[2] * weights_[2];

        return cost;
    }

protected:


    double goalDistance(const Trajectory& traj, const Tpoint& goal)
    {
        double gd = 0.0;
        int index = 0;
        for (const Tpoint& waypoint: traj)
        {
            gd += edist(waypoint, goal) * std::pow(gamma_, float(index));
            index++;
        }

        return std::move(gd);
    }

    double density(const Trajectory& traj, const VPersons& people)
    {
        if (people.size() < 1)
            return 0.0;

        double phi = 0.0;
        for (unsigned int t = 0; t < traj.size(); t++)
        {

            int den = 0;
            for (const Person& p : people)
            {
                if (edist(traj[t], p) <= zone_radius_)
                    den++;
            }

            phi += den * std::pow(gamma_, float(t));
        }

        return std::move(phi);
    }


    double relativeHeading(const Trajectory& traj,
                           const VPersons& people,
                           const Tpoint& goal)
    {
        if (people.size() < 1)
            return 0.0;

        double phi = 0.0;
        for (unsigned int t = 0; t < traj.size(); t++)
        {
            double density = 0;
            double goal_pull = 0.0;
            for (const Person& p : people)
            {
                if (edist(traj[t], p) <= zone_radius_)
                {
                    density++;
                    // goal_pull += goalOrientation(p, goal);
                    // goal_pull += sigmoid(goalOrientation(p, goal));
                    goal_pull += tanh(goalOrientation(p, goal));
                }
            }

            if (density > 0)
                goal_pull /= density;

            phi += goal_pull * std::pow(gamma_, float(t));
        }

        return std::move(phi);
    }


private:

    double gamma_;
    std::vector<double> weights_;

    double zone_radius_;
};



} // end namespace

#endif
