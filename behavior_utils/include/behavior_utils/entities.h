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

#ifndef NORMATIVE_BEHAVIOR_ENTITIES_H
#define NORMATIVE_BEHAVIOR_ENTITIES_H

#include <cassert>
#include <cmath>

// #include <geometry_msgs/Point.h>


namespace behavior_utils
{


/// ------------------------------------------------------------------------
/// 2D covariance matrix
struct Covariance {
    double xx, yy, xy;

    Covariance() = default;
    Covariance(const double& x, const double& y, const double& xy)
        : xx{ x }
        , yy{ y }
        , xy{ xy }
    {
    }
    Covariance(const Covariance& other)
        : xx{ other.xx }
        , yy{ other.yy }
        , xy{ other.xy }
    {
    }

    // move, copy constructors, operators
    Covariance(Covariance&&) = default;
    Covariance& operator=(const Covariance&) = default;
    Covariance& operator=(Covariance&&) = default;
};


/// ------------------------------------------------------------------------

struct Person {
    double x, y, vx, vy;
    Covariance pos_cov;

    Person() = default;
    Person(const double& x, const double& y)
        : x(x)
        , y(y)
        , vx(0)
        , vy(0)
    {
    }
    Person(const double& x, const double& y,
        const double& vx, const double& vy)
        : x(x)
        , y(y)
        , vx(vx)
        , vy(vy)
    {
    }
    Person(const Person& p)
        : x(p.x)
        , y(p.y)
        , vx(p.vx)
        , vy(p.vy)
        , pos_cov(Covariance())
    {
    }

    // move, copy constructors, operators
    Person(Person&&) = default;
    Person& operator=(const Person&) = default;
    Person& operator=(Person&&) = default;
    double operator[](size_t index) const
    {
        assert(index <= 3);

        if (index == 0)
            return x;
        else if (index == 1)
            return y;
        else if (index == 2)
            return vx;
        else
            return vy;
    }

    double heading() const { return atan2(vy, vx); }
};


/// ------------------------------------------------------------------------

struct TrajectoryWaypoint {
    double x, y, vx, vy;

    TrajectoryWaypoint() = default;
    TrajectoryWaypoint(const double& x, const double& y)
        : x(x)
        , y(y)
        , vx(0)
        , vy(0)
    {
    }
    TrajectoryWaypoint(const double& x, const double& y,
        const double& vx, const double& vy)
        : x(x)
        , y(y)
        , vx(vx)
        , vy(vy)
    {
    }
    TrajectoryWaypoint& operator=(const TrajectoryWaypoint&) = default;
    double operator[](size_t index) const
    {
        assert(index <= 3);

        if (index == 0)
            return x;
        else if (index == 1)
            return y;
        else if (index == 2)
            return vx;
        else
            return vy;
    }
};

/// ------------------------------------------------------------------------

struct point_t {
    double x, y;

    point_t() = default;
    point_t(const double& x, const double& y)
        : x(x)
        , y(y)
    {
    }

    point_t& operator=(const point_t&) = default;
    double operator[](size_t index) const
    {
        assert(index <= 1);
        if (index == 0)
            return x;
        else
            return y;
    }
};

// using point_t = geometry_msgs::Point;

/// ------------------------------------------------------------------------

/**
 * @brief Pairwise relation
 */
struct PairWiseRelation {
    double xs, ys, xe, ye;
    double probability;

    PairWiseRelation() = default;
    PairWiseRelation(const double& x1, const double& y1,
        const double& x2, const double& y2)
        : xs(x1)
        , ys(y1)
        , xe(x2)
        , ye(y2)
        , probability(0.0)
    {
    }
    PairWiseRelation(const double& x1, const double& y1,
        const double& x2, const double& y2, const double& p)
        : xs(x1)
        , ys(y1)
        , xe(x2)
        , ye(y2)
        , probability(p)
    {
    }

    PairWiseRelation(const point_t& p1, const point_t& p2)
        : xs(p1.x)
        , ys(p1.y)
        , xe(p2.x)
        , ye(p2.y)
        , probability(0.0)
    {
    }
    PairWiseRelation(const point_t& p1, const point_t& p2, const double& p)
        : xs(p1.x)
        , ys(p1.y)
        , xe(p2.x)
        , ye(p2.y)
        , probability(p)
    {
    }

    PairWiseRelation(PairWiseRelation&&) = default;
    PairWiseRelation& operator=(const PairWiseRelation&) = default;
    PairWiseRelation& operator=(PairWiseRelation&&) = default;
};


/// Convenient aliases
using Tpoint = TrajectoryWaypoint;
using Trajectory = std::vector<Tpoint>;
using VPersons = std::vector<Person>;
using VRelations = std::vector<PairWiseRelation>;


} // end of namespace

#endif // NORMATIVE_BEHAVIOR_ENTITIES_H
