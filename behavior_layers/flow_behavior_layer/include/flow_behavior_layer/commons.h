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



#ifndef COMMONS_H
#define COMMONS_H


#include <cmath>
#include <iterator>
#include <algorithm>

namespace flow_behavior_layer
{


/// ------------------------------------------------------------------------


struct Person
{
    double x, y, vx, vy;

    Person() = default;
    Person(const double& x, const double& y)
    : x(x), y(y), vx(0), vy(0) {}
    Person(const double& x, const double& y,
           const double& vx, const double& vy)
    : x(x), y(y), vx(vx), vy(vy) {}
    Person(const Person& co)
    : x(co.x), y(co.y), vx(co.vx), vy(co.vy) {}

    // move, copy constructors, operators
    Person(Person&&) = default;
    Person& operator=(const Person&) = default;
    Person& operator=(Person&&) = default;
    double operator[] (size_t index) const
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


struct TrajectoryWaypoint
{
    double x, y, vx, vy;

    TrajectoryWaypoint() = default;
    TrajectoryWaypoint(const double& x, const double& y)
    : x(x), y(y), vx(0), vy(0) {}
    TrajectoryWaypoint(const double& x, const double& y,
           const double& vx, const double& vy)
    : x(x), y(y), vx(vx), vy(vy) {}
    TrajectoryWaypoint& operator=(const TrajectoryWaypoint&) = default;
    double operator[] (size_t index) const
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

struct point_t
{
    double x, y;

    point_t() = default;
    point_t(const double& x, const double& y) : x(x), y(y) {}

    point_t& operator=(const point_t&) = default;
    double operator[] (size_t index) const
    {
        assert(index <= 1);
        if (index == 0)
            return x;
        else
            return y;
    }
};

/// ------------------------------------------------------------------------


using Tpoint = TrajectoryWaypoint;
using Trajectory = std::vector<Tpoint>;
using VPersons = std::vector<Person>;


/// ------------------------------------------------------------------------


template<typename PointType>
inline double angleBetween(const PointType& p1, const PointType& p2,
                           const PointType& p3, const PointType& p4)
{
    double heading1 = atan2(p2[1]-p1[1], p2[0]-p1[0]);
    double heading2 = atan2(p4[1]-p3[1], p4[0]-p3[0]);
    return (heading1 - heading2);
}


/// ------------------------------------------------------------------------



/**
 * @brief edist
 * @details Euclidean distance between two poses, points or a pose and point
 *
 * @param v1 Indexable array
 * @param v2 Indexable array
 *
 * @return double scalar value of the distance
 */
template<typename IndexableA, typename IndexableB>
inline double edist(const IndexableA& v1, const IndexableB& v2)
{
    return std::hypot((v1[0]-v2[0]), (v1[1]-v2[1]));
}


/// ------------------------------------------------------------------------


/**
 * @brief mapRange
 * @details scale a number from one range to another
 *
 * @param value number to scale
 * @param mina start original range
 * @param maxa end original range
 * @param mint start target range
 * @param maxt end target range
 * @return result
 */
inline double mapRange(const double& value,
    const double& mina, const double& maxa,
    const double& mint, const double& maxt)
{
    double denom = maxa - mina;
    if (abs(denom) < std::numeric_limits<float>::min())
        denom = std::numeric_limits<float>::min();

    return mint + ((value - mina) * (maxt - mint) / denom);
}


/// ------------------------------------------------------------------------

inline double goalOrientation(const Person& person, const Tpoint& goal)
{
    auto speed = std::hypot(person.vx, person.vy);
    point_t pnext = point_t{person.x + speed * person.vx,
                            person.y + speed * person.vy};
    double dnow = edist(goal, person);
    double dnext = edist(goal, pnext);

    return (dnext - dnow);
}


inline double sigmoid(const double& value)
{
    return 1.0 / (1.0 + exp(-value));
}



} // end namespace

#endif
