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

#ifndef UTILS_H
#define UTILS_H

#include <cmath>
#include <iterator>
#include <algorithm>

namespace social_compliance_layer {

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
    Person(const Person& co)
        : x(co.x)
        , y(co.y)
        , vx(co.vx)
        , vy(co.vy)
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

using Tpoint = TrajectoryWaypoint;
using Trajectory = std::vector<Tpoint>;
using VPersons = std::vector<Person>;
using VRelations = std::vector<PairWiseRelation>;

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
template <typename IndexableA, typename IndexableB>
inline double edist(const IndexableA& v1, const IndexableB& v2)
{
    return std::hypot((v1[0] - v2[0]), (v1[1] - v2[1]));
}

/**
 * @brief distance2Segment
 * @details Compute the distance to a line segment in 2D. Other dimensions of
 * the points are ignored
 *
 * @param x 2D point to which we want the distance
 * @param line_start 2D point of the start of the line
 * @param line_end 2D point of the end of the line
 * @return a pair of (scalar, bool) with distance and whether the point is
 * 'inside' the segment of not i.e. which the perpendicular line limits.
 * true means inside
 */
template <typename WaypointType, typename PointType>
inline std::pair<double, bool>
distance2Segment(const WaypointType& x,
    const PointType& line_start,
    const PointType& line_end)
{
    double xa = line_start[0];
    double ya = line_start[1];
    double xb = line_end[0];
    double yb = line_end[1];
    double xp = x[0];
    double yp = x[1];

    // x-coordina&tes
    double f_1 = xb - xa;
    double f_2 = yb - ya;
    double f_3 = yp * f_2 + xp * f_1;
    double a = 2 * ((f_2 * f_2) + (f_1 * f_1));
    double b = -4 * f_1 * f_3 + (2 * yp + ya + yb) * f_1 * f_2 - (2 * xp + xa + xb) * (f_2 * f_2);
    double c = 2 * (f_3 * f_3) - (2 * yp + ya + yb) * f_3 * f_2 + (yp * (ya + yb) + xp * (xa + xb)) * (f_2 * f_2);
    double x1 = (-b + sqrt((b * b) - 4 * a * c)) / (2 * a);
    double x2 = (-b - sqrt((b * b) - 4 * a * c)) / (2 * a);

    // y-coordinates
    double A = yb - ya;
    double B = xb - xa;
    double C = xp * B + yp * A;
    a = 2 * ((B * B) + (A * A));
    b = -4 * A * C + (2 * xp + xa + xb) * A * B - (2 * yp + ya + yb) * (B * B);
    c = 2 * (C * C) - (2 * xp + xa + xb) * C * B + (xp * (xa + xb) + yp * (ya + yb)) * (B * B);
    double y1 = (-b + sqrt((b * b) - 4 * a * c)) / (2 * a);
    double y2 = (-b - sqrt((b * b) - 4 * a * c)) / (2 * a);

    // % Put point candidates together
    std::vector<double> dvec;
    dvec.reserve(4);
    point_t xfm1 = { x1, y1 };
    point_t xfm2 = { x2, y2 };
    point_t xfm3 = { x1, y2 };
    point_t xfm4 = { x2, y1 };

    dvec.push_back(edist(xfm1, x));
    dvec.push_back(edist(xfm2, x));
    dvec.push_back(edist(xfm3, x));
    dvec.push_back(edist(xfm4, x));

    double dmax = -1;
    double imax = -1;
    for (int i = 0; i < 4; i++) {
        if (dvec[i] > dmax) {
            dmax = dvec[i];
            imax = i;
        }
    }

    point_t xf;
    if (imax == 0)
        xf = xfm1;
    else if (imax == 1)
        xf = xfm2;
    else if (imax == 2)
        xf = xfm3;
    else if (imax == 3)
        xf = xfm4;

    point_t line_start_xf = { line_start[0] - xf[0], line_start[1] - xf[1] };
    point_t line_end_xf = { line_end[0] - xf[0], line_end[1] - xf[1] };
    double dotp = (line_start_xf[0] * line_end_xf[0]) + (line_start_xf[1] * line_end_xf[1]);

    bool inside = false;
    if (dotp <= 0.0)
        inside = true;

    return std::make_pair(dmax, inside);
}

/// ------------------------------------------------------------------------

/**
 * @brief normAngle
 * @details Normalize an angle to be proper within the selected range like
 * [0, 2pi] or [-pi, pi]. Useful for angle arithmetic to handle the fact that
 * angles exists on a manifold.
 *
 * @param theta angle
 * @param start start angle of the expected range
 *
 * @return normalized angle
 */
inline double normAngle(double theta, const double& start = 0)
{
    constexpr double inf = std::numeric_limits<double>::infinity();
    if (theta < inf) {
        while (theta >= (start + 2 * M_PI))
            theta -= 2 * M_PI;
        while (theta < start)
            theta += 2 * M_PI;
        return theta;
    }
    else {
        return inf;
    }
}

/// ------------------------------------------------------------------------

/**
 * @brief angleBetween
 * @details Angle between two 2D vectors
 *
 * @param p1 start of first vector
 * @param p2 end of first vector
 * @param p3 start of second vector
 * @param p4 end of second vector
 * @return angle
 */
template <typename PointType>
inline double angleBetween(const PointType& p1, const PointType& p2,
    const PointType& p3, const PointType& p4,
    const bool normalize = false)
{
    double heading1 = normAngle(atan2(p2[1] - p1[1], p2[0] - p1[0]));
    double heading2 = normAngle(atan2(p4[1] - p3[1], p4[0] - p3[0]));
    if (normalize)
        return normAngle(heading1 - heading2);
    else
        return (heading1 - heading2);
}


template <typename VecType>
inline double angleBetween(const VecType& v1, const VecType& v2, const bool normalize=false)
{
    double heading1 = normAngle(atan2(v1[1], v1[0]));
    double heading2 = normAngle(atan2(v2[1], v2[0]));

    if (normalize)
        return normAngle(heading1 - heading2);
    else
        return (heading1 - heading2);
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

/**
 * @brief generateCombinations
 * @details Generate combinations of k elements from a a range on n, i.e. C_k^n
 *
 * @param n Length of the range to choose from
 * @param k Size of combination tuples
 *
 * @return Combinations in an vector of vectors
 */
std::vector<std::vector<int> > generateCombinations(const int& n, const int& k = 2)
{
    std::vector<std::vector<int> > combinations;

    std::vector<int> selected;
    std::vector<int> selector(n);
    std::fill(selector.begin(), selector.begin() + k, 1);

    do {
        for (int i = 0; i < n; i++) {
            if (selector[i])
                selected.push_back(i);
        }
        combinations.push_back(selected);
        selected.clear();
    } while (std::prev_permutation(selector.begin(), selector.end()));

    return combinations;
}

/// ------------------------------------------------------------------------

/**
 * @brief Inner product of two vectors
 */
double vdot(const std::vector<double>& v1, const std::vector<double>& v2)
{
    assert(v1.size() == v2.size());
    double value = 0.0;
    for (unsigned int i = 0; i < v1.size(); i++)
        value += v1[i] * v2[i];
    return value;
}

/// -----------------------------------------------------------
/// \brief Compute the social influece of an action point on
/// the pedestrians using the social force model
/// -----------------------------------------------------------
// inline double anisotropicDistance(const Person& person, const Tpoint& wp,
//     const double& ak = 2.48, const double& bk = 1.0,
//     const double& lambda = 0.35, const double& rij = 0.9)
inline double anisotropicDistance(const Person& person, const Tpoint& wp,
    const double& ak = 2.48, const double& bk = 1.0,
    const double& lambda = 0.4, const double& rij = 0.9)
{
    point_t ei = point_t{ -person.vx, -person.vy };
    // point_t ei = point_t{ person.vx, person.vy };
    double len_ei = std::hypot(ei[0], ei[1]);
    ei.x /= len_ei;
    ei.y /= len_ei;

    double phi = atan2(wp[1] - person[1], wp[0] - person[0]);
    point_t nij = point_t{ cos(phi), sin(phi) };

    double dij = edist(person, wp);
    point_t alpha = point_t{ ak * exp((rij - dij) / bk) * nij[0],
        ak * exp((rij - dij) / bk) * nij[1] };
    double cosphi = (-nij[0] * ei[0]) + (-nij[1] * ei[1]);
    double beta = (lambda + (1 - lambda)) * ((1.0 + cosphi) / 2.0);

    point_t ap = point_t{ alpha[0] * beta, alpha[1] * beta };
    double dc = std::hypot(ap[0], ap[1]);
    return dc;


    // point_t ei = { person.vx, person.vy };
    // point_t nij = {wp.x - person.x, wp.y - person.y};
    // double len_nij = std::hypot(ei[0], ei[1]);
    // nij.x /= len_nij;
    // nij.y /= len_nij;

    // double dij = edist(person, wp);
    // double cos_phi = (-nij.x * ei.x) + (-nij.y * ei.y);
    // double alpha = ak * exp((rij - dij) / bk);
    // double beta = (lambda + (1 - lambda) * ((1.0 + cos_phi) / 2.0));
    // double social_force = (alpha * nij.x * beta) + (alpha * nij.y * beta);
    // return social_force;
}

/// ------------------------------------------------------------------------

template <typename PersonType, typename GoalType>
inline double goalOrientation(const PersonType& person, const GoalType& goal)
{
    auto speed = std::hypot(person.vx, person.vy);
    point_t pnext = point_t{ person.x + speed * person.vx,
        person.y + speed * person.vy };
    double dnow = edist(goal, person);
    double dnext = edist(goal, pnext);

    return (dnext - dnow);
}

/// ------------------------------------------------------------------------

// Check if a point on the 'front' side of the robot based on a robot pose,
// and heading.
inline bool inFrontOfRobot(const double& rx, const double& ry,
                           const double& vrx, const double& vry,
                           const double& px, const double& py)
{
    // detemine robot baseline
    double rtheta = normAngle(atan2(vry, vrx));
    // point_t rline_s = { rx+5.0*cos(rtheta - M_PI_2), ry+5.0*sin(rtheta - M_PI_2) };
    // point_t rline_s = { rx, ry };
    point_t rline_e = { rx+5.0*cos(rtheta + M_PI), ry+5.0*sin(rtheta + M_PI) };

    // determine which side the robot of the line(baseline) the robot
    // is 'heading' and detemine the side
    // point_t head_point = { rx+5.0*cos(rtheta), ry+5.0*sin(rtheta) };
    point_t C = { rx, ry };
    point_t B = rline_e;
    point_t P = { px, py };

    double dotp = ((P.x - C.x) * (B.x - C.x)) + ((P.y - C.y) * (B.y - C.y));
    if (dotp > 0.0)
        return false;
    else
        return true;
}

/// ------------------------------------------------------------------------


} // end namespace

#endif
