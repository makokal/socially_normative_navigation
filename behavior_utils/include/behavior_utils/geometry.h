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

#ifndef NORMATIVE_BEHAVIOR_GEOMETRY_H
#define NORMATIVE_BEHAVIOR_GEOMETRY_H

#include <cmath>
#include <iterator>
#include <vector>
#include <limits>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_datatypes.h>

#include <behavior_utils/entities.h>

namespace behavior_utils {

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
inline std::pair<double, bool> distance2Segment(const WaypointType& x,
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
inline double angleBetween(const PointType& p1,
    const PointType& p2,
    const PointType& p3,
    const PointType& p4,
    const bool normalize = false)
{
    double heading1 = normAngle(atan2(p2[1] - p1[1], p2[0] - p1[0]));
    double heading2 = normAngle(atan2(p4[1] - p3[1], p4[0] - p3[0]));
    if (normalize)
        return normAngle(heading1 - heading2);
    else
        return (heading1 - heading2);
}

template <typename Vector>
inline double angleBetween(const Vector& v1,
    const Vector& v2,
    const bool normalize = false)
{
    double heading1 = normAngle(atan2(v1[1], v1[0]));
    double heading2 = normAngle(atan2(v2[1], v2[0]));

    if (normalize)
        return normAngle(heading1 - heading2);
    else
        return (heading1 - heading2);
}

/**
 * @brief anisotropicDistacne
 * @details Anisotropic distance of a point from a person
 *
 * @param person Pose of a person including orientation vector
 * @param wp Point of interest
 * @param ak Anisotropic model parameter
 * @param bk Anisotropic model parameter
 * @param lambda Model 'circleness' parameter
 * @param rij Total radii of person and point (model parameter)
 * @return distance
 */
inline double anisotropicDistance(const Person& person,
    const Tpoint& wp,
    const double& ak = 2.48,
    const double& bk = 1.0,
    const double& lambda = 0.4,
    const double& rij = 0.9)
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
}

/**
 * @brief Goal orientation
 * @details Get the goal orientation of an agent relative to the robot, i.e is
 * the agent heading towards the robot's goal.
 *
 * @note This is a crude heuristic that does not cover all the cases, esp when
 * the goal is in some curvature.
 *
 * @param person Pose of the person of interest including orientation vector
 * @param goal Goal position that the robot is heading towards
 * @return goal orientedness of the agent
 */
template <typename PersonType, typename GoalType>
inline double goalOrientation(const PersonType& person, const GoalType& goal)
{
    double speed = std::hypot(person.vx, person.vy);
    point_t pnext = point_t{ person.x + speed * person.vx,
        person.y + speed * person.vy };
    double dnow = edist(goal, person);
    double dnext = edist(goal, pnext);

    return (dnext - dnow);
}

/**
 * @brief Check if a point is infront of the robot
 * @details Find out whether a point in 2D is in front of the robot or not
 *
 * @param robot Robot pose
 * @param point 2D location
 * @return bool
 */
template <typename PointType>
inline bool inFrontOfRobot(const Person& robot, const PointType& point)
{
    // detemine robot baseline
    double rtheta = normAngle(atan2(robot.vy, robot.vx));
    // point_t rline_e = { robot.x+5.0*cos(rtheta + M_PI), robot.y+5.0*sin(rtheta + M_PI) };

    double lx = robot.x+5.0*cos(rtheta + M_PI);
    double ly = robot.y+5.0*sin(rtheta + M_PI);

    // determine which side the robot of the line(baseline) the robot
    // is 'heading' and detemine the side
    // point_t C = { robot.x, robot.y };
    // point_t B = rline_e;
    // point_t P = { px, py };

    double dotp = ((point.x - robot.x) * (lx - robot.x)) + ((point.y - robot.y) * (ly - robot.y));
    if (dotp > 0.0)
        return false;
    else
        return true;
}


inline double relativeAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const double& angle)
{
    double actual_angle = acos((fabs(p1.x - p2.x)) / edist(point_t(p1.x, p1.y), point_t(p2.x, p2.y)));

    // Trigonometric adjustment
    if (p1.x < p2.x)
        actual_angle = 3.1416 - actual_angle;

    if (p2.y < p1.y)
        actual_angle = -actual_angle;

    return angle - actual_angle;
}

inline double isInAngle(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2, const double& angle,
    double angle_threshold)
{
    double angle_result = relativeAngle(p1, p2, angle);

    if (fabs(angle_result) > angle_threshold) {
        return 0.0;
    }
    else {
        return (angle_threshold - fabs(angle_result)) / angle_threshold;
    }
}

inline bool isFacing(const geometry_msgs::Pose& p1, const geometry_msgs::Point& p2)
{
    tf::Quaternion quaternion;
    tf::quaternionMsgToTF(p1.orientation, quaternion);
    double yaw = tf::getYaw(quaternion);
    return isInAngle(p1.position, p2, yaw, 0.5);
}


} // end of namespace

#endif // GEOMETRY_H
