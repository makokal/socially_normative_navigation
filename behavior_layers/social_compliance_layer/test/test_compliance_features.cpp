/**
* Copyright 2014-2015 Social Robotics Lab, University of Freiburg
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


#include <gtest/gtest.h>
#include <social_compliance_layer/utils.h>
#include <social_compliance_layer/social_compliance_layer.h>
#include <cmath>


using namespace social_compliance_layer;

/**
 * @brief Test correctness of the euclidean distance function
 * @details Check if our method robustly computes euclidean distances
 */
TEST(edist, Correctness)
{
    const point_t p1 = {0.0, 0.0};
    const point_t p2 = {3.0, 0.0};
    const point_t p3 = {-3.0, 0.0};
    ASSERT_EQ(edist(p1, p2), 3.0);
    ASSERT_EQ(edist(p1, p1), 0.0);
    ASSERT_EQ(edist(p2, p2), 0.0);
    ASSERT_EQ(edist(p1, p3), 3.0);
}


TEST(distance2Segment, Correctness)
{
    // test points
    const TrajectoryWaypoint x1 = {2.0, 2.0, 0, 0}; // colinear inside
    const TrajectoryWaypoint x2 = {4.0, 0.0, 0, 0}; // colinear outside
    const TrajectoryWaypoint x3 = {4.0, 1.0, 0, 0}; // outside not colinear
    const TrajectoryWaypoint x4 = {0.0, -1.0, 0, 0}; // inside not colinear
    const TrajectoryWaypoint x5 = {1.0, 2.0, 0, 0}; // inside not colinear
    const TrajectoryWaypoint x6 = {2.7, 2.7, 0, 0}; // inside not colinear

    // line
    const point_t ls = {1.0, 3.0};
    const point_t le = {3.0, 1.0};
    ASSERT_EQ(distance2Segment(x1, ls, le).second, true);
    ASSERT_EQ(distance2Segment(x1, ls, le).first, 0.0);

    ASSERT_EQ(distance2Segment(x2, ls, le).second, false);

    ASSERT_EQ(distance2Segment(x3, ls, le).second, false);

    ASSERT_EQ(distance2Segment(x6, ls, le).second, true);
    ASSERT_EQ(distance2Segment(x4, ls, le).second, true);
    ASSERT_EQ(distance2Segment(x5, ls, le).second, true);
}

TEST(normAngle, Correctness)
{
    const double theta1 = M_PI;
    const double theta2 = -M_PI;
    const double theta3 = 2*M_PI;
    const double theta4 = 3*M_PI;
    const double theta5 = -5*M_PI;
    ASSERT_EQ(normAngle(theta1), M_PI);
    ASSERT_EQ(normAngle(theta2), M_PI);
    ASSERT_EQ(normAngle(theta3), 0);
    ASSERT_EQ(normAngle(theta4), M_PI);
    ASSERT_EQ(normAngle(theta5), M_PI);
}

TEST(angleBetween, Correctness)
{
    // simple case
    const point_t p1 = {0.0, 0.0};
    const point_t p2 = {4.0, 0.0};

    const point_t p3 = {1.0, 4.0};
    const point_t p4 = {1.0, 0.0};

    ASSERT_EQ(angleBetween(p1, p2, p3, p4), M_PI/2);

    // more complex cases
    const point_t p5 = {0.0, 0.0};
    const point_t p6 = {-3.0, 0.0};

    ASSERT_EQ(angleBetween(p1, p2, p5, p6), M_PI);
}

TEST(mapRange, Correctness)
{
    ASSERT_EQ(mapRange(2, 0, 10, 0, 10), 2.0);
    ASSERT_EQ(mapRange(2, 0, 10, 0, 100), 20.0);
    ASSERT_EQ(mapRange(12, 10, 20, 0, 10), 2.0);
    ASSERT_EQ(mapRange(2, 0, 10, -5, 5), -3.0);
    ASSERT_EQ(mapRange(30, 0, 180, 0, 360), 60.0);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "comliance_features_tests");
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
