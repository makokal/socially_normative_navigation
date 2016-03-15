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

#ifndef NORMATIVE_BEHAVIOR_HELPERS_H
#define NORMATIVE_BEHAVIOR_HELPERS_H

#include <vector>
#include <cmath>

#include <behavior_utils/entities.h>


const double ROOT_2PI = 2.50662827463;


namespace behavior_utils
{

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
    assert(mint < maxt);

    double denom = maxa - mina;
    if (fabs(denom) < std::numeric_limits<float>::min())
        denom = std::numeric_limits<float>::min();

    return mint + ((value - mina) * (maxt - mint) / denom);
}


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


/**
 * @brief Inner product of two vectors
 *
 * @param v1 vector 1
 * @param v2 vector 2
 *
 * @return dot product
 */
inline double vdot(const std::vector<double>& v1, const std::vector<double>& v2)
{
    assert(v1.size() == v2.size());
    double value = 0.0;
    for (unsigned int i = 0; i < v1.size(); i++)
        value += v1[i] * v2[i];
    return value;
}


/**
 * @brief sigmoid function
 * @details Compute the sigmoid function
 *
 * @param value input response
 * @return response after sigmoid transformation
 */
inline double sigmoid(const double& value)
{
    return 1.0 / (1.0 + exp(-value));
}


inline double costDecay(const double& value, const double horizon, const double& dt=1.0)
{
    // return init_value * exp(-value * dt);
    double h2 = horizon * 2.0;
    return ((h2 - dt) / h2) * value;
    // return value * (1.0 / dt);
}

inline double gaussianPdf(const double& x, const double& mu = 0.0, const double& sigma = 1.0)
{
    const double a = 1.0 / (sigma * ROOT_2PI);
    return a * exp(-((x - mu) * (x - mu)) / (2.0 * sigma * sigma));
}

inline double laplacePdf(const double& x, const double& mu = 0.0, const double& b = 1.0)
{
    const double a = 1.0 / (2.0 * b);
    return a * exp(-(fabs(x - mu)) / b);
}


} // end of namespace


#endif // HELPERS_H
