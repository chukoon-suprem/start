/**
 * @file clark_park_transform.c
 * @brief Clark 변환 및 Park 변환 구현
 */

#include "clark_park_transform.h"
#include <math.h>

/* sqrt(3) / 2 상수 (반복 계산 방지) */
#define SQRT3_OVER_2  0.86602540378443864676

AlphaBeta clark_transform(double a, double b, double c)
{
    /*
     * 진폭 불변 Clark 변환 행렬:
     *   α = (2/3) * (a - b/2 - c/2)
     *   β = (2/3) * (√3/2) * (b - c)
     */
    AlphaBeta result;
    result.alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c);
    result.beta  = (2.0 / 3.0) * SQRT3_OVER_2 * (b - c);
    return result;
}

PhaseABC inverse_clark_transform(double alpha, double beta)
{
    /*
     * 역 Clark 변환:
     *   a =  alpha
     *   b = -alpha/2 + (√3/2) * beta
     *   c = -alpha/2 - (√3/2) * beta
     */
    PhaseABC result;
    result.a =  alpha;
    result.b = -0.5 * alpha + SQRT3_OVER_2 * beta;
    result.c = -0.5 * alpha - SQRT3_OVER_2 * beta;
    return result;
}

DQ park_transform(double alpha, double beta, double theta)
{
    /*
     * Park 변환 행렬:
     *   d =  α·cos(θ) + β·sin(θ)
     *   q = -α·sin(θ) + β·cos(θ)
     */
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    DQ result;
    result.d =  alpha * cos_theta + beta * sin_theta;
    result.q = -alpha * sin_theta + beta * cos_theta;
    return result;
}

AlphaBeta inverse_park_transform(double d, double q, double theta)
{
    /*
     * 역 Park 변환:
     *   α = d·cos(θ) - q·sin(θ)
     *   β = d·sin(θ) + q·cos(θ)
     */
    double cos_theta = cos(theta);
    double sin_theta = sin(theta);

    AlphaBeta result;
    result.alpha = d * cos_theta - q * sin_theta;
    result.beta  = d * sin_theta + q * cos_theta;
    return result;
}
