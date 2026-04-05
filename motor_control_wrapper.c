/**
 * @file motor_control_wrapper.c
 * @brief 모터 제어 변환 Wrapper 구현
 */

#include "motor_control_wrapper.h"

DQ abc_to_dq(double a, double b, double c, double theta)
{
    AlphaBeta ab = clark_transform(a, b, c);
    return park_transform(ab.alpha, ab.beta, theta);
}

PhaseABC dq_to_abc(double d, double q, double theta)
{
    AlphaBeta ab = inverse_park_transform(d, q, theta);
    return inverse_clark_transform(ab.alpha, ab.beta);
}
