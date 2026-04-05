/**
 * @file motor_control_wrapper.h
 * @brief 모터 제어 변환 Wrapper 모듈
 *
 * Clark/Park 변환을 조합한 편의 함수를 제공하여
 * 외부 모듈에서 쉽게 접근할 수 있도록 한다.
 *
 * 사용 예시:
 *   #include "motor_control_wrapper.h"
 *
 *   DQ result = abc_to_dq(ia, ib, ic, theta);
 *   PhaseABC phase = dq_to_abc(vd, vq, theta);
 */

#ifndef MOTOR_CONTROL_WRAPPER_H
#define MOTOR_CONTROL_WRAPPER_H

#include "clark_park_transform.h"

/**
 * 3상(a, b, c) → dq 회전 좌표계 변환 (Clark + Park 연쇄 변환)
 *
 * @param a      a상 전류 또는 전압
 * @param b      b상 전류 또는 전압
 * @param c      c상 전류 또는 전압
 * @param theta  회전자 전기각 (라디안 단위)
 * @return       dq 회전 좌표계 성분
 */
DQ abc_to_dq(double a, double b, double c, double theta);

/**
 * dq 회전 좌표계 → 3상(a, b, c) 변환 (역Park + 역Clark 연쇄 변환)
 *
 * @param d      회전 좌표계 d 성분
 * @param q      회전 좌표계 q 성분
 * @param theta  회전자 전기각 (라디안 단위)
 * @return       3상 전류 또는 전압
 */
PhaseABC dq_to_abc(double d, double q, double theta);

#endif /* MOTOR_CONTROL_WRAPPER_H */
