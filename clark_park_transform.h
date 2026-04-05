/**
 * @file clark_park_transform.h
 * @brief Clark 변환 및 Park 변환 핵심 모듈
 *
 * 모터 제어(FOC: Field-Oriented Control)에서 사용되는
 * Clark 변환과 Park 변환의 정변환 및 역변환을 제공한다.
 *
 * - Clark 변환: 3상(a, b, c) → 정지 좌표계(α, β)
 * - Park 변환: 정지 좌표계(α, β) → 회전 좌표계(d, q)
 */

#ifndef CLARK_PARK_TRANSFORM_H
#define CLARK_PARK_TRANSFORM_H

/** αβ 정지 좌표계 성분 */
typedef struct {
    double alpha;
    double beta;
} AlphaBeta;

/** dq 회전 좌표계 성분 */
typedef struct {
    double d;
    double q;
} DQ;

/** 3상(a, b, c) 성분 */
typedef struct {
    double a;
    double b;
    double c;
} PhaseABC;

/**
 * Clark 변환 (3상 → αβ 정지 좌표계)
 * 진폭 불변(amplitude-invariant) 방식
 *
 * @param a  a상 전류 또는 전압
 * @param b  b상 전류 또는 전압
 * @param c  c상 전류 또는 전압
 * @return   αβ 정지 좌표계 성분
 */
AlphaBeta clark_transform(double a, double b, double c);

/**
 * 역 Clark 변환 (αβ 정지 좌표계 → 3상)
 *
 * @param alpha  정지 좌표계 α 성분
 * @param beta   정지 좌표계 β 성분
 * @return       3상 전류 또는 전압
 */
PhaseABC inverse_clark_transform(double alpha, double beta);

/**
 * Park 변환 (αβ 정지 좌표계 → dq 회전 좌표계)
 *
 * @param alpha  정지 좌표계 α 성분
 * @param beta   정지 좌표계 β 성분
 * @param theta  회전자 전기각 (라디안 단위)
 * @return       dq 회전 좌표계 성분
 */
DQ park_transform(double alpha, double beta, double theta);

/**
 * 역 Park 변환 (dq 회전 좌표계 → αβ 정지 좌표계)
 *
 * @param d      회전 좌표계 d 성분
 * @param q      회전 좌표계 q 성분
 * @param theta  회전자 전기각 (라디안 단위)
 * @return       αβ 정지 좌표계 성분
 */
AlphaBeta inverse_park_transform(double d, double q, double theta);

#endif /* CLARK_PARK_TRANSFORM_H */
