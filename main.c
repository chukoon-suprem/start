/**
 * @file main.c
 * @brief Clark & Park 변환 데모 프로그램
 */

#include <stdio.h>
#include <math.h>
#include "motor_control_wrapper.h"

#define DEG_TO_RAD(deg)  ((deg) * M_PI / 180.0)
#define TOLERANCE        1e-10

int main(void)
{
    printf("==================================================\n");
    printf("  Clark & Park 변환 데모 (C 언어)\n");
    printf("==================================================\n");

    /* 균형 3상 신호 생성 (120도 위상차) */
    double theta_signal = DEG_TO_RAD(30.0);  /* 신호 위상각 30도 */
    double amplitude = 10.0;

    double ia = amplitude * cos(theta_signal);
    double ib = amplitude * cos(theta_signal - 2.0 * M_PI / 3.0);
    double ic = amplitude * cos(theta_signal + 2.0 * M_PI / 3.0);

    printf("\n[입력] 3상 전류 (진폭=%.1f, 위상각=30도)\n", amplitude);
    printf("  Ia = %.4f\n", ia);
    printf("  Ib = %.4f\n", ib);
    printf("  Ic = %.4f\n", ic);
    printf("  합계 = %.6f (균형 3상이면 0에 근접)\n", ia + ib + ic);

    /* Clark 변환 */
    AlphaBeta ab = clark_transform(ia, ib, ic);
    printf("\n[Clark 변환] 3상 -> ab 정지 좌표계\n");
    printf("  alpha = %.4f\n", ab.alpha);
    printf("  beta  = %.4f\n", ab.beta);

    /* Park 변환 */
    double theta_rotor = theta_signal;
    DQ dq = park_transform(ab.alpha, ab.beta, theta_rotor);
    printf("\n[Park 변환] ab -> dq 회전 좌표계 (theta=30도)\n");
    printf("  d = %.4f\n", dq.d);
    printf("  q = %.4f\n", dq.q);

    /* Wrapper 연쇄 변환 (abc -> dq) */
    DQ dq2 = abc_to_dq(ia, ib, ic, theta_rotor);
    printf("\n[Wrapper] abc_to_dq 연쇄 변환 결과\n");
    printf("  d = %.4f\n", dq2.d);
    printf("  q = %.4f\n", dq2.q);

    /* 역변환 (dq -> abc) */
    PhaseABC restored = dq_to_abc(dq.d, dq.q, theta_rotor);
    printf("\n[역변환] dq -> abc 복원 결과\n");
    printf("  Ia' = %.4f\n", restored.a);
    printf("  Ib' = %.4f\n", restored.b);
    printf("  Ic' = %.4f\n", restored.c);

    /* 왕복 변환 정확도 검증 */
    printf("\n[검증] 왕복 변환 정확도\n");
    int a_ok = fabs(ia - restored.a) < TOLERANCE;
    int b_ok = fabs(ib - restored.b) < TOLERANCE;
    int c_ok = fabs(ic - restored.c) < TOLERANCE;

    printf("  Ia 일치: %s (오차: %.2e)\n", a_ok ? "통과" : "실패", fabs(ia - restored.a));
    printf("  Ib 일치: %s (오차: %.2e)\n", b_ok ? "통과" : "실패", fabs(ib - restored.b));
    printf("  Ic 일치: %s (오차: %.2e)\n", c_ok ? "통과" : "실패", fabs(ic - restored.c));

    if (a_ok && b_ok && c_ok)
        printf("\n  -> 모든 왕복 변환 검증 통과!\n");
    else
        printf("\n  -> 일부 검증 실패. 확인이 필요합니다.\n");

    printf("\n==================================================\n");
    return 0;
}
