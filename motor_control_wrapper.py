"""
모터 제어 변환 Wrapper 모듈

Clark/Park 변환을 조합한 편의 함수를 제공하여
외부 모듈에서 쉽게 접근할 수 있도록 한다.

사용 예시:
    from motor_control_wrapper import abc_to_dq, dq_to_abc

    # 3상 → dq 회전 좌표계
    d, q = abc_to_dq(ia, ib, ic, theta)

    # dq 회전 좌표계 → 3상
    a, b, c = dq_to_abc(vd, vq, theta)

    # 개별 변환도 사용 가능
    from motor_control_wrapper import clark_transform, park_transform
"""

import math

# 개별 변환 함수를 re-export하여 단계별 접근도 가능하게 한다
from clark_park_transform import (
    clark_transform,
    inverse_clark_transform,
    park_transform,
    inverse_park_transform,
)


def abc_to_dq(a: float, b: float, c: float, theta: float) -> tuple[float, float]:
    """
    3상(a, b, c) → dq 회전 좌표계 변환 (Clark + Park 연쇄 변환)

    3상 전류/전압을 한 번에 dq 회전 좌표계로 변환한다.

    Args:
        a: a상 전류 또는 전압
        b: b상 전류 또는 전압
        c: c상 전류 또는 전압
        theta: 회전자 전기각 (라디안 단위)

    Returns:
        (d, q): 회전 좌표계의 d, q 성분
    """
    alpha, beta = clark_transform(a, b, c)
    d, q = park_transform(alpha, beta, theta)
    return d, q


def dq_to_abc(d: float, q: float, theta: float) -> tuple[float, float, float]:
    """
    dq 회전 좌표계 → 3상(a, b, c) 변환 (역Park + 역Clark 연쇄 변환)

    dq 회전 좌표계 값을 한 번에 3상으로 변환한다.

    Args:
        d: 회전 좌표계 d 성분
        q: 회전 좌표계 q 성분
        theta: 회전자 전기각 (라디안 단위)

    Returns:
        (a, b, c): 3상 전류 또는 전압
    """
    alpha, beta = inverse_park_transform(d, q, theta)
    a, b, c = inverse_clark_transform(alpha, beta)
    return a, b, c


if __name__ == "__main__":
    print("=" * 50)
    print("  Clark & Park 변환 데모")
    print("=" * 50)

    # 균형 3상 신호 생성 (120도 위상차)
    theta_signal = math.radians(30)  # 신호 위상각 30도
    amplitude = 10.0  # 진폭

    ia = amplitude * math.cos(theta_signal)
    ib = amplitude * math.cos(theta_signal - 2 * math.pi / 3)
    ic = amplitude * math.cos(theta_signal + 2 * math.pi / 3)

    print(f"\n[입력] 3상 전류 (진폭={amplitude}, 위상각={math.degrees(theta_signal)}°)")
    print(f"  Ia = {ia:.4f}")
    print(f"  Ib = {ib:.4f}")
    print(f"  Ic = {ic:.4f}")
    print(f"  합계 = {ia + ib + ic:.6f} (균형 3상이면 0에 근접)")

    # Clark 변환
    alpha, beta = clark_transform(ia, ib, ic)
    print(f"\n[Clark 변환] 3상 → αβ 정지 좌표계")
    print(f"  α = {alpha:.4f}")
    print(f"  β = {beta:.4f}")

    # Park 변환 (회전자 각도 = 신호 위상각과 동일하게 설정)
    theta_rotor = theta_signal
    d, q = park_transform(alpha, beta, theta_rotor)
    print(f"\n[Park 변환] αβ → dq 회전 좌표계 (θ={math.degrees(theta_rotor)}°)")
    print(f"  d = {d:.4f}")
    print(f"  q = {q:.4f}")

    # 연쇄 변환 (abc → dq 한 번에)
    d2, q2 = abc_to_dq(ia, ib, ic, theta_rotor)
    print(f"\n[Wrapper] abc_to_dq 연쇄 변환 결과")
    print(f"  d = {d2:.4f}")
    print(f"  q = {q2:.4f}")

    # 역변환 (dq → abc)
    a_restored, b_restored, c_restored = dq_to_abc(d, q, theta_rotor)
    print(f"\n[역변환] dq → abc 복원 결과")
    print(f"  Ia' = {a_restored:.4f}")
    print(f"  Ib' = {b_restored:.4f}")
    print(f"  Ic' = {c_restored:.4f}")

    # 왕복 변환 정확도 검증
    print(f"\n[검증] 왕복 변환 정확도")
    tol = 1e-10
    a_ok = math.isclose(ia, a_restored, abs_tol=tol)
    b_ok = math.isclose(ib, b_restored, abs_tol=tol)
    c_ok = math.isclose(ic, c_restored, abs_tol=tol)

    print(f"  Ia 일치: {'✓ 통과' if a_ok else '✗ 실패'} (오차: {abs(ia - a_restored):.2e})")
    print(f"  Ib 일치: {'✓ 통과' if b_ok else '✗ 실패'} (오차: {abs(ib - b_restored):.2e})")
    print(f"  Ic 일치: {'✓ 통과' if c_ok else '✗ 실패'} (오차: {abs(ic - c_restored):.2e})")

    if a_ok and b_ok and c_ok:
        print("\n  → 모든 왕복 변환 검증 통과!")
    else:
        print("\n  → 일부 검증 실패. 확인이 필요합니다.")

    print("\n" + "=" * 50)
