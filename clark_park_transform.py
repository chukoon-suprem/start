"""
Clark 변환 및 Park 변환 모듈

모터 제어(FOC: Field-Oriented Control)에서 사용되는
Clark 변환과 Park 변환의 정변환 및 역변환을 구현한다.

- Clark 변환: 3상(a, b, c) → 정지 좌표계(α, β)
- Park 변환: 정지 좌표계(α, β) → 회전 좌표계(d, q)
"""

import math


def clark_transform(a: float, b: float, c: float) -> tuple[float, float]:
    """
    Clark 변환 (3상 → αβ 정지 좌표계)

    진폭 불변(amplitude-invariant) 방식을 사용한다.
    균형 3상 시스템(a + b + c = 0)을 가정한다.

    변환 행렬:
        [α]   2/3 * [ 1    -1/2    -1/2  ] [a]
        [β] =       [ 0   √3/2   -√3/2  ] [b]
                                            [c]

    Args:
        a: a상 전류 또는 전압
        b: b상 전류 또는 전압
        c: c상 전류 또는 전압

    Returns:
        (alpha, beta): 정지 좌표계의 α, β 성분
    """
    alpha = (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c)
    beta = (2.0 / 3.0) * (math.sqrt(3) / 2.0) * (b - c)
    return alpha, beta


def inverse_clark_transform(alpha: float, beta: float) -> tuple[float, float, float]:
    """
    역 Clark 변환 (αβ 정지 좌표계 → 3상)

    Args:
        alpha: 정지 좌표계 α 성분
        beta: 정지 좌표계 β 성분

    Returns:
        (a, b, c): 3상 전류 또는 전압
    """
    a = alpha
    b = -0.5 * alpha + (math.sqrt(3) / 2.0) * beta
    c = -0.5 * alpha - (math.sqrt(3) / 2.0) * beta
    return a, b, c


def park_transform(alpha: float, beta: float, theta: float) -> tuple[float, float]:
    """
    Park 변환 (αβ 정지 좌표계 → dq 회전 좌표계)

    정지 좌표계의 α, β 성분을 회전자 각도 θ를 기준으로
    회전 좌표계의 d, q 성분으로 변환한다.

    변환 행렬:
        [d]   [ cos(θ)   sin(θ)] [α]
        [q] = [-sin(θ)   cos(θ)] [β]

    Args:
        alpha: 정지 좌표계 α 성분
        beta: 정지 좌표계 β 성분
        theta: 회전자 전기각 (라디안 단위)

    Returns:
        (d, q): 회전 좌표계의 d, q 성분
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    d = alpha * cos_theta + beta * sin_theta
    q = -alpha * sin_theta + beta * cos_theta
    return d, q


def inverse_park_transform(d: float, q: float, theta: float) -> tuple[float, float]:
    """
    역 Park 변환 (dq 회전 좌표계 → αβ 정지 좌표계)

    Args:
        d: 회전 좌표계 d 성분
        q: 회전 좌표계 q 성분
        theta: 회전자 전기각 (라디안 단위)

    Returns:
        (alpha, beta): 정지 좌표계의 α, β 성분
    """
    cos_theta = math.cos(theta)
    sin_theta = math.sin(theta)

    alpha = d * cos_theta - q * sin_theta
    beta = d * sin_theta + q * cos_theta
    return alpha, beta
