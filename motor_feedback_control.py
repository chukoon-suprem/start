"""
모터 이중 루프 피드백 제어 (Cascade PID Control)
- 외부 루프: 속도 제어 (Speed Loop)
- 내부 루프: 전류 제어 (Current Loop)

구조: 속도 설정값 → [속도 PID] → 전류 설정값 → [전류 PID] → 전압 → 모터 모델
"""

import time
import math


class PID:
    """기본 PID 제어기"""

    def __init__(self, kp: float, ki: float, kd: float,
                 output_min: float = -float('inf'),
                 output_max: float = float('inf')):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min = output_min
        self.output_max = output_max

        self._integral = 0.0
        self._prev_error = 0.0

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0

    def update(self, setpoint: float, measured: float, dt: float) -> float:
        error = setpoint - measured

        self._integral += error * dt
        derivative = (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        output = self.kp * error + self.ki * self._integral + self.kd * derivative

        # Anti-windup: 출력 클램핑 시 적분항 보정
        clamped = max(self.output_min, min(self.output_max, output))
        if output != clamped:
            self._integral -= error * dt  # 포화 시 적분 취소

        return clamped


class MotorModel:
    """
    간단한 DC 모터 모델
    - 전기 모델: L * di/dt = V - R*i - Ke*w
    - 기계 모델: J * dw/dt = Kt*i - B*w - T_load
    """

    def __init__(self):
        # 전기 파라미터
        self.R = 1.0       # 저항 [Ω]
        self.L = 0.005     # 인덕턴스 [H]
        self.Ke = 0.1      # 역기전력 상수 [V·s/rad]

        # 기계 파라미터
        self.Kt = 0.1      # 토크 상수 [N·m/A]
        self.J = 0.01      # 관성 모멘트 [kg·m²]
        self.B = 0.001     # 점성 마찰 계수 [N·m·s/rad]

        # 상태 변수
        self.current = 0.0    # 전류 [A]
        self.speed = 0.0      # 각속도 [rad/s]
        self.load_torque = 0.0

    def step(self, voltage: float, dt: float):
        # 전기 방정식: di/dt = (V - R*i - Ke*w) / L
        di_dt = (voltage - self.R * self.current - self.Ke * self.speed) / self.L
        self.current += di_dt * dt

        # 기계 방정식: dw/dt = (Kt*i - B*w - T_load) / J
        dw_dt = (self.Kt * self.current - self.B * self.speed - self.load_torque) / self.J
        self.speed += dw_dt * dt

    def apply_load(self, torque: float):
        self.load_torque = torque


class CascadeMotorController:
    """
    캐스케이드(이중 루프) 모터 제어기
    - 내부 루프(전류): 더 빠른 샘플링 주기
    - 외부 루프(속도): 더 느린 샘플링 주기
    """

    def __init__(self):
        # 전류 루프 PID (빠른 응답, 높은 대역폭)
        self.current_pid = PID(
            kp=10.0, ki=100.0, kd=0.01,
            output_min=-24.0, output_max=24.0   # 전압 한계 [V]
        )

        # 속도 루프 PID (전류 루프보다 느린 응답)
        self.speed_pid = PID(
            kp=0.5, ki=2.0, kd=0.005,
            output_min=-10.0, output_max=10.0   # 전류 지령 한계 [A]
        )

        self.motor = MotorModel()

    def run_simulation(self, target_speed: float, duration: float,
                       speed_dt: float = 0.01, current_dt: float = 0.001):
        """
        시뮬레이션 실행

        Args:
            target_speed: 목표 속도 [rad/s]
            duration: 시뮬레이션 시간 [s]
            speed_dt: 속도 루프 샘플링 주기 [s]
            current_dt: 전류 루프 샘플링 주기 [s]
        """
        print(f"{'Time':>8} | {'Target Spd':>12} | {'Motor Spd':>12} | {'Target Cur':>12} | {'Motor Cur':>12} | {'Voltage':>10}")
        print("-" * 80)

        t = 0.0
        speed_timer = 0.0
        target_current = 0.0

        while t < duration:
            # --- 외부 루프: 속도 제어 ---
            if speed_timer >= speed_dt:
                target_current = self.speed_pid.update(
                    setpoint=target_speed,
                    measured=self.motor.speed,
                    dt=speed_dt
                )
                speed_timer = 0.0

            # --- 내부 루프: 전류 제어 ---
            voltage = self.current_pid.update(
                setpoint=target_current,
                measured=self.motor.current,
                dt=current_dt
            )

            # 모터 모델 업데이트
            self.motor.step(voltage, current_dt)

            # 결과 출력 (0.1초 간격)
            if abs(t % 0.1) < current_dt / 2:
                print(f"{t:>8.2f} | {target_speed:>12.3f} | {self.motor.speed:>12.3f} | "
                      f"{target_current:>12.3f} | {self.motor.current:>12.3f} | {voltage:>10.3f}")

            t += current_dt
            speed_timer += current_dt

        print(f"\n최종 속도: {self.motor.speed:.3f} rad/s (목표: {target_speed:.3f} rad/s)")
        print(f"최종 전류: {self.motor.current:.3f} A")
        steady_state_error = abs(target_speed - self.motor.speed)
        print(f"정상상태 오차: {steady_state_error:.4f} rad/s ({steady_state_error/target_speed*100:.2f}%)")


def main():
    print("=" * 80)
    print("  DC 모터 이중 루프 피드백 제어 시뮬레이션")
    print("  Cascade PID: 속도 루프(외부) + 전류 루프(내부)")
    print("=" * 80)

    controller = CascadeMotorController()

    # 시나리오 1: 기본 속도 추종
    print("\n[시나리오 1] 목표 속도 100 rad/s 추종 (2초)")
    controller.run_simulation(target_speed=100.0, duration=2.0)

    # 시나리오 2: 부하 변동 대응
    print("\n[시나리오 2] 목표 속도 100 rad/s, 1초 후 부하 토크 0.5 N·m 인가")
    controller2 = CascadeMotorController()

    duration = 3.0
    speed_dt = 0.01
    current_dt = 0.001
    target_speed = 100.0
    t = 0.0
    speed_timer = 0.0
    target_current = 0.0

    print(f"{'Time':>8} | {'Target Spd':>12} | {'Motor Spd':>12} | {'Load Torque':>12}")
    print("-" * 60)

    while t < duration:
        # 1초 후 부하 인가
        if t >= 1.0:
            controller2.motor.apply_load(0.5)

        if speed_timer >= speed_dt:
            target_current = controller2.speed_pid.update(
                setpoint=target_speed,
                measured=controller2.motor.speed,
                dt=speed_dt
            )
            speed_timer = 0.0

        voltage = controller2.current_pid.update(
            setpoint=target_current,
            measured=controller2.motor.current,
            dt=current_dt
        )
        controller2.motor.step(voltage, current_dt)

        if abs(t % 0.2) < current_dt / 2:
            print(f"{t:>8.2f} | {target_speed:>12.3f} | {controller2.motor.speed:>12.3f} | "
                  f"{controller2.motor.load_torque:>12.3f}")

        t += current_dt
        speed_timer += current_dt

    print(f"\n최종 속도: {controller2.motor.speed:.3f} rad/s (목표: {target_speed:.3f} rad/s)")


if __name__ == "__main__":
    main()
