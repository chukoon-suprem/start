/**
 * motor_feedback_control.c
 *
 * DC 모터 이중 루프 피드백 제어 (Cascade PID)
 * - 외부 루프: 속도 제어 (Speed Loop)
 * - 내부 루프: 전류 제어 (Current Loop)
 *
 * 구조: 목표속도 → [속도PID] → 목표전류 → [전류PID] → 전압 → 모터
 *
 * 컴파일: gcc -o motor_ctrl motor_feedback_control.c -lm
 */

#include <stdio.h>
#include <math.h>

/* ------------------------------------------------------------------ */
/*  PID 제어기                                                          */
/* ------------------------------------------------------------------ */

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float integral;
    float prev_error;
} PID;

void pid_init(PID *pid, float kp, float ki, float kd,
              float out_min, float out_max)
{
    pid->kp        = kp;
    pid->ki        = ki;
    pid->kd        = kd;
    pid->out_min   = out_min;
    pid->out_max   = out_max;
    pid->integral  = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_reset(PID *pid)
{
    pid->integral   = 0.0f;
    pid->prev_error = 0.0f;
}

float pid_update(PID *pid, float setpoint, float measured, float dt)
{
    float error      = setpoint - measured;
    float derivative = (dt > 0.0f) ? (error - pid->prev_error) / dt : 0.0f;

    pid->integral   += error * dt;
    pid->prev_error  = error;

    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    /* 출력 클램핑 + Anti-windup */
    if (output > pid->out_max) {
        pid->integral -= error * dt;   /* 포화 시 적분 취소 */
        output = pid->out_max;
    } else if (output < pid->out_min) {
        pid->integral -= error * dt;
        output = pid->out_min;
    }

    return output;
}

/* ------------------------------------------------------------------ */
/*  DC 모터 모델                                                        */
/*                                                                      */
/*  전기: L·di/dt = V - R·i - Ke·w                                    */
/*  기계: J·dw/dt = Kt·i - B·w - T_load                               */
/* ------------------------------------------------------------------ */

typedef struct {
    /* 전기 파라미터 */
    float R;    /* 저항       [Ω]        */
    float L;    /* 인덕턴스   [H]        */
    float Ke;   /* 역기전력상수 [V·s/rad] */

    /* 기계 파라미터 */
    float Kt;   /* 토크 상수    [N·m/A]   */
    float J;    /* 관성 모멘트  [kg·m²]   */
    float B;    /* 점성 마찰    [N·m·s/rad] */

    /* 상태 변수 */
    float current;      /* [A]     */
    float speed;        /* [rad/s] */
    float load_torque;  /* [N·m]   */
} Motor;

void motor_init(Motor *m)
{
    m->R   = 1.0f;
    m->L   = 0.005f;
    m->Ke  = 0.1f;
    m->Kt  = 0.1f;
    m->J   = 0.01f;
    m->B   = 0.001f;
    m->current     = 0.0f;
    m->speed       = 0.0f;
    m->load_torque = 0.0f;
}

void motor_step(Motor *m, float voltage, float dt)
{
    /* 오일러 적분 */
    float di_dt = (voltage - m->R * m->current - m->Ke * m->speed) / m->L;
    m->current += di_dt * dt;

    float dw_dt = (m->Kt * m->current - m->B * m->speed - m->load_torque) / m->J;
    m->speed += dw_dt * dt;
}

/* ------------------------------------------------------------------ */
/*  캐스케이드 제어기 상태                                              */
/* ------------------------------------------------------------------ */

typedef struct {
    PID   speed_pid;    /* 외부 루프 */
    PID   current_pid;  /* 내부 루프 */
    Motor motor;
} CascadeCtrl;

void cascade_init(CascadeCtrl *c)
{
    /* 속도 PID: 출력 = 목표전류 [A] */
    pid_init(&c->speed_pid,   0.5f,   2.0f,  0.005f, -10.0f, 10.0f);

    /* 전류 PID: 출력 = 인가전압 [V] */
    pid_init(&c->current_pid, 10.0f, 100.0f, 0.01f,  -24.0f, 24.0f);

    motor_init(&c->motor);
}

/* ------------------------------------------------------------------ */
/*  시나리오 1: 기본 속도 계단 응답                                     */
/* ------------------------------------------------------------------ */

static void scenario_step_response(void)
{
    CascadeCtrl c;
    cascade_init(&c);

    const float target_speed = 100.0f;  /* [rad/s] */
    const float duration     = 2.0f;    /* [s]     */
    const float speed_dt     = 0.01f;   /* 속도 루프 주기 [s] */
    const float current_dt   = 0.001f;  /* 전류 루프 주기 [s] */

    printf("\n[시나리오 1] 목표 속도 %.1f rad/s 계단 응답 (%.1f초)\n",
           target_speed, duration);
    printf("%8s | %12s | %12s | %12s | %12s | %10s\n",
           "Time", "Target Spd", "Motor Spd", "Target Cur", "Motor Cur", "Voltage");
    printf("%.80s\n", "--------------------------------------------------------------------------------");

    float t            = 0.0f;
    float speed_timer  = 0.0f;
    float target_cur   = 0.0f;

    while (t < duration) {
        /* 외부 루프: 속도 → 목표전류 */
        if (speed_timer >= speed_dt) {
            target_cur = pid_update(&c.speed_pid, target_speed, c.motor.speed, speed_dt);
            speed_timer = 0.0f;
        }

        /* 내부 루프: 전류 → 전압 */
        float voltage = pid_update(&c.current_pid, target_cur, c.motor.current, current_dt);

        /* 모터 적분 */
        motor_step(&c.motor, voltage, current_dt);

        /* 0.1초 간격 출력 */
        if (fmodf(t, 0.1f) < current_dt / 2.0f) {
            printf("%8.2f | %12.3f | %12.3f | %12.3f | %12.3f | %10.3f\n",
                   t, target_speed, c.motor.speed,
                   target_cur, c.motor.current, voltage);
        }

        t           += current_dt;
        speed_timer += current_dt;
    }

    float err = fabsf(target_speed - c.motor.speed);
    printf("\n최종 속도 : %.3f rad/s  (목표: %.3f rad/s)\n", c.motor.speed, target_speed);
    printf("최종 전류 : %.3f A\n", c.motor.current);
    printf("정상상태 오차: %.4f rad/s  (%.2f%%)\n", err, err / target_speed * 100.0f);
}

/* ------------------------------------------------------------------ */
/*  시나리오 2: 외란(부하 토크) 인가                                    */
/* ------------------------------------------------------------------ */

static void scenario_load_disturbance(void)
{
    CascadeCtrl c;
    cascade_init(&c);

    const float target_speed  = 100.0f;
    const float load_torque   = 0.5f;   /* 1초 후 인가 [N·m] */
    const float duration      = 3.0f;
    const float speed_dt      = 0.01f;
    const float current_dt    = 0.001f;

    printf("\n[시나리오 2] 목표 속도 %.1f rad/s, 1초 후 부하 %.2f N·m 인가\n",
           target_speed, load_torque);
    printf("%8s | %12s | %12s | %12s\n",
           "Time", "Target Spd", "Motor Spd", "Load Torque");
    printf("%.60s\n", "------------------------------------------------------------");

    float t           = 0.0f;
    float speed_timer = 0.0f;
    float target_cur  = 0.0f;

    while (t < duration) {
        if (t >= 1.0f)
            c.motor.load_torque = load_torque;

        if (speed_timer >= speed_dt) {
            target_cur = pid_update(&c.speed_pid, target_speed, c.motor.speed, speed_dt);
            speed_timer = 0.0f;
        }

        float voltage = pid_update(&c.current_pid, target_cur, c.motor.current, current_dt);
        motor_step(&c.motor, voltage, current_dt);

        if (fmodf(t, 0.2f) < current_dt / 2.0f) {
            printf("%8.2f | %12.3f | %12.3f | %12.3f\n",
                   t, target_speed, c.motor.speed, c.motor.load_torque);
        }

        t           += current_dt;
        speed_timer += current_dt;
    }

    printf("\n최종 속도 : %.3f rad/s  (목표: %.3f rad/s)\n",
           c.motor.speed, target_speed);
}

/* ------------------------------------------------------------------ */
/*  main                                                                */
/* ------------------------------------------------------------------ */

int main(void)
{
    printf("================================================================================\n");
    printf("  DC 모터 이중 루프 피드백 제어 시뮬레이션 (C 구현)\n");
    printf("  Cascade PID: 속도 루프(외부, 10ms) + 전류 루프(내부, 1ms)\n");
    printf("================================================================================\n");

    scenario_step_response();
    scenario_load_disturbance();

    return 0;
}
