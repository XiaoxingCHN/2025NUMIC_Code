#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led_strip.h"
#include "WS2812BS_Signal.h"
#include "Device_ICM42688P.h"
#include "esp_timer.h"  // esp_timer_get_time
#include <math.h>
#include <stdbool.h>

//Begin0
// === 姿态解算：Mahony，保持四元数贯穿全链路，最后再转欧拉 ===
typedef struct {
    float q0, q1, q2, q3;   // 四元数
    float exInt, eyInt, ezInt;
    float twoKp;
    float twoKi;
} MahonyFilter;

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void mahony_init(MahonyFilter *f, float kp, float ki) {
    f->q0 = 1.0f; f->q1 = f->q2 = f->q3 = 0.0f;
    f->exInt = f->eyInt = f->ezInt = 0.0f;
    f->twoKp = 2.0f * kp;
    f->twoKi = 2.0f * ki;
}

static void mahony_update(MahonyFilter *f, float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    // 若加速度模长偏离 1g 太多，认为处于强动力阶段，不用加计修正以减少姿态抖动
    float acc_norm_sq = ax * ax + ay * ay + az * az;
    const float g_ref_sq_min = (0.7f * 9.80665f) * (0.7f * 9.80665f);
    const float g_ref_sq_max = (1.3f * 9.80665f) * (1.3f * 9.80665f);
    if (acc_norm_sq < g_ref_sq_min || acc_norm_sq > g_ref_sq_max) {
        // 仅用陀螺积分，保持姿态连续
        gx *= 0.5f * dt;
        gy *= 0.5f * dt;
        gz *= 0.5f * dt;
        float qa = f->q0, qb = f->q1, qc = f->q2, qd = f->q3;
        f->q0 += (-qb * gx - qc * gy - qd * gz);
        f->q1 += (qa * gx + qc * gz - qd * gy);
        f->q2 += (qa * gy - qb * gz + qd * gx);
        f->q3 += (qa * gz + qb * gy - qc * gx);
        float norm_i = 1.0f / sqrtf(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
        f->q0 *= norm_i; f->q1 *= norm_i; f->q2 *= norm_i; f->q3 *= norm_i;
        return;
    }

    float norm = 1.0f / sqrtf(acc_norm_sq);
    ax *= norm; ay *= norm; az *= norm;

    // 重力方向估计
    float vx = 2.0f * (f->q1 * f->q3 - f->q0 * f->q2);
    float vy = 2.0f * (f->q0 * f->q1 + f->q2 * f->q3);
    float vz = f->q0 * f->q0 - f->q1 * f->q1 - f->q2 * f->q2 + f->q3 * f->q3;

    // 误差为测量重力与估计重力的叉乘
    float ex = (ay * vz - az * vy);
    float ey = (az * vx - ax * vz);
    float ez = (ax * vy - ay * vx);

    // 积分误差（限幅防积累漂移）；Ki=0 时保持为 0
    if (f->twoKi > 0.0f) {
        f->exInt = clampf(f->exInt + ex * f->twoKi * dt, -0.1f, 0.1f);
        f->eyInt = clampf(f->eyInt + ey * f->twoKi * dt, -0.1f, 0.1f);
        f->ezInt = clampf(f->ezInt + ez * f->twoKi * dt, -0.1f, 0.1f);
    } else {
        f->exInt = f->eyInt = f->ezInt = 0.0f;
    }

    // PI 校正
    gx += f->twoKp * ex + f->exInt;
    gy += f->twoKp * ey + f->eyInt;
    gz += f->twoKp * ez + f->ezInt;

    // 四元数微分更新
    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;

    float qa = f->q0, qb = f->q1, qc = f->q2, qd = f->q3;
    f->q0 += (-qb * gx - qc * gy - qd * gz);
    f->q1 += (qa * gx + qc * gz - qd * gy);
    f->q2 += (qa * gy - qb * gz + qd * gx);
    f->q3 += (qa * gz + qb * gy - qc * gx);

    // 归一化
    norm = 1.0f / sqrtf(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= norm; f->q1 *= norm; f->q2 *= norm; f->q3 *= norm;
}

static void mahony_get_euler(const MahonyFilter *f, float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f * (f->q0 * f->q1 + f->q2 * f->q3), 1.0f - 2.0f * (f->q1 * f->q1 + f->q2 * f->q2));
    *pitch = asinf(2.0f * (f->q0 * f->q2 - f->q3 * f->q1));
    *yaw   = atan2f(2.0f * (f->q0 * f->q3 + f->q1 * f->q2), 1.0f - 2.0f * (f->q2 * f->q2 + f->q3 * f->q3));
}

static inline float rad_to_deg180(float rad) {
    float deg = rad * 57.2957795f;
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

typedef struct {
    float w, x, y, z;
} Quat;

static void quat_normalize(Quat *q) {
    float n = sqrtf(q->w * q->w + q->x * q->x + q->y * q->y + q->z * q->z);
    if (n < 1e-6f) {
        q->w = 1.0f; q->x = q->y = q->z = 0.0f;
        return;
    }
    float inv = 1.0f / n;
    q->w *= inv; q->x *= inv; q->y *= inv; q->z *= inv;
}

// SLERP 作为简单平滑，alpha 小时偏向前一帧，保持四元数连续
static Quat quat_slerp(Quat a, Quat b, float alpha) {
    float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    if (dot < 0.0f) { b.w = -b.w; b.x = -b.x; b.y = -b.y; b.z = -b.z; dot = -dot; }
    const float DOT_THRESH = 0.9995f;
    if (dot > DOT_THRESH) {
        Quat r = {
            a.w + alpha * (b.w - a.w),
            a.x + alpha * (b.x - a.x),
            a.y + alpha * (b.y - a.y),
            a.z + alpha * (b.z - a.z)
        };
        quat_normalize(&r);
        return r;
    }
    float theta = acosf(dot);
    float sin_theta = sinf(theta);
    float w1 = sinf((1.0f - alpha) * theta) / sin_theta;
    float w2 = sinf(alpha * theta) / sin_theta;
    Quat r = {
        w1 * a.w + w2 * b.w,
        w1 * a.x + w2 * b.x,
        w1 * a.y + w2 * b.y,
        w1 * a.z + w2 * b.z
    };
    quat_normalize(&r);
    return r;
}

static void quat_to_euler(const Quat *q, float *roll, float *pitch, float *yaw) {
    *roll  = atan2f(2.0f * (q->w * q->x + q->y * q->z), 1.0f - 2.0f * (q->x * q->x + q->y * q->y));
    *pitch = asinf(2.0f * (q->w * q->y - q->z * q->x));
    *yaw   = atan2f(2.0f * (q->w * q->z + q->x * q->y), 1.0f - 2.0f * (q->y * q->y + q->z * q->z));
}

// 通过加速度初始静置姿态来设定 Mahony 四元数，减少初始零偏
static void mahony_seed_from_acc(MahonyFilter *f, float ax, float ay, float az) {
    // 归一化加速度
    float norm = ax * ax + ay * ay + az * az;
    if (norm < 1e-6f) return;
    norm = 1.0f / sqrtf(norm);
    ax *= norm; ay *= norm; az *= norm;

    float roll = atan2f(ay, az);
    float pitch = atan2f(-ax, sqrtf(ay * ay + az * az));
    float half_r = roll * 0.5f;
    float half_p = pitch * 0.5f;
    float cr = cosf(half_r), sr = sinf(half_r);
    float cp = cosf(half_p), sp = sinf(half_p);
    // yaw 设为 0
    f->q0 = cr * cp;
    f->q1 = sr * cp;
    f->q2 = cr * sp;
    f->q3 = sr * sp;
    float norm_q = 1.0f / sqrtf(f->q0 * f->q0 + f->q1 * f->q1 + f->q2 * f->q2 + f->q3 * f->q3);
    f->q0 *= norm_q; f->q1 *= norm_q; f->q2 *= norm_q; f->q3 *= norm_q;
}
    //End0

void app_main(void)
{

    configure_WS2812B();
    while (1)
    {
        // 红 -> 黄
        for (int g = 0; g <= 255; g += 5) {
            WS2812B_Color_Set(255, g, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 黄 -> 绿
        for (int r = 255; r >= 0; r -= 5) {
            WS2812B_Color_Set(r, 255, 0);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 绿 -> 青
        for (int b = 0; b <= 255; b += 5) {
            WS2812B_Color_Set(0, 255, b);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 青 -> 蓝
        for (int g = 255; g >= 0; g -= 5) {
            WS2812B_Color_Set(0, g, 255);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 蓝 -> 紫
        for (int r = 0; r <= 255; r += 5) {
            WS2812B_Color_Set(r, 0, 255);
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 紫 -> 红
        for (int b = 255; b >= 0; b -= 5) {
            WS2812B_Color_Set(255, 0, b);
            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}