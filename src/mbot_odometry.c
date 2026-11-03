#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Trust the gyro over the encoders when their yaw rates differ by more than this (rad/s).
// Must stay above the encoder yaw rate step (about 0.04 rad/s) and the gyro noise at rest.
#define GYRODOM_THRESHOLD 0.05f

void mbot_calculate_odometry(float vx, float vy, float wz, float dt, float* x, float* y, float* theta) {
    // Update pose
    *x += vx * dt * cos(*theta) - vy * dt * sin(*theta);
    *y += vx * dt * sin(*theta) + vy * dt * cos(*theta);
    *theta += wz * dt;

    // Normalize theta to [-pi, pi]
    while (*theta > PI) *theta -= 2.0 * PI;
    while (*theta <= -PI) *theta += 2.0 * PI;
}

void mbot_calculate_gyrodometry(float vx, float vy, float wz, float dt, float gyro_z, float* x, float* y, float* theta) {
    // Disagreement suggests wheel slip, so use the gyro. Otherwise use the encoders, which don't drift at rest.
    float yaw_rate = (fabsf(gyro_z - wz) > GYRODOM_THRESHOLD) ? gyro_z : wz;
    mbot_calculate_odometry(vx, vy, yaw_rate, dt, x, y, theta);
}