#include "mbot_odometry.h"
#include <math.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif

// Loops without encoder ticks before the robot counts as still (0.2 s at 100 Hz). Slow turns can
// go a few loops without a tick, so a single empty loop isn't enough.
#define GYRODOM_STILL_LOOPS 20
// How fast the gyro bias estimate follows the gyro while still (about a 1 s time constant at 100 Hz).
#define GYRODOM_BIAS_ALPHA 0.01f

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
    static int still_loops = 0;
    static float gyro_bias = 0.0f;

    // No encoder ticks on either wheel gives exactly zero velocity.
    if (vx == 0.0f && wz == 0.0f) {
        if (still_loops < GYRODOM_STILL_LOOPS) still_loops++;
    } else {
        still_loops = 0;
    }

    float yaw_rate;
    if (still_loops >= GYRODOM_STILL_LOOPS) {
        // Still: keep the encoders' zero yaw rate so the heading doesn't drift, and learn the gyro's bias.
        gyro_bias += GYRODOM_BIAS_ALPHA * (gyro_z - gyro_bias);
        yaw_rate = wz;
    } else {
        // Moving: use the gyro. The encoders misjudge turns in place by a few percent (tire scrub),
        // and the error changes from run to run, so a disagreement threshold can't catch it.
        yaw_rate = gyro_z - gyro_bias;
    }
    mbot_calculate_odometry(vx, vy, yaw_rate, dt, x, y, theta);
}
