#ifndef __ESKF_H
#define __ESKF_H

#include "FreeRTOS.h"
#include "queue.h"
#include "utils/matrix.h"
#include "utils/math.h"
#include "utils/utils.h"

#define StateEstimatorInit EskfInit

// Indexes to access the ESKF's state, stored as a column vector
typedef enum {
    STATE_PX,
    STATE_PY,
    STATE_PZ,
    STATE_VX,
    STATE_VY,
    STATE_VZ,
    STATE_ROLL,
    STATE_PITCH,
    STATE_YAW,
    STATE_ACC_BIAS_X,
    STATE_ACC_BIAS_Y,
    STATE_ACC_BIAS_Z,
    STATE_GYRO_BIAS_X,
    STATE_GYRO_BIAS_Y,
    STATE_GYRO_BIAS_Z,
    STATE_DIM
} eskf_error_state_idx;

// The data used by the ESKF.
typedef struct {
    /**
   * Quadrocopter State
   *
   * The internally-estimated state is:
   * - p: the quad's position in the global frame
   * - v: the quad's velocity in its global frame
   * - q:  The quad's attitude as a quaternion (w,x,y,z) in its global frame
   * - acc_bias: acceleration bias in its local frame
   * - gyro_bias: gyroscope bias in its local frame 
   * For more information, refer to the paper [Quaternion kinematics for the error-state Kalman ﬁlter]
   */

    float p[3];
    float v[3];

    // The quad's attitude as a quaternion (qw, qx, qy, qz)
    // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
    // while also being robust against singularities (in comparison to euler angles)
    float q[4];

    float R[3][3];

    // bias
    float acc_bias[3];
    float gyro_bias[3];

    // The covariance matrix
    arm_matrix_instance_f32 COV;
    __attribute__((aligned(4))) float COV_arr[16][16];

    uint64_t timestamp;
    float acc[3];
    float gyro[3];

} eskf_state;

bool EskfInit(const float32_t _gravity,
              const float32_t _acc_noise,
              const float32_t _gyro_noise,
              const float32_t _acc_bias_noise,
              const float32_t _gyro_bias_noise);

void EskfPredict(const uint64_t *timestamp, const float *acc, const float *gyro);
#endif /* __ESKF_H */
