#ifndef __MATH_H
#define __MATH_H

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#include "arm_math.h"

#define PI 3.14159265358979323846f
#define DEGREE2RADIAN 0.017453292519943295f
#define RADIAN2DEGREE 57.29577951308232f


// /**
//  * @brief Make length x length Square Identity  Matrix
//  *

//  * @param[in out] M length x length Matrix
//  * @param[in] length matrix length
//  */
// static inline void SetSquareMatrixIdentity(float *M,  const uint16_t length) {

// }

/**
 * @brief Rotation vector to Rotation matrix
 *
 * See equation (77) of [Quaternion kinematics for the error-state Kalman ﬁlter]
 *
 * @param[in] v 3x1 rotation vector
 * @param[out] R 3x3 rotation matrix
 */
static inline void RotVec2Rot(const float *v, float *R) {
    float v_sum = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    float theta;
    arm_sqrt_f32(v_sum, &theta);

    if (theta < 1e-12) {
        R[0] = 1.0f; R[1] = 0.0f; R[2] = 0.0f;
        R[3] = 0.0f; R[4] = 1.0f; R[5] = 0.0f;
        R[6] = 0.0f; R[7] = 0.0f; R[8] = 1.0f;
    } else {
        float ux = v[0] / theta;
        float uy = v[1] / theta;
        float uz = v[2] / theta;

        float cos_theta = arm_cos_f32(theta);
        float sin_theta = arm_sin_f32(theta);

        float cos_theta_1 = 1 - cos_theta;

        float ux_uy_1_cos = ux * uy * cos_theta_1;
        float ux_uz_1_cos = ux * uz * cos_theta_1;
        float uy_uz_1_cos = uy * uz * cos_theta_1;

        float ux_sin = ux * sin_theta;
        float uy_sin = uy * sin_theta;
        float uz_sin = uz * sin_theta;

        R[0] = cos_theta + ux * ux * cos_theta_1;
        R[1] = ux_uy_1_cos - uz_sin;
        R[2] = ux_uz_1_cos + uy_sin;
        R[3] = ux_uy_1_cos + uz_sin;
        R[4] = cos_theta + uy * uy * cos_theta_1;
        R[5] = uy_uz_1_cos - ux_sin;
        R[6] = ux_uz_1_cos - uy_sin;
        R[7] = uy_uz_1_cos + ux_sin;
        R[8] = cos_theta + uz * uz * cos_theta_1;
    }
}


#endif /* __MATH_H */
