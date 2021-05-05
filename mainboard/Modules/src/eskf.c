#include "eskf.h"

float gravity[3];
// MAT_ALLOC(gravity, 3, 1);
// MAT_ALLOC(acc_var, 3, 1);
// MAT_ALLOC(gyro_var, 3, 1);
// MAT_ALLOC(acc_bias_var, 3, 1);
// MAT_ALLOC(gyro_bias_var, 3, 1);

// MAT_ALLOC(p, 3, 1);
// MAT_ALLOC(v, 3, 1);
// MAT_ALLOC(R, 3, 1);

eskf_state state;

bool EskfInit(const float32_t _gravity,
              const float32_t _acc_var,
              const float32_t _gyro_var,
              const float32_t _acc_bias_var,
              const float32_t _gyro_bias_var) {

    gravity[0] = 0.f;
    gravity[1] = 0.f;
    gravity[2] = -_gravity;

    /* initialize the nominal state */
    memset(state.p, 0, 3*sizeof(float));
    memset(state.v, 0, 3*sizeof(float));
    memset(state.acc_bias, 0, 3*sizeof(float));
    memset(state.gyro_bias, 0, 3*sizeof(float));

    state.q[0] = 1.0f; state.q[1] = 0.0f; state.q[2] = 0.0f; state.q[3] = 0.0f;

    state.R[0][0] = 1.0f; state.R[0][1] = 0.0f; state.R[0][2] = 0.0f;
    state.R[1][0] = 0.0f; state.R[1][1] = 1.0f; state.R[1][2] = 0.0f;
    state.R[2][0] = 0.0f; state.R[2][1] = 0.0f; state.R[2][2] = 1.0f;

    MAT_INIT(state.COV, 16, 16);
    memset(state.COV_arr, 0, 256*sizeof(float));
    state.COV_arr[STATE_PX][STATE_PX] = 1.f * 1.f; // position x
    state.COV_arr[STATE_PY][STATE_PY] = 1.f * 1.f; // position y
    state.COV_arr[STATE_PZ][STATE_PZ] = 1.f * 1.f; // position z
    state.COV_arr[STATE_PX][STATE_PX] = 1.f * 1.f; // velocity x
    state.COV_arr[STATE_PY][STATE_PY] = 1.f * 1.f; // velocity y
    state.COV_arr[STATE_PZ][STATE_PZ] = 1.f * 1.f; // velocity z
    state.COV_arr[STATE_ROLL][STATE_ROLL] = (10.f * DEGREE2RADIAN) * (10.f * DEGREE2RADIAN); // roll
    state.COV_arr[STATE_PITCH][STATE_PITCH] = (10.f * DEGREE2RADIAN) * (10.f * DEGREE2RADIAN); // pitch
    state.COV_arr[STATE_YAW][STATE_YAW] = (10.f * DEGREE2RADIAN) * (10.f * DEGREE2RADIAN); // yaw
    state.COV_arr[STATE_ACC_BIAS_X][STATE_ACC_BIAS_X] = 0.1f * 0.1f; // ass bias x
    state.COV_arr[STATE_ACC_BIAS_Y][STATE_ACC_BIAS_Y] = 0.1f * 0.1f; // ass bias y
    state.COV_arr[STATE_ACC_BIAS_Y][STATE_ACC_BIAS_Z] = 0.1f * 0.1f; // ass bias z
    state.COV_arr[STATE_GYRO_BIAS_X][STATE_GYRO_BIAS_X] = 0.01f * 0.01f; // ass gyro x
    state.COV_arr[STATE_GYRO_BIAS_Y][STATE_GYRO_BIAS_Y] = 0.01f * 0.01f; // ass gyro y
    state.COV_arr[STATE_GYRO_BIAS_Z][STATE_GYRO_BIAS_Z] = 0.01f * 0.01f; // ass gyro z

    printf("Success to Init Eskf!\r\n\r\n");
    
    return true;
}

void EskfPredict(const uint64_t *timestamp, const float *_acc, const float *_gyro) {
    static float acc_unbias[3];
    static float gyro_unbias[3];
    static float acc_world[3];

    float dt = (float)(*timestamp - state.timestamp);
    float dt2 = dt * dt;

    acc_unbias[0] = _acc[0] - state.acc_bias[0];
    acc_unbias[1] = _acc[1] - state.acc_bias[1];
    acc_unbias[2] = _acc[2] - state.acc_bias[2];

    gyro_unbias[0] = _gyro[0] - state.gyro_bias[0];
    gyro_unbias[1] = _gyro[1] - state.gyro_bias[1];
    gyro_unbias[2] = _gyro[2] - state.gyro_bias[2];

    acc_world[0] = state.R[0][0] * acc_unbias[0] + state.R[0][1] * acc_unbias[1] + state.R[0][2] * acc_unbias[2] + gravity[0];
    acc_world[1] = state.R[1][0] * acc_unbias[0] + state.R[1][1] * acc_unbias[1] + state.R[1][2] * acc_unbias[2] + gravity[1];
    acc_world[2] = state.R[2][0] * acc_unbias[0] + state.R[2][1] * acc_unbias[1] + state.R[2][2] * acc_unbias[2] + gravity[2];

    state.p[0] = state.p[0] + state.v[0] * dt + 0.5 * acc_world[0] * dt2;
    state.p[1] = state.p[1] + state.v[1] * dt + 0.5 * acc_world[1] * dt2;
    state.p[2] = state.p[2] + state.v[2] * dt + 0.5 * acc_world[2] * dt2;

    state.v[0] = state.v[0] + acc_world[0] * dt;
    state.v[1] = state.v[1] + acc_world[1] * dt;
    state.v[2] = state.v[2] + acc_world[2] * dt;

    



    // MAT_STATIC_ALLOC_INIT(acc_unbias, 3, 1);
    // MAT_STATIC_ALLOC_INIT(acc_world, 3, 1);
    // MAT_STATIC_ALLOC_INIT(gyro_unbias, 3, 1);
    
    // MAT_DATA(acc_unbias)[0] = _acc[0] - MAT_DATA(state.acc_bias)[0];
    // MAT_DATA(acc_unbias)[1] = _acc[1] - MAT_DATA(state.acc_bias)[1];
    // MAT_DATA(acc_unbias)[2] = _acc[2] - MAT_DATA(state.acc_bias)[2];

    // MAT_DATA(gyro_unbias)[0] = _gyro[0] - MAT_DATA(state.gyro_bias)[0];
    // MAT_DATA(gyro_unbias)[1] = _gyro[1] - MAT_DATA(state.gyro_bias)[1];
    // MAT_DATA(gyro_unbias)[2] = _gyro[2] - MAT_DATA(state.gyro_bias)[2];

    // MAT_MULT(&state.R, &acc_unbias, &acc_world);

    // MAT_DATA(acc_world)[0] = MAT_DATA(acc_world)[0] - MAT_DATA(gravity)[0];
    // MAT_DATA(acc_world)[1] = MAT_DATA(acc_world)[1] - MAT_DATA(gravity)[1];
    // MAT_DATA(acc_world)[2] = MAT_DATA(acc_world)[2] - MAT_DATA(gravity)[2];

    // MAT_DATA(state.p)[0] = MAT_DATA(state.p)[0] + MAT_DATA(state.v)[0] * dt + 0.5 * MAT_DATA(acc_world)[0] * dt2;
    // MAT_DATA(state.p)[1] = MAT_DATA(state.p)[1] + MAT_DATA(state.v)[1] * dt + 0.5 * MAT_DATA(acc_world)[1] * dt2;
    // MAT_DATA(state.p)[2] = MAT_DATA(state.p)[2] + MAT_DATA(state.v)[2] * dt + 0.5 * MAT_DATA(acc_world)[2] * dt2;

    // MAT_DATA(state.v)[0] = MAT_DATA(state.v)[0] + MAT_DATA(acc_world)[0] * dt;
    // MAT_DATA(state.v)[1] = MAT_DATA(state.v)[1] + MAT_DATA(acc_world)[1] * dt;
    // MAT_DATA(state.v)[2] = MAT_DATA(state.v)[2] + MAT_DATA(acc_world)[2] * dt;

    // arm_mat_mult_f32(&state.R, &acc_unbias, &acc_world);

    // printf("%llu, %f, %f, %f\r\n", *timestamp, _acc[0], _acc[1], _acc[2]); 
    // printf("%llu, %f, %f, %f\r\n", *timestamp, MAT_DATA(acc_world)[0], MAT_DATA(acc_world)[1], MAT_DATA(acc_world)[2]); 
    
}
