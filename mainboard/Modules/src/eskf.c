#include "eskf.h"

MAT_ALLOC(gravity, 3, 1);
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

    MAT_INIT(gravity, 3, 1);
    // MAT_INIT(acc_var, 3, 1);
    // MAT_INIT(gyro_var, 3, 1);
    // MAT_INIT(acc_bias_var, 3, 1);
    // MAT_INIT(gyro_bias_var, 3, 1);

    MAT_DATA(gravity)[0] = 0.f;
    MAT_DATA(gravity)[1] = 0.f;
    MAT_DATA(gravity)[2] = -_gravity;

    // MAT_DATA(acc_var)[0] = _acc_var;
    // MAT_DATA(acc_var)[1] = _acc_var;
    // MAT_DATA(acc_var)[2] = _acc_var;

    // MAT_DATA(gyro_var)[0] = _gyro_var;
    // MAT_DATA(gyro_var)[1] = _gyro_var;
    // MAT_DATA(gyro_var)[2] = _gyro_var;

    // MAT_DATA(acc_bias_var)[0] = _acc_bias_var;
    // MAT_DATA(acc_bias_var)[1] = _acc_bias_var;
    // MAT_DATA(acc_bias_var)[2] = _acc_bias_var;

    // MAT_DATA(gyro_bias_var)[0] = _gyro_bias_var;
    // MAT_DATA(gyro_bias_var)[1] = _gyro_bias_var;
    // MAT_DATA(gyro_bias_var)[2] = _gyro_bias_var;


    /* initialize the nominal state */
    MAT_INIT(state.p, 3, 1);
    arm_fill_f32(0.f, MAT_DATA(state.p), 3);

    MAT_INIT(state.v, 3, 1);
    arm_fill_f32(0.f, MAT_DATA(state.v), 3);

    MAT_INIT(state.acc_bias, 3, 1);
    arm_fill_f32(0.f, MAT_DATA(state.acc_bias), 3);

    MAT_INIT(state.gyro_bias, 3, 1);
    arm_fill_f32(0.f, MAT_DATA(state.gyro_bias), 3);

    MAT_INIT(state.q, 4, 1);
    MAT_DATA(state.q)[0] = 1.0f;
    MAT_DATA(state.q)[1] = 0.0f;
    MAT_DATA(state.q)[2] = 0.0f;
    MAT_DATA(state.q)[3] = 0.0f;

    MAT_INIT(state.R, 3, 3);
    MAT_DATA(state.R)[0] = 1.0f; MAT_DATA(state.R)[1] = 0.0f; MAT_DATA(state.R)[2] = 0.0f;
    MAT_DATA(state.R)[3] = 0.0f; MAT_DATA(state.R)[4] = 1.0f; MAT_DATA(state.R)[5] = 0.0f;
    MAT_DATA(state.R)[6] = 0.0f; MAT_DATA(state.R)[7] = 0.0f; MAT_DATA(state.R)[8] = 1.0f;

    MAT_INIT(state.COV, 16, 16);
    arm_fill_f32(0.f, MAT_DATA(state.COV), 256);
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
    float dt = (float)(*timestamp - state.timestamp);
    float dt2 = dt * dt;

    MAT_STATIC_ALLOC_INIT(acc_unbias, 3, 1);
    MAT_STATIC_ALLOC_INIT(acc_world, 3, 1);
    
    // Midward Integration
    MAT_DATA(acc_unbias)[0] = _acc[0] - MAT_DATA(state.acc_bias)[0];
    MAT_DATA(acc_unbias)[1] = _acc[1] - MAT_DATA(state.acc_bias)[1];
    MAT_DATA(acc_unbias)[2] = _acc[2] - MAT_DATA(state.acc_bias)[2];

    


    printf("%llu, %f, %f, %f\r\n", *timestamp, MAT_DATA(acc_unbias)[0], MAT_DATA(acc_unbias)[1], MAT_DATA(acc_unbias)[2]); 
    
}
