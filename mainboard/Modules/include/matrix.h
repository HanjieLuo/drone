#ifndef __MATRIX_H
#define __MATRIX_H

#ifndef ARM_MATH_CM4
#define ARM_MATH_CM4
#endif

#include "arm_math.h"
#include "stdio.h"

#define MAT_ALLOC(mat, row, col) \
	arm_matrix_instance_f32 mat; \
	__attribute__((aligned(4))) float mat ## _arr[row * col]

#define MAT_INIT(mat, row, col) \
	arm_mat_init_f32(&mat, row, col, (float32_t *)mat ## _arr)

#define MAT_ALLOC_INIT(mat, row, col) \
	arm_matrix_instance_f32 mat; \
	__attribute__((aligned(4))) float mat ## _arr[row * col]; \
	arm_mat_init_f32(&mat, row, col, (float32_t *)mat ## _arr)

#define MAT_STATIC_ALLOC(mat, row, col) \
	static arm_matrix_instance_f32 mat; \
	__attribute__((aligned(4))) static float mat ## _arr[row * col]

#define MAT_STATIC_ALLOC_INIT(mat, row, col) \
	static arm_matrix_instance_f32 mat; \
	__attribute__((aligned(4))) static float mat ## _arr[row * col]; \
	arm_mat_init_f32(&mat, row, col, (float32_t *)mat ## _arr)

#define MAT_ADD(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_add_f32(mat_a, mat_b, mat_result); \

#define MAT_SUB(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_sub_f32(mat_a, mat_b, mat_result); \

#define MAT_MULT(mat_a, mat_b, mat_result) \
	mat_op_status = arm_mat_mult_f32(mat_a, mat_b, mat_result); \

#define MAT_SCALE(mat_in, scale, mat_out) \
	mat_op_status = arm_mat_scale_f32(mat_in, scale, mat_out); \

#define MAT_TRANS(mat, mat_trans) \
	mat_op_status = arm_mat_trans_f32(mat, mat_trans); \

#define MAT_INV(mat, mat_inv) \
	mat_op_status = arm_mat_inverse_f32(mat, mat_inv); \

#define MAT_DATA(mat) mat.pData

#endif /* __MATRIX_H */
