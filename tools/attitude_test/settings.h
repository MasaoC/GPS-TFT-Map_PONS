#pragma once
#define IMULOG_RAW_REPORTS_ENABLED 1
#define IMU_RATE_GYRO_HZ 50
#define ESKF_SIGMA_G    2e-3f
#define ESKF_SIGMA_A    3e-2f
#define ESKF_SIGMA_BG   1e-5f
#define ESKF_SIGMA_BA   1e-4f
#define ESKF_MAX_GYRO_BIAS   0.01745f
#define ESKF_MAX_ACCEL_BIAS  1.0f
#define ESKF_SACC_MIN   0.05f
#define ESKF_SACC_MAX   1.0f
#define ESKF_STATIC_GYRO_RADS   0.026f
#define ESKF_STATIC_ACCEL_TOL   0.30f
#define ESKF_STATIC_INIT_US     2000000UL
#define ESKF_INIT_ATT_SIGMA_DEG         3.0f
#define ESKF_INIT_ATT_SIGMA_MOVING_DEG  30.0f

#define ESKF_INIT_USE_RV        1
#define ESKF_RV_ACC_MAX_DEG     25.0f
#define ESKF_MAG_DECLINATION_DEG  (-8.0f)
