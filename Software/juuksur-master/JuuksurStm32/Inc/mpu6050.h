#pragma once

//#include <stm32f1xx_hal_i2c.h>
#include "stm32f3xx_hal.h"

#define MPU6050_I2C_ADDR (0b1101000<<1)

#define MPU6050_REG_CONFIG 0x1A
#define MPU6050_REG_TEMP_OUT_H 0x41
#define MPU6050_REG_TEMP_OUT_L 0x42
#define MPU6050_REG_SIGNAL_PATH_RESET 0x68
#define MPU6050_REG_PWR_MGMT_1 0x6B

#define MPU6050_REG_GYRO_XOUT_H 0x43
#define MPU6050_REG_GYRO_XOUT_L 0x44
#define MPU6050_REG_GYRO_YOUT_H 0x45
#define MPU6050_REG_GYRO_YOUT_L 0x46
#define MPU6050_REG_GYRO_ZOUT_H 0x47
#define MPU6050_REG_GYRO_ZOUT_L 0x48

typedef struct Mpu6050Data {
    uint8_t rawData[14];
    float gyro[3];
    float acc[3];
    float temp;
    uint32_t lastData; // last tick raw data was received
    uint32_t lastProcess; // last tick data was processed
} Mpu6050Data;

// Kalman filter state
// not estimating yaw because it would drift (would need magnetometer)
// matrices are in column major memory order (first index is row)
typedef struct MpuFusion {
    float pitch, roll; // current estimations
    float pitchRateBias, rollRateBias;
    float pitchRate, rollRate; // rate estimations
    float pitchP[2][2], rollP[2][2]; // estimate covariance matrix
    float pitchF[2][2], rollF[2][2]; // state transition matrix
    float pitchQ[2][2], rollQ[2][2]; // process variance

    float pitchR, rollR; // measurement covariance

    // debug
    float pitchK[2], rollK[2];
    float pitchY;
} MpuFusion;

uint8_t mpu_read_reg(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t reg);
void mpu_write_reg(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t reg, uint8_t value);
HAL_StatusTypeDef mpu_detect(I2C_HandleTypeDef *i2c, uint8_t addr);
HAL_StatusTypeDef mpu_init(I2C_HandleTypeDef *i2c, uint8_t addr);
void mpu_process_data(uint8_t *data, Mpu6050Data *dst);

void mpu_update_angle_estimates(Mpu6050Data *data, MpuFusion *fusion, float dt);
void mpu_fusion_init(MpuFusion *state);
void mpu_fusion_prediction(MpuFusion *state, float rollRate, float pitchRate, float dt);
void mpu_fusion_update(MpuFusion *state, float roll, float pitch, float dt);
