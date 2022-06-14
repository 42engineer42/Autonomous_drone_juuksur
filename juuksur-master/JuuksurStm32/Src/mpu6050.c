#include "mpu6050.h"

#define RAD2DEG_F 57.295779513f
#include <math.h>

uint8_t mpu_read_reg(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t reg) {
    uint8_t data[1], ret[1];
    data[0] = reg;
    ret[0] = 0xFF;
    HAL_I2C_Master_Transmit(i2c, addr, data, 1, 100);
    HAL_I2C_Master_Receive(i2c, addr, ret, 1, 100);
    return ret[0];
}

void mpu_write_reg(I2C_HandleTypeDef *i2c, uint8_t addr, uint8_t reg, uint8_t value) {
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    HAL_I2C_Master_Transmit(i2c, addr|0x1, data, 2, 100);
}

HAL_StatusTypeDef mpu_detect(I2C_HandleTypeDef *i2c, uint8_t addr) {
    HAL_StatusTypeDef present = HAL_I2C_IsDeviceReady(i2c, addr, 20, 100);
    uint8_t whoami = mpu_read_reg(i2c, addr, 0x75);
    if(present == HAL_OK && whoami == 0x68) {
        return HAL_OK;
    }
    return HAL_ERROR;
}

HAL_StatusTypeDef mpu_init(I2C_HandleTypeDef *i2c, uint8_t addr) {
    HAL_StatusTypeDef status = mpu_detect(i2c, addr);
    if(status != HAL_OK) {
        return status;
    }
    // reset the chip
    mpu_write_reg(i2c, addr, MPU6050_REG_PWR_MGMT_1, 0x80);
    HAL_Delay(10);
    // reset signal paths
    mpu_write_reg(i2c, addr, MPU6050_REG_SIGNAL_PATH_RESET, 0x7);
    // use Z gyro oscillator
    mpu_write_reg(i2c, addr, MPU6050_REG_PWR_MGMT_1, 0x3);
    // use DLPF (Gyro 184Hz bandwidth, 2ms delay, Acc 188Hz bandwidth 1.9ms delay)
    // restricts output at 1khz (DLPF - Digital Low Pass Filter)
    mpu_write_reg(i2c, addr, MPU6050_REG_CONFIG, 0x6);
    return HAL_OK;
}

void mpu_process_data(uint8_t *data, Mpu6050Data *mpudata) {
    int16_t value;
    if(mpudata->lastData == 0) {
        return;
    }
    value = ((data[0]<<8) | data[1]);
    mpudata->acc[0] = ((float)value/32768.f)*2.0f*9.81f;
    value = (data[2]<<8) | data[3];
    mpudata->acc[1] = ((float)value/32768.f)*2.0f*9.81f;
    value = (data[4]<<8) | data[5];
    mpudata->acc[2] = ((float)value/32768.f)*2.0f*9.81f;

    value = (data[6]<<8) | data[7];
    mpudata->temp = ((float)(value)/340.0f + 36.53f); 

    value = (data[8]<<8) | data[9];
    mpudata->gyro[0] = ((float)value/32768.f)*250.0f;
    value = (data[10]<<8) | data[11];
    mpudata->gyro[1] = ((float)value/32768.f)*250.0f;
    value = (data[12]<<8) | data[13];
    mpudata->gyro[2] = ((float)value/32768.f)*250.0f;

    mpudata->lastProcess = HAL_GetTick();
}

void mpu_fusion_init(MpuFusion *state) {
    // decreasing Q[1][1] increases drift probablity
    // decreasing Q[0][0] increases responsiveness
    state->pitchQ[0][0] = 0.001f;
    state->pitchQ[1][1] = 0.003f;
    state->pitchR = 0.03f;
    state->pitch = 0.0f;
    state->pitchRateBias = 0.0f;
    state->pitchRate = 0.0f;

    state->rollQ[0][0] = 0.001f;
    state->rollQ[1][1] = 0.003f;
    state->rollR = 0.03f;
    state->roll = 0.0f;
    state->rollRateBias = 0.0f;
    state->rollRate = 0.0f;

    // NOTE: if initial state is unknown [0][0] and [1][1] should be large values
    state->pitchP[0][0] = 0.0f; state->pitchP[0][1] = 0.0f;
    state->pitchP[1][0] = 0.0f; state->pitchP[1][1] = 0.0f;

    state->rollP[0][0] = 0.0f; state->rollP[0][1] = 0.0f;
    state->rollP[1][0] = 0.0f; state->rollP[1][1] = 0.0f;
}

void mpu_fusion_prediction(MpuFusion *state, float rollRate, float pitchRate, float dt) {
    float pr = pitchRate - state->pitchRateBias;
    float rr = rollRate - state->rollRateBias;
    state->pitch += dt*pr;
    state->roll += dt*rr;

    // update error covariance
    state->pitchP[0][0] += dt * (dt*state->pitchP[1][1] - state->pitchP[0][1] - state->pitchP[1][0] + state->pitchQ[0][0]);
    state->pitchP[0][1] -= dt*state->pitchP[1][1];
    state->pitchP[1][0] -= dt*state->pitchP[1][1];
    state->pitchP[1][1] += dt*state->pitchQ[1][1];

    state->rollP[0][0] += dt * (dt*state->rollP[1][1] - state->rollP[0][1] - state->rollP[1][0] + state->rollQ[0][0]);
    state->rollP[0][1] -= dt*state->rollP[1][1];
    state->rollP[1][0] -= dt*state->rollP[1][1];
    state->rollP[1][1] += dt*state->rollQ[1][1];

}

void mpu_fusion_update(MpuFusion *state, float roll, float pitch, float dt) {
    // TODO: if you use full Q matrix this is wrong! Either do actual matrix multiplication or only expose the [0][0] and [1][1] parts of the matrix

    // calculate innovation
    float py = pitch - state->pitch;
    float ry = roll - state->roll;
    // calculate error
    float ps = state->pitchP[0][0] + state->pitchR;
    float rs = state->rollP[0][0] + state->rollR;
    // calculate kalman gain
    float pk[2], rk[2];
    pk[0] = state->pitchP[0][0] / ps;
    pk[1] = state->pitchP[1][0] / ps;
    rk[0] = state->rollP[0][0] / rs;
    rk[1] = state->rollP[1][0] / rs;

    float pP00_temp = state->pitchP[0][0];
    float pP01_temp = state->pitchP[0][1];
    state->pitchP[0][0] -= pk[0] * pP00_temp;
    state->pitchP[0][1] -= pk[0] * pP01_temp;
    state->pitchP[1][0] -= pk[1] * pP00_temp;
    state->pitchP[1][1] -= pk[1] * pP01_temp;

    float rP00_temp = state->rollP[0][0];
    float rP01_temp = state->rollP[0][1];
    state->rollP[0][0] -= rk[0] * rP00_temp;
    state->rollP[0][1] -= rk[0] * rP01_temp;
    state->rollP[1][0] -= rk[1] * rP00_temp;
    state->rollP[1][1] -= rk[1] * rP01_temp;

    //debug
    state->pitchK[0] = pk[0];
    state->pitchK[1] = pk[1];
    state->rollK[0] = rk[0];
    state->rollK[1] = rk[1];

    // correct estimated state
    state->pitch += pk[0]*py;
    state->pitchRateBias += pk[1]*py;
    state->roll += rk[0]*ry;
    state->rollRateBias += rk[1]*ry;
}

// TODO: gyro and acc data could be updates separately?
void mpu_update_angle_estimates(Mpu6050Data *data, MpuFusion *fusion, float dt) {
    // have not received any data from IMU yet or the data is invalid
    if(data->lastProcess == 0 
            || (data->acc[0] == 0.0f && data->acc[1] == 0.0f && data->acc[2] == 0.0f)) {
        return;
    }
    // roll is around X axis
    // pitch is around Y axis
    mpu_fusion_prediction(fusion, data->gyro[0], data->gyro[1], dt);
    // TODO: calculate angles from acc
    float accX = data->acc[0], accY = data->acc[1], accZ = data->acc[2];
    float accRoll = atan2f(accY, accZ) * RAD2DEG_F;
    float accPitch = atanf(-accX / sqrtf(accY*accY + accZ*accZ)) * RAD2DEG_F;

    mpu_fusion_update(fusion, accRoll, accPitch, dt);
}
