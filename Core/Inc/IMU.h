#include <stdint.h>
#ifndef IMU_H
#define IMU_H
void BMI088_ACCEL_NS_L(void);
void BMI088_ACCEL_NS_H(void);
void BMI088_GYRO_NS_L(void);
void BMI088_GYRO_NS_H(void);
void BMI088_ReadReg_ACCEL(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_ReadReg_GYRO(uint8_t reg, uint8_t *return_data, uint8_t length);
void BMI088_WriteReg(uint8_t reg, uint8_t write_data);
void BMI088_Init(void);
void BMI_Init();
void GetAccelData(uint8_t *return_data, uint8_t range_acc_receive, float *Accel_X_in_mg, float *Accel_Y_in_mg, float *Accel_Z_in_mg);
void GetGyroData(uint8_t *return_data, uint8_t range_gyro_receive, float *Rate_X, float *Rate_Y, float *Rate_Z);
#endif //IMU_H
