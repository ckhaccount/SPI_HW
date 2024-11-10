#include "main.h"
#include "gpio.h"
#include "spi.h"
#include "IMU.h"
#include <math.h>
extern uint8_t rx_data;
extern uint8_t range_acc;
extern uint8_t range_acc_receive;

void BMI088_ACCEL_NS_L(void) {
    HAL_GPIO_WritePin(CS_Accel_GPIO_Port, CS_Accel_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void) {
    HAL_GPIO_WritePin(CS_Accel_GPIO_Port, CS_Accel_Pin, GPIO_PIN_SET);
}
void BMI088_GYRO_NS_L(void) {
    HAL_GPIO_WritePin(CS_Gyro_GPIO_Port, CS_Gyro_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void) {
    HAL_GPIO_WritePin(CS_Gyro_GPIO_Port, CS_Gyro_Pin, GPIO_PIN_SET);
}

void BMI088_ReadReg_ACCEL(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_ACCEL_NS_L();
    uint8_t rx_adress= reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &rx_adress, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    // HAL_SPI_Receive(&hspi1, return_data, 1, 1000);
    // while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX);
    HAL_SPI_Receive(&hspi1, return_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX); // 等待SPI接收完成
    BMI088_ACCEL_NS_H();
}
void BMI088_ReadReg_GYRO(uint8_t reg, uint8_t *return_data, uint8_t length) {
    BMI088_GYRO_NS_L();
    uint8_t rx_adress= reg | 0x80;
    HAL_SPI_Transmit(&hspi1, &rx_adress, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Receive(&hspi1, return_data, length, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_RX); // 等待SPI接收完成
    BMI088_GYRO_NS_H();
}
void BMI088_WriteReg(uint8_t reg, uint8_t write_data) {
    uint8_t tx_adress= reg & 0x7F;
    HAL_SPI_Transmit(&hspi1, &tx_adress, 1, 1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX);
    HAL_SPI_Transmit(&hspi1,&write_data,1,1000);
    while (HAL_SPI_GetState(&hspi1) == HAL_SPI_STATE_BUSY_TX); // 等待SPI接收完成
}

void BMI088_Init(void) {
    // Soft Reset ACCEL
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x7E, 0xB6); // Write 0xB6 to ACC_SOFTRESET(0x7E)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();

    // Soft Reset GYRO
    BMI088_GYRO_NS_L();
    BMI088_WriteReg(0x14, 0xB6); // Write 0xB6 to GYRO_SOFTRESET(0x14)
    HAL_Delay(30);
    BMI088_GYRO_NS_H();

    // Switch ACCEL to Normal Mode
    BMI088_ACCEL_NS_L();
    HAL_Delay(1);
    BMI088_WriteReg(0x7D, 0x04); // Write 0x04 to ACC_PWR_CTRL(0x7D)
    HAL_Delay(1);
    BMI088_ACCEL_NS_H();
}
void GetAccelData(uint8_t *return_data, uint8_t range_acc_receive, float *Accel_X_in_mg, float *Accel_Y_in_mg, float *Accel_Z_in_mg){
    int16_t Accel_X_int16=0, Accel_Y_int16=0, Accel_Z_int16=0;
    BMI088_ReadReg_ACCEL(0x12,return_data,7);
    Accel_X_int16=return_data[2]<<8 | return_data[1];
    Accel_Y_int16=return_data[4]<<8 | return_data[3];
    Accel_Z_int16=(return_data[6]-0x15)<<8 | return_data[5];
    *Accel_X_in_mg=Accel_X_int16*10*pow(2,range_acc_receive+1)*1.5/32768;
    *Accel_Y_in_mg=Accel_Y_int16*10*pow(2,range_acc_receive+1)*1.5/32768;
    *Accel_Z_in_mg=Accel_Z_int16*10*pow(2,range_acc_receive+1)*1.5/32768;
}
void GetGyroData(uint8_t *return_data, uint8_t range_gyro_receive, float *Rate_X, float *Rate_Y, float *Rate_Z)
{
    int16_t Rate_X_int16=0, Rate_Y_int16=0, Rate_Z_int16=0;
    BMI088_ReadReg_GYRO(0x02,return_data,6);
    Rate_X_int16=return_data[1]<<8 | return_data[0];
    Rate_Y_int16=return_data[3]<<8 | return_data[2];
    Rate_Z_int16=return_data[5]<<8 | return_data[4];
    *Rate_X=Rate_X_int16*1000*pow(2,-range_gyro_receive+1)/32768;
    *Rate_Y=Rate_Y_int16*1000*pow(2,-range_gyro_receive+1)/32768;
    *Rate_Z=Rate_Z_int16*1000*pow(2,-range_gyro_receive+1)/32768;
}
void BMI_Init()
{
    BMI088_Init();
    BMI088_ReadReg_GYRO(0x00,&rx_data, 1);
    BMI088_ACCEL_NS_L();
    BMI088_WriteReg(0x41,0x02);
    BMI088_ACCEL_NS_H();
    BMI088_ReadReg_ACCEL(0x41,&range_acc_receive,1);
}