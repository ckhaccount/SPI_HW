#include "IMU.h"
#include "main.h"
#include <stdint.h>
extern uint8_t return_acc_data[7];
extern uint8_t return_gyro_data[6];
extern uint8_t range_acc_receive;
extern uint8_t range_gyro;
extern float acc_x, acc_y, acc_z;
extern float gyro_x, gyro_y, gyro_z;
void  HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        GetAccelData(return_acc_data,range_acc_receive,&acc_x,&acc_y,&acc_z);
        GetGyroData(return_gyro_data,range_gyro,&gyro_x,&gyro_y,&gyro_z);
        HAL_GPIO_TogglePin(LED_G_GPIO_Port,LED_G_Pin);
    }
}