#ifndef INC_CAN_H_
#define INC_CAN_H_

#include "main.h"
#include "stm32h7xx_hal_fdcan.h"

#define MOTOR_CONTROLLER_CAN_ADDRESS 0xC0
#define TPS1_CAN_ADDRESS 0x500
#define TPS2_CAN_ADDRESS 0x501
#define BPS_CAN_ADDRESS 0x502


HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef* can, uint32_t id, uint8_t *data, uint8_t len);
void SendMotorCommand(FDCAN_HandleTypeDef* can, uint16_t torque, uint16_t speed, uint8_t direction, uint8_t inverter_en, uint16_t torque_limit);
void SendSensorReading01(FDCAN_HandleTypeDef* can, uint32_t adc_value);
void SendSensorReading02(FDCAN_HandleTypeDef* can, uint32_t adc_value);


#endif /* INC_CAN_H_ */
