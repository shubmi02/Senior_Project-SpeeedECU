#include "can.h"

HAL_StatusTypeDef CAN_SendData(FDCAN_HandleTypeDef* can, uint32_t id, uint8_t *data, uint8_t len) {
	FDCAN_TxHeaderTypeDef txHeader;

	// Configure CAN header
	txHeader.Identifier = id;
	txHeader.IdType = FDCAN_STANDARD_ID;
	txHeader.TxFrameType = FDCAN_DATA_FRAME;
	txHeader.DataLength = FDCAN_DLC_BYTES_8;
	txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	txHeader.BitRateSwitch = FDCAN_BRS_OFF;
	txHeader.FDFormat = FDCAN_CLASSIC_CAN;
	txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	txHeader.MessageMarker = 0;

	// Transmit the CAN message
	return HAL_FDCAN_AddMessageToTxFifoQ(can, &txHeader, data);
}

void SendMotorCommand(FDCAN_HandleTypeDef* can, uint16_t torque, uint16_t speed, uint8_t direction, uint8_t inverter_en, uint16_t torque_limit)
{
    uint8_t motorData[8] = {0};

    motorData[0] = torque;
    motorData[1] = torque >> 8;
    motorData[2] = speed;
    motorData[3] = speed >> 8;
    motorData[4] = direction;
    motorData[5] = inverter_en;
    motorData[6] = torque_limit;
    motorData[7] = torque_limit >> 8;


    if (CAN_SendData(can, MOTOR_CONTROLLER_CAN_ADDRESS, motorData, 8) != HAL_OK)
    {
        Error_Handler();
    }
}

void SendSensorReading01(FDCAN_HandleTypeDef* can, uint32_t adc_value)
{
    uint8_t adcData[6] = {0};

    adcData[0] = 0; //
    adcData[1] = 0; //
    adcData[2] = adc_value; // TPS1 Voltage
    adcData[3] = adc_value >> 8; // TPS1 Voltage
    adcData[4] = adc_value >> 16; // TPS1 Voltage
    adcData[5] = adc_value >> 24; // TPS1 Voltage



    if (CAN_SendData(can, TPS1_CAN_ADDRESS, adcData, 6) != HAL_OK)
    {
        Error_Handler();
    }
}

void SendSensorReading02(FDCAN_HandleTypeDef* can, uint32_t adc_value)
{
    uint8_t adcData[6] = {0};

    adcData[0] = 0; //
    adcData[1] = 0; //
    adcData[2] = adc_value; // TPS1 Voltage
    adcData[3] = adc_value >> 8; // TPS1 Voltage
    adcData[4] = adc_value >> 16; // TPS1 Voltage
    adcData[5] = adc_value >> 24; // TPS1 Voltage



    if (CAN_SendData(can, TPS2_CAN_ADDRESS, adcData, 6) != HAL_OK)
    {
        Error_Handler();
    }
}




