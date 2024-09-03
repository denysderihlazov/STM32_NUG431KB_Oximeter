/*
 * MAX30102_Oximeter.c
 *
 *  Created on: Sep 1, 2024
 *      Author: denys
 */


#include "MAX30102_Oximeter.h"

// Check connection of OXIMETER MAX30102
OXIMETER_Status MAX30102_CheckConnection()
{
	// Check if sensor is connected by reading part ID
	//
	// According to MAX30102 documentation part ID register has address 0xFF
	uint8_t part_id = 0;
	if(HAL_I2C_Mem_Read(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0xFF, I2C_MEMADD_SIZE_8BIT, &part_id, 1, HAL_MAX_DELAY))
	{
		return OXIMETER_OK; // MAX30102 is connected successfully
	}
	else
	{
		return OXIMETER_ERROR; // The sensor isn't connected properly or it has another part ID
	}
}


// Reset OXIMETER MAX30102
void MAX30102_ResetSensor()
{
	  uint8_t mode_reset = 0x40; // Reset bit
	  HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &mode_reset, 1, HAL_MAX_DELAY);
	  HAL_Delay(100); // Wait for reset
}


// Configure LED current (max for both LEDs)
OXIMETER_Status MAX30102_LEDConfig()
{
	  uint8_t led1_pa = 0x7F; // Red LED current
	  uint8_t led2_pa = 0x7F; // IR LED current

	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, &led1_pa, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR;
	  }


	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x0D, I2C_MEMADD_SIZE_8BIT, &led2_pa, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR;
	  }

	  return OXIMETER_OK;
}


// Set SpO2 configuration
OXIMETER_Status MAX30102_SetSpO2Mode()
{
	  // Set SpO2 by clearing reset bit and setting up sensor to measure oxygen saturation
	  uint8_t mode_config = 0x03; // Set to SpO2 mode
	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x09, I2C_MEMADD_SIZE_8BIT, &mode_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR;
	  }


	  // Set SpO2 configuration (ADC range, sample rate, pulse width)
	  uint8_t spo2_config = 0x27; // 0x27 sets sample rate to 100 Hz, pulse width to 411us
	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x0A, I2C_MEMADD_SIZE_8BIT, &spo2_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR;
	  }

	  return OXIMETER_OK;
}


// Slots determine the sequence of LED activations
OXIMETER_Status MAX30102_SlotCofig()
{
	  uint8_t slot_config = 0x21; // Slot 1 for Red, Slot 2 for IR
	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x11, I2C_MEMADD_SIZE_8BIT, &slot_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR; // Slot configuration failed
	  }

      return OXIMETER_OK; // Slot configuration succeeded
}


// Reset FIFO pointers
OXIMETER_Status MAX30102_ResetFIFOPointers()
{
	  uint8_t fifo_config = 0x00; // 0 to reset
	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x04, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR; // FIFO Write Pointer ERROR
	  }

	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x05, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR; // FIFO Overflow Counter ERROR
	  }

	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x06, I2C_MEMADD_SIZE_8BIT, &fifo_config, 1, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR; // FIFO Read Pointer ERROR
	  }

	  return OXIMETER_OK;
}


// Start data collecting
OXIMETER_Status MAX30102_DataCollectTurnOn()
{
	  uint8_t start_collect[2] =  {0x00, 0x01}; // Command to start data collection
	  if(HAL_I2C_Mem_Write(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x20, I2C_MEMADD_SIZE_8BIT, (uint8_t*)&start_collect, 2, HAL_MAX_DELAY) != HAL_OK)
	  {
		  return OXIMETER_ERROR; // Cannot collect data
	  }

	  return OXIMETER_OK;
}


// This function measures Heart beats (pulse) and SpO2 (oxygen saturation in blood red cells)
void MAX30102_GetHeartbeatSPO2(uint8_t *spo2, uint32_t *heartbeat)
{
	  uint8_t rbuf[8]; // Buffer to store the read data

	  // Read from the register where SPO2 and Heartbeat data are stored
	  HAL_I2C_Mem_Read(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x0C, I2C_MEMADD_SIZE_8BIT, rbuf, 8, HAL_MAX_DELAY);

	  // Extract SPO2 and Heartbeat data
	  *spo2 = rbuf[0]; // SPO2 level
	  *heartbeat = (rbuf[2] << 24) | (rbuf[3] << 16) | (rbuf[4] << 8) | rbuf[5]; // Heartbeat rate
}

// ATTENTION! For this function Properties -> C\C++ Build -> Settings -> MCU/MPU Settings
//     Use float with printf
//     Use float with scanf
// Options should be turned on!
float MAX30102_GetTemperature(void)
{
	  uint8_t temp_buf[2];
	  HAL_I2C_Mem_Read(&hi2c2, MAX30102_I2C_ADDRESS << 1, 0x14, I2C_MEMADD_SIZE_8BIT, temp_buf, 2, HAL_MAX_DELAY);

	  // Convert the temperature data
	  float temperature = temp_buf[0] + temp_buf[1] / 100.0;
	  return temperature;
}

