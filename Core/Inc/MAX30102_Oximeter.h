/*
 * MAX30102_Oximeter.h
 *
 *  Created on: Sep 1, 2024
 *      Author: denys
 */

#ifndef INC_MAX30102_OXIMETER_H_
#define INC_MAX30102_OXIMETER_H_


#include "main.h"
#include "i2c.h"

#define MAX30102_I2C_ADDRESS    0x57


// OXIMETER MAX30102 Success status
typedef enum
{
	OXIMETER_OK       = 0,
	OXIMETER_ERROR    = 1
} OXIMETER_Status;




// Check connection of OXIMETER MAX30102
//
// Returns 1 while OK | 0 while MAX30102 isn't connected properly
//
OXIMETER_Status MAX30102_CheckConnection();


// Reset OXIMETER MAX30102
void MAX30102_ResetSensor();


// Configure LED current (max for both LEDs)
//
// Returns 1 while OK | 0 while MAX30102 LEDs cannot be configured
//
OXIMETER_Status MAX30102_LEDConfig();


// Set SpO2 configuration
//
// Returns 1 while OK | 0 while MAX30102 SpO2 cannot be configured
//
OXIMETER_Status MAX30102_SetSpO2Mode();


// Slots determine the sequence of LED activations
//
// Returns 1 while OK | 0 while MAX30102 Slots cannot be configured
//
OXIMETER_Status MAX30102_SlotCofig();

// Reset FIFO pointers
//
// Returns 1 while OK | 0 while MAX30102 FIFO registers cannot be reseted
//
OXIMETER_Status MAX30102_ResetFIFOPointers();


// Start data collecting
OXIMETER_Status MAX30102_DataCollectTurnOn();


// This function measures Heart beats (pulse) and SpO2 (oxygen saturation in blood red cells)
void MAX30102_GetHeartbeatSPO2(uint8_t *spo2, uint32_t *heartbeat);


// ATTENTION! For this function Properties -> C\C++ Build -> Settings -> MCU/MPU Settings
//     Use float with printf
//     Use float with scanf
// Options should be turned on!
// This function gets temperature from MAX30102 sensor
float MAX30102_GetTemperature(void);

#endif /* INC_MAX30102_OXIMETER_H_ */
