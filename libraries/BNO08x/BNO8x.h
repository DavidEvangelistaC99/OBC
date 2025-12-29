/*
 * BNO8x.h
 *
 *  Created on: May 16, 2025
 *      Author: David Evangelista
 */

#include "stm32h7xx_hal.h"

#ifndef INC_BNO8X_H_
#define INC_BNO8X_H_

#define RAD_TO_DEG 			(180.0/M_PI)
//#define SCALE_Q(n) (1.0 / (1 << n))
#define SCALE_Q(n) 			(pow(0.5, n))
// Address 0x4A (verification in ESP32)
#define BNO_I2C_ADDRESS 	(0x4A << 1) 	// Adafruit
//#define BNO_I2C_ADDRESS (0x4B << 1) 	// SparkFun BNO086
//#define BNO_I2C_HANDLE &hi2c1
#define BNO_MSG_LENGTH 			21
#define BNO_READ_PERIOD 		20 				// ms

// I2C variable
extern I2C_HandleTypeDef *hi2cBNO;

// Functions
void setI2CInterface_BNO08x(I2C_HandleTypeDef *hi2c);
void BNO_08x_Init();
double* get_Orientation();

#endif /* INC_BNO8X_H_ */
