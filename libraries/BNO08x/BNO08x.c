/*
 * BNO08x.c
 *
 *  Created on: May 16, 2025
 *      Author: David Evangelista
 */

#include "BNO8x.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

I2C_HandleTypeDef *hi2cBNO = NULL;

uint8_t START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ[] =
{ 0x15, 0x00, 0x02, 0x00, 0xFD, 0x28, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf
 * 6.5.4 Set Feature Command (0xFD)
 * Hint: 0x00002710 = 10000 us (100 Hz)
*/

uint8_t FishedOutMessage[BNO_MSG_LENGTH];
uint8_t BnoRxBuff[BNO_MSG_LENGTH];
int16_t Data1;
int16_t Data2;
int16_t Data3;
int16_t Data4;

// Yaw, Pitch, Roll in this sequence
double Orientation[3];
double Qi, Qj, Qk, Qr;

uint32_t BnoSoftTimer;

void BNO_08x_Init()
{
	HAL_I2C_Master_Transmit(hi2cBNO, BNO_I2C_ADDRESS, START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ, sizeof(START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ), 10);

	HAL_Delay(100);
	// Necessary for BNO085
	BnoSoftTimer = HAL_GetTick();

}
void setI2CInterface_BNO08x(I2C_HandleTypeDef *hi2c)
{
	hi2cBNO=hi2c;
}

double* get_Orientation()
{
  if (HAL_GetTick() - BnoSoftTimer > BNO_READ_PERIOD)
  {
		BnoSoftTimer = HAL_GetTick();

		HAL_I2C_Master_Receive(hi2cBNO, BNO_I2C_ADDRESS, BnoRxBuff, BNO_MSG_LENGTH, 10);

			if ((BnoRxBuff[9] == 0x28))
			/* https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf
			 * 6.5.42 ARVR-Stabilized Rotation Vector (0x28)
			 */
			{
				memcpy(FishedOutMessage, BnoRxBuff, BNO_MSG_LENGTH);
				Data1 = (((uint16_t) FishedOutMessage[14]) << 8)
						| FishedOutMessage[13];
				Data2 = (((uint16_t) FishedOutMessage[16]) << 8)
						| FishedOutMessage[15];
				Data3 = (((uint16_t) FishedOutMessage[18]) << 8)
						| FishedOutMessage[17];
				Data4 = (((uint16_t) FishedOutMessage[20]) << 8)
						| FishedOutMessage[19];

				Qi = ((double) Data2) * SCALE_Q(14);
				Qj = ((double) Data3) * SCALE_Q(14);
				Qk = ((double) Data4) * SCALE_Q(14);
				Qr = ((double) Data1) * SCALE_Q(14);

				Orientation[0] = ((double) atan2(2.0 * (Qi * Qj + Qk * Qr),
						(Qi * Qi - Qj * Qj - Qk * Qk + Qr * Qr)) * RAD_TO_DEG);
				Orientation[1] = ((double) asin(
								-2.0 * (Qi * Qk - Qj * Qr)
										/ (Qi * Qi + Qj * Qj + Qk * Qk + Qr * Qr)) * RAD_TO_DEG);
				Orientation[2] = ((double)atan2(2.0 * (Qj * Qk + Qi * Qr),
						(-Qi * Qi - Qj * Qj + Qk * Qk + Qr * Qr)) * RAD_TO_DEG);

			}
  }

  return Orientation;

}
