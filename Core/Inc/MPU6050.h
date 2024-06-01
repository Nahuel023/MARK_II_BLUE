/*
 * MPU6050.h
 *
 *  Created on: Nov 18, 2023
 *      Author: Nahuel Medina
 */

#include "util.h"

#ifndef MPU6050_H_
#define MPU6050_H_

#define MPU6050_ADDR 		0xD0

#define SMPLRT_DIV_REG 		0x19
#define GYRO_CONFIG_REG 	0x1B
#define ACCEL_CONFIG_REG 	0x1C
#define ACCEL_XOUT_H_REG 	0x3B
#define TEMP_OUT_H_REG 		0x41
#define GYRO_XOUT_H_REG 	0x43
#define PWR_MGMT_1_REG 		0x6B
#define WHO_AM_I_REG 		0x75

#define OFFSET_AX 			450
#define OFFSET_AY 			450
#define OFFSET_AZ 			20000

#define OFFSET_GX 			450
#define OFFSET_GY 			350
#define OFFSET_GZ 			350

#define GRAVEDAD            9.81
#define MULTIPLICADORFLOAT  100

void MPU6050_Init ();
void MPU6050_Read_Accel ();
void MPU6050_Read_Gyro ();

#endif /* MPU6050_H_ */
