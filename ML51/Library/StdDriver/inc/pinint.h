/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* SPDX-License-Identifier: Apache-2.0                                                                     */
/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#define PIT0  0
#define PIT1  1
#define PIT2  2
#define PIT3  3
#define PIT4  4
#define PIT5  5
#define PIT6  6
#define PIT7  7

#define LOW     0
#define HIGH    1
#define FALLING 0
#define RISING  1
#define BOTH    2
#define LEVEL   0
#define EDGE    1

void GPIO_EnableInt(unsigned char u8PIT, unsigned char u8IntStatus,unsigned char u8IntMode, unsigned char u8Port, unsigned char u8PinMask);