/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* SPDX-License-Identifier: Apache-2.0                                                                     */
/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#define LEVEL0   0
#define LEVEL1   1
#define LEVEL2   2
#define LEVEL3   3
#define LEVEL4   4

void VREF_Open(unsigned char u8VREFValue);
void VREF_Reload(unsigned char u8VREFtrimValue);
void VREF_Disable();