/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* SPDX-License-Identifier: Apache-2.0                                                                     */
/* Copyright(c) 2020 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/


#include "numicro_8051.h"
//***********************************************************************************************************
//  File Function: MUG51 I2C master mode demo code, the Slave address = 0xA4
//
//   ____________            _____________ 
//  |            |   SDA    |             |
//  |            |<-------->|             |
//  |            |          |             |
//  |ML51(M)     |          | ML51(S)     |
//  |(I2C_Master)|          | (I2C_Slave) |
//  |            |   SCL    |             |
//  |            |--------->|             |
//  |____________|          |_____________|
//
//  The protocol of I2C is master: start -> write 10 byte(ACK) ->stop -> start ->read 10 byte(ACK) -> stop
//***********************************************************************************************************

#define EEPROM_ADDRESS          0xA0
#define I2C_WR                     0
#define I2C_RD                     1

#define LOOP_SIZE                 10

unsigned int Tx_Addr = 0;
unsigned char Tx_Dat = 0;
unsigned int Rx_Addr = 0;
unsigned char Rx_Dat = 0;
BIT Write_End_Flag = 0;
BIT Read_End_Flag = 0;
BIT wdtintflag = 0;

unsigned char PinIntFlag;
unsigned char LED_ON=0, WDT_CNT=0;

/************************************************************************************************************/
/* FUNCTION_PURPOSE: ADC interrupt Service Routine                                                          */
/************************************************************************************************************/
void WDT_ISR (void)   interrupt 10
{
PUSH_SFRS;
    CLEAR_WDT_INTERRUPT_FLAG;
  /* Config Enable WDT reset and not clear couter trig reset */
    wdtintflag = 1;
POP_SFRS;
}

/************************************************************************************************************/
/* FUNCTION_PURPOSE: Timer0 interrupt Service Routine                                                       */
/************************************************************************************************************/
void Timer0_ISR (void) interrupt 1        // Vector @  0x0B
{
    PUSH_SFRS;
      SFRS = 0;
/* following setting for reload Timer 0 counter */
      TH0 = TH0TMP;
      TL0 = TL0TMP;
/* following clear flag for next interrupt */
      clr_TCON_TF0;
			LED_ON++;
			WDT_CNT++;
      
   POP_SFRS;
}

/******************************************************************************
Pin interrupt subroutine.
******************************************************************************/
void PinInterrupt_ISR (void) interrupt 7
{
PUSH_SFRS;

    SFRS = 0;
    switch(PIF)
    {
      case (SET_BIT0): PinIntFlag = SET_BIT0; PIF&=CLR_BIT0; break;
			
      default: break;
    }

POP_SFRS;
}

void (*I2C_Func)(void);
void I2C_ISR(void) interrupt 6
{
    I2C_Func();
}

/*========================================================================================================*/
void I2C0_Master_Tx_Isr(void)
{
    static uint8_t addr_flag = 0;
    static uint8_t count = 0;

PUSH_SFRS;
    SFRS = 0;
    printf ("\n I2C Transmit Interrupt" );
    printf("\n I2STAT = 0x%BD", I2STAT);
    switch (I2STAT)
    {
       /* Bus error */
       case 0x00: set_I2CON_STO; break;
        
      /* I2C start */
       case 0x08:
            clr_I2CON_STA;
            I2DAT = (EEPROM_ADDRESS | I2C_WR);
       break;

       /* Master Transmit Address ACK  */
       case 0x18:
            I2DAT = HIBYTE(Rx_Addr);          //address high byte of I2C EEPROM
            addr_flag = 1;
       break;

       /* Master Transmit Data ACK  */
       case 0x28:
            if(addr_flag)
            {
                I2DAT = LOBYTE(Tx_Addr);      //address low byte of I2C EEPROM
                addr_flag = 0;
                count = 0;
            }
            else
            {
                if(count == 0)
                {
                    I2DAT = Tx_Dat;
                    count++;
                }
                else
                {
                    Write_End_Flag = 1;
                    set_I2CON_STO;
                }
            }
        break;
    }

    I2C0_SI_Check();
    //while(STO);
POP_SFRS;
}

/*========================================================================================================*/
void I2C0_Master_Rx_Isr(void)
{
    static uint8_t addr_flag = 0;
    static uint8_t count = 0;
PUSH_SFRS;

    SFRS = 0;
    printf ("\n I2C Receive Interrupt" );
    printf("\n I2STAT = 0x%BD", I2STAT);
    switch (I2STAT)
    {
       /* Bus error */
       case 0x00: set_I2CON_STO; break;

      /* I2C start */
       case 0x08:
            clr_I2CON_STA;
            I2DAT = (EEPROM_ADDRESS | I2C_WR);
       break;

       /* Master Transmit Address ACK  */
       case 0x18:
            I2DAT = HIBYTE(Rx_Addr);          //address high byte of I2C EEPROM
            addr_flag = 1;
       break;

       /* Master Transmit Data ACK */ 
       case 0x28:
            if(addr_flag)
            {
                I2DAT = LOBYTE(Rx_Addr);       //address low byte of I2C EEPROM
                addr_flag = 0;
                count = 0;
            }
            else
            {
                set_I2CON_STA;
            }
       break;  

       /* Master Repeat Start  */
       case 0x10: 
           clr_I2CON_STA;
           I2DAT = (EEPROM_ADDRESS | I2C_RD);
       break;

      /* Master Receive Address ACK  */
       case 0x40:  set_I2CON_AA; break;
       
      /* Master Receive Data ACK  */
       case 0x50:
            Rx_Dat = I2DAT;
            set_I2CON_STO;
            Read_End_Flag = 1;
       break;
    }

    I2C0_SI_Check();
    //while(STO);

POP_SFRS;
}


//========================================================================================================
bit I2C0_Write(unsigned int u16I2Caddr, unsigned char u8I2Cdat)
{
    unsigned long count = 0;

    Write_End_Flag = 0;
    I2C_Func = I2C0_Master_Tx_Isr;
    Tx_Addr = u16I2Caddr;
    Tx_Dat = u8I2Cdat;

    SFRS=0; printf ("\n Write n24LC64 data 0x%bd", u8I2Cdat);
    set_I2CON_STA;             /* Start transmit */
    while(1)
    {
        count++;
        if(Write_End_Flag == 1)
        {
            return 1;
        }
        
        if(count > 100000)
        {
            return 0;
        }
    }
}

/*========================================================================================================*/
bit I2C0_Read(unsigned int u8I2Caddr, unsigned char *u8I2Cdat)
{
    uint32_t count = 0;
    Read_End_Flag = 0;
    I2C_Func = I2C0_Master_Rx_Isr;
    Rx_Addr = u8I2Caddr;
  
    SFRS=0; printf ("\n Receive data from n24LC64" );
    set_I2CON_STA; 
    
    while(1)
    {
        count++;
        if(Read_End_Flag == 1)
        {
            *u8I2Cdat = Rx_Dat;
            return 1;
        }
        
        if(count > 100000)
        {
            return 0;
        }
    }
}
/*========================================================================================================*/
void Init_I2C(void)
{
    /* Set I2C GPIO */
    P13_OPENDRAIN_MODE;          /* External pull high resister in circuit */
    P14_OPENDRAIN_MODE;          /* External pull high resister in circuit */
    P13_ST_ENABLE;               /* Setting schmit tigger mode */
    P14_ST_ENABLE;               /* Setting schmit tigger mode */

    /* Set I2C clock rate and enable*/
     I2C_Master_Open(24000000,100000);
    /* Set I2C Interrupt enable*/
     I2C_Interrupt(ENABLE);
}

/*========================================================================================================*/
void main(void)
{
//    uint8_t dat;
	
    MODIFY_HIRC(HIRC_24);  
    Enable_UART0_VCOM_printf_24M_115200();
    printf("\n UART initial...");
	
    P15_PUSHPULL_MODE; // Test PIN
		P15 = 0;
		SFRS = 0;
    printf("\n Test PIN initial...");
	

    P16_PUSHPULL_MODE; // LED
		P16 = 0;
		SFRS = 0;
    printf("\n LED initial...");
	

    P17_INPUT_MODE; // Key Input
    P17_PULLDOWN_ENABLE; 
    ENABLE_PIT0_P17_RISINGEDGE;
    ENABLE_PIN_INTERRUPT;
    /*GPIO_Pull_Enable(Port1,SET_BIT7,PULLDOWN);   */
    /*GPIO_EnableInt(PIT0,RISING,EDGE,Port1,7);   */
    SFRS = 0;
    printf("\n GPIO Input initial...");

    Timer0_AutoReload_Interrupt_Initial(24,1000); // Timer 1ms
		SFRS = 0;
    printf("\n Timer initial...");

    Init_I2C(); // I2C EEPROM
    SFRS = 0;
    printf("\n I2C EEPROM intial...");

		WDT_Open(64); // Watchdog 106.66ms time-out ; 64/(FIRC*Clock divider)
//	  WDT_Interrupt(ENABLE);
    SFRS = 0;
    printf("\n Watchdog intial...");
	
    ENABLE_GLOBAL_INTERRUPT;	
           

		while(1)
		{
			if(LED_ON==50) //100ms flash
			{
					LED_ON = 0;
					P16 ^= 1; //LED
			}
			if(WDT_CNT==64) //64ms clear watchdog count
			{
					WDT_CNT = 0;
					P15 ^= 1;
					WDT_Clear();
			}
			
			if(PinIntFlag == SET_BIT0) // Key pressed
			{
//					printf("\n I2C EEPROM Writing...");
					PinIntFlag = CLR_BIT0;
					I2C0_Write(0x0000, 0x55);
//					if(I2C0_Write(0x0000, 0x55) == 1)
//					{
//								
//								if(I2C0_Read(0x0000, &dat) == 1)
//								{
//										if(dat == 0x55)
//										{
//												SFRS=0; printf("\n EEPROM write and read Pass! ");
//										}
//										else
//										{
//												printf("\n FAIL! ");
//										}
//								}
//					 }						
			}
			
		}

}

