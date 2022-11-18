/****************************************Copyright (c)****************************************************
**                                 http://www.PowerAVR.com
**								   http://www.PowerMCU.com
**--------------File Info---------------------------------------------------------------------------------
** File name:           wdt.c
** Last modified Date:  2010-05-12
** Last Version:        V1.00
** Descriptions:        
**
**--------------------------------------------------------------------------------------------------------
** Created by:          PowerAVR
** Created date:        2010-05-10
** Version:             V1.00
** Descriptions:        ±àD′ê?ày′ú??
**
**--------------------------------------------------------------------------------------------------------       
*********************************************************************************************************/
#include "LPC17xx.h"
#include "type.h"
#include "IAP.h"

#define WDEN		0x00000001
#define WDRESET		0x00000002
#define WDTOF		0x00000004
#define WDINT		0x00000008

#define WDT_FEED_VALUE	0x09FFFFFF

extern void error_handle(void);   //1ê??′|àí
extern void WDT_IRQHandler(void);
extern uint32_t WDTInit( void );
extern void WDTFeed(void);

volatile uint32_t wdt_counter;

/*****************************************************************************
** Function name:		WDT_IRQHandler
**
** Descriptions:		Watchdog timer interrupt handler
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDT_IRQHandler(void) 
{
	uint32 i;
	
	LPC_WDT->WDMOD &= ~WDTOF;		/* clear the time-out terrupt flag */
 	wdt_counter= 0xFFFFFFFF;
	OnOff_Disable;
	SystemStatus = SystemProtect;
	MainErrorCode =30;							//记录故障为30
	alarm_time = RTCGetTime();					//打时间搓
	error_handle();              				//故障处理
 	led4_ON;
	led3_ON;
 	led2_ON;
 	led1_ON;
 	for(i=60000000;i>0;i--);
 	led3_OFF;
	for(i=60000000;i>0;i--);
	led4_OFF;
	for(i=60000000;i>0;i--);
	led1_OFF;
	for(i=60000000;i>0;i--);
	led2_OFF;
	LPC_WDT->WDMOD = WDRESET;
  //LPC_GPIO2->FIOCLR = 1<<12;    		//断电重启继电器
  	return;
}


/*****************************************************************************
** Function name:		WDTInit
**
** Descriptions:		Initialize watchdog timer, install the
**				watchdog timer interrupt handler
**
** parameters:			None
** Returned value:		true or false, return false if the VIC table
**				is full and WDT interrupt handler can be
**				installed.
** 
*****************************************************************************/
uint32_t WDTInit( void )
{
  wdt_counter = 0;

  NVIC_EnableIRQ(WDT_IRQn);

  LPC_WDT->WDTC = WDT_FEED_VALUE;	/* once WDEN is set, the WDT will start after feeding */
  LPC_WDT->WDMOD = WDEN;

  LPC_WDT->WDFEED = 0xAA;		/* Feeding sequence */
  LPC_WDT->WDFEED = 0x55;    
  return( 1 );
}

/*****************************************************************************
** Function name:		WDTFeed
**
** Descriptions:		Feed watchdog timer to prevent it from timeout
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void WDTFeed( void )
{
	/*LPC_UART0->IER = 0;
	LPC_SSP0->IMSC = 0x00;*/

  	LPC_WDT->WDFEED = 0xAA;		/* Feeding sequence */
  	LPC_WDT->WDFEED = 0x55;

	/*LPC_UART0->IER = 1;
	LPC_UART3->IER = 1; 
	LPC_SSP0->IMSC = 0x04;*/

  wdt_counter++;
  return;
}

/******************************************************************************
**                            End Of File
******************************************************************************/

