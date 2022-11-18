/*****************************************************************************
 *  can.c:  CAN module API file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *	 说明：经汤涛以滤波FULLCAN模式改过
*****************************************************************************/
#include "lpc17xx.h"
#include "type.h"
#include "can.h"
extern uint8	cj_ID;
/* Receive Queue: one queue for each CAN port */
CAN_MSG MsgBuf_TX1, MsgBuf_TX2; /* TX and RX Buffers for CAN message */
volatile uint32_t	CANStatus;
extern int	dizhi;
extern uint8	gz_dan[1060];
uint32_t	address = 0;
uint32_t	i_bl;

/*****************************************************************************
** 函数名:		CAN_Handler
**
** 功能:		CAN中断函数
**
** 说明：       无
*****************************************************************************/
void CAN_IRQHandler(void)
{
// 	LPC_SSP0->IMSC = 4;
 	LPC_CAN2->CMR = 0x04;
 	LPC_CAN1->CMR = 0x04;
 	if(*((volatile uint32_t *)(0x4003811c)) == 0x07082082 + cj_ID*0x80)
 	{
  		if( (*((volatile uint32_t *)(0x40038120))>>16) == 0x0022 )//从机1参数上传使能
  		{
   			dizhi = *((volatile uint32_t *)(0x40038120)) & 0xffff;//读取参数首地址
   			MsgBuf_TX1.Frame = 0x00080000; // 11-bit, no RTR, DLC is 8 bytes 
   			MsgBuf_TX1.MsgID = 0x00000090 + cj_ID*0x80;
   			MsgBuf_TX1.DataA = (dizhi<<16) | (Parameter[dizhi]<<8) | (Parameter[dizhi+1]);
   			MsgBuf_TX1.DataB = ((Parameter[dizhi+2]<<24) | (Parameter[dizhi+3])<<16)| (Parameter[dizhi+4]<<8) | (Parameter[dizhi+5]);
   			CAN1_SendMessage( &MsgBuf_TX1 );//发送参数
   			(*((volatile uint32_t *)(0x4003811c))) = 0;//清零
  		}
 	}
 	if(*((volatile uint32_t *)(0x40038128))	== 0x07082083 + cj_ID*0x80)
 	{
  		dizhi = *((volatile uint32_t *)(0x4003812c)) & 0xffff;//读取参数首地址
  		MsgBuf_TX1.Frame = 0x00080000; // 11-bit, no RTR, DLC is 8 bytes 
  		MsgBuf_TX1.MsgID = 0x000000B8 + cj_ID*0x80;
  		MsgBuf_TX1.DataA = (gz_dan[0]<<24) | (gz_dan[dizhi*7-6]<<16) | (gz_dan[dizhi*7-5]<<8) | (gz_dan[dizhi*7-4]);					  
  		MsgBuf_TX1.DataB = (gz_dan[dizhi*7-3]<<24) | (gz_dan[dizhi*7-2]<<16) | (gz_dan[dizhi*7-1]<<8) | (gz_dan[dizhi*7]);					  
  		CAN1_SendMessage( &MsgBuf_TX1 );//发送参数																				  
  		(*((volatile uint32_t *)(0x40038128))) = 0;//清零
 	}
// 	LPC_SSP0->IMSC=0x04;	    //使能接受一半中断置位
} 
/******************************************************************************
** 函数名:		CAN_Init
**
** 功能:		CAN初始化
**
** 说明：       配置CAN管脚、使能CAN1和CAN2、设置CAN模式和波特率、开中断
******************************************************************************/
void CAN_INIT(void)
{
  	LPC_SC->PCONP|= ((1<<13) | (1<<14));  //使能CAN1和CAN2
  	LPC_CAN1->MOD = 1;                 //禁止CAN功能，进行寄存器初始化
  	LPC_CAN2->MOD = 1;
  	LPC_CAN1->IER = 0;
  	LPC_CAN2->IER = 0;	
  	LPC_CAN1->GSR = 0;
  	LPC_CAN2->GSR = 0;
  	LPC_CAN1->EWL = 0xFE;          //CAN错误累计峰值寄存器设置
  	LPC_CAN2->EWL = 0xFE;
  	//BitRate = Fcclk/(APBDIV * (BRP+1) * ((Tseg1+1)+(Tseg2+1)+1));
  	LPC_CAN1->BTR = 0x00230002;      //Fcclk=40MHz,Fpclk=20MHz,BRP=3,tseg1=6,tseg2=1;500K 40MHz
  	LPC_CAN2->BTR = 0x00230002;
   	LPC_CAN1->MOD = 0x0;             //CAN功能使能
    LPC_CAN2->MOD = 0x0;
	NVIC_EnableIRQ(CAN_IRQn);
	LPC_CAN1->IER=1;
  	LPC_CAN2->IER=1;
    LPC_CAN2->CMR|=(0x01<<2);    //释放接收缓冲区
  	LPC_CAN1->CMR|=(0x01<<2);
	//LPC_CANAF->AFMR=0x2;         //bypass验收过滤器旁路
}
/**********************************************************************************
** 函数名:		CAN_SetACCF_Lookup
**
** 功  能:		配置滤波模式下的ID查找表格
**              
** 说  明:      具体看PDF。如果一个表格的起始地址等于下一个表格的起始地址或表格终止
*               寄存器的数值，则该表格为空，在处理中将被忽略。表格的大小由其前后2个
*               表格起始地址寄存器的差值决定。例如，(SFF_GRP_sa) - (SFF_sa)为独立标
*               准帧标识符查找表格大小。若其值为0，即SFF_GRP_sa=SFF_sa，则独立标准              
*               帧标识符查找表格大小为0，在查找时此表格将被忽略。查找表结束寄存器              
*               ENDofTable代表查找表结束地址，(ENDofTable) - (EFF_GRP_sa)为扩展帧组              
*               标识符查找表大小。              
**********************************************************************************/
void CAN_SetACCF_Lookup( void )
{
 	uint32_t	ID_high, ID_low;
 	/* Set explicit standard Frame */
 	address=0;
 	*((volatile uint32_t *)(LPC_CANAF_RAM_BASE))  = (0x2000<<16) | (0x2081+cj_ID*0x80);//ID_low | ID_high;
 	address += 4;
 	*((volatile uint32_t *)(LPC_CANAF_RAM_BASE+address)) = ((0x2882+cj_ID*0x80)<<16) | (0x2883+cj_ID*0x80);//ID_low | ID_high;
 	address += 4;
 	for(i_bl=0; i_bl<62; i_bl++) 
 	{
  		ID_low = (0x2084+cj_ID*0x80+2*i_bl)<<16;
  		ID_high = 0x2084+cj_ID*0x80+2*i_bl+1;
  		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE+address )) = ID_low|ID_high;//将ID写入FULLCAN表格
  		address += 4;
 	}	 
 	LPC_CANAF->SFF_sa = address; //FullCAN（标准帧格式）标识符区
 	address += 4; 
 	LPC_CANAF->SFF_GRP_sa = address;//明确的标准帧格式标识符区	
 	/* Set group standard Frame */
 	/*for ( i = 0; i < ACCF_IDEN_NUM; i += 2 )
  	{
		ID_low = (i << 29) | (GRP_STD_ID << 16);
		ID_high = ((i+1) << 13) | (GRP_STD_ID << 0);
		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low | ID_high;
		address += 4; 
  	}
	*/   
  	/* Set explicit extended Frame */ 
 	LPC_CANAF->EFF_sa = address; //标准帧组格式标识符区
	/*for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  	{
		ID_low = (i << 29) | (EXP_EXT_ID << 0);
		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
		address += 4; 
 	}
	*/ 
  	/* Set group extended Frame */
 	LPC_CANAF->EFF_GRP_sa = address;// ; //明确的扩展帧格式标识符区
	/*  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  	{
		ID_low = (i << 29) | (GRP_EXT_ID << 0);
		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
		address += 4; 
 	}
  	*/
  	/* Set End of Table */
 	LPC_CANAF->ENDofTable = address; //扩展帧组格式标识符区
 	for(i_bl=0; i_bl<446; i_bl++)//用于将滤波RAM的数据清零
 	{
  		address+=4;
  		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE+address )) = 0;
 	}
 	return;
}
/******************************************************************************
** 函数名:		CAN_SetACCF
**
** 功  能:		设置滤波模式 
******************************************************************************/
void CAN_SetACCF( uint32_t ACCFMode )
{
	switch ( ACCFMode )
  	{
		case ACCF_OFF:
			LPC_CANAF->AFMR = ACCFMode;
	  		LPC_CAN1->MOD = LPC_CAN2->MOD = 1;	// Reset CAN
	  		LPC_CAN1->IER = LPC_CAN2->IER = 0;	// Disable Receive Interrupt
	  		LPC_CAN1->GSR = LPC_CAN2->GSR = 0;	// Reset error counter when CANxMOD is in reset
			break;
		case ACCF_BYPASS:
	  		LPC_CANAF->AFMR = ACCFMode;
			break;
		case ACCF_ON:
			break;
		case ACCF_FULLCAN:
	  		LPC_CANAF->AFMR = ACCF_OFF;//复位滤波，以便写入寄存器
	  		CAN_SetACCF_Lookup();		 //设置过滤表格
	  		LPC_CANAF->FCANIE |= 0x1;//开启FULLCAN中断
	  		LPC_CANAF->AFMR = ACCFMode;//开启FULLCAN模式
		default:break;
  	}
  	return;
}
/******************************************************************************
** 函数名:		CAN1_SendMessage
**
** 功  能:		CAN1发送	 
******************************************************************************/
uint32_t CAN1_SendMessage( CAN_MSG *pTxBuf )
{
 	uint32_t	CANStatus1;
  	CANStatus1 = LPC_CAN1->SR;
  	if ( CANStatus1 & 0x00000004 )
  	{
		LPC_CAN1->TFI1 = pTxBuf->Frame & 0xC00F0000;
		LPC_CAN1->TID1 = pTxBuf->MsgID;
		LPC_CAN1->TDA1 = pTxBuf->DataA;
		LPC_CAN1->TDB1 = pTxBuf->DataB;
		LPC_CAN1->CMR = 0x21;
		return ( TRUE );
  	}
  	else if ( CANStatus1 & 0x00000400 )
  	{
		LPC_CAN1->TFI2 = pTxBuf->Frame & 0xC00F0000;
		LPC_CAN1->TID2 = pTxBuf->MsgID;
		LPC_CAN1->TDA2 = pTxBuf->DataA;
		LPC_CAN1->TDB2 = pTxBuf->DataB;
		LPC_CAN1->CMR = 0x41;
		return ( TRUE );
  	}
  	else if ( CANStatus1 & 0x00040000 )
  	{	
		LPC_CAN1->TFI3 = pTxBuf->Frame & 0xC00F0000;
		LPC_CAN1->TID3 = pTxBuf->MsgID;
		LPC_CAN1->TDA3 = pTxBuf->DataA;
		LPC_CAN1->TDB3 = pTxBuf->DataB;
		LPC_CAN1->CMR = 0x81;
		return ( TRUE );
  	}
  	return ( FALSE );
}
/******************************************************************************
**                            End Of File
******************************************************************************/
