/*****************************************************************************
 *  can.c:  CAN module API file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *	 ˵�������������˲�FULLCANģʽ�Ĺ�
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
** ������:		CAN_Handler
**
** ����:		CAN�жϺ���
**
** ˵����       ��
*****************************************************************************/
void CAN_IRQHandler(void)
{
// 	LPC_SSP0->IMSC = 4;
 	LPC_CAN2->CMR = 0x04;
 	LPC_CAN1->CMR = 0x04;
 	if(*((volatile uint32_t *)(0x4003811c)) == 0x07082082 + cj_ID*0x80)
 	{
  		if( (*((volatile uint32_t *)(0x40038120))>>16) == 0x0022 )//�ӻ�1�����ϴ�ʹ��
  		{
   			dizhi = *((volatile uint32_t *)(0x40038120)) & 0xffff;//��ȡ�����׵�ַ
   			MsgBuf_TX1.Frame = 0x00080000; // 11-bit, no RTR, DLC is 8 bytes 
   			MsgBuf_TX1.MsgID = 0x00000090 + cj_ID*0x80;
   			MsgBuf_TX1.DataA = (dizhi<<16) | (Parameter[dizhi]<<8) | (Parameter[dizhi+1]);
   			MsgBuf_TX1.DataB = ((Parameter[dizhi+2]<<24) | (Parameter[dizhi+3])<<16)| (Parameter[dizhi+4]<<8) | (Parameter[dizhi+5]);
   			CAN1_SendMessage( &MsgBuf_TX1 );//���Ͳ���
   			(*((volatile uint32_t *)(0x4003811c))) = 0;//����
  		}
 	}
 	if(*((volatile uint32_t *)(0x40038128))	== 0x07082083 + cj_ID*0x80)
 	{
  		dizhi = *((volatile uint32_t *)(0x4003812c)) & 0xffff;//��ȡ�����׵�ַ
  		MsgBuf_TX1.Frame = 0x00080000; // 11-bit, no RTR, DLC is 8 bytes 
  		MsgBuf_TX1.MsgID = 0x000000B8 + cj_ID*0x80;
  		MsgBuf_TX1.DataA = (gz_dan[0]<<24) | (gz_dan[dizhi*7-6]<<16) | (gz_dan[dizhi*7-5]<<8) | (gz_dan[dizhi*7-4]);					  
  		MsgBuf_TX1.DataB = (gz_dan[dizhi*7-3]<<24) | (gz_dan[dizhi*7-2]<<16) | (gz_dan[dizhi*7-1]<<8) | (gz_dan[dizhi*7]);					  
  		CAN1_SendMessage( &MsgBuf_TX1 );//���Ͳ���																				  
  		(*((volatile uint32_t *)(0x40038128))) = 0;//����
 	}
// 	LPC_SSP0->IMSC=0x04;	    //ʹ�ܽ���һ���ж���λ
} 
/******************************************************************************
** ������:		CAN_Init
**
** ����:		CAN��ʼ��
**
** ˵����       ����CAN�ܽš�ʹ��CAN1��CAN2������CANģʽ�Ͳ����ʡ����ж�
******************************************************************************/
void CAN_INIT(void)
{
  	LPC_SC->PCONP|= ((1<<13) | (1<<14));  //ʹ��CAN1��CAN2
  	LPC_CAN1->MOD = 1;                 //��ֹCAN���ܣ����мĴ�����ʼ��
  	LPC_CAN2->MOD = 1;
  	LPC_CAN1->IER = 0;
  	LPC_CAN2->IER = 0;	
  	LPC_CAN1->GSR = 0;
  	LPC_CAN2->GSR = 0;
  	LPC_CAN1->EWL = 0xFE;          //CAN�����ۼƷ�ֵ�Ĵ�������
  	LPC_CAN2->EWL = 0xFE;
  	//BitRate = Fcclk/(APBDIV * (BRP+1) * ((Tseg1+1)+(Tseg2+1)+1));
  	LPC_CAN1->BTR = 0x00230002;      //Fcclk=40MHz,Fpclk=20MHz,BRP=3,tseg1=6,tseg2=1;500K 40MHz
  	LPC_CAN2->BTR = 0x00230002;
   	LPC_CAN1->MOD = 0x0;             //CAN����ʹ��
    LPC_CAN2->MOD = 0x0;
	NVIC_EnableIRQ(CAN_IRQn);
	LPC_CAN1->IER=1;
  	LPC_CAN2->IER=1;
    LPC_CAN2->CMR|=(0x01<<2);    //�ͷŽ��ջ�����
  	LPC_CAN1->CMR|=(0x01<<2);
	//LPC_CANAF->AFMR=0x2;         //bypass���չ�������·
}
/**********************************************************************************
** ������:		CAN_SetACCF_Lookup
**
** ��  ��:		�����˲�ģʽ�µ�ID���ұ��
**              
** ˵  ��:      ���忴PDF�����һ��������ʼ��ַ������һ��������ʼ��ַ������ֹ
*               �Ĵ�������ֵ����ñ��Ϊ�գ��ڴ����н������ԡ����Ĵ�С����ǰ��2��
*               �����ʼ��ַ�Ĵ����Ĳ�ֵ���������磬(SFF_GRP_sa) - (SFF_sa)Ϊ������
*               ׼֡��ʶ�����ұ���С������ֵΪ0����SFF_GRP_sa=SFF_sa���������׼              
*               ֡��ʶ�����ұ���СΪ0���ڲ���ʱ�˱�񽫱����ԡ����ұ�����Ĵ���              
*               ENDofTable������ұ������ַ��(ENDofTable) - (EFF_GRP_sa)Ϊ��չ֡��              
*               ��ʶ�����ұ��С��              
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
  		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE+address )) = ID_low|ID_high;//��IDд��FULLCAN���
  		address += 4;
 	}	 
 	LPC_CANAF->SFF_sa = address; //FullCAN����׼֡��ʽ����ʶ����
 	address += 4; 
 	LPC_CANAF->SFF_GRP_sa = address;//��ȷ�ı�׼֡��ʽ��ʶ����	
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
 	LPC_CANAF->EFF_sa = address; //��׼֡���ʽ��ʶ����
	/*for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  	{
		ID_low = (i << 29) | (EXP_EXT_ID << 0);
		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
		address += 4; 
 	}
	*/ 
  	/* Set group extended Frame */
 	LPC_CANAF->EFF_GRP_sa = address;// ; //��ȷ����չ֡��ʽ��ʶ����
	/*  for ( i = 0; i < ACCF_IDEN_NUM; i++  )
  	{
		ID_low = (i << 29) | (GRP_EXT_ID << 0);
		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE + address)) = ID_low;
		address += 4; 
 	}
  	*/
  	/* Set End of Table */
 	LPC_CANAF->ENDofTable = address; //��չ֡���ʽ��ʶ����
 	for(i_bl=0; i_bl<446; i_bl++)//���ڽ��˲�RAM����������
 	{
  		address+=4;
  		*((volatile uint32_t *)(LPC_CANAF_RAM_BASE+address )) = 0;
 	}
 	return;
}
/******************************************************************************
** ������:		CAN_SetACCF
**
** ��  ��:		�����˲�ģʽ 
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
	  		LPC_CANAF->AFMR = ACCF_OFF;//��λ�˲����Ա�д��Ĵ���
	  		CAN_SetACCF_Lookup();		 //���ù��˱��
	  		LPC_CANAF->FCANIE |= 0x1;//����FULLCAN�ж�
	  		LPC_CANAF->AFMR = ACCFMode;//����FULLCANģʽ
		default:break;
  	}
  	return;
}
/******************************************************************************
** ������:		CAN1_SendMessage
**
** ��  ��:		CAN1����	 
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
