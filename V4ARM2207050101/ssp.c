/*****************************************************************************
 *   Uart.c:  ����ARM����
 *   History
 *   2017.10.24  ver 1710240101
 *	 ����:	ARMӲ�����ð��� RTC��I2C��SSP0��Uart0��Uart1��Uart3�����账��
 	 ����:	SPI���ݽ��մ���SPI���Ͳ�������SPI�жϺ�����SPI��ʼ��
 	 �ж�:	���ļ����жϺ�������:��ʱ�жϣ�SSP�жϣ�Uart0�жϣ�Uart1�жϣ�Uart3�жϣ�Ӳ�������жϣ����Ź��ж�
*****************************************************************************/

#include "lpc17xx.h"
#include "type.h"
#include "IAP.h"
		
#define SendSpiBufMax   	108
#define	ReceSpiBufMax		120
#define SpiWrite_Disable	{LPC_GPIO2->FIOSET=1<<12;}		  	//��1����ʾSPIд��ʹ�ܣ�������Ч
#define SpiWrite_Enable		{LPC_GPIO2->FIOCLR=1<<12;}			//��0����ʾSPIдʹ��
#define	led1_ON				{LPC_GPIO3->FIOCLR=1<<25;}			//LED1
#define	led1_OFF			{LPC_GPIO3->FIOSET=1<<25;}			//LED1


extern void SPI_ReceiveProcess(void);//��DSP���ݽ��պ���	
extern uint8 SPI_CommumWithDSP(void);//��DSP���Ͳ�������
extern  volatile uint8 SPIRecvSuccess;					//SPI���ճɹ���־λ	
extern	uint16	APFStatus;								//�豸��ʾ״̬
extern	uint32 	TotalEnergyHB,TotalEnergyLB;			//���첹����
extern	uint8	NextReceFrameID;						//��һ֡����Ҫ����֡ͷ
extern  uint16	SPICommCount;							//SPI�ۼƼ���
extern  uint8	SPICommError;							//SPI�������
extern	uint16 	puyaotest;
extern  uint16 	SlaveErrorCode[16];						//�ӻ����ϴ���
extern	uint8		bluetooth_int;	//�ж��Ƿ���Ҫ��ʼ��
uint16  y_led =0;
uint16  RecvSpiBuf[ReceSpiBufMax];                    	//SPI�������ݻ���
uint16  SendSpiBuf[SendSpiBufMax];						//SPI�������ݻ���
uint16	Problem[100];
//�¶ȱ��
int16	temp_list[294]={-26,9466,-25,9003,-24,8568,-23,8157,-22,7770,-21,7404,
											-20,7058,-19,6730,-18,6418,-17,6122,-16,5841,-15,5573,
											-14,5317,-13,5075,-12,4843,-11,4622,-10,4412,-9,4212,
											-8,4021,-7,3840,-6,3668,-5,3504,-4,3348,-3,3200,
											-2,3060,-1,2928,  0,2802,  1,2683,  2,2570,  3,2463,
												4,2362,  5,2266,  6,2175,  7,2089,  8,2007,  9,1930,
											 10,1856, 11,1848, 12,1815, 13,1763, 14,1699, 15,1628,
											 16,1554, 17,1479, 18,1406, 19,1335, 20,1269, 21,1207,
											 22,1149, 23,1095, 24,1046, 25,1000, 26, 958, 27, 918,
											 28, 882, 29, 848, 30, 816, 31, 786, 32, 758, 33, 731,
											 34, 706, 35, 681, 36, 658, 37, 636, 38, 614, 39, 593,
											 40, 573, 41, 554, 42, 535, 43, 517, 44, 500, 45, 483,
											 46, 467, 47, 451, 48, 435, 49, 421, 50, 407, 51, 393,
											 52, 379, 53, 366, 54, 354, 55, 341, 56, 329, 57, 318,
											 58, 306, 59, 294, 60, 283, 61, 278, 62, 272, 63, 265,
											 64, 258, 65, 251, 66, 243, 67, 236, 68, 228, 69, 221,
											 70, 214, 71, 207, 72, 200, 73, 194, 74, 188, 75, 182,
											 76, 177, 77, 172, 78, 167, 79, 163, 80, 159, 81, 155,
											 82, 151, 83, 147, 84, 144, 85, 140, 86, 137, 87, 133,
											 88, 130, 89, 127, 90, 124, 91, 120, 92, 117, 93, 114,
											 94, 111, 95, 107, 96, 104, 97, 101, 98,  98, 99,  95,
											 100, 92,101,  89,102,  85,103,  83, 104, 81,105,  79,
											 106, 77, 107, 75,108,  73,109,  72, 110, 71,
												111,68, 112,66, 113,65, 114,63, 115,62, 116,61,
												117,59, 118,57, 119,56, 120,55										
												};

int16	temp_list1[258]={ -10,2312,  -9,2201,  -8,2095,
												-7,1996,  -6,1902,  -5,1813,  -4,1728,  -3,1649,  -2,1573,
											 -1,1502, 0,1434, 1,1370, 2,1309, 3,1251, 4,1196,
											 5,1144, 6,1094, 7,1047, 8,1003, 9,960, 10,920,
											 11,881, 12,845, 13,810, 14,777, 15, 745, 16, 715,
											 17, 689, 18, 659, 19, 633, 20, 608, 21, 584, 22, 562,
											 23, 540, 24, 519, 25, 500, 26, 481, 27, 463, 28, 445,
											 29, 429, 30, 413, 31, 398, 32, 383, 33, 370, 34, 356,
											 35, 344, 36, 331, 37, 320, 38, 309, 39, 298, 40, 288,
											 41, 278, 42, 268, 43, 259, 44, 250, 45, 242, 46, 234,
											 47, 226, 48, 219, 49, 212, 50, 205, 51, 198, 52, 192,
											 53, 186, 54, 180, 55, 174, 56, 169, 57, 163, 58, 158,
											 59, 153, 60, 149, 61, 144, 62, 140, 63, 136, 64, 131,
											 65, 128, 66, 124, 67, 120, 68, 117, 69, 113, 70, 110,
											 71, 107, 72, 104, 73, 101, 74, 98, 75, 95, 76, 93,
											 77, 90, 78, 87, 79, 85, 80, 83, 82, 78, 83, 76,
											 84, 74, 85, 72, 86, 70, 87, 68, 88,  67, 89,  65,
											 90, 63,91,  62,92,  60,93,  58, 94, 57,95,  55,
											 96, 54,97,  53,98,  51,99,  50, 100, 49,101,48,
											102,47,103,46,104,45,105,44,106,43,107,42,108,41,
											109,40,110,39,111,38,112,37,113,36,114,35,115,34,
											116,33,118,32,119,31,120,30			};



//��DSP			��������		��������					ѡ�β���				��Դ����					�ֶ���Դͨ��			�ֶ���ԴͶ��		���Ҳ���
uint16		dsp_data[100],	main_parameter[101],	Selected_parameter[51],		Passive_parameter[100],		ManualPassive_aisle[32]/*,ManualPassiveSwitch*/,Factory_parameter[100],FactoryEnable,PassiveSign,PassiveSign_aisle[16];
//���ز�	A����������			B����������			C����������						���ܵ���
uint16	load_correctionA[100],load_correctionB[100],load_correctionC[100],		IntPassive_aisle[32],spcial_parameter[100];
//ϵͳ��	A����������			B����������			C����������
//uint16	system_correctionA[50],system_correctionB[50],system_correctionC[50];

//ϵͳ��	A�������ֵ			B�������ֵ			C�������ֵ		A�������λ��		B�������ֵ		C�������ֵ
uint16	system_currentA[50],system_currentB[50],system_currentC[50],system_phaseA[50],system_phaseB[50],system_phaseC[50];
//���ز�	A�������ֵ			B�������ֵ			C�������ֵ		A�������λ��		B�������ֵ		C�������ֵ
uint16	load_currentA[50],	load_currentB[50],	load_currentC[50],	load_phaseA[50],	load_phaseB[50],load_phaseC[50];
//�����	A�������ֵ			B�������ֵ			C�������ֵ		A�������λ��		B�������ֵ		C�������ֵ
uint16	out_currentA[50],	out_currentB[50],	out_currentC[50],	out_phaseA[50],		out_phaseB[50],	out_phaseC[50];
//��ѹ		A���ѹ��ֵ			B���ѹ��ֵ			C���ѹ��ֵ		A���ѹ��λ��		B���ѹ��ֵ		C���ѹ��ֵ		
uint16		votageA[50]	,		votageB[50]	,		votageC[50]	,	votage_phaseA[50], votage_phaseB[50],votage_phaseC[50];
//���ݹ��	��������1��	��������2����������3	
uint16	capa[300];
//��DSP					��������			��ʼ������			

uint8	Eep_parameter[1600],Init_parameter[1600];/*,Eep_passiveparameter[224],Init_passiveparameter[224];*/

uint8 AI_select,enhance,ntc_type;//����ѡ�ο���,��ǿģʽ������Ч����0:�أ�������10s������	1�����������󲹳������ﵽ��ֵ������
//�ӻ�		��������			�����Сֵ			��������
uint16  slave_data[16][10],slave_Maxmin[16][10],slave_parameter[16][100]/*,slave_Enable[16],slave_Reset[16]*/;	

uint16	slave_Num;
uint16	CTSyscurA,CTSyscurB,CTSyscurC,CTOutcurA,CTOutcurB,CTOutcurC,Ubalance;
uint16	HighVolA,HighVolB,HighVolC;
uint32	ManualPassiveSwitch;//�ֶ���ԴͶ��


extern uint16 ini_version_a,ini_version_b,ini_version_flag;//Ĭ�ϳ�ʼ�汾��DSP��ʼ�汾����ʼ�汾���±�־��

//						��Ҫ������־λ					ѡ�β�����־λ				�û�������־λ			���Բ�����־λ		
extern volatile uint8	WriteMainParameterFlag,			WriteSelectedtimesFlag,	  WriteUserParameterFlag,/*WriteDebugParameterFlag,*/
//						У׼1������־λ					У׼2������־λ				У׼3������־λ					��Դ������־λ
					WriteCorrectionParameterFlag, 	WriteCorrectionParameter2Flag,WriteCorrectionParameter3Flag,	WritePassiveParameterFlag,
//						�ӻ�1������־λ					�ӻ�2������־λ				�ӻ�3������־λ					�ӻ�4������־λ	
					WriteSlaveParameterFlag,	 	WriteSlaveParameter2Flag,		WriteSlaveParameter3Flag,		WriteSlaveParameter4Flag,
//						�ӻ�5������־λ 				�ӻ�6������־λ 			�ӻ�7������־λ 				�ӻ�8������־λ 
					WriteSlaveParameter5Flag,	  	WriteSlaveParameter6Flag, 	WriteSlaveParameter7Flag,		WriteSlaveParameter8Flag,
//						�ӻ�9������־λ 				�ӻ�10������־λ			�ӻ�11������־λ				�ӻ�12������־λ 
					WriteSlaveParameter9Flag,		WriteSlaveParameter10Flag,	WriteSlaveParameter11Flag,		WriteSlaveParameter12Flag,
//						�ӻ�13������־λ				�ӻ�14������־λ			�ӻ�15������־λ				�ӻ�16������־λ 
					WriteSlaveParameter13Flag,		WriteSlaveParameter14Flag,	WriteSlaveParameter15Flag,		WriteSlaveParameter16Flag,
//						г��������־λ
					WriteHrmonicParameterFlag;

//*****************************************************��������־λ**********************************************************************//
//						��Ҫ������־λ					ѡ�β�����־λ				�û�������־λ					���Բ�����־λ		
extern	volatile uint8	ReadMainParameterFlag,			ReadSelectedtimesFlag,	ReadUserParameterFlag,	/*ReadDebugParameterFlag,*/
//						У׼1������־λ					У׼2������־λ				У׼3������־λ					��Դ������־λ
					ReadCorrectionParameterFlag,	ReadCorrectionParameter2Flag,ReadCorrectionParameter3Flag,	ReadPassiveParameterFlag,
//						�ӻ�1������־λ 				�ӻ�2������־λ 			�ӻ�3������־λ 				�ӻ�4������־λ 
					ReadSlaveParameterFlag,	  		ReadSlaveParameter2Flag, 	ReadSlaveParameter3Flag,		ReadSlaveParameter4Flag,
//						�ӻ�5������־λ 				�ӻ�6������־λ 			�ӻ�7������־λ 				�ӻ�8������־λ 
					ReadSlaveParameter5Flag,	 	ReadSlaveParameter6Flag, 	ReadSlaveParameter7Flag,		ReadSlaveParameter8Flag,
//						�ӻ�9������־λ 				�ӻ�10������־λ			�ӻ�11������־λ				�ӻ�12������־λ
					ReadSlaveParameter9Flag,		ReadSlaveParameter10Flag,	ReadSlaveParameter11Flag,		ReadSlaveParameter12Flag, 
//						�ӻ�13������־λ				�ӻ�14������־λ			�ӻ�15������־λ				�ӻ�16������־λ
					ReadSlaveParameter13Flag,		ReadSlaveParameter14Flag,	ReadSlaveParameter15Flag,		ReadSlaveParameter16Flag, 
//						г��������־λ
					ReadHrmonicParameterFlag;

volatile uint8 ParameterFinishFlag;

uint16 crc_chk_value(uint16 *data_value,uint16 tcrclength)   
{ 
	uint16	tii,tcrc_value;
 	tcrc_value = 0xffff;
 	while(tcrclength--)
 	{
  		tcrc_value ^= *data_value++;
  		for(tii=0; tii<16; tii++)
  		{ 
   			if(tcrc_value&0x0001) tcrc_value = (tcrc_value>>1)^0xa001;
   			else tcrc_value = tcrc_value>>1;
  		}
 	}
 	return(tcrc_value);
}
/******************************************************************************
** ������: Get_temp
**
** ���ܣ��¶ȼ���
**		  
** �÷���Get_temp(����ֵ)
******************************************************************************/
int16 Get_temp( int16 var_temp  )
{
	uint16  i;
	int16	value_temp;
	//var_list=22000*var_temp*3.3/4095/(5-var_temp*3.3/4095);
	
	if(ntc_type==0)//Ĭ�ϳ�ʼ���¶�ϵ��
	{
		for(i=0; i<294; i+=2)
		{
			if(i==292 )
			{
				value_temp=120;
			}
			else
			{
				if( var_temp<=temp_list[i+1] && var_temp>temp_list[i+3] )
				{
					value_temp=temp_list[i+2];
					break;
				}
			}
		}
	}
	else if(ntc_type==1)//690���¶�ϵ��
	{
		for(i=0; i<258; i+=2)
		{
			if(i==256)
			{
				value_temp=120;
			}
			else
			{
				if( var_temp<=temp_list1[i+1] && var_temp>temp_list1[i+3] )
				{
					value_temp=temp_list1[i+2];
					break;
				}
			}
		}
	}	
	return value_temp;
}

void SSP0_Init(void)
{
		uint16 temp;
 	//SSP�ӿ�����ΪSPIģʽ
 	LPC_SC->PCONP |=  (1 << 21);			//ʹ��SSPI0  
 	LPC_SC->PCLKSEL1 &= ~(3<<10);          	// PCLKSP1 = CCLK/4 (18MHz) ;PCLKSP1 = HCLK/8 (9MHz)
 	LPC_SC->PCLKSEL1 |=  (1<<10);          	// PCLKSP1 = CCLK   (72MHz)
 	LPC_SSP0->CR0 = (0x0F<<0)|  			//DSS ���ݳ��ȣ�0000-0010=����,0011=4λ,0111=8λ,1111=16λ
               		(0x00<<4)|  			//FRF ѡ��SPIģʽ 00=SPI 01=SSI 10=Microwire
			   		(0x00<<6)|  			//CPOL
			   		(0x00<<7)|  			//CPHA
			   		(0x00<<8);  			//SCR SPIʱ�ӷ�Ƶ
 	LPC_SSP0->CR1 = (0x00<<3)|  			//SOD �ӻ�������ܣ�1=��ֹ��0=����
               		(0x01<<2)|  			//MS ����ѡ��0=������1=�ӻ���
			   		(0x01<<1)|  			//SSE SSPʹ�ܣ�1=����SSP�������豸ͨ��
			   		(0x00<<0);  			//LBM ��дģʽ
 	//LPC_SSP0->CPSR=72;	    			//f=Fpck/(CPSR*(SCR+1) =72/72*1=1
	if((LPC_SSP0->SR & 0x04) != 0)
	{temp = LPC_SSP0->DR;}
	
 	LPC_SSP0->IMSC = 0x00;	    			//ʹ�ܽ���һ���ж���λ	 //����ж�

	SPIRecvSuccess=0;
	SPICommError =0;
	SPICommCount =0;
 	NVIC_EnableIRQ(SSP0_IRQn);
 	LPC_SSP0->IMSC = 0x04;			  		//��ֹ�ж�
}

uint16 GetSend_Word(uint16 data)
{
	uint16 temp;
	uint32 j;
	j=96000;
	LPC_SSP0->DR = data;
   // while((LPC_SSP0->SR & 0x10) == 0x10);	 	                            /* �ȴ�����֡�������	        */ 
	//for(i=0;i<0xfffffff0;i++)
	//{
	while((LPC_SSP0->SR & 0x04) == 0)
	{
		j--;
		if(j==0)
		{
			SPICommError=1;
			break;
		}
	};
	if(SPICommError==0)temp= LPC_SSP0->DR;	
																	/*	�ȴ����ݽ������			*/	
	return(temp);
}



/*void SSP0_IRQHandler()
{
 	uint16	i_ssp;
 	uint32	n_ssp;
 	LPC_SSP0->IMSC = 0x00;			  //��ֹ�ж�
 	LPC_UART0->IER =0;//		  ��ֹUART1�ж�
 	LPC_UART1->IER =0;//		  ��ֹUART1�ж�
	if(y_led==0)
	{
		LPC_GPIO3->FIOSET = 1<<25;
		y_led=1;
	}
	else
	{
		LPC_GPIO3->FIOCLR = 1<<25;
		y_led=0;
	}
	
 	ssp0[0] = LPC_SSP0->DR;
 	if(ssp0[0] == 0xaacc)
 	{
  		tang1=0;	
		for(n_ssp=0; n_ssp<0xfffffff0; n_ssp++)
		{
			if((LPC_SSP0->SR&0x04) != 0){n_ssp = 0xfffffff3;ssp0[1] = LPC_SSP0->DR;}
			tang1++;
			if( tang1==350000 && tang2==0 ){LPC_GPIO0->FIOCLR=1<<30;tang2=1;tang1=0;}
			if( tang1==350000 && tang2==1 ){LPC_GPIO0->FIOSET=1<<30;tang2=0;tang1=0;}
		}
  		if((ssp0[1] == 0x100)||(ssp0[1] == 0)||(ssp0[1] == 0x10)||(ssp0[1] == 1))
  		{
   			tang1=0;
			for(n_ssp=0; n_ssp<0xfffffff0; n_ssp++)
			{
				if((LPC_SSP0->SR&0x04) != 0){n_ssp = 0xfffffff3;ssp0[2] = LPC_SSP0->DR;}
				tang1++;
				if( tang1==350000 && tang2==0 ){LPC_GPIO0->FIOCLR=1<<30;tang2=1;tang1=0;}
				if( tang1==350000 && tang2==1 ){LPC_GPIO0->FIOSET=1<<30;tang2=0;tang1=0;}
			}
			tang1=0;
   		for(n_ssp=0; n_ssp<0xfffffff0; n_ssp++)
			{
				if((LPC_SSP0->SR&0x04) != 0){n_ssp = 0xfffffff3;ssp0[3] = LPC_SSP0->DR;}
				tang1++;
				if( tang1==350000 && tang2==0 ){LPC_GPIO0->FIOCLR=1<<30;tang2=1;tang1=0;}
				if( tang1==350000 && tang2==1 ){LPC_GPIO0->FIOSET=1<<30;tang2=0;tang1=0;}
			}
   		if( (ssp0[3]==20) && (ssp0[1]==1) )//�����д�����
   		{
				for(i_ssp=0; i_ssp<ssp0[3]+1; i_ssp++)
				{
	 				LPC_SSP0->DR = canshuzhong2[i_ssp];
					for(n_ssp=0; n_ssp<0xfffffff0; n_ssp++)					  
	     			{
	      				if((LPC_SSP0->SR&0x04) != 0){n_ssp = 0xfffffff3;ssp0[4+i_ssp] = LPC_SSP0->DR;}
	     			}
				}    
   		}
   		if((ssp0[3] == 100)||(ssp0[3] == 400))
   		{
				for(i_ssp=0; i_ssp<ssp0[3]+1; i_ssp++)
				{
	 				if(ssp0[1] == 0x100) LPC_SSP0->DR = canshu[ssp0[2]+i_ssp];
	 				tang1=0;
					for(n_ssp=0; n_ssp<0xfffffff0; n_ssp++)
					{
					 	if((LPC_SSP0->SR&0x04) != 0){n_ssp = 0xfffffff3;ssp0[4+i_ssp] = LPC_SSP0->DR;}
						tang1++;
						if( tang1==350000 && tang2==0 ){LPC_GPIO0->FIOCLR=1<<30;tang2=1;tang1=0;}
						if( tang1==350000 && tang2==1 ){LPC_GPIO0->FIOSET=1<<30;tang2=0;tang1=0;}\
					}
    			}
				if(ssp0[1] == 0)
				{
	 				pointer16 = ssp0;
					pointer16 += 4;
     				if(crc_chk_value(pointer16,ssp0[3]) == ssp0[ssp0[3]+4])
	 				{
	  					for(i_ssp=0; i_ssp<ssp0[3]; i_ssp++) dsp_tran[ssp0[2]+i_ssp] = ssp0[4+i_ssp];
	  					SPIRecvSuccess=1;
							
	 				}         		   
				}
   			}
  		}
 	}
 	LPC_SSP0->IMSC=0x04;	    //ʹ�ܽ���һ���ж���λ
 	if(ParametersConfigflag==0)
	{
		LPC_UART0->IER=0x01;
		LPC_UART1->IER =1;
	}
}
*/
void SSP0_IRQHandler()
{ 	
	uint8 	i=0;
	uint16	temp;
	uint32 	j=0;
	puyaotest =3;
 	LPC_SSP0->IMSC = 0x00;			  		//		  ��ֹ�ж�
 	//LPC_UART0->IER =0;						//		  ��ֹUART1�ж�
 	//LPC_UART3->IER =0;						//		  ��ֹUART1�ж�
	SPICommError =0;							//		���SPI����
	//if(SPIRecvSuccess ==0)
	{
		//�ж��Ƿ񴥷�SPI�ж�
		bluetooth_int=1;
		
		if(y_led==0)
		{
			//LPC_GPIO3->FIOSET = 1<<25;
			led1_ON
			y_led=1;
		}
		else
		{
			//LPC_GPIO3->FIOCLR = 1<<25;
			led1_OFF
			y_led=0;
		}														/*	�ȴ����ݽ������			*/	
		while((LPC_SSP0->SR & 0x04) != 0)
		{
			RecvSpiBuf[i++] = LPC_SSP0->DR;							//		 Catch FrameHead
		}

		if(RecvSpiBuf[0] == 0xaacc&&100 ==RecvSpiBuf[3])
		{	
			for(j=0;j<SendSpiBufMax;j++)
			{
				RecvSpiBuf[i++] = GetSend_Word(SendSpiBuf[j]);
				if(SPICommError==1)break;
				
			}
				//count =i;

			if(crc_chk_value(RecvSpiBuf+4,100)==RecvSpiBuf[104])
			{	
				SPI_ReceiveProcess();
				SPIRecvSuccess =1;
			}
		}
		else//���걾������
		{
			j=9600;
			while(1)
			{
				if((LPC_SSP0->SR & 0x04) == 0)
				{
					j--;
					if(j==0)break;
				}
				else 
				{
					temp = LPC_SSP0->DR;
					j=9600;
				}
			}
		}
	}
	/*else//���걾������
	{
		j=9600;
		while(1)
		{
			if((LPC_SSP0->SR & 0x04) == 0)
			{
				j--;
				if(j==0)break;
			}
			else 
			{
				temp = LPC_SSP0->DR;
				j=9600;
			}
		}
	}*/

	LPC_SSP0->IMSC=0x04;
	//LPC_UART0->IER =1;						//		  ��ֹUART0�ж�
	//ClrRevFIFO();
 	//LPC_UART1->IER =1;						//		  ��ֹUART1�ж�
	//ClrRevFIFO1();
 	puyaotest =4;
}
/******************************************************************************
** ������: SPI_ReceiveProcess
**
** ���ܣ�DSP��ARM������
******************************************************************************/
void SPI_ReceiveProcess(void)
{
	uint8 	i,WriteSPIFailure=0;
	int16	tmp16;

	//APFStatus= RecvSpiBuf[1];

	if(RecvSpiBuf[2]<9999)				//  data Address
	{
		if(RecvSpiBuf[2] ==0)
		{
			for(i=0;i<100;i++)
			{
				dsp_data[i] =RecvSpiBuf[4+i];
			}
			if(ParameterFinishFlag==1)
			{
				TotalEnergyHB =(uint32)((dsp_data[73]<<16)|dsp_data[74]);
				TotalEnergyLB =(uint32)((dsp_data[75]<<16)|dsp_data[76]);
			}
		}
		else if(RecvSpiBuf[2] ==100)
		{
			for(i=0;i<50;i++)
			{
				system_currentA[i]	=RecvSpiBuf[4+i];		//ϵͳA �������ֵ
				system_phaseA[i] 	=RecvSpiBuf[4+i+50];	//ϵͳA �������λ��
			}
			//NextAddressID =16;
		}
		else if(RecvSpiBuf[2] ==200)
		{
			for(i=0;i<50;i++)
			{
				system_currentB[i]	=RecvSpiBuf[4+i];		//ϵͳB �������ֵ
				system_phaseB[i] 	=RecvSpiBuf[4+i+50];	//ϵͳB �������λ��
			}
			//NextAddressID =17;
		}
		else if(RecvSpiBuf[2] ==300)
		{
			for(i=0;i<50;i++)
			{
				system_currentC[i]	=RecvSpiBuf[4+i];
				system_phaseC[i]	=RecvSpiBuf[4+i+50];
			}
			//NextAddressID =18;
		}
		else if(RecvSpiBuf[2] ==400)
		{
			for(i=0;i<50;i++)
			{
				load_currentA[i] =RecvSpiBuf[4+i];
				load_phaseA[i]	 =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =19;
		}
		else if(RecvSpiBuf[2] ==500)
		{
			for(i=0;i<50;i++)
			{
				load_currentB[i] =RecvSpiBuf[4+i];
				load_phaseB[i] 	 =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =20;
		}
		else if(RecvSpiBuf[2] ==600)
		{
			for(i=0;i<50;i++)
			{
				load_currentC[i] =RecvSpiBuf[4+i];
				load_phaseC[i] 	 =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =21;
		}
		else if(RecvSpiBuf[2] ==700)
		{
			for(i=0;i<50;i++)
			{
				out_currentA[i] =RecvSpiBuf[4+i];
				out_phaseA[i] 	=RecvSpiBuf[4+i+50];
			}
			//NextAddressID =22;
		}
		else if(RecvSpiBuf[2] ==800)
		{
			for(i=0;i<50;i++)
			{
				out_currentB[i] =RecvSpiBuf[4+i];
				out_phaseB[i] 	=RecvSpiBuf[4+i+50];
			}
			//NextAddressID =23;
		}
		else if(RecvSpiBuf[2] ==900)
		{
			for(i=0;i<50;i++)
			{
				out_currentC[i] =RecvSpiBuf[4+i];
				out_phaseC[i] 	=RecvSpiBuf[4+i+50];
			}
			//NextAddressID =24;
		}
		else if(RecvSpiBuf[2] ==1000)
		{
			for(i=0;i<50;i++)
			{
				votageA[i] =RecvSpiBuf[4+i];
				votage_phaseA[i] =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =25;
		}
		else if(RecvSpiBuf[2] ==1100)
		{
			for(i=0;i<50;i++)
			{
				votageB[i] =RecvSpiBuf[4+i];
				votage_phaseB[i] =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =26;
		}
		else if(RecvSpiBuf[2] ==1200)
		{
			for(i=0;i<50;i++)
			{
				votageC[i] =RecvSpiBuf[4+i];
				votage_phaseC[i] =RecvSpiBuf[4+i+50];
			}
			//NextAddressID =27;
		}
		else if(RecvSpiBuf[2] ==1300)
		{
			for(i=0;i<100;i++)
			{
				capa[i] =RecvSpiBuf[4+i];
			}
			//NextAddressID =28;
		}
		else if(RecvSpiBuf[2] ==1400)
		{
			for(i=0;i<100;i++)
			{
				capa[100+i] =RecvSpiBuf[4+i];
			}
			//NextAddressID =29;
		}
		else if(RecvSpiBuf[2] ==1500)
		{
			for(i=0;i<100;i++)
			{
				capa[200+i] =RecvSpiBuf[4+i];
			}
			//NextAddressID =30;
		}
		else if(RecvSpiBuf[2] ==2000)
		{
			for(i=0;i<10;i++)
			{	
				slave_data[0][i] =RecvSpiBuf[4+i];
				slave_data[1][i] =RecvSpiBuf[4+i+10];
				slave_data[2][i] =RecvSpiBuf[4+i+20];
				slave_data[3][i] =RecvSpiBuf[4+i+30];
				slave_data[4][i] =RecvSpiBuf[4+i+40];
				slave_data[5][i] =RecvSpiBuf[4+i+50];
				slave_data[6][i] =RecvSpiBuf[4+i+60];
				slave_data[7][i] =RecvSpiBuf[4+i+70];
				slave_data[8][i] =RecvSpiBuf[4+i+80];
				slave_data[9][i] =RecvSpiBuf[4+i+90];
			}
			/*for(i=0;i<10;i++)
			{
				//tmp16 = slave_data[i][2]*10;
				slave_data[2][i] = slave_data[0][i] ;			// ɢ�����¶� fortest
			}
			for(i=0;i<10;i++)
			{
				//tmp16 = slave_data[i][2]*10;
				slave_data[3][i] = slave_data[1][i] ;			// ɢ�����¶� fortest
			}*/
			for(i=0;i<10;i++)
			{
				//if(i==2||i==3)continue;
				tmp16 = (slave_data[i][2]);
				slave_data[i][2] = Get_temp(tmp16)*10;			// ɢ�����¶�
			}
		
			//NextAddressID =31;
		}
		else if(RecvSpiBuf[2] ==2100)
		{
			for(i=0;i<10;i++)
			{	
				slave_Maxmin[0][i] =RecvSpiBuf[4+i];
				slave_Maxmin[1][i] =RecvSpiBuf[4+i+10];
				slave_Maxmin[2][i] =RecvSpiBuf[4+i+20];
				slave_Maxmin[3][i] =RecvSpiBuf[4+i+30];
				slave_Maxmin[4][i] =RecvSpiBuf[4+i+40];
				slave_Maxmin[5][i] =RecvSpiBuf[4+i+50];
				slave_Maxmin[6][i] =RecvSpiBuf[4+i+60];
				slave_Maxmin[7][i] =RecvSpiBuf[4+i+70];
				slave_Maxmin[8][i] =RecvSpiBuf[4+i+80];
				slave_Maxmin[9][i] =RecvSpiBuf[4+i+90];
			}
			for(i=0;i<10;i++)
			{
				tmp16 =(slave_Maxmin[i][8]);
				slave_Maxmin[i][8] = Get_temp(tmp16)*10;		// ɢ�����¶�����
			}
			for(i=0;i<10;i++)
			{
				tmp16 = (slave_Maxmin[i][9]);
				slave_Maxmin[i][9] = Get_temp(tmp16)*10;		// ɢ�����¶�����
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2] ==2200)
		{
			for(i=0;i<10;i++)
			{	
				slave_data[10][i] =RecvSpiBuf[4+i];
				slave_data[11][i] =RecvSpiBuf[4+i+10];
				slave_data[12][i] =RecvSpiBuf[4+i+20];
				slave_data[13][i] =RecvSpiBuf[4+i+30];
				slave_data[14][i] =RecvSpiBuf[4+i+40];
				slave_data[15][i] =RecvSpiBuf[4+i+50];
			}
			for(i=10;i<16;i++)
			{
				//if(i==2||i==3)continue;
				tmp16 = (slave_data[i][2]);
				slave_data[i][2] = Get_temp(tmp16)*10;			// ɢ�����¶�
			}
			for(i=0;i<6;i++)
			{
				SlaveErrorCode[i+10] =RecvSpiBuf[4+i+60];
			}
			CTSyscurA =RecvSpiBuf[70];
			CTSyscurB =RecvSpiBuf[71];
			CTSyscurC =RecvSpiBuf[72];
			CTOutcurA =RecvSpiBuf[73];
			CTOutcurB =RecvSpiBuf[74];
			CTOutcurC =RecvSpiBuf[75];
			Ubalance  =RecvSpiBuf[76];
			HighVolA  =RecvSpiBuf[77];
			HighVolB  =RecvSpiBuf[78];
			HighVolC  =RecvSpiBuf[79];
		}
		else if(RecvSpiBuf[2] ==2300)
		{
			for(i=0;i<10;i++)
			{	
				slave_Maxmin[10][i] =RecvSpiBuf[4+i];
				slave_Maxmin[11][i] =RecvSpiBuf[4+i+10];
				slave_Maxmin[12][i] =RecvSpiBuf[4+i+20];
				slave_Maxmin[13][i] =RecvSpiBuf[4+i+30];
				slave_Maxmin[14][i] =RecvSpiBuf[4+i+40];
				slave_Maxmin[15][i] =RecvSpiBuf[4+i+50];		
			}
			for(i=10;i<16;i++)
			{
				tmp16 =(slave_Maxmin[i][8]);
				slave_Maxmin[i][8] = Get_temp(tmp16)*10;		// ɢ�����¶�����
			}
			for(i=10;i<16;i++)
			{
				tmp16 = (slave_Maxmin[i][9]);
				slave_Maxmin[i][9] = Get_temp(tmp16)*10;		// ɢ�����¶�����
			}
		}
	}
	else if(RecvSpiBuf[2]==10000)								//  main_parameter Address
	{
		for(i=0;i<100;i++)
		{
			if(main_parameter[i] !=RecvSpiBuf[4+i])						//spi���ܵ�������������һ��
			{
				WriteSPIFailure =1;
			}
		}
		if(WriteSPIFailure!=1)
		{
			WriteMainParameterFlag=0;
			SPICommCount =0;
			SPICommError =0;
		}
		else 
		{
			SPICommCount++;
			if(SPICommCount>1000)
			{
				WriteMainParameterFlag =0;
				SPICommCount =0;
				SPICommError =1;
			}
		}
	}
	else if(RecvSpiBuf[2]==10100||RecvSpiBuf[2]==10200||RecvSpiBuf[2]==10300||RecvSpiBuf[2]==10400||RecvSpiBuf[2]==11000)//  correction_parameter Address
	{
		if(RecvSpiBuf[2]==10100)
		{
			for(i=0;i<100;i++)
			{
				if(load_correctionA[i] !=RecvSpiBuf[4+i])
				{
					WriteSPIFailure =1;
				}
			}
			if(WriteSPIFailure!=1)
			{
				WriteCorrectionParameterFlag=0;
				SPICommCount =0;
				SPICommError =0;
			}
			else 
			{
				SPICommCount++;
				if(SPICommCount>1000)
				{
					WriteCorrectionParameterFlag =0;
					SPICommCount =0;
					SPICommError =1;
				}
			}
		}
		else if(RecvSpiBuf[2]==10200)
		{
			for(i=0;i<100;i++)
			{
				if(load_correctionB[i] !=RecvSpiBuf[4+i])
				{
					WriteSPIFailure =1;
				}
			}
			if(WriteSPIFailure!=1)
			{
				WriteCorrectionParameter2Flag=0;
				SPICommCount =0;
				SPICommError =1;
			}
			else 
			{
				SPICommCount++;
				if(SPICommCount>1000)
				{
					WriteCorrectionParameter2Flag =0;
					SPICommCount =0;
					SPICommError =1;
				}
			}
		}
		else if(RecvSpiBuf[2]==10300)
		{
			for(i=0;i<100;i++)
			{
				if(load_correctionC[i] !=RecvSpiBuf[4+i])
				{
					WriteSPIFailure =1;
				}
				if(WriteSPIFailure!=1)
				{
					WriteCorrectionParameter3Flag=0;
					SPICommCount =0;
					SPICommError =1;
				}
				else 
				{
					SPICommCount++;
					if(SPICommCount>1000)
					{
						WriteCorrectionParameter3Flag =0;
						SPICommCount =0;
						SPICommError =1;
					}
				}
			}
			//NextAddressID =4;
		}
		else if(RecvSpiBuf[2]==10400)
		{
			for(i=0;i<100;i++)
			{
				if(Passive_parameter[i] !=RecvSpiBuf[4+i])
				{
					WriteSPIFailure =1;
				}
				if(WriteSPIFailure!=1)
				{
					WritePassiveParameterFlag=0;
					SPICommCount =0;
					SPICommError =1;
				}
				else 
				{
					SPICommCount++;
					if(SPICommCount>1000)
					{
						WritePassiveParameterFlag =0;
						SPICommCount =0;
						SPICommError =1;
					}
				}
			}
			//NextAddressID =4;
		}
		else if(RecvSpiBuf[2]==11000)
		{
			if((TotalEnergyHB !=((RecvSpiBuf[4]<<16)|RecvSpiBuf[5]))||(TotalEnergyLB!=((RecvSpiBuf[6]<<16)|RecvSpiBuf[7])))
			{
				WriteSPIFailure =1;
				//TotalEnergyHB =((RecvSpiBuf[4]<<16)|RecvSpiBuf[5]);
				//TotalEnergyLB =((RecvSpiBuf[6]<<16)|RecvSpiBuf[7]);
			}
			if(WriteSPIFailure!=1)	
			{
				WriteUserParameterFlag=0;
				SPICommCount =0;
				SPICommError =1;
			}
			else 
			{
				SPICommCount++;
				if(SPICommCount>1000)
				{
					WriteUserParameterFlag =0;
					SPICommCount =0;
					SPICommError =1;
				}
			}
		}
	}
	else if(  RecvSpiBuf[2]==12000
			||RecvSpiBuf[2]==12100
			||RecvSpiBuf[2]==12200
			||RecvSpiBuf[2]==12300
			||RecvSpiBuf[2]==12400
			||RecvSpiBuf[2]==12500
			||RecvSpiBuf[2]==12600
			||RecvSpiBuf[2]==12700
			||RecvSpiBuf[2]==12800
			||RecvSpiBuf[2]==12900
			||RecvSpiBuf[2]==13000
			||RecvSpiBuf[2]==13100
			||RecvSpiBuf[2]==13200
			||RecvSpiBuf[2]==13300
			||RecvSpiBuf[2]==13400
			||RecvSpiBuf[2]==13500
			)															//  slave_parameter Address
	{
		if(RecvSpiBuf[2]==12000)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameterFlag==1)
				{
					if(slave_parameter[0][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameterFlag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameterFlag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else 
				{
					ReadSlaveParameterFlag=0;
					slave_parameter[0][i] =RecvSpiBuf[4+i];
					if(slave_parameter[0][99]!=0)
					{
						ParameterFinishFlag =1;
						//�ж�Ĭ�ϲ����汾��
						ini_version_b=slave_parameter[0][95];
												
					}
				}
			}
			//NextAddressID =5;
		}
		else if(RecvSpiBuf[2]==12100)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter2Flag==1)
				{
					if(slave_parameter[1][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter2Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter2Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}	
				}
				else
				{
					ReadSlaveParameter2Flag=0;
					slave_parameter[1][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =6;
		}
		else if(RecvSpiBuf[2]==12200)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter3Flag==1)
				{
					if(slave_parameter[2][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter3Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter3Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter3Flag=0;
					slave_parameter[2][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =7;
		}
		else if(RecvSpiBuf[2]==12300)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter4Flag==1)
				{
					if(slave_parameter[3][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter4Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter4Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter4Flag=0;
					slave_parameter[3][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =8;
		}
		else if(RecvSpiBuf[2]==12400)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter5Flag==1)
				{
					if(slave_parameter[4][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter5Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter5Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter5Flag=0;
					slave_parameter[4][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =9;
		}
		else if(RecvSpiBuf[2]==12500)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter6Flag==1)
				{
					if(slave_parameter[5][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter6Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter6Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter6Flag=0;
					slave_parameter[5][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =10;
		}
		else if(RecvSpiBuf[2]==12600)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter7Flag==1)
				{
					if(slave_parameter[6][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter7Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter7Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter7Flag=0;
					slave_parameter[6][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =11;
		}
		else if(RecvSpiBuf[2]==12700)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter8Flag==1)
				{
					if(slave_parameter[7][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter8Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter8Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter8Flag=0;
					slave_parameter[7][i] =RecvSpiBuf[4+i];
				}
			}
		}
		else if(RecvSpiBuf[2]==12800)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter9Flag==1)
				{
					if(slave_parameter[8][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter9Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter9Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter9Flag=0;
					slave_parameter[8][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =13;
		}
		else if(RecvSpiBuf[2]==12900)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter10Flag==1)
				{
					if(slave_parameter[9][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter10Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter10Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter10Flag=0;
					slave_parameter[9][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2]==13000)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter11Flag==1)
				{
					if(slave_parameter[10][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter11Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter11Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter11Flag=0;
					slave_parameter[10][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2]==13100)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter12Flag==1)
				{
					if(slave_parameter[11][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter12Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter12Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter12Flag=0;
					slave_parameter[11][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2]==13200)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter13Flag==1)
				{
					if(slave_parameter[12][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter13Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter13Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter13Flag=0;
					slave_parameter[12][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2]==13300)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter14Flag==1)
				{
					if(slave_parameter[13][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter14Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter14Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter14Flag=0;
					slave_parameter[13][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		else if(RecvSpiBuf[2]==13400)
		{
			for(i=0;i<100;i++)
			{
				if(WriteSlaveParameter15Flag==1)
				{
					if(slave_parameter[14][i] !=RecvSpiBuf[4+i])
					{
						WriteSPIFailure =1;
					}
					if(WriteSPIFailure!=1)
					{
						WriteSlaveParameter15Flag=0;
						SPICommCount =0;
						SPICommError =1;
					}
					else 
					{
						SPICommCount++;
						if(SPICommCount>1000)
						{
							WriteSlaveParameter15Flag =0;
							SPICommCount =0;
							SPICommError =1;
						}
					}
				}
				else
				{
					ReadSlaveParameter15Flag=0;
					slave_parameter[14][i] =RecvSpiBuf[4+i];
				}
			}
			//NextAddressID =14;
		}
		// else if(RecvSpiBuf[2]==13500)
		// {
		// 	for(i=0;i<100;i++)
		// 	{
		// 		if(WriteSlaveParameter16Flag==1)
		// 		{
		// 			if(slave_parameter[15][i] !=RecvSpiBuf[4+i])
		// 			{
		// 				WriteSPIFailure =1;
		// 			}
		// 			if(WriteSPIFailure!=1)
		// 			{
		// 				WriteSlaveParameter16Flag=0;
		// 				SPICommCount =0;
		// 				SPICommError =1;
		// 			}
		// 			else 
		// 			{
		// 				SPICommCount++;
		// 				if(SPICommCount>1000)
		// 				{
		// 					WriteSlaveParameter16Flag =0;
		// 					SPICommCount =0;
		// 					SPICommError =1;
		// 				}
		// 			}
		// 		}
		// 		else
		// 		{
		// 			ReadSlaveParameter16Flag=0;
		// 			slave_parameter[15][i] =RecvSpiBuf[4+i];
		// 		}
		// 	}
		// 	//NextAddressID =14;
		// }
	}
//	return(NextAddressID);
	else if(RecvSpiBuf[2]==60000)								//  main_parameter Address
	{	
		WriteSpcialParameterFlag=0;
		SPICommCount =0;
		SPICommError =0;
	}
}
/******************************************************************************
** ������: SPI_CommumWithDSP
**
** ���ܣ�ARMװ�ظ�DSP���͵�����
******************************************************************************/
uint8 SPI_CommumWithDSP(void)
{
	uint8 i,CurrentAddr =0;
	static uint8 FirstCommuWithDSP=0;
	

				
	if(FirstCommuWithDSP==0)
	{
		WriteMainParameterFlag =1;
		WriteCorrectionParameterFlag =1;
		WriteCorrectionParameter2Flag =1;
		WriteCorrectionParameter3Flag =1;
		WriteUserParameterFlag =1;
		WritePassiveParameterFlag =1;
		FirstCommuWithDSP =1;
	}

	if(WriteMainParameterFlag ==1)
	{
		CurrentAddr =1;
//		if(main_parameter[30]>7)
//		{	
//			main_parameter[30]=7;
//			for (i=38;i<50;i++)		//��7�����ϵ�ѡ��ȥ��
//			{
//				main_parameter[i]=0;
//			}
//	  }		
		for(i=0;i<100;i++)
		{
			SendSpiBuf[i+3] =main_parameter[i];					//�������������͸�SPI
		}
		NextReceFrameID =0;
		SpiWrite_Enable;
	}
	else if(WriteCorrectionParameterFlag ==1)
	{
		CurrentAddr =2;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[i+3] =load_correctionA[i];
		}
		NextReceFrameID =1;
		SpiWrite_Enable;
	}
	else if(WriteCorrectionParameter2Flag ==1)
	{
		CurrentAddr =3;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[i+3] =load_correctionB[i];
		}
		NextReceFrameID =2;
		SpiWrite_Enable;
	}
	else if(WriteCorrectionParameter3Flag ==1)
	{
		CurrentAddr =4;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[i+3] =load_correctionC[i];
		}
		NextReceFrameID =3;
		SpiWrite_Enable;
	}
	else if(WritePassiveParameterFlag ==1)
	{
		CurrentAddr =5;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[i+3] =Passive_parameter[i];
		}
		NextReceFrameID =4;
		SpiWrite_Enable;
	}
	else if(WriteUserParameterFlag ==1)
	{
		CurrentAddr =6;
		SendSpiBuf[3] =(uint16)(TotalEnergyHB>>16);
		SendSpiBuf[4] =(uint16)TotalEnergyHB;
		SendSpiBuf[5] =(uint16)(TotalEnergyLB>>16);
		SendSpiBuf[6] =(uint16)TotalEnergyLB;
		for(i=4;i<100;i++)
		{
			SendSpiBuf[3+i] =0;
		}
		NextReceFrameID =5;
		SpiWrite_Enable;
	}
	else if(WriteSlaveParameterFlag ==1)
	{
		CurrentAddr =7;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[0][i];
		}
		//WriteSlaveParameterFlag = 0;
		NextReceFrameID =6;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameterFlag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =6;
	}
	else if(WriteSlaveParameter2Flag ==1)
	{
		CurrentAddr =8;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[1][i];
		}
		//WriteSlaveParameter2Flag = 0;
		NextReceFrameID =7;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter2Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =7;
	}
	else if(WriteSlaveParameter3Flag ==1)
	{
		CurrentAddr =9;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[2][i];
		}
		//WriteSlaveParameter3Flag = 0;
		NextReceFrameID =8;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter3Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =8;
	}

	else if(WriteSlaveParameter4Flag ==1)
	{
		CurrentAddr =10;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[3][i];
		}
		//WriteSlaveParameter4Flag = 0;
		NextReceFrameID =9;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter4Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =9;
	}

	else if(WriteSlaveParameter5Flag ==1)
	{
		CurrentAddr =11;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[4][i];
		}
		//WriteSlaveParameter5Flag = 0;
		NextReceFrameID =10;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter5Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =10;
	}

	else if(WriteSlaveParameter6Flag ==1)
	{
		CurrentAddr =12;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[5][i];
		}
		//WriteSlaveParameter6Flag = 0;
		NextReceFrameID =11;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter6Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =11;
	}

	else if(WriteSlaveParameter7Flag ==1)
	{
		CurrentAddr =13;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[6][i];
		}
		//WriteSlaveParameter7Flag = 0;
		NextReceFrameID =12;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter7Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =12;
	}

	else if(WriteSlaveParameter8Flag ==1)
	{
		CurrentAddr =14;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[7][i];
		}
		//WriteSlaveParameter8Flag = 0;
		NextReceFrameID =13;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter8Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =13;
	}

	else if(WriteSlaveParameter9Flag ==1)
	{
		CurrentAddr =15;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[8][i];
		}
		//WriteSlaveParameter9Flag = 0;
		NextReceFrameID =14;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter9Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =14;
	}

	else if(WriteSlaveParameter10Flag ==1)
	{
		CurrentAddr =16;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[9][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =15;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter10Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =15;
	}
	else if(WriteSlaveParameter11Flag ==1)
	{
		CurrentAddr =17;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[10][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =16;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter11Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =16;
	}
	else if(WriteSlaveParameter12Flag ==1)
	{
		CurrentAddr =18;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[11][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =17;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter12Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =17;
	}
	else if(WriteSlaveParameter13Flag ==1)
	{
		CurrentAddr =19;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[12][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =18;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter13Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =18;
	}
	else if(WriteSlaveParameter14Flag ==1)
	{
		CurrentAddr =20;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[13][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =19;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter14Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =19;
	}
	else if(WriteSlaveParameter15Flag ==1)
	{
		CurrentAddr =21;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =slave_parameter[14][i];
		}
		//WriteSlaveParameter10Flag = 0;
		NextReceFrameID =20;
		SpiWrite_Enable;
	}
	else if(ReadSlaveParameter15Flag==1)
	{
		CurrentAddr =0;
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
		NextReceFrameID =20;
	}
	// else if(WriteSlaveParameter16Flag ==1)
	// {
	// 	CurrentAddr =22;
	// 	for(i=0;i<100;i++)
	// 	{
	// 		SendSpiBuf[3+i] =slave_parameter[15][i];
	// 	}
	// 	//WriteSlaveParameter10Flag = 0;
	// 	NextReceFrameID =21;
	// 	SpiWrite_Enable;
	// }
	// else if(ReadSlaveParameter16Flag==1)
	// {
	// 	CurrentAddr =0;
	// 	for(i=0;i<100;i++)
	// 	{
	// 		SendSpiBuf[3+i] =0xFFFF;
	// 	}
	// 	NextReceFrameID =21;
	// }
	else if(WriteSpcialParameterFlag==1)
	{
		CurrentAddr =22;
		SendSpiBuf[3]=0xAAAA;
		for(i=0;i<60;i++)
		{
			SendSpiBuf[4+i] =spcial_parameter[i];
		}
		NextReceFrameID =21;
		SpiWrite_Enable;

		WriteSpcialParameterFlag=0;
	}
	else 	
	{
		CurrentAddr =0;
		SpiWrite_Disable; //������Ч
		for(i=0;i<100;i++)
		{
			SendSpiBuf[3+i] =0xFFFF;
		}
	}
	SendSpiBuf[103] = crc_chk_value(SendSpiBuf+3,100);
	return(CurrentAddr);

}
/******************************************************************************
**                            End Of File
******************************************************************************/
