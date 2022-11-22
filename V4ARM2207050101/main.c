/*****************************************************************************
 *   mian.c:  ����ARM����
 *   History
 *   2017.10.24  ver 1710240101
 *	 ����:	ARMӲ�����ð��� RTC��I2C��SSP0��Uart0��Uart1��Uart3�����账��
 	 ����:	���ļ��������ܰ��� �ܽų�ʼ������ʱ����ʼ����I2C��ʼ����I2C��д��������ʼ����������ʼ����
 	 		���ϼ�¼��ʼ�������ϴ����¼�����SPIͨѶ�������ػ�������ʱ���ػ������ۼ��¼�����
 	 		HexתBCD�룬Ӳ�����ϴ�����Դ�����źŴ��������ۼƴ���
 	 �ж�:	���ļ����жϺ�������:��ʱ�жϣ�	���ڴ������Ӽ�����
 	 							 SSP�жϣ�	���ڴ���DSP��������
 	 							 Uart0�жϣ����ڴ����������ݽ���
 	 							 Uart1�жϣ����ڴ�����Ļ���ݽ���
 	 							 Uart3�жϣ����ڴ������485���ݽ���
 	 							 Ӳ�������жϣ�����ARM����Ӳ�����Ϻ�ĸ�λ
 	 							 ���Ź��ж�	����ARM�������Ź���ĸ�λ
*****************************************************************************/
/*****����ͷ�ļ�*****/
#include "Lpc17xx.h"
#include "math.h"
#include "type.h"
#include "ssp.c" 
#include "IAP.c"
#include "IAP.h"
#include "uart.c"
#include "rtc.c"
#include "WDT.c"

#define I2C_FEQ 400000								//I2CƵ��



extern void	delay(uint32_t dly);//�ӳٺ��������ڳ����ӳ٣���ȷ��ʱ��
extern uint16 crc_chk(uint8 *buf,uint32 length);//CRC16����
extern void I2c1_Init(void);//I2C��ʼ��
extern void I2c1_Recv(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num);//I2C��ȡ����
extern void I2c1_Send(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num);//I2Cд�뺯��
extern void WriteEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//I2Cд���װ����(��ʼ��ַ��Ŀ��ָ�룬��ȡ����)ÿ��д8��byte����ʼ��ַ��Ҫȡ8��������
extern void ReadEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//I2C��ȡ��װ����(��ʼ��ַ��Ŀ��ָ�룬��ȡ����)
extern void Parameter_init(void);//������ʼ�����״��ϵ������ʼ��
extern void GZ_init(void);//���ϳ�ʼ�����״��ϵ������0
extern void GPIO_init(void);//IO�ܽų�ʼ��
extern void Bluetooth_init(void);//������ʼ��
extern uint32_t TIME_init(uint8 time_num);//��ʱ����ʼ��(��ʱ��������)��Ŀǰ��ʼ��Ϊ1ms�ж�
extern void error_handle(void); //���ϴ�����¼��������
extern void Event_handle(void); //�¼���¼����¼�¼���Ϣ���������ػ�����ʱ���ػ���Զ�̿��ػ�
extern void SPIReadyData(void); //SPIͨѶ������׼��SPI��һ֡��������װ��
extern void OnOffProcess(void); //���ػ�����
extern void AlarmTimeProcess(void);//��ʱ���ػ�����
extern void AccumEventProcess(void);//�ۼ��¼�������Ҫ�ۼ������ۼӣ��������ݼ�¼
extern uint8 HEX2BCD(uint64 Dec, uint8 *Bcd, uint8 length);     //HEXתΪBCD�ӳ���  
extern void PassiveReturnSignProcess(void);//��ԴͶ�з����źŴ���M3���ձ��豸ʹ��
extern void Reset(void);//ARMϵͳ��λ����
extern void DatalogProcess(void);//���ݼ�¼����
extern void TIMER0_IRQHandler (void) ;//��ʱ���жϣ��������ֱ��ۼ�1ms��10ms��100ms��1s
 void UART0_IRQHandler(void) ;//Uart0�жϣ���������ģ��ͨѶ��������9600
extern void UART1_IRQHandler(void); //Uart1�жϣ�����DGUS��ĻͨѶ��������115200
extern void UART3_IRQHandler(void) ;//Uart3�жϣ�����DTUͨѶ�������ʿ���
extern void hard_fault_handler_c(unsigned int * hardfault_args);//ARMӲ�������жϣ�����ͻ����Ӳ�����ϸ�λ

//extern void ARM_proess(void);
/************************������**********************************/
/**ģ��**/
extern uint16 Testtime_time1;
extern uint16 test_time;								//����ʱ������
extern uint16 time_flag;							//дi2c��־��
extern uint16 sendok;									//���ͳɹ���־��


/**********************�û����ò���******************************/
__align(4) uint8 Parameter[ParameterNumMax],ParameterBak[ParameterInitNumMax];//�洢���޸�,�޸���߼��������ֽڱ߽�,��֤���ݶ�������Ӧ����
extern uint32_t SystemFrequency;										//ϵͳƵ��
uint8 	led2_flag,led3_flag,led4_flag;									//Led��˸��־λ
uint8 	gz_dan[160];													//����
uint8 	SlaveFault[5][100];											//�ӻ�����
uint8 	Event_Logging[160];												//�¼���¼
uint8   DR_LOG[800];
uint8   capa_bak;

double  pclock;															//ϵͳʱ��
uint8	RemoteEnable;													//Զ��ʹ��
uint16	AlarmTime1[5],AlarmTime2[5],AlarmTime3[5],AlarmTime4[5];		//��ʱ���ػ�����
uint8	APFStartRunning =0;												//���ػ�״̬
extern Aisle	IntPassive;												//��Դͨ��
uint16	TimeDelay=0,AccumDelay=0,ResetDelay =0,DatalogDelay;			//�ӳ�������
uint32	moni_delay =0;													//ģ��
uint32	ProjectNo;														//��Ŀ��
uint16	ProductionNo;													//������
uint16	VolOnOffEnable,CurOnOffEnable;									//��ѹ���ػ����������ػ�
//����������װλ�ã���������A�෽����������B�෽��,��������C�෽��,��CT����,�����������װλ�ã����������A�෽�����������B�෽��,���������C�෽��,�������������,
uint16	MainCTLocation,MainCTDirectionA,MainCTDirectionB,MainCTDirectionC,MainCTPhase,OutCTDirectionA,OutCTDirectionB,OutCTDirectionC,OutCTPhase;
uint16	Position[15],Group[15],Capacitance[15];							//��λ����
uint16  PassiveChannel[32],PassiveValue[32],PassiveCom;					//��Դ����
uint16	CorrectParam[11];												//��������
uint32	IntPassiveState,IntPassiveFault,IntPassiveSwitch;				//��Դ״̬����Դ���ϣ���Դ����
uint16	ActiveBal,ReactiveBal,HarmonicBal,ApparentBal;					//�й���ƽ�⣬�޹���ƽ�⣬г����ƽ�⣬���ڲ�ƽ��,���ڲ�ƽ�ⱸ��
uint16	SetupMode;														//��λ����
uint64	SerialNumber,tmp64;												//���к�
uint8	Event_Code,RecordEventFlag,ACSwitchFlag,AlarmSwitchFlag;		//�¼����룬�¼���־λ�������ת���ر�־λ����ʱ���ػ���־λ
uint16	Datalog[38],DatalogMax,DatalogStart;							//���ݼ�¼�����ݼ�¼���ֵ�����ݼ�¼��ʼλ��
volatile uint8 DatalogReset;											//���ݱ�����λ

/***********************Timerʱ�ӱ�����*************************/
uint8  	time_1us,time_10us,time_100us,time_1ms,time_10ms,time_100ms,time_1s;
uint8	time_SpiDelay;
uint16  TimeCount[4];
RTCTime local_time, alarm_time, current_time,Event_time,DR_time;   				//RTC�ṹ��
/***********************Uart0������*****************************/
extern volatile uint8		RecvBTFlag;                               	// ���ڽ������ݱ�־λ
extern uint8        		RecvSciBuf[150];                         	// ���ڽ������ݻ���
extern uint32      			RecvSciNum;                                 // ���ڽ������ݸ���
//extern uint16		 		RecvSuccessTest;							// ���ڽ��ճɹ�����
//extern uint16 		 	SciSuccessNUM;								// ���ڽ��ճɹ�����
extern	uint16				SciWaitingCount;							// ���ڵȴ���ʱ
extern	uint8 				BluetoothTestOK;							// �������Գɹ�
uint8 						BluetoothTestOK;							// �������Գɹ�
uint8						BluetoothChangeNameOK;						// �����豸�����޸�
uint8						BluetoothATcmd;								// ����ATָ��״̬

uint8		bluetooth_int;	//�ж��Ƿ���Ҫ��ʼ��
uint16 count_swj;
uint8   first_int;				//��һ�γ�ʼ��
/***********************Uart3������*****************************/
extern volatile uint8		RecvNewFlag;                               	// ���ڽ������ݱ�־λ
extern uint8        		RecvSci3Buf[150];                         	// ���ڽ������ݻ���
extern uint32      			RecvSci3Num;                                // ���ڽ������ݸ���
//extern uint16		 		RecvSuccessTest3;							// ���ڽ��ճɹ�����
//extern uint16 		 	SciSuccessNUM3;								// ���ڽ��ճɹ�����
extern	uint16				SciWaitingCount3;							// ���ڵȴ���ʱ
uint8						Uart3Reset,Uart1Reset,Uart0Reset;			// ���ڸ�λ��־λ
/********************** *SSP������*************************************************/
volatile uint8				SPIRecvSuccess;                             // SPI�������ݱ�־λ
uint8						SPICommDelay;								// SPIͨѶ�ӳ�
uint8						NextReceFrameID;							//�´ν�������֡ID
uint8						SPICommError;								//SPIͨѶ����
uint16						SPICommCount;								//SPIͨѶ����

uint16	ReadCount;					//������ȡ����

//								 ����������ַ	����������ַ	����������ַ	����������ַ	  ��Դ����		�û�������ַ
//									0				1				2				3				4				5
uint16	SPI_NextStartAddress[44]={	10000,			10100,			10200,			10300,			10400,			11000,
//								�ӻ�0������ַ	�ӻ�1������ַ	�ӻ�2������ַ	�ӻ�3������ַ	�ӻ�4������ַ
//									6				7				8				9				10
									12000,			12100,			12200,			12300,			12400,
//								�ӻ�5������ַ	�ӻ�6������ַ	�ӻ�7������ַ	�ӻ�8������ַ	�ӻ�9������ַ
//									11				12				13				14				15
									12500,			12600,			12700,			12800,			12900,
//								�ӻ�10������ַ	�ӻ�11������ַ	�ӻ�12������ַ	�ӻ�13������ַ	�ӻ�14������ַ	60000������ַ
//									16				17				18				19				20				21
									13000,			13100,			13200,			13300,			13400,			60000,
//								�������ݵ�ַ	A��ϵͳ���ַ	B��ϵͳ���ַ	C��ϵͳ���ַ	A�ฺ�ز��ַ
//									22				23				24				25				26
									0,				100,			200,			300,			400,
//								B�ฺ�ز��ַ	C�ฺ�ز��ַ	A��������ַ	B��������ַ	C��������ַ
//									27				28				29				30				31
									500,			600,			700,			800,			900,
//								A���ѹ��ַ		B���ѹ��ַ		C���ѹ��ַ		A����ݹ��ַ	B����ݹ��ַ
//									32				33				34				35				36
									1000,			1100,			1200,			1300,			1400,
//								C����ݹ��ַ	�ӻ���������	�ӻ������С���� �ӻ�11-16����	�ӻ�11-16����	��Դ�����ź�	�ֶ���ԴͶ��
//									37				38				39				40				41			42				43
									1500,			2000,			2100,			2200,			2300,		9100,			9000};


//								 	��Ч��ַ		����������ַ	����������ַ	����������ַ	����������ַ	 ��Դ����		�û�������ַ
//										0				1				2				3				4			  	5				6
uint16	SPI_CurrentStartAddress[22]={	0xFFFF,			10000,			10100,			10200,			10300,			10400,			11000,
//									�ӻ�0������ַ	�ӻ�1������ַ	�ӻ�2������ַ	�ӻ�3������ַ	�ӻ�4������ַ
//										7				8				9				10				11
										12000,			12100,			12200,			12300,			12400,
//									�ӻ�5������ַ	�ӻ�6������ַ	�ӻ�7������ַ	�ӻ�8������ַ	�ӻ�9������ַ
//										12				13				14				15				16
										12500,			12600,			12700,			12800,			12900,
//									�ӻ�10������ַ	�ӻ�11������ַ	�ӻ�12������ַ	�ӻ�13������ַ	�ӻ�14������ַ	      60000����
//										17				18				19				20				21				22     
										13000,			13100,			13200,			13300,			13400,			60000};
/******************************************************************************
** ������: delay
**
** ���ܣ���ʱ
**		  
** �÷���delay(Ҫ��ʱ��ʱ��)
******************************************************************************/
void delay(uint32_t dly) 
{
  uint32_t	  i_delay;
  for(; dly>0; dly--)
  for(i_delay=0; i_delay<100000; i_delay++); 
}
/******************************************************************************
** ������: Get_temp
**
** ���ܣ��¶ȼ���
**		  
** �÷���Get_temp(����ֵ)
******************************************************************************/
/*unsigned int Get_temp( float var_temp  )
{
	var_list=22000*var_temp*3.3/4095/(5-var_temp*3.3/4095);
	for(var_temp0=0; var_temp0<222; var_temp0+=2)
	{
		if( var_temp0==220 )
		{
			value_temp=110;
		}
		else
		{
			if( var_list<temp_list[var_temp0+1] && var_list>temp_list[var_temp0+3] )
			{
				value_temp=temp_list[var_temp0];
				break;
			}
		}
	}
	return value_temp;
}
*/
/*unsigned int GetAD(uint8 Aisle )
{
	static uint8 i;
// Keil: function replaced to handle LPC1768 A/D converter.
	unsigned int val,t;
	unsigned int adcr;
	adcr = 0x01000000 | (1 << Aisle);
	LPC_ADC->ADCR = adcr | 0x00200100;			
 	LPC_ADC->ADCR |=  (1<<24);             
	while ( (!(LPC_ADC->ADGDR & (1UL<<31))) && tang!=0xffffff  ) 
	{
		tang++;
	}
	LPC_ADC->ADCR &= ~(7<<24);              
	if(tang!=0xffffff)
	{
		val = ((LPC_ADC->ADGDR >> 4) & 0xFFF);  
		if(i<GetADMax)
		{
			data_temp[Aisle][i]=Get_temp(val);
			i++;
		}
		else i=0;
		
	}
	return(t);  
}
*/
//AD ��ʼ��
/*void AD_ini(void)
{
  LPC_SC->PCONP	|=	(1<<12);              
  LPC_ADC->ADCR	=	15 |                  	
   		            (6<< 8) |              
                  (1<<21);               						 
}
*/
/************************************************************************
** ������: crc_chk
**
** ���ܣ�����CRC������У��
**			
** �÷���crc_chk(Ҫ�������,����)
************************************************************************/
uint16	crc_chk(uint8 *buf,uint32 length)
{
 	uint8	j;
 	uint16	reg_crc	= 0xffff;
 	while(length--)
 	{
 	    if(length>255)break;
  		reg_crc ^= *buf++;
  		for(j=0; j<8; j++)
  		{
   			if(reg_crc&0x01){reg_crc = (reg_crc>>1)^0xa001;}
   			else{reg_crc = reg_crc>>1;}
  		}
 	}
 	return	reg_crc;
}
/************************************************************************
** ������: I2c1_Init 
**
** ���ܣ�I2C�ĳ�ʼ��
************************************************************************/
void I2c1_Init(void)
{
	
 	LPC_I2C2->I2SCLH = (pclock/I2C_FEQ+1)/2;	//I2C1ʱ����400KHz
 	LPC_I2C2->I2SCLL = (pclock/I2C_FEQ)/2;
 	LPC_I2C2->I2CONCLR = 0x2C;              	//���AAC��SIC��STACλ
 	LPC_I2C2->I2CONSET = 0x40;              	//ʹ��I2C���ܣ���ģʽ
}

/************************************************************************
** ������: I2c1_Recv 
**
** ���ܣ�I2C�Ķ�ȡ
**
** �÷���I2c1_Recv(I2C��ַ,��ڵ�ַ,��ȡ������,����ĸ���)
************************************************************************/
void I2c1_Recv(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num)
{ 
 	uint32_t	var_i2c;
 	LPC_I2C2->I2CONCLR = (1<<2)|(1<<3)|(1<<5);	//��AA//��SI//��STA          
 	for(var_i2c=10000; var_i2c>0; var_i2c--);           
 	LPC_I2C2->I2CONSET = 0x60;                	//��ʼ�źţ�ѡ��Ϊ��ģʽ         
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
	LPC_I2C2->I2DAT	= sla;                    	//slave address+W
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);       
 	for(var_i2c=10000; var_i2c>0; var_i2c--);  
 	LPC_I2C2->I2DAT = ((suba>>8)&0xff);       	//sub-address H
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);     
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2DAT = (suba&0xff);            	//sub-address L
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);           
 	LPC_I2C2->I2CONSET = (1<<5);              	//�ظ���ʼ����
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2CONCLR = (1<<3);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);  
 	LPC_I2C2->I2DAT = sla + 1;                  //salve address+R
 	for(var_i2c=10000; var_i2c>0; var_i2c--); 
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);           
 	do
	{
		LPC_I2C2->I2CONSET = (1<<2);
    for(var_i2c=10000; var_i2c>0; var_i2c--);
    LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);
    for(var_i2c=10000; var_i2c>0; var_i2c--);            
    *pointeri2c++ = LPC_I2C2->I2DAT;	//rev data
    num--;
	}
 	while(num>1);
 	LPC_I2C2->I2CONCLR = (1<<2)|(1<<5);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);           
 	LPC_I2C2->I2CONCLR = (1<<3);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	*pointeri2c = LPC_I2C2->I2DAT;
 	for(var_i2c=10000; var_i2c>0; var_i2c--); 
 	LPC_I2C2->I2CONSET = (1<<4);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);
 	for(var_i2c=10000; var_i2c>0; var_i2c--);
}

/************************************************************************
** ������: I2c1_Send 
**
** ���ܣ�I2C�ı���
**
** �÷���I2c1_Send(I2C��ַ����ڵ�ַ���������飬����ĸ���)
************************************************************************/
void I2c1_Send(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num)
{ 
 	uint32_t	var_i2c;  
 	LPC_I2C2->I2CONCLR = (1<<2)|(1<<3)|(1<<5);	//��AA//��SI//��STA    
 	LPC_I2C2->I2CONSET = 0x60;                  //��ʼ�źţ�ѡ��Ϊ��ģʽ
 	for(var_i2c=0; var_i2c<10000; var_i2c++);
 	LPC_I2C2->I2DAT = sla;                      //slave address+W
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);       
 	for(var_i2c=0; var_i2c<10000; var_i2c++);
 	LPC_I2C2->I2DAT = ((suba>>8)&0xff);         //sub-address H
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);      
  for(var_i2c=0; var_i2c<10000; var_i2c++); 
 	LPC_I2C2->I2DAT = (suba&0xff);              //sub-address L
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);
 	for(var_i2c=0; var_i2c<10000; var_i2c++);           
 	while(num>0)
	{
		LPC_I2C2->I2DAT = *pointeri2c++;    //data to be sent
    LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);      
    for(var_i2c=0; var_i2c<10000; var_i2c++);            
    num--;
	}
 	LPC_I2C2->I2CONSET = (1<<4);
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);         //��������                
 	for(var_i2c=0; var_i2c<10000; var_i2c++);
}
/************************************************************************
** ������: WriteEeprom 
**
** ���ܣ�Eeprom�ı���
**
** �÷���WriteEeprom(��ַ���������飬����)
************************************************************************/
void WriteEeprom(uint32_t addr,uint8_t *pointer,uint32_t num)
{ 
	uint8 *pointer8;
	uint16 i,j,counter,EepromAddr;
	uint32 delaytime;
	
	//LPC_UART0->IER = 0;
	//LPC_UART1->IER = 0;
	//LPC_UART3->IER = 0; 
	LPC_SSP0->IMSC = 0x00;
	
	if(num>8)
	{
		i=num/8;
		j=num%8;
		for(counter=0; counter<i; counter++)
	  	{
	  		pointer8 =pointer;
	  		pointer8 += counter<<3;
			EepromAddr =addr;
			EepromAddr += counter<<3;
	   		I2c1_Send(0xa0,EepromAddr,pointer8,8);   				//����������д����ʽ������
	   		for(delaytime=0x80000; delaytime>0; delaytime--);
	  	}
		if(j>0)
		{
			pointer8 =pointer;
			pointer8 += counter<<3;
			EepromAddr =addr;
			EepromAddr += counter<<3;
			I2c1_Send(0xa0,EepromAddr,pointer8,j);   				//����������д����ʽ������
	   		for(delaytime=0x80000; delaytime>0; delaytime--);
		}
	}
	else
	{
		I2c1_Send(0xa0,addr,pointer,num);   				//����������д����ʽ������
		for(delaytime=0x80000; delaytime>0; delaytime--);
	}
	
	//LPC_UART0->IER = 1;
	//LPC_UART1->IER = 1; 
	//LPC_UART3->IER = 1; 
	LPC_SSP0->IMSC = 0x04;
}
/************************************************************************
** ������: ReadEeprom 
**
** ���ܣ�Eeprom�ı���
**
** �÷���WriteEeprom(��ַ���������飬����)
************************************************************************/
void ReadEeprom(uint32_t addr,uint8_t *pointer,uint32_t num)
{ 
	I2c1_Recv(0xa0,addr,pointer,num);				//��ȡ24C32�ڵĳ�������
}
/******************************************************************************
** ������:Parameter_init 
**
** ���ܣ�������ʼ��
******************************************************************************/
void Parameter_init(void)
{
	//uint8	temp;
 	uint16	i,j,tmp[100],tmp16;
	uint32 	tmp32;
	uint64	tmp64H,tmp64L,tmp64Val;
/****************************************************��ʼ��ȫ�ֱ���*******************************************************/		
	SendSpiBuf[0]=0xACAC;
	NextReceFrameID =5;
	SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];

	//for(i=0;i<11;i++)
	//{FaultBak[i] =0;}
	ini_version_b=0;
	Remote_Password =0;
	/*temp =55;
	I2C_write;
	WriteEeprom(0x0000,Eep_parameter,ParameterInitNumMax); 
	I2C_read;
	ReadEeprom(0x0000,Eep_parameter,ParameterInitNumMax);		//��ȡ24C32�ڵĳ�������
	*/


/****************************************************��ʼ��������������***************************************************/	
 	ReadEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);		//��ȡ24C32�ڵĳ�������

	tmp32 = ((Eep_parameter[ParameterInitNumMax-4]<<24)
			|(Eep_parameter[ParameterInitNumMax-3]<<16)
			|(Eep_parameter[ParameterInitNumMax-2]<<8)
			|(Eep_parameter[ParameterInitNumMax-1]));
	if(tmp32!=EepromVersion)											//�жϳ��������汾�Ƿ����
 	{
 		i=0;
	  	Eep_parameter[0]=0;		Eep_parameter[1]=230;							//����ϵͳ�Ķ���ѹ	
	  	Eep_parameter[2]=0;		Eep_parameter[3]=0;							//����ϵͳ�ĶƵ��	
	  	Eep_parameter[4]=0;		Eep_parameter[5]=0;							//��ѹ֧����Ӧ�ٶ�			   			  
		Eep_parameter[6]=0;		Eep_parameter[7]=2;								//ϵͳ��ѹ������
		Eep_parameter[8]=0;		Eep_parameter[9] =0; 							//ϵͳ�Ľ��߷�ʽ			   
		Eep_parameter[10]=0;	Eep_parameter[11] =1;   				  		//�豸�Ļ�������ģʽ			   	
		Eep_parameter[12]=0;	Eep_parameter[13] =1;  				 			//�Ƿ������й���ƽ������		   
		Eep_parameter[14]=0;	Eep_parameter[15] =0;   						//�Ƿ���������Ӧ   
		Eep_parameter[16]=0;	Eep_parameter[17] =0;   						//�Ƿ�߱���ͬ�������ù���     	      
		Eep_parameter[18]=0;	Eep_parameter[19] =0;							//�Ƿ�������������	   
		Eep_parameter[20]=0;	Eep_parameter[21] =0;							//�������������İ�װλ��			   
		Eep_parameter[22]=0;	Eep_parameter[23] =200;    					//��������������һ�ζ����	   
		Eep_parameter[24]=0;	Eep_parameter[25] =1; 							//�������������Ķ��ζ����		  
		Eep_parameter[26]=0;	Eep_parameter[27] =0;							//�������������İ�װ����	   
		Eep_parameter[28]=0;	Eep_parameter[29] =16;    						//LCL��̬����	   
		Eep_parameter[30]=0;	Eep_parameter[31] =0;   						//��������������İ�װλ��
		Eep_parameter[32]=0;	Eep_parameter[33] =0;    						//ģ������仯����	   
		Eep_parameter[34]=0;	Eep_parameter[35] =0xF0;     					//���ܵ���ͨѶ������ 2400
		Eep_parameter[36]=0;	Eep_parameter[37] =200;     					//���������������һ�ζ����    
		Eep_parameter[38]=0;	Eep_parameter[39] =1;							//��������������Ķ��ζ����   
		Eep_parameter[40]=0;	Eep_parameter[41] =0;							//���ȼ�����ģʽ
		Eep_parameter[42]=0x02;	Eep_parameter[43] =0x58; 						//�����������ֵ����
		Eep_parameter[44]=0x02;	Eep_parameter[45] =0x58; 						//�����޹���ֵ��������
		Eep_parameter[46]=0;	Eep_parameter[47] =200; 						//��г����ֵ��������
		Eep_parameter[48]=0x02;	Eep_parameter[49] =0x58; 						//��ƽ�������ֵ����
		Eep_parameter[50]=0x03;	Eep_parameter[51] =0xE8; 						//Ŀ�깦������
		Eep_parameter[52]=0;	Eep_parameter[53] =20;							//�豸�����������
		Eep_parameter[54]=0;	Eep_parameter[55] =10;							//�豸�ػ��������
		Eep_parameter[56]=0;	Eep_parameter[57] =10;							//�������������ʱ
		Eep_parameter[58]=0;	Eep_parameter[59] =30;  						//�ػ����������ʱ
		Eep_parameter[60]=0;	Eep_parameter[61] =1;							//�豸��ѡ���˶���Ƶ�εĴ���������
		Eep_parameter[62]=0;	Eep_parameter[63] =1; 							//ѡ�еĵ�һ������Ƶ��
		for(i=0;i<18;i++)
	  	{
	  		Eep_parameter[i*2+64]=0;Eep_parameter[i*2+65]=0;					//��ѡ�еĵ�2~19��Ƶ��
	  	}
	  	Eep_parameter[100]=0;	Eep_parameter[101]=0; 							//��ѹ֧��ģʽ
		Eep_parameter[102]=0;	Eep_parameter[103]=230;							//��ѹ�ϸ�����
		Eep_parameter[104]=0;	Eep_parameter[105]=210; 						//��ѹ�ϸ�����
		Eep_parameter[106]=0x03;Eep_parameter[107]=0x52;						//���ߵ�������850
		Eep_parameter[108]=0;	Eep_parameter[109]=1;							//���������������ۺ�ϵ��1
		Eep_parameter[110]=0x0E;Eep_parameter[111]=0x4F;						//���������������ۺ�ϵ��2 3663
		Eep_parameter[112]=0;	Eep_parameter[113]=240; 						//��ѹ��������
		Eep_parameter[114]=0;	Eep_parameter[115]=200; 						//��ѹ��������         
		Eep_parameter[116]=0;	Eep_parameter[117]=1;  							//����������������ۺ�ϵ��1          
		Eep_parameter[118]=0x0E;Eep_parameter[119]=0x4F; 						//����������������ۺ�ϵ��2         
		Eep_parameter[120]=0x27;Eep_parameter[121]=0x10; 						//A���������������ķ�ֵ����          
		Eep_parameter[122]=0x27;Eep_parameter[123]=0x10;						//B���������������ķ�ֵ����          
		Eep_parameter[124]=0x27;Eep_parameter[125]=0x10; 						//C���������������ķ�ֵ����        
		Eep_parameter[126]=0; Eep_parameter[127]=120;  						//����״̬�µĹ�������ʱ����    
		Eep_parameter[128]=0;	Eep_parameter[129]=30;							//����״̬�µı�������ʱ����         
		Eep_parameter[130]=0;	Eep_parameter[131]=1; 							//���������ӳ�      
		Eep_parameter[132]=0x27;Eep_parameter[133]=0x10; 						//A����������������ķ�ֵ����       
		Eep_parameter[134]=0x27;Eep_parameter[135]=0x10; 						//B����������������ķ�ֵ����        
		Eep_parameter[136]=0x27;Eep_parameter[137]=0x10; 						//C����������������ķ�ֵ����
		Eep_parameter[138]=0;	Eep_parameter[139]=0;   						//����������������λ����
		Eep_parameter[140]=0;	Eep_parameter[141]=0;  							//��Դ��������
		Eep_parameter[142]=0;	Eep_parameter[143]=0;						  	//�����������������λ����
		Eep_parameter[144]=0;	Eep_parameter[145]=0;							//�Ƿ�������Դ����
		Eep_parameter[146]=0;	Eep_parameter[147]=0;							//���ػ������ж�
		Eep_parameter[148]=0;	Eep_parameter[149]=0;							//��ѹ������
		Eep_parameter[150]=0;Eep_parameter[151]=2;						//�����ȴ���ʱ
		Eep_parameter[152]=0x13;Eep_parameter[153]=0x88;						//0״̬�ȴ�ͨѶ��ʱ����
		Eep_parameter[154]=0x06;Eep_parameter[155]=0x40;						//��0״̬�ȴ�ͨѶ��ʱ����
		Eep_parameter[156]=0;	Eep_parameter[157]=2;							//�Ƿ����ø��ز������
		Eep_parameter[158]=0x1F;Eep_parameter[159]=0x40;						//ϵͳ��Ĳ�������ϵ��
		Eep_parameter[160]=0x3;Eep_parameter[161]=0xE8;						//���ز�Ĳ�������ϵ��
		Eep_parameter[162]=0x27;Eep_parameter[163]=0x10;						//��ƽ��ϵͳ��Ĳ�������ϵ�� 10000
		Eep_parameter[164]=0x1;Eep_parameter[165]=0x2C;						//��ƽ�⸺�ز�Ĳ�������ϵ��  300
		Eep_parameter[166]=0;	Eep_parameter[167]=0;							//Ŀ���������λ����1
		Eep_parameter[168]=0;	Eep_parameter[169]=0;							//Ŀ���������λ����2	
		Eep_parameter[170]=0;Eep_parameter[171]=10;						//�ϵ���������ʱ����
		Eep_parameter[172]=0;	Eep_parameter[173]=0;							//����ģʽ
		Eep_parameter[174]=0;Eep_parameter[175]=10;						//��������������ĵȴ�ʱ��
		Eep_parameter[176]=0;	Eep_parameter[177]=1;							//����
		Eep_parameter[178]=0;	Eep_parameter[179]=14;							//����
		Eep_parameter[180]=0;	Eep_parameter[181]=0;							//��ѹ��ѹ�����
		Eep_parameter[182]=0;	Eep_parameter[183]=0;							//��ѹ����λ��
		Eep_parameter[184]=0x01;	Eep_parameter[185]=0xf4;							//��������Ӧ�ٶ�
		Eep_parameter[186]=0;	Eep_parameter[187]=10;							//��������ά��ʱ��
		Eep_parameter[188]=0;	Eep_parameter[189]=10;							//����������ֵ1
		Eep_parameter[190]=0;	Eep_parameter[191]=20;							//����������ֵ2
		Eep_parameter[192]=0;	Eep_parameter[193]=30;							//����������ֵ3
		Eep_parameter[194]=0;	Eep_parameter[195]=40;							//����������ֵ4
		Eep_parameter[196]=3;	Eep_parameter[197]=0xE8;							// 10��Ԫ��ʹ��
		Eep_parameter[198]=0;	Eep_parameter[199]=1;							//�ӻ�����
		
		//�޹�����
		Eep_parameter[200]= 0;	Eep_parameter[201]= 10;
		Eep_parameter[202]= 0;	Eep_parameter[203]= 200;
		Eep_parameter[204]= 0;	Eep_parameter[205]= 1;
		Eep_parameter[206]= 0;	Eep_parameter[207]= 8;
		Eep_parameter[208]= 0;	Eep_parameter[209]= 30;
		Eep_parameter[210]= 0;	Eep_parameter[211]= 100;
		Eep_parameter[212]= 0;	Eep_parameter[213]= 20;
		Eep_parameter[214]= 0x01;Eep_parameter[215]= 0x90;
		Eep_parameter[216]= 0;	Eep_parameter[217]= 230;
		Eep_parameter[218]= 0x01;Eep_parameter[219]= 0x90;
		Eep_parameter[220]= 0;	Eep_parameter[221]= 0;
		Eep_parameter[222]= 0;	Eep_parameter[223]= 0;
		Eep_parameter[224]= 0;	Eep_parameter[225]= 0;
		Eep_parameter[226]= 0;	Eep_parameter[227]= 0;
		Eep_parameter[228]= 0;	Eep_parameter[229]= 0;
		Eep_parameter[230]= 0;	Eep_parameter[231]= 0;
		Eep_parameter[232]= 0;	Eep_parameter[233]= 0;
		Eep_parameter[234]= 0;	Eep_parameter[235]= 0;
		Eep_parameter[236]= 0;	Eep_parameter[237]= 0;
		Eep_parameter[238]= 0;	Eep_parameter[239]= 0;
		Eep_parameter[240]= 0;	Eep_parameter[241]= 0;
		Eep_parameter[242]= 0;	Eep_parameter[243]= 0;
		Eep_parameter[244]= 0;	Eep_parameter[245]= 0;
		Eep_parameter[246]= 0;	Eep_parameter[247]= 0;
		Eep_parameter[248]= 0;	Eep_parameter[249]= 0;
		Eep_parameter[250]= 0;	Eep_parameter[251]= 0;
		Eep_parameter[252]= 0;	Eep_parameter[253]= 0;
		Eep_parameter[254]= 0;	Eep_parameter[255]= 0;
		Eep_parameter[256]= 0;	Eep_parameter[257]= 0;
		Eep_parameter[258]= 0;	Eep_parameter[259]= 0;
		Eep_parameter[260]= 0;	Eep_parameter[261]= 0;
		Eep_parameter[262]= 0;	Eep_parameter[263]= 0;
		Eep_parameter[264]= 0;	Eep_parameter[265]= 0;
		Eep_parameter[266]= 0;	Eep_parameter[267]= 0;
		Eep_parameter[268]= 0;	Eep_parameter[269]= 0;
		Eep_parameter[270]= 0;	Eep_parameter[271]= 0;
		Eep_parameter[272]= 0;	Eep_parameter[273]= 0;
		Eep_parameter[274]= 0;	Eep_parameter[275]= 0;
		Eep_parameter[276]= 0;	Eep_parameter[277]= 0;
		Eep_parameter[278]= 0;	Eep_parameter[279]= 0;
		Eep_parameter[280]= 0;	Eep_parameter[281]= 0;
		Eep_parameter[282]= 0;	Eep_parameter[283]= 0;
		Eep_parameter[284]= 0;	Eep_parameter[285]= 0;
		Eep_parameter[286]= 0;	Eep_parameter[287]= 0;
		Eep_parameter[288]= 0;	Eep_parameter[289]= 5;
		Eep_parameter[290]= 0;	Eep_parameter[291]= 20;
		Eep_parameter[292]= 0x04;Eep_parameter[293]= 0xB0;
		Eep_parameter[294]= 0;	Eep_parameter[295]= 0;
		Eep_parameter[296]= 0x01;Eep_parameter[297]= 0xF4;
		Eep_parameter[298]= 0	;Eep_parameter[299]= 100;
		Eep_parameter[300]= 0;	Eep_parameter[301]= 5; //10450#��Դ����
		Eep_parameter[302]= 0;	Eep_parameter[303]= 1;
		Eep_parameter[304]= 0;	Eep_parameter[305]= 2;	
		
		for(i=0;i<94;i++)
		{
			Eep_parameter[306+i]= 0;
		}
		
		Eep_parameter[306]= 0;	Eep_parameter[307]= 5;	
		Eep_parameter[310]= 0;	Eep_parameter[311]= 10;		//���й���10455#
		Eep_parameter[312]= 0;	Eep_parameter[313]= 10;		//��WU����10456#
		Eep_parameter[314]= 0;	Eep_parameter[315]= 0;		//2
		Eep_parameter[316]= 0;	Eep_parameter[317]= 10;		//3
		Eep_parameter[318]= 0;	Eep_parameter[319]= 0;		//4
		Eep_parameter[320]= 0;	Eep_parameter[321]= 10;		//5
		Eep_parameter[322]= 0;	Eep_parameter[323]= 0;		//6
		Eep_parameter[324]= 0;	Eep_parameter[325]= 10;		//7
		Eep_parameter[326]= 0;	Eep_parameter[327]= 0;		//��8
		Eep_parameter[328]= 0;	Eep_parameter[329]= 10;		//��9
		Eep_parameter[330]= 0;	Eep_parameter[331]= 0;		//10
		Eep_parameter[332]= 0;	Eep_parameter[333]= 10;		//11
		Eep_parameter[334]= 0;	Eep_parameter[335]= 0;		//��12
		Eep_parameter[336]= 0;	Eep_parameter[337]= 10;		//13
		Eep_parameter[338]= 0;	Eep_parameter[339]= 0;		//14
		Eep_parameter[340]= 0;	Eep_parameter[341]= 5;		//15
		
		Eep_parameter[342]= 0;	Eep_parameter[343]= 0;		//16
		Eep_parameter[344]= 0;	Eep_parameter[345]= 5;		//17
		Eep_parameter[346]= 0;	Eep_parameter[347]= 0;		//18
		Eep_parameter[348]= 0;	Eep_parameter[349]= 5;		//19
		Eep_parameter[350]= 0;	Eep_parameter[351]= 0;		//20
		
		Eep_parameter[352]= 0;	Eep_parameter[353]= 5;		//21
		Eep_parameter[354]= 0;	Eep_parameter[355]= 0;		//22
		Eep_parameter[356]= 0;	Eep_parameter[357]= 5;		//23
		Eep_parameter[358]= 0;	Eep_parameter[359]= 0;		//24
		Eep_parameter[360]= 0;	Eep_parameter[361]= 5;		//25
	
/****************************************************��ʼ��������������***************************************************/
		for(i=0; i<50; i++)	
		{
		 	Eep_parameter[2*i+400] =0x27;Eep_parameter[2*i+401] =0x10;			//���ز�A�������������
			Eep_parameter[2*i+500] =0;Eep_parameter[2*i+501] =0;
			Eep_parameter[2*i+600] =0x27;Eep_parameter[2*i+601] =0x10;			//���ز�B�������������
			Eep_parameter[2*i+700] =0;Eep_parameter[2*i+701] =0;
			Eep_parameter[2*i+800] =0x27;Eep_parameter[2*i+801] =0x10;			//���ز�C�������������
			Eep_parameter[2*i+900] =0;Eep_parameter[2*i+901] =0;
		}
		
		Eep_parameter[500] =0x27;Eep_parameter[501] =0x10;//������Ǹ�Ϊ��ƽ���ֵ����
		Eep_parameter[700] =0x27;Eep_parameter[701] =0x10;
		Eep_parameter[900] =0x27;Eep_parameter[901] =0x10;
		//30K��Ĭ����������
//		Axiang
		Eep_parameter[400] =0x29;Eep_parameter[401] =0x04;//0 10500
		Eep_parameter[412] =0x27;Eep_parameter[413] =0xAB;//7
		Eep_parameter[432] =0x2A;Eep_parameter[433] =0xF8;//17		
		
		Eep_parameter[504] =0x2;Eep_parameter[505] =0xC3;//3
		Eep_parameter[508] =0x4;Eep_parameter[509] =0xAB;//5
		Eep_parameter[512] =0x6;Eep_parameter[513] =0xBD;//7
		Eep_parameter[516] =0x8;Eep_parameter[517] =0xB1;//9
		Eep_parameter[520] =0xA;Eep_parameter[521] =0x48;//11
		Eep_parameter[524] =0xC;Eep_parameter[525] =0xE1;//13
		Eep_parameter[528] =0x10;Eep_parameter[529] =0x18;//15
		Eep_parameter[532] =0x11;Eep_parameter[533] =0x7C;//17
		Eep_parameter[536] =0x11;Eep_parameter[537] =0x7C;//19
		Eep_parameter[540] =0x13;Eep_parameter[541] =0x0C;//21
		Eep_parameter[544] =0x16;Eep_parameter[545] =0x2C;//23
		Eep_parameter[548] =0x17;Eep_parameter[549] =0xBC;//25
		
		//b
		Eep_parameter[604] =0x27;Eep_parameter[605] =0x74;
		Eep_parameter[608] =0x26;Eep_parameter[609] =0xBB;
		Eep_parameter[612] =0x26;Eep_parameter[613] =0xE7;
		Eep_parameter[616] =0x29;Eep_parameter[617] =0x7B;
		Eep_parameter[620] =0x24;Eep_parameter[621] =0xBC;
		Eep_parameter[624] =0x23;Eep_parameter[625] =0xA8;
	//	Eep_parameter[644] =0x25;Eep_parameter[645] =0x33;
	//	Eep_parameter[648] =0x21;Eep_parameter[649] =0x9C;
		
	//	Eep_parameter[704] =0x00;Eep_parameter[705] =0xB4;
	//	Eep_parameter[708] =0x00;Eep_parameter[709] =0x9D;
		Eep_parameter[712] =0x00;Eep_parameter[713] =0x64;
		Eep_parameter[716] =0x01;Eep_parameter[717] =0xA4;
		Eep_parameter[720] =0x8C;Eep_parameter[721] =0x3C;
		Eep_parameter[724] =0x00;Eep_parameter[725] =0x32;
		Eep_parameter[744] =0x05;Eep_parameter[745] =0x46;
		Eep_parameter[748] =0x05;Eep_parameter[749] =0x78;
	
/****************************************************��ʼ��ARM��������***************************************************/			
		for(i=0;i<600;i++)
		{
			Eep_parameter[i+1000] =0;
		}
		Eep_parameter[1000]=0;	Eep_parameter[1001]=1;							//ͨѶ��ַ	
		Eep_parameter[1002]=0;	Eep_parameter[1003]=3;							//ͨѶ������
		Eep_parameter[1004]=0;	Eep_parameter[1005]=1;							//ͨѶ���ȼ� 0:��������  1:Զ������
		Eep_parameter[1006]=0;	Eep_parameter[1007]=1;							//�ϵ�Ĭ�Ͽ���״̬ 0:�ػ� 1:����
		Eep_parameter[1008]=0;	Eep_parameter[1009]=0;							//��ʱ1	ʹ��
		Eep_parameter[1010]=0;	Eep_parameter[1011]=0;							//��ʱ1	ʱ
		Eep_parameter[1012]=0;	Eep_parameter[1013]=0;							//��ʱ1	��
		Eep_parameter[1014]=0;	Eep_parameter[1015]=0;							//��ʱ1	ʱ
		Eep_parameter[1016]=0;	Eep_parameter[1017]=0;							//��ʱ1	��
		Eep_parameter[1018]=0;	Eep_parameter[1019]=0;							//��ʱ2	ʹ��
		Eep_parameter[1020]=0;	Eep_parameter[1021]=0;							//��ʱ2	ʱ
		Eep_parameter[1022]=0;	Eep_parameter[1023]=0;							//��ʱ2	��
		Eep_parameter[1024]=0;	Eep_parameter[1025]=0;							//��ʱ2	ʱ
		Eep_parameter[1026]=0;	Eep_parameter[1027]=0;							//��ʱ2	��
		Eep_parameter[1028]=0;	Eep_parameter[1029]=0;							//��ʱ3	ʹ��
		Eep_parameter[1030]=0;	Eep_parameter[1031]=0;							//��ʱ3	ʱ
		Eep_parameter[1032]=0;	Eep_parameter[1033]=0;							//��ʱ3	��
		Eep_parameter[1034]=0;	Eep_parameter[1035]=0;							//��ʱ3	ʱ
		Eep_parameter[1036]=0;	Eep_parameter[1037]=0;							//��ʱ3	��
		Eep_parameter[1038]=0;	Eep_parameter[1039]=0;							//��ʱ4 	ʹ��
		Eep_parameter[1040]=0;	Eep_parameter[1041]=0;							//��ʱ4	ʱ
		Eep_parameter[1042]=0;	Eep_parameter[1043]=0;							//��ʱ4	��
		Eep_parameter[1044]=0;	Eep_parameter[1045]=0;							//��ʱ4	ʱ
		Eep_parameter[1046]=0;	Eep_parameter[1047]=0;							//��ʱ4	��
		Eep_parameter[1048]=0;	Eep_parameter[1049]=1;							//�й���ƽ��
		Eep_parameter[1050]=0;	Eep_parameter[1051]=0;							//�޹���ƽ��
		Eep_parameter[1052]=0;	Eep_parameter[1053]=0;							//г����ƽ��
		Eep_parameter[1054]=0;	Eep_parameter[1055]=0;							//���ڲ�ƽ��
		Eep_parameter[1056]=0;	Eep_parameter[1057]=1;							//ֱ��ѡ��
		Eep_parameter[1058]=0x27;Eep_parameter[1059]=0x10;							//����1
		Eep_parameter[1060]=0x27;Eep_parameter[1061]=0x10;							//����2
		Eep_parameter[1062]=0x27;Eep_parameter[1063]=0x10;							//����3
		Eep_parameter[1064]=0x27;Eep_parameter[1065]=0x10;							//����4
		Eep_parameter[1066]=0x27;Eep_parameter[1067]=0x10;							//����5
		Eep_parameter[1068]=0x27;Eep_parameter[1069]=0x10;							//����6
		
		Eep_parameter[1188]=0x27;Eep_parameter[1189]=0x10;							//����7
		Eep_parameter[1191]=1;
		Eep_parameter[1192]=0;Eep_parameter[1193]=0x1E;							//��������Ĭ��30
		Eep_parameter[1194]=0;//����ģʽΪ0���ֶ�
		Eep_parameter[1195]=100;//��������Ϊ100
		
		Eep_parameter[1196]=0x5A;Eep_parameter[1197]=0x3C;							//��ʼ�汾A
		Eep_parameter[1198]=0x27;Eep_parameter[1199]=0x10;							//����8	��ѹ������
		
		Eep_parameter[1070]=0;	Eep_parameter[1071]=0;							//��Ŀ����
		Eep_parameter[1072]=0;	Eep_parameter[1073]=0;							//��Ŀ����
		Eep_parameter[1074]=0;	Eep_parameter[1075]=0;							//�������
		
		tmp32 =EepromVersion;													//�����汾
		Eep_parameter[ParameterInitNumMax-4]=(uint8)(tmp32>>24);
		Eep_parameter[ParameterInitNumMax-3]=(uint8)(tmp32>>16);				//Eeprom �����汾��λ
		Eep_parameter[ParameterInitNumMax-2]=(uint8)(tmp32>>8);
		Eep_parameter[ParameterInitNumMax-1]=(uint8)(tmp32);					//Eeprom �����汾��λ
		
		I2C_write;  
		WriteEeprom(EepParameterAddr,Eep_parameter,ParameterInitNumMax); 
		I2C_read;
		//I2C_write;                           									//д��I2C
		//WriteEeprom(EepParameterInitAddr,Eep_parameter,2);	//д����������I2C	
	  //I2C_read;

		delay(500);
		
		I2C_write;                           									//д��I2C
		WriteEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);	//д����������I2C	
	  	I2C_read;
		//ReadEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);		//��ȡ24C32�ڵĳ�������
	}	
/****************************************************��ʼ����ǰ����***************************************************/ 	
 	ReadEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);        		//��ȡ24C32�ڵĵ�ǰ�豸����

	if(Init_parameter[ParameterNumMax-4]!=0xA5
	&&Init_parameter[ParameterNumMax-3]!=0xA5
	&&Init_parameter[ParameterNumMax-2]!=0xA5
	&&Init_parameter[ParameterNumMax-1]!=0xA5)	//�ж��Ƿ�д��
 	{
	  	for(i=0;i<1600;i++)
		{
			Init_parameter[i] =Eep_parameter[i];								//�ָ�����	
		}
		
		Init_parameter[1568]=0;	Init_parameter[1569]=0;							//�ۼƽ����� HH
		Init_parameter[1570]=0;	Init_parameter[1571]=0;							//�ۼƽ����� H
		Init_parameter[1572]=0;	Init_parameter[1573]=0;							//�ۼƽ����� L
		Init_parameter[1574]=0;	Init_parameter[1575]=0;							//�ۼƽ����� LL
		Init_parameter[1576]=0;	Init_parameter[1577]=0;							//�ۼ�����ʱ��H
		Init_parameter[1578]=0;	Init_parameter[1579]=0;							//�ۼ�����ʱ��L
		Init_parameter[1580]=0;	Init_parameter[1581]=0;							//���ս����� H
		Init_parameter[1582]=0;	Init_parameter[1583]=0;							//���ս����� L
		Init_parameter[1584]=0;	Init_parameter[1585]=0;							//��������ʱ��H
		Init_parameter[1586]=0;	Init_parameter[1587]=0;							//��¼��
		Init_parameter[1588]=0;	Init_parameter[1589]=0;							//��¼��
		Init_parameter[1590]=0;	Init_parameter[1291]=0;							//��¼��
		Init_parameter[1592]=0;	Init_parameter[1593]=0;							//Ԥ��
		Init_parameter[1594]=0;	Init_parameter[1595]=0;							//Ԥ��

		Init_parameter[ParameterNumMax-4] =0xA5;
		Init_parameter[ParameterNumMax-3] =0xA5;
		Init_parameter[ParameterNumMax-2] =0xA5;
		Init_parameter[ParameterNumMax-1] =0xA5;
		I2C_write;
		WriteEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);   					//����������д����ʽ������
		I2C_read;																			//��ȡI2C

		delay(500);

		I2C_write;
		WriteEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);   					//����������д����ʽ������
		I2C_read;
		
 	}
/*********************************************************************************************************************/	
	for(i=0;i<100;i++)
	{
		Passive_parameter[i] =(Init_parameter[2*i+200]<<8)|Init_parameter[2*i+201];						//��ʼ����Դ������
	}
	for(i=0;i<100;i++)
	{	
		main_parameter[i]=(Init_parameter[2*i]<<8)|Init_parameter[2*i+1];								//��ʼ��������
	}
	if(main_parameter[72]==2)																			//�ֶ�Ͷ��
	{
		ManualPassiveSwitchFlag=1;
	}
	else 
	{
		ManualPassiveSwitchFlag =0;
	}
	/*for(i=0;i<10;i++)
	{
		slave_Enable[i]= main_parameter[i+89];															//�ӻ�ʹ��
	}*/
	// for(i=0;i<50;i++)
	// {
	// 	Selected_parameter[i] =0;																		//ѡ�β���
	// }
	// for(i=0;i<19;i++)																	//��ѡ��Ҫ������Ƶ�δ�
	// {
	// 	j=main_parameter[31+i];
	// 	if(j>0)Selected_parameter[j-1]=1;
	// }
	for(i=0; i<100; i++)	
	{
		load_correctionA[i] =	(Init_parameter[2*i+400]<<8)|Init_parameter[2*i+401];
		load_correctionB[i] =	(Init_parameter[2*i+600]<<8)|Init_parameter[2*i+601];
		load_correctionC[i] =	(Init_parameter[2*i+800]<<8)|Init_parameter[2*i+801];							//��ʼ����������

	}
	
	LocalAddr =((Init_parameter[1000]<<8)|Init_parameter[1001]);													//ARM ������
	UART_BPS  =((Init_parameter[1002]<<8)|Init_parameter[1003]);
	RemoteEnable=((Init_parameter[1004]<<8)|Init_parameter[1005]);
	
	
	OnOffStatus=(Init_parameter[1006]<<8|Init_parameter[1007]);
	if(OnOffStatus ==0)
	{
		OnOffCommand =0;
		//RemoteOnOff =0;
	}
	else 
	{
		OnOffCommand =1;
		//RemoteOnOff =1;
	}
	
	CT_MAIN1=(Init_parameter[1008]<<8|Init_parameter[1009]);//CT1
	CT_MAIN2=(Init_parameter[1010]<<8|Init_parameter[1011]);//CT2

	// AlarmTime1[0]=(Init_parameter[1008]<<8|Init_parameter[1009]);//��ʱ1	ʹ��
	// AlarmTime1[1]=(Init_parameter[1010]<<8|Init_parameter[1011]);//��ʱ1	ʱ
	// AlarmTime1[2]=(Init_parameter[1012]<<8|Init_parameter[1013]);//��ʱ1	��
	// AlarmTime1[3]=(Init_parameter[1014]<<8|Init_parameter[1015]);//��ʱ1	ʹ��
	// AlarmTime1[4]=(Init_parameter[1016]<<8|Init_parameter[1017]);//��ʱ1	ʱ
	
	// AlarmTime2[0]=(Init_parameter[1018]<<8|Init_parameter[1019]);//��ʱ2	ʹ��
	// AlarmTime2[1]=(Init_parameter[1020]<<8|Init_parameter[1021]);//��ʱ2	ʱ
	// AlarmTime2[2]=(Init_parameter[1022]<<8|Init_parameter[1023]);//��ʱ2	��
	// AlarmTime2[3]=(Init_parameter[1024]<<8|Init_parameter[1025]);//��ʱ2	ʱ					
	// AlarmTime2[4]=(Init_parameter[1026]<<8|Init_parameter[1027]);//��ʱ2	��
	
	// AlarmTime3[0]=(Init_parameter[1028]<<8|Init_parameter[1029]);//��ʱ3	ʱ
	// AlarmTime3[1]=(Init_parameter[1030]<<8|Init_parameter[1031]);//��ʱ3	��	
	// AlarmTime3[2]=(Init_parameter[1032]<<8|Init_parameter[1033]);//��ʱ3 ʹ��
	// AlarmTime3[3]=(Init_parameter[1034]<<8|Init_parameter[1035]);//��ʱ3	ʱ
	// AlarmTime3[4]=(Init_parameter[1036]<<8|Init_parameter[1037]);//��ʱ3	��	

	ntc_type=Init_parameter[1038];
	//��ʹ��
//	AlarmTime4[0]=(Init_parameter[1038]<<8|Init_parameter[1039]);//��ʱ4 ʹ��
//	AlarmTime4[1]=(Init_parameter[1040]<<8|Init_parameter[1041]);//��ʱ4	ʱ
//	AlarmTime4[2]=(Init_parameter[1042]<<8|Init_parameter[1043]);//��ʱ4	��
//	AlarmTime4[3]=(Init_parameter[1044]<<8|Init_parameter[1045]);//��ʱ4	ʱ
//	AlarmTime4[4]=(Init_parameter[1046]<<8|Init_parameter[1047]);//��ʱ4	��
	
	// ActiveBal =(Init_parameter[1048]<<8|Init_parameter[1049]);//�й���ƽ��
	// ReactiveBal=(Init_parameter[1050]<<8|Init_parameter[1051]);//�޹���ƽ��
	// HarmonicBal=(Init_parameter[1052]<<8|Init_parameter[1053]);//г����ƽ��
	// ApparentBal=(Init_parameter[1054]<<8|Init_parameter[1055]);//���ڲ�ƽ��
	// SetupMode=(Init_parameter[1056]<<8|Init_parameter[1057]);
	xiuzhen_dianliujibian=(Init_parameter[1058]<<8|Init_parameter[1059]);
	xiuzhen_fuzhi=(Init_parameter[1060]<<8|Init_parameter[1061]);
	xiuzhen_pf=(Init_parameter[1062]<<8|Init_parameter[1063]);
	xiuzhen_cos=(Init_parameter[1064]<<8|Init_parameter[1065]);
	xiuzhen_shuchu=(Init_parameter[1066]<<8|Init_parameter[1067]);
	xiuzhen_wugong=(Init_parameter[1068]<<8|Init_parameter[1069]);
	


	ProjectNo=	((Init_parameter[1070]<<24)\
				 |(Init_parameter[1071]<<16)\
				 |(Init_parameter[1072]<<8)\
				 |(Init_parameter[1073]));
	ProductionNo=(Init_parameter[1074]<<8|Init_parameter[1075]);
	tmp64Val=1000;
	tmp64Val =(uint64) (ProjectNo)*tmp64Val;
	SerialNumber = 2000000000000000+tmp64Val+ProductionNo;
	
	// Position[0]=(Init_parameter[1092]<<8|Init_parameter[1093]);
	// Group[0]=(Init_parameter[1094]<<8|Init_parameter[1095]);
	// Capacitance[0]=(Init_parameter[1096]<<8|Init_parameter[1097]);

	// Position[1]=(Init_parameter[1098]<<8|Init_parameter[1099]);
	// Group[1]=(Init_parameter[1100]<<8|Init_parameter[1101]);
	// Capacitance[1]=(Init_parameter[1102]<<8|Init_parameter[1103]);

	// Position[2]=(Init_parameter[1104]<<8|Init_parameter[1105]);
	// Group[2]=(Init_parameter[1106]<<8|Init_parameter[1107]);
	// Capacitance[2]=(Init_parameter[1108]<<8|Init_parameter[1109]);

	// Position[3]=(Init_parameter[1110]<<8|Init_parameter[1111]);
	// Group[3]=(Init_parameter[1112]<<8|Init_parameter[1113]);
	// Capacitance[3]=(Init_parameter[1114]<<8|Init_parameter[1115]);

	// Position[4]=(Init_parameter[1116]<<8|Init_parameter[1117]);
	// Group[4]=(Init_parameter[1118]<<8|Init_parameter[1119]);
	// Capacitance[4]=(Init_parameter[1120]<<8|Init_parameter[1121]);

	// Position[5]=(Init_parameter[1122]<<8|Init_parameter[1123]);
	// Group[5]=(Init_parameter[1124]<<8|Init_parameter[1125]);
	// Capacitance[5]=(Init_parameter[1126]<<8|Init_parameter[1127]);

	// Position[6]=(Init_parameter[1128]<<8|Init_parameter[1129]);
	// Group[6]=(Init_parameter[1130]<<8|Init_parameter[1131]);
	// Capacitance[6]=(Init_parameter[1132]<<8|Init_parameter[1133]);

	// Position[7]=(Init_parameter[1134]<<8|Init_parameter[1135]);
	// Group[7]=(Init_parameter[1136]<<8|Init_parameter[1137]);
	// Capacitance[7]=(Init_parameter[1138]<<8|Init_parameter[1139]);

	// Position[8]=(Init_parameter[1140]<<8|Init_parameter[1141]);
	// Group[8]=(Init_parameter[1142]<<8|Init_parameter[1143]);
	// Capacitance[8]=(Init_parameter[1144]<<8|Init_parameter[1145]);

	// OutCTDirectionA=(Init_parameter[1146]<<8|Init_parameter[1147]);
	// OutCTDirectionB=(Init_parameter[1148]<<8|Init_parameter[1149]);
	// OutCTDirectionC=(Init_parameter[1150]<<8|Init_parameter[1151]);
	
	// Position[9]=(Init_parameter[1152]<<8|Init_parameter[1153]);
	// Group[9]=(Init_parameter[1154]<<8|Init_parameter[1155]);
	// Capacitance[9]=(Init_parameter[1156]<<8|Init_parameter[1157]);

	// Position[10]=(Init_parameter[1158]<<8|Init_parameter[1159]);
	// Group[10]=(Init_parameter[1160]<<8|Init_parameter[1161]);
	// Capacitance[10]=(Init_parameter[1162]<<8|Init_parameter[1163]);

	// Position[11]=(Init_parameter[1164]<<8|Init_parameter[1165]);
	// Group[11]=(Init_parameter[1166]<<8|Init_parameter[1167]);
	// Capacitance[11]=(Init_parameter[1168]<<8|Init_parameter[1169]);

	// Position[12]=(Init_parameter[1170]<<8|Init_parameter[1171]);
	// Group[12]=(Init_parameter[1172]<<8|Init_parameter[1173]);
	// Capacitance[12]=(Init_parameter[1174]<<8|Init_parameter[1175]);

	// Position[13]=(Init_parameter[1176]<<8|Init_parameter[1177]);
	// Group[13]=(Init_parameter[1178]<<8|Init_parameter[1179]);
	// Capacitance[13]=(Init_parameter[1180]<<8|Init_parameter[1181]);

	// Position[14]=(Init_parameter[1182]<<8|Init_parameter[1183]);
	// Group[14]=(Init_parameter[1184]<<8|Init_parameter[1185]);
	// Capacitance[14]=(Init_parameter[1186]<<8|Init_parameter[1187]);
	xiuzhen_jibian=(Init_parameter[1188]<<8|Init_parameter[1189]);
	// AI_select=Init_parameter[1190];
	enhance=Init_parameter[1191];
	xiuzhen_rongliang=(Init_parameter[1192]<<8|Init_parameter[1193]);
	xiuzhen_mode=Init_parameter[1194];
	xiuzhen_bili=Init_parameter[1195];
	
	ini_version_a=(Init_parameter[1196]<<8|Init_parameter[1197]);
	xiuzhen_dianyajibian=(Init_parameter[1198]<<8|Init_parameter[1199]);
	
	//����Ͷ��
	for(i=0;i<100;i++)
	{
		spcial_parameter[i]=(Init_parameter[1200+2*i]<<8|Init_parameter[1201+2*i]);
	}

	
	// for(i=0;i<32;i++)
	// {
	// 	PassiveChannel[i]=(Init_parameter[1200+2*i]<<8|Init_parameter[1201+2*i]);
	// }
	// for(i=0;i<32;i++)
	// {
	// 	PassiveValue[i]=(Init_parameter[1264+2*i]<<8|Init_parameter[1265+2*i]);
	// }
	// PassiveCom =(Init_parameter[1328]<<8|Init_parameter[1329]);
	/*for(i=0;i<11;i++)
	{
		CorrectParam[i]=(Init_parameter[1330+2*i]<<8|Init_parameter[1331+2*i]);
	}*/
	// if(SetupMode==0)
	// {
	// 	CabinProcess();
	// }
	// if(SetupMode==1)
	// {
	// 	ChannelProcess();
	// }
	//ProjectNo =2016072801;
	//ProductionNo =1;
	
//ARM������ʼ��	

	for(i=0;i<100;i++) tmp[i]=0;//����

	tmp[0] = LocalAddr;				//0564
	tmp[1] = UART_BPS;				//0565
	tmp[2] =RemoteEnable;			//0566
	tmp[3] =local_time.RTC_Year;	//0567
	tmp[4] =local_time.RTC_Mon;		//0568
	tmp[5] =local_time.RTC_Mday;	//0569
	tmp[6] =local_time.RTC_Hour;	//056A
	tmp[7] =local_time.RTC_Min;		//056B
	
	tmp[8] =OnOffStatus;			//056C
	
	tmp[9] = CT_MAIN1;			//056D
	tmp[10] =CT_MAIN2;			//056E
	tmp[11] =AlarmTime1[2];			//056F
	tmp[12] =AlarmTime1[3];			//0570
	tmp[13] =AlarmTime1[4];			//0571
	
	tmp[14] =AlarmTime2[0];			//0572
	tmp[15] =AlarmTime2[1];			//0573
	tmp[16] =AlarmTime2[2];			//0574
	tmp[17] =AlarmTime2[3];			//0575
	tmp[18] =AlarmTime2[4];			//0576
	
	tmp[19] =AlarmTime3[0];			//0577
	tmp[20] =AlarmTime3[1];			//0578
	tmp[21] =AlarmTime3[2];			//0579
	tmp[22] =AlarmTime3[3];			//057A
	tmp[23] =AlarmTime3[4];			//057B

	tmp[24] =ntc_type;//AlarmTime4[0];			//057C
	tmp[25] =0;			//057D
	tmp[26] =0;			//057E
	tmp[27] =0;			//057F
	tmp[28] =0;			//0580

	// HarmonicBal =main_parameter[6]/100;tmp16 =main_parameter[6]%100;
	// ReactiveBal =tmp16/10;			   tmp16 =main_parameter[6]%10;
	// ActiveBal	=tmp16;
	// if(ReactiveBal==1||HarmonicBal==1)ApparentBal=1;
	
	// tmp[29] =ActiveBal;			//0581
	// tmp[30] =ReactiveBal;		//0582
	// tmp[31] =HarmonicBal;		//0583
	// tmp[32] =ApparentBal;		//0584
	// tmp[33] =SetupMode;		//0585
	// tmp[34] =enhance;		//0586
	tmp[35] =0;		//0587
	tmp[36] =0;		//0588
	tmp[37] =0;		//0589
	tmp[38] =0;		//058A
	tmp[39] =0;		//058B

	tmp[40] =(uint16)(ProjectNo>>16);//058C
	tmp[41] =(uint16)ProjectNo;		//058D
	tmp[42] =ProductionNo;			//058E

	//VolOnOffEnable =main_parameter[73]%10;
	// tmp[43] =0;						//��ѹ����ʹ��//058F
	// CurOnOffEnable =main_parameter[73]/10;
	// tmp[44] =CurOnOffEnable;		//��������ʹ��//0590

	// MainCTPhase =main_parameter[10]/10000;tmp16 =main_parameter[10]%10000;
	// MainCTDirectionC=tmp16/1000;tmp16 =main_parameter[10]%1000;
	// MainCTDirectionB=tmp16/100;tmp16 =main_parameter[10]%100;
	// MainCTDirectionA=tmp16/10;tmp16 =main_parameter[10]%10;
	// MainCTLocation  =tmp16;
	
	// tmp[45] =MainCTLocation;		//��������λ��//0591
	// tmp[46] =MainCTDirectionA;		//������������A//0592
	// tmp[47] =MainCTDirectionB;		//������������B//0593
	// tmp[48] =MainCTDirectionC;		//������������C//0594
	// tmp[49] =MainCTPhase; 			//������������//0595
	
	// OutCTPhase =main_parameter[15]/10000;tmp16 =main_parameter[15]%10000;
	// OutCTDirectionC=tmp16/1000;tmp16 =main_parameter[15]%1000;
	// OutCTDirectionB=tmp16/100;tmp16 =main_parameter[15]%100;
	// OutCTDirectionA=tmp16/10;tmp16 =main_parameter[15]%10;
	
	// tmp[50] =OutCTPhase; 			//��������������//0596
	
	// tmp[51] =Position[0];			//��λ2	
	// tmp[52] =Group[0];				//��2
	// tmp[53] =Capacitance[0];		//��ֵ2

	// tmp[54] =Position[1];			//��λ3	
	// tmp[55] =Group[1];				//��3
	// tmp[56] =Capacitance[1];		//��ֵ3

	// tmp[57] =Position[2];			//��λ4	
	// tmp[58] =Group[2];				//��4
	// tmp[59] =Capacitance[2];		//��ֵ4

	// tmp[60] =Position[3];			//��λ5	
	// tmp[61] =Group[3];				//��5
	// tmp[62] =Capacitance[3];		//��ֵ5

	// tmp[63] =Position[4];			//��λ6	
	// tmp[64] =Group[4];				//��6
	// tmp[65] =Capacitance[4];		//��ֵ6

	// tmp[66] =Position[5];			//��λ7	
	// tmp[67] =Group[5];				//��7
	// tmp[68] =Capacitance[5];		//��ֵ7

	// tmp[69] =Position[6];			//��λ8	
	// tmp[70] =Group[6];				//��8
	// tmp[71] =Capacitance[6];		//��ֵ8

	// tmp[72] =Position[7];			//��λ9	
	// tmp[73] =Group[7];				//��9
	// tmp[74] =Capacitance[7];		//��ֵ9

	// tmp[75] =Position[8];			//��λ10	
	// tmp[76] =Group[8];				//��10
	// tmp[77] =Capacitance[8];		//��ֵ10

	// tmp[78] =OutCTDirectionA;		//�������������A
	// tmp[79] =OutCTDirectionB;		//�������������B
	// tmp[80] =OutCTDirectionC;		//�������������C

	// tmp[81] =Position[9];			//��λ11	
	// tmp[82] =Group[9];				//��11
	// tmp[83] =Capacitance[9];		//��ֵ11

	// tmp[84] =Position[10];			//��λ12	
	// tmp[85] =Group[10];				//��12
	// tmp[86] =Capacitance[10];		//��ֵ12

	// tmp[87] =Position[11];			//��λ13	
	// tmp[88] =Group[11];				//��13
	// tmp[89] =Capacitance[11];		//��ֵ13

	// tmp[90] =Position[12];			//��λ14	
	// tmp[91] =Group[12];				//��14
	// tmp[92] =Capacitance[12];		//��ֵ14

	// tmp[93] =Position[13];			//��λ15	
	// tmp[94] =Group[13];				//��15
	// tmp[95] =Capacitance[13];		//��ֵ15

	// tmp[96] =Position[14];			//��λ16
	// tmp[97] =Group[14];				//��16
	// tmp[98] =Capacitance[14];		//��16
	// tmp[99] =0;						//Ԥ��
	
	for(i=0; i<100; i++)	
	{
		ARM_param[i] =	tmp[i];//ARM������
	}
	
	
	TotalEnergyHB =((Init_parameter[1568]<<24)\
				 |(Init_parameter[1569]<<16)\
				 |(Init_parameter[1570]<<8)\
				 |(Init_parameter[1571]));
	
	TotalEnergyLB =((Init_parameter[1572]<<24)\
				 |(Init_parameter[1573]<<16)\
				 |(Init_parameter[1574]<<8)\
				 |(Init_parameter[1575]));
	//TotalEnergyHB =0;
	//TotalEnergyLB =1000;
	
	TotalRunningTime =((Init_parameter[1576]<<24)\
					|(Init_parameter[1577]<<16)\
					|(Init_parameter[1578]<<8)\
					|(Init_parameter[1579]));

	tmp64H =(uint64)(TotalEnergyHB);
	tmp64L =(uint64)(TotalEnergyLB);
	tmp64Val =(uint64)((tmp64H<<32|tmp64L)*1000/997);
	Co2HB =(uint32)(tmp64Val>>32);
	Co2LB =(uint32)(tmp64Val);

	DailyEnergy =((Init_parameter[1580]<<24)|(Init_parameter[1581]<<16)|(Init_parameter[1582]<<8)|Init_parameter[1583]);
	DailyRunningTime =((Init_parameter[1584]<<8)|Init_parameter[1585]);
	
	EepromYear =(Init_parameter[1586]<<8|Init_parameter[1587]);
	EepromMon =(Init_parameter[1588]<<8|Init_parameter[1589]);
	EepromDay =(Init_parameter[1590]<<8|Init_parameter[1591]);

	//TotalEnergyBakHB =((Init_parameter[1208]<<24)|(Init_parameter[1209]<<16)|(Init_parameter[1210]<<8)|Init_parameter[1211]);
	//TotalEnergyBakLB =((Init_parameter[1212]<<24)|(Init_parameter[1213]<<16)|(Init_parameter[1214]<<8)|Init_parameter[1215]);
	//TotalRunningTimeBak =((Init_parameter[1216]<<24)|(Init_parameter[1217]<<16)|(Init_parameter[1018]<<8)|Init_parameter[1219]);

}
/******************************************************************************
** ������: GZ_init
**
** ���ܣ���ȡ���Ϻ�г��������������
******************************************************************************/
void GZ_init(void)
{
	uint8 tmp[1664];
	uint16 i=0;
 	ReadEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);                        //��ȡ���ϼ�¼
 	
 	if(gz_dan[159]!=0)                                                   		//��һ��ʹ�ã���¼��0
 	{	  															   
  		for(i=0; i<FaultDataNumMax; i++) {gz_dan[i] = 0;}						//������
  		I2C_write;                                                           	//д��I2C	   				  
   		WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);		    		//�����¼��I2C		    
  		I2C_read; 		    
 	}

	ReadEeprom(EepSlaveFaultAddr,tmp,SlaveFaultNumMax);                        //��ȡ���ϼ�¼

	if(tmp[1663] !=0)
	{
		for(i=0; i<SlaveFaultNumMax; i++) {tmp[i] = 0;}						//������
  		I2C_write;                                                           	//д��I2C	   				  
   		WriteEeprom(EepSlaveFaultAddr,tmp,SlaveFaultNumMax);		    		//�����¼��I2C		    
  		I2C_read; 
	}
	for(i=0;i<100;i++)
	{
		SlaveFault[0][i] =tmp[i];
		SlaveFault[1][i] =tmp[i+104];
		SlaveFault[2][i] =tmp[i+208];
		SlaveFault[3][i] =tmp[i+312];
		SlaveFault[4][i] =tmp[i+416];
		// SlaveFault[5][i] =tmp[i+520];
		// SlaveFault[6][i] =tmp[i+624];
		// SlaveFault[7][i] =tmp[i+728];
		// SlaveFault[8][i] =tmp[i+832];
		// SlaveFault[9][i] =tmp[i+936];
		// SlaveFault[10][i] =tmp[i+1040];
		// SlaveFault[11][i] =tmp[i+1144];
		// SlaveFault[12][i] =tmp[i+1248];
		// SlaveFault[13][i] =tmp[i+1352];
		// SlaveFault[14][i] =tmp[i+1456];
		// SlaveFault[15][i] =tmp[i+1560];
	}

	for(i=0;i<800;i++)
	{
		DR_LOG[i]=tmp[i+520];//��ȡ���ݼ�¼
	}

	ReadEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);                        //��ȡ���ϼ�¼

	if(Event_Logging[159] !=0)
	{
		for(i=0; i<EventLogNumMax; i++) {Event_Logging[i] = 0;}						//������
  		I2C_write;                                                           	//д��I2C	   				  
   		WriteEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);		    		//�����¼��I2C		    
  		I2C_read; 
	}
	
	ReadEeprom(EepDatalogMaxAddr,tmp,2);
	if(tmp[0]>13||tmp[1]>13)	
	{
		DatalogMax=0;
		DatalogStart=0;
		tmp[0]=(uint8)DatalogMax;
		tmp[1]=(uint8)DatalogStart;
		
		I2C_write;                                                           	//д��I2C	   				  
   		WriteEeprom(EepDatalogMaxAddr,tmp,2);		    		//�����¼��I2C		    
  		I2C_read;

		for(i=0; i<EepDatalogNum; i++) {tmp[i] = 0;}
		
		I2C_write; 
		for(i=0;i<13;i++)
		{
			WriteEeprom(DatalogStAddr[i],tmp,EepDatalogNum);
		}
		I2C_read;
	}
	else
	{
			DatalogMax= tmp[0];
			DatalogStart=tmp[1];
	}
}
/******************************************************************************
** ������:GPIO_init 
**
** ����:�ܽų�ʼ��
******************************************************************************/
void GPIO_init(void)
{
 	/********�ܽų�ʼ��*************
 	P2.8�����¶ȿ���
 	*******************************/
	LPC_PINCON->PINSEL0 = 0x80A00a5A;	//P0.0-RD1;P0.1-TD1;P0.2-ARMTXD0;P0.3-ARMRXD0;P0.4-RD2;P0.5-TD2;P0.10-astopin2;P0.11-astopin1;P0.15-SPICL;P0.0-TXD3;P0.1-RXD3	
	LPC_PINCON->PINSEL1 = 0x000003ea;	//P0.16-SPICS;P0.17-MISO;P0.18-MOSI;P0.19-aSDA;P0.20-aSCL;P0.21-awp;P0.22-ARMOUT2;P0.23-aADIN3;P0.24-aADIN4;P0.25-aADIN1;P0.26-aADIN2;P0.27-aclren;P0.29-aled4;P0.30-aled3
	LPC_PINCON->PINSEL2 = 0;			//P1.0-aioin1;P1.1-UARTCON0;P1.4-aout6;P1.9-aout7;
	LPC_PINCON->PINSEL3 = 0;			//P1.20-protect1;P1.22-protect2;P1.24-astop;P1.25-dastartin;
	LPC_PINCON->PINSEL3 = 0;
	LPC_PINCON->PINSEL4 = 0x0000000a;	//P2.0-ARMTXD1;P2.1-ARMRXD1;P2.3-aout1;P2.4-aout2;P2.5-aout3;P2.6-aout4;P2.7-aout5;P2.11-ARMOUT1;P2.12-ARMIN1;P2.13-ARMIN2
	LPC_PINCON->PINSEL5 = 0;
	LPC_PINCON->PINSEL6 = 0;
	LPC_PINCON->PINSEL7 = 0;			//P3.25-aled1;P3.26-aled2 
	LPC_PINCON->PINSEL8 = 0;
	LPC_PINCON->PINSEL9 = 0;	
	LPC_PINCON->PINSEL10 = 0;
	LPC_PINCON->PINMODE_OD0 = 0xC00;		//I2C2��Ϊ��©
	//LPC_PINCON->PINMODE3 = 0xf300;		//�����������Ϊ��������	
	LPC_GPIO0->FIODIR = 0x62E00200; 	//0110 0010 1110 0000 1000 0010 0000 0000 
	LPC_GPIO1->FIODIR = 0x30000013;		//0011 0000 0000 0000 0000 0000 0001 0011
	LPC_GPIO2->FIODIR = 0x00003804; 	//0000 0000 0000 0000 0011 1000 0000 0100
	LPC_GPIO3->FIODIR = 0x06000000; 	//0000 0110 0000 0000 0000 0000 0000 0000
	LPC_GPIO4->FIODIR = 0;
	LPC_GPIO1->FIOSET = 1<<28;			//awp
	//LPC_GPIO0->FIOSET = 1<<15;		//awp
	LPC_GPIO0->FIOSET = 1<<22;			// ARM�Ŀ��ػ�ʹ�ܹܽţ�
										//��0����ʾ����DSP�������������״̬5
										//��1����ʾ������DSP������DSP����״̬3
	

	LPC_GPIO0->FIOSET = 1<<13;			// ARM�Ĵ󿪹ػ�ʹ�ܹܽţ�
										//��0����ʾ����DSP�������������״̬3
										//��1����ʾ������DSP������DSP����״̬2
	
	LPC_GPIO2->FIOSET = 1<<11;			// ARM��SPIͨѶʹ�ܹܽ�
										//��0: ����SPIͨѶ
										//��1��������SPIͨѶ
										
	LPC_GPIO2->FIOSET = 1<<12;			//ARM��SPIд����ʹ��
										//��1����ʾSPIд��ʹ��
										//��0: ��ʾSPIдʹ��
	LPC_GPIO2->FIOSET = 1<<13;			//�ϵ� �����ܽ�Ϊ�߲�����
	LPC_GPIO0->FIOSET = 1<<27;			//aclren
	//LPC_GPIO1->FIOSET = 1<<1;			//UARTCON0
	LPC_GPIO0->FIOSET = 1<<23;			//UARTCON1
	//LPC_GPIO1->FIOCLR = 1<<24;		//astop
	//LPC_GPIO1->FIOCLR = 1<<9; 		//aout7
	//LPC_GPIO1->FIOCLR = 1<<4; 		//aout6
	//LPC_GPIO2->FIOCLR = 1<<7; 		//aout5
	//LPC_GPIO2->FIOCLR = 1<<6; 		//aout4    
	//LPC_GPIO2->FIOCLR = 1<<5; 		//aout3
	//LPC_GPIO2->FIOCLR = 1<<4; 		//aout2
	//LPC_GPIO2->FIOCLR = 1<<3; 		//aout1
	LPC_GPIO3->FIOSET = 1<<25;			//aled1
	LPC_GPIO3->FIOSET = 1<<26;			//aled2
	LPC_GPIO0->FIOSET = 1<<29;			//aled4
	LPC_GPIO0->FIOSET = 1<<30;			//aled3
	//LPC_GPIO0->FIOSET = 1<<25;			//�����
	LPC_GPIO0->FIOSET = 1<<9;			//�����
	
	
	LPC_GPIO1->FIOSET = 1<<4;			//����Reset ����
	LPC_GPIO1->FIOCLR = 1<<0;			//����Key ����-ATģʽ
	
	
}
/******************************************************************************
** ������:Bluetooth_init 
**
** ����:����ģ���ʼ��
******************************************************************************/
void Bluetooth_init(void)
{
	uint8 i,j,*SendPtr,SendData,ATcmd[26];
	uint64 temp64;
	LPC_GPIO1->FIOCLR = 1<<0;			//����Key ����-ATģʽ
/*****************************��������Ӳ��**********************************/	
	BluetoothATcmd =1;
	
	//ATcmd[4]={'A','T','\r','\N'};
	ATcmd[0]='A';
	ATcmd[1]='T';
	ATcmd[2]='\r';
	ATcmd[3]='\n';
	SendPtr =ATcmd;
	uart_write0;  //UART0 send
	for(i=4;i>0;i--)
	{
		SendData=*SendPtr;UART0_SendByte(SendData);SendPtr++;
	}
	uart_read0;  //UART0 receive
	delay(300);
	if(RecvBTFlag==1)
	{
		if(RecvSciBuf[0]=='O'||RecvSciBuf[1]=='K')
		{
			BluetoothTestOK=1;
			RecvSciBuf[0]=0;
			RecvSciBuf[1]=0;
			RecvSciBuf[2]=0;
			RecvSciBuf[3]=0;
			//BluetoothATcmd =0;
		}
	}
/*****************************�����豸�����޸�**********************************/
	if(BluetoothTestOK==1)
	{
		//ATcmd[8]={'A','T','+','N','A','M','E','='};
		for(j=0;j<5;j++)
		{
			ATcmd[0]='A';
			ATcmd[1]='T';
			ATcmd[2]='+';
			ATcmd[3]='N';
			ATcmd[4]='A';
			ATcmd[5]='M';
			ATcmd[6]='E';
			ATcmd[7]='=';
			SendPtr =ATcmd;
			uart_write0;  //UART0 send
			for(i=8;i>0;i--)
			{
				SendData=*SendPtr;UART0_SendByte(SendData);SendPtr++;
			}
			//HEX2BCD(SerialNumber,ATcmd,8);
			if(bluetooth_int==1)
			{
			ATcmd[0]=0x30+SerialNumber/1000000000000000;temp64 =SerialNumber%1000000000000000;
			ATcmd[1]=0x30+temp64/100000000000000;temp64 =temp64%100000000000000;
			ATcmd[2]=0x30+temp64/10000000000000; temp64 =temp64%10000000000000;
			ATcmd[3]=0x30+temp64/1000000000000;  temp64 =temp64%1000000000000;
			ATcmd[4]=0x30+temp64/100000000000;   temp64 =temp64%100000000000;
			ATcmd[5]=0x30+temp64/10000000000;    temp64 =temp64%10000000000;
			ATcmd[6]=0x30+temp64/1000000000;     temp64 =temp64%1000000000;
			ATcmd[7]=0x30+temp64/100000000;      temp64 =temp64%100000000;
			ATcmd[8]=0x30+temp64/10000000;		 temp64 =temp64%10000000;
			ATcmd[9]=0x30+temp64/1000000;		 temp64 =temp64%1000000;
			ATcmd[10]=0x30+temp64/100000;		 temp64 =temp64%100000;
			ATcmd[11]=0x30+temp64/10000;		 temp64 =temp64%10000;
			ATcmd[12]=0x30+temp64/1000;			 temp64 =temp64%1000;
			ATcmd[13]=0x30+temp64/100;			 temp64 =temp64%100;
			ATcmd[14]=0x30+temp64/10;			 temp64 =temp64%10;
			ATcmd[15]=0x30+temp64;
			}
			else
			{
				for(i=0;i<16;i++)
				ATcmd[i]=0x30;
			}
			SendPtr =ATcmd;
			for(i=16;i>0;i--)
			{
				SendData=*SendPtr;UART0_SendByte(SendData);SendPtr++;
			}
			ATcmd[0]='\r';
			ATcmd[1]='\n';
			SendPtr =ATcmd;
			for(i=2;i>0;i--)
			{
				SendData=*SendPtr;UART0_SendByte(SendData);SendPtr++;
			}
			uart_read0;  //UART0 receive
			delay(300);
			if(RecvBTFlag==1)
			{
				if(RecvSciBuf[0]=='O'||RecvSciBuf[1]=='K')
				{
					BluetoothChangeNameOK=1;
					BluetoothATcmd =0;
					LPC_GPIO1->FIOSET = 1<<0;			//����Key ����-͸��ģʽ
					break;
				}
			}
		}
	}
		
	
}

/******************************************************************************
** ������:TIME_init 
**
** ����:��ʱ����ʼ��
******************************************************************************/
uint32_t TIME_init(uint8 time_num)
{
  	pclock=SystemFrequency/4; 		//�ڲ�RCʱ��Ƶ��Ϊ4M Hz
 	if(time_num == 0)
	{
	 	LPC_TIM0->TC = 0;   	    //��ʱ��0��ʼ��
 		LPC_TIM0->PR = 0;   		//1��Ƶ
 		LPC_TIM0->MCR = 0x3;       	//��ʱ���Ƚϲ����жϱ�־λ��TC����������
 		//LPC_TIM0->MR0 = pclock*0.5;	//��Ƶ��96Mhz Fpclk��Ƶ����24Mhz  
 		LPC_TIM0->MR0 = pclock/1000-1; //  ((1000000/1000)-1+1)/24000000 =1ms
 		LPC_TIM0->IR = 0xff;	    // reset all interrrupts 
 		//LPC_TIM0->TCR = 0;         	//��ʹ�ܶ�ʱ������0
 		NVIC_EnableIRQ(TIMER0_IRQn);
 		LPC_TIM0->TCR =1 ;		    //ʹ�ܶ�ʱ������0
	  return (TRUE);
	}
  else if(time_num == 1)
	{
	 	LPC_TIM1->TC = 0;   	    //��ʱ��1��ʼ��
 		LPC_TIM1->PR = 0;   		//1��Ƶ
 		LPC_TIM1->MCR = 0x3;       	//��ʱ���Ƚϲ����жϱ�־λ��TC����������
 		LPC_TIM1->MR0 = pclock*2.5;//*0.5;	//��Ƶ��96Mhz Fpclk��Ƶ����24Mhz  
 		LPC_TIM1->IR = 0xff;	    // reset all interrrupts
		NVIC_EnableIRQ(TIMER1_IRQn); 
 		LPC_TIM1->TCR = 0;         	//��ʹ�ܶ�ʱ������1
	  return (TRUE);
	}
	else if(time_num == 2)
	{ 
		LPC_TIM2->TC = 0;   	    //��ʱ��0��ʼ��
 		LPC_TIM2->PR = 0;   		//1��Ƶ
 		LPC_TIM2->MCR = 0x3;       	//��ʱ���Ƚϲ����жϱ�־λ��TC����������
 		LPC_TIM2->MR0 = pclock*1.0;	//��Ƶ��96Mhz Fpclk��Ƶ����24Mhz  
 		LPC_TIM2->IR = 0xff;	    // reset all interrrupts 
		NVIC_EnableIRQ(TIMER2_IRQn); 
 		LPC_TIM2->TCR = 0;         	//��ʹ�ܶ�ʱ������0
	  return (TRUE);
	}
	else if(time_num == 3)
	{
	 	LPC_TIM3->TC = 0;   	    //��ʱ��0��ʼ��
 		LPC_TIM3->PR = 0;   		//1��Ƶ
 		LPC_TIM3->MCR = 0x3;       	//��ʱ���Ƚϲ����жϱ�־λ��TC����������
 		LPC_TIM3->MR0 = pclock*1.0;	//��Ƶ��96Mhz Fpclk��Ƶ����24Mhz  
 		LPC_TIM3->IR = 0xff;	    // reset all interrrupts 
		NVIC_EnableIRQ(TIMER3_IRQn); 
 		LPC_TIM3->TCR = 0;         	//��ʹ�ܶ�ʱ������0
	  return (TRUE);	
	}
 	return (FALSE);
}
/******************************************************************************
** ������:TIMER0_IRQHandler 
**
** ����:��ʱ��
******************************************************************************/
void TIMER0_IRQHandler (void) 
{		
	uint16 i;
	LPC_TIM0->IR = 1;


	time_1ms++;	
	time_SpiDelay++;
	TimeCount[t1ms]++;
	if(Delay35Count<0xFF)Delay35Count++;
	
	if(time_1ms>9)
	{
		/*i =ADCAisleMax;
		while(i--)
		{
			GetAD(i);
		}*/
		time_10ms++;
		time_1ms=0;
		TimeCount[t10ms]++;
	}
	if(time_10ms>9)
	{
		time_100ms++;
		time_10ms=0;
		TimeCount[t100ms]++;
		if(count_swj<0xffff)count_swj++;
	}
	if(time_100ms>9)
	{
//		DeratingJudgeFlag =1;
		time_100ms=0;
		TimeCount[t1s]++;
		if(SPICommDelay<0xff)SPICommDelay++;
		//if(OnOffDelayCount<0xffff)OnOffDelayCount++;
		//if(FaultDelayCount<0xffff)FaultDelayCount++;
		//if(WorkTimeCount<0xffff) WorkTimeCount++;
		//if(BoostTimeCount<0xffff)BoostTimeCount++;
		//if(StartDelayCount<0xffff)StartDelayCount++;
		if(Testtime_function==1)Testtime_time1++;							//������ܿ����͵ݼ�
		if(ReadCount<0xffff)ReadCount++;
		if(TimeDelay<0xffff)TimeDelay++;
		if(AccumDelay<0xffff)AccumDelay++;
		if(ResetDelay<0xffff)ResetDelay++;
		if(moni_delay<0xffff)moni_delay++;
		if(SciWaitingCount<0xffff)SciWaitingCount++;
		if(SciWaitingCount3<0xffff)SciWaitingCount3++;
		if(DatalogDelay<0xffff)DatalogDelay++;
		
		if(SystemStatus==SystemRunning)
		{
			xiuzhen_time++;
		}
		else
		{
			xiuzhen_time=0;
		}
		
		
		if(SystemStatus==SystemRunning)
		{
			if(enhance==0)
			{
				if((dsp_data[32]>(dsp_data[93]*xiuzhen_bili/100))||( dsp_data[33]>(dsp_data[93]*xiuzhen_bili/100)) ||(dsp_data[34]>(dsp_data[93]*xiuzhen_bili/100)))
				{
					if(xiuzhen_mode==2)//APF����
					{
						i=10000*(xiuzhen_rongliang+1)/dsp_data[93];
						xiuzhen_dianliujibian2=i;
						xiuzhen_fuzhi2=i;
						xiuzhen_pf2=i;
						xiuzhen_cos2=10000;
						xiuzhen_shuchu2=i;
						xiuzhen_wugong2=10000;
						xiuzhen_jibian2=i;
					}
					else if(xiuzhen_mode==1)//svg����
					{
						i=10000*(xiuzhen_rongliang+1)/dsp_data[93];
						xiuzhen_dianliujibian2=10000;
						xiuzhen_fuzhi2=10000;
						xiuzhen_pf2=10000;
						xiuzhen_cos2=i;
						xiuzhen_shuchu2=i;
						xiuzhen_wugong2=i;
						xiuzhen_jibian2=i;
					}
					else//�ֶ�����
					{
						xiuzhen_dianliujibian2=xiuzhen_dianliujibian;
						xiuzhen_fuzhi2=xiuzhen_fuzhi;
						xiuzhen_pf2=xiuzhen_pf;
						xiuzhen_cos2=xiuzhen_cos;
						xiuzhen_shuchu2=xiuzhen_shuchu;
						xiuzhen_wugong2=xiuzhen_wugong;
						xiuzhen_jibian2=xiuzhen_jibian;
						xiuzhen_dianyajibian2=xiuzhen_dianyajibian;
					}
					
				}
				else
				{
					xiuzhen_dianliujibian2=10000;
					xiuzhen_fuzhi2=10000;
					xiuzhen_pf2=10000;
					xiuzhen_cos2=10000;
					xiuzhen_shuchu2=10000;
					xiuzhen_wugong2=10000;
					xiuzhen_jibian2=10000;
					xiuzhen_dianyajibian2=10000;
				}
			}
			else 
			{
				if(xiuzhen_time>40)//ǿ�Ƶ��ֶ�����
				{
					if(xiuzhen_mode==2)//APF����
					{
						i=10000*(xiuzhen_rongliang+1)/dsp_data[93];
						xiuzhen_dianliujibian2=i;
						xiuzhen_fuzhi2=i;
						xiuzhen_pf2=i;
						xiuzhen_cos2=10000;
						xiuzhen_shuchu2=i;
						xiuzhen_wugong2=10000;
						xiuzhen_jibian2=i;
					}
					else if(xiuzhen_mode==1)//svg����
					{
						i=10000*(xiuzhen_rongliang+1)/dsp_data[93];
						xiuzhen_dianliujibian2=10000;
						xiuzhen_fuzhi2=10000;
						xiuzhen_pf2=10000;
						xiuzhen_cos2=i;
						xiuzhen_shuchu2=i;
						xiuzhen_wugong2=i;
						xiuzhen_jibian2=i;
					}
					else//�ֶ�����
					{
						xiuzhen_dianliujibian2=xiuzhen_dianliujibian;
						xiuzhen_fuzhi2=xiuzhen_fuzhi;
						xiuzhen_pf2=xiuzhen_pf;
						xiuzhen_cos2=xiuzhen_cos;
						xiuzhen_shuchu2=xiuzhen_shuchu;
						xiuzhen_wugong2=xiuzhen_wugong;
						xiuzhen_jibian2=xiuzhen_jibian;
						xiuzhen_dianyajibian2=xiuzhen_dianyajibian;
					}
				}
			}
		 }
		 else
		{
			xiuzhen_dianliujibian2=10000;
			xiuzhen_fuzhi2=10000;
			xiuzhen_pf2=10000;
			xiuzhen_cos2=10000;
			xiuzhen_shuchu2=10000;
			xiuzhen_wugong2=10000;
			xiuzhen_jibian2=10000;
			xiuzhen_dianyajibian2=10000;
		}

		
	}	

	LPC_TIM0->IR = 0;
}
/******************************************************************************
** ������:error_handle 
**
** ����:���ϴ���
******************************************************************************/
void error_handle(void)
{
	uint16 i,j,FaultNum,FaultCode;
	uint8 Tmp_Fault[104];

	if(SystemStatus !=SystemProtect && SystemStatus !=SystemPfault && SystemStatus !=SystemParameterError)	//�����Ƿ����
	{
		SystemStatusBak =SystemStatus;	//״̬���� 
		//led3_OFF;
	}
	else
	{
		//led3_ON;
		if(SystemStatus==SystemParameterError)	//��������
		{
			SystemStatusBak =SystemStatus;		//״̬���� 
		}
		if(SystemStatusBak!=SystemProtect &&SystemStatusBak!=SystemPfault&&SystemStatusBak!=SystemParameterError)//�ж��ϴ��Ƿ�Ϊ���ϵ�״̬
		{
			/*if(SystemStatus==SystemProtect)
			{
				if(ResetCount>0)
				{
					ResetCount--;
					temp[0] =(uint8)(ResetCount>>8);
					temp[1] =(uint8)(ResetCount);
					I2C_write;													//д��I2C
					WriteEeprom(EepParameterAddr+1048,temp,2);   			//����������д����ʽ������
					I2C_read;
					DUGSReadMainParameterFlag =1;
				}
			}*/
			SystemStatusBak =SystemStatus;
			if(MainErrorCode !=0)	
			{
				FaultNum =gz_dan[0];
				FaultCode =MainErrorCode;
				if(FaultNum==20)		   									//���豸����¼140������
				{ 
					for(i=1; i<141; i++)
					{
						if(i%7!=0) gz_dan[i] = gz_dan[7+i];					//��ǰ����	
					} 	   
					gz_dan[140] = 1;	                                  	//���
					gz_dan[139] = alarm_time.RTC_Year - 2000;				//��
					gz_dan[138] = alarm_time.RTC_Mon;	                    //��
					gz_dan[137] = alarm_time.RTC_Mday;	                //��
					gz_dan[136] = alarm_time.RTC_Hour;                   	//ʱ
					gz_dan[135] = alarm_time.RTC_Min;                     //��
					gz_dan[134] = FaultCode;                             	//���ϴ���
										
				}
				else
				{
					gz_dan[0]++;
					FaultNum =gz_dan[0];  
					gz_dan[7*FaultNum] = FaultNum;	                    	//���
					gz_dan[7*FaultNum-1] = alarm_time.RTC_Year - 2000;   	//��
					gz_dan[7*FaultNum-2] = alarm_time.RTC_Mon;	        //��
					gz_dan[7*FaultNum-3] = alarm_time.RTC_Mday;	        //��
					gz_dan[7*FaultNum-4] = alarm_time.RTC_Hour;          	//ʱ
					gz_dan[7*FaultNum-5] = alarm_time.RTC_Min;           	//��
					gz_dan[7*FaultNum-6] = FaultCode;                     	//���ϴ���
					for(i=0; i<FaultNum; i++)
					{
						gz_dan[(i+1)*7] = FaultNum-i;						//��ŵ���		
					}
				}
				I2C_write;                                                  //д��I2C	   
				WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);       //�����¼��I2C						    
				I2C_read;
			}
		}
	}
	
	for(j=0;j<main_parameter[99];j++)
	{
		//if(slave_Enable[j]!=22) continue;
		
		if( SlaveFaultFlag[j]!=SystemProtect && SlaveFaultFlag[j] !=SystemPfault && SlaveFaultFlag[j] !=SystemParameterError)//�ж��Ƿ���ڹ���
		{
			SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//û�й��Ͼͼ�¼��ǰ״̬����
		}
		else//���ڹ���
		{
			if(SlaveFaultFlag[j]==SystemParameterError)//�ж��Ƿ��ǲ����������
			{
				SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//���ݲ����������
			}
			if(SlaveFaultFlagBak[j]!=SystemProtect &&SlaveFaultFlagBak[j]!=SystemPfault&&SlaveFaultFlagBak[j]!=SystemParameterError)//����ϴα��ݵ�״̬���ǹ���״̬
			{
				/*if(SlaveFaultFlag[j]==SystemPfault)
				{
					if(slave_Reset[j]>0)
					{
						slave_Reset[j]--;
						temp[0] =(uint8)(slave_Reset[j]>>8);
						temp[1] =(uint8)(slave_Reset[j]);
		
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr+1050+2*j,temp,2);   			//����������д����ʽ������
						I2C_read;

						DUGSReadMainParameterFlag =1;
					}
				}*/
				SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//���ݵ�ǰ״̬
				if(SlaveErrorCode[j]	>	4)		//�����ǰ���ݵ�״̬����4
				{
					if(SlaveErrorCode[j]	!=	0)		//�ӻ����ϴ���budengyu 0�ż�¼
					{
						FaultNum =SlaveFault[j][0];//��ȡ�ϴι��ϼ�¼����
						FaultCode =SlaveErrorCode[j];//��ȡ��ǰ���ϴ���
						
						if(FaultNum==14)												//�ӻ�����¼14������
						{
							for(i=1; i<99; i++)
							{
								if(i%7!=0) SlaveFault[j][i] = SlaveFault[j][7+i];		//��ǰ����	
							} 	   
							SlaveFault[j][98] = 1;	                                  	//���
							SlaveFault[j][97] = alarm_time.RTC_Year - 2000;			//��
							SlaveFault[j][96] = alarm_time.RTC_Mon;	                //��
							SlaveFault[j][95] = alarm_time.RTC_Mday;	                //��
							SlaveFault[j][94] = alarm_time.RTC_Hour;                 //ʱ
							SlaveFault[j][93] = alarm_time.RTC_Min;                  //��
							SlaveFault[j][92] = FaultCode;                            //���ϴ���
													
						}
						else
						{
							SlaveFault[j][0]++;  
							FaultNum =SlaveFault[j][0];
							SlaveFault[j][7*FaultNum] = FaultNum;	                    	//���
							SlaveFault[j][7*FaultNum-1] = alarm_time.RTC_Year - 2000; 	//��
							SlaveFault[j][7*FaultNum-2] = alarm_time.RTC_Mon;	        //��
							SlaveFault[j][7*FaultNum-3] = alarm_time.RTC_Mday;	        //��
							SlaveFault[j][7*FaultNum-4] = alarm_time.RTC_Hour;          	//ʱ
							SlaveFault[j][7*FaultNum-5] = alarm_time.RTC_Min;           	//��
							SlaveFault[j][7*FaultNum-6] = FaultCode;                     	//���ϴ���
							for(i=0; i<SlaveFault[j][0]; i++)
							{
								SlaveFault[j][(i+1)*7] = SlaveFault[j][0]-i;//ǰ����ϼ�¼��ż�1
							}
						}
						for(i=0;i<100;i++)
						{
							Tmp_Fault[i]=SlaveFault[j][i];
						}
						I2C_write;                                                   		//д��I2C	   
						WriteEeprom(EepSlaveFaultAddr+j*SingleFaultNum,Tmp_Fault,SingleFaultNum);    	//�����¼��I2C						    
						I2C_read;

						//I2C_write;                                                   		//д��I2C	   
						ReadEeprom(EepSlaveFaultAddr+j*SingleFaultNum,Tmp_Fault,SingleFaultNum);    	//�����¼��I2C		

						for(i=0;i<100;i++)
						{
							SlaveFault[j][i]=Tmp_Fault[i];
						}
						//I2C_read;
					
					}
					
				}
			}
		}
	}
}
/******************************************************************************
** ������:Event_handle 
**
** ����:���ϴ���
******************************************************************************/
void Event_handle(void)
{
	uint16 i,FaultNum,FaultCode;
	//uint8 /*Tmp_Fault[104],*/temp[2];
	uint8 fault_flag;

	//�жϵ��ݵĹ��ϵ�ַ
	fault_flag=0;
	if(capa[3]>0)//���ڹ���
	{
		if(capa_bak!=capa[3])//��ι������ϴι��ϲ�һ��
		{
			fault_flag=1;
			capa_bak=capa[3];
		}
	}

	DR_Logging[0]++;

	if(RecordEventFlag ==1)
	{
		RecordEventFlag =0;
		if(Event_Code !=0)	
		{
			FaultNum =Event_Logging[0];
			FaultCode =Event_Code;
			if(FaultNum==20)		   									//���豸����¼140������
			{ 
				for(i=1; i<141; i++)
				{
					if(i%7!=0) Event_Logging[i] = Event_Logging[7+i];					//��ǰ����	
				}
				Event_Logging[140] = 1;	                                  	//���
				Event_Logging[139] = Event_time.RTC_Year - 2000;				//��
				Event_Logging[138] = Event_time.RTC_Mon;	                    //��
				Event_Logging[137] = Event_time.RTC_Mday;	                //��
				Event_Logging[136] = Event_time.RTC_Hour;                   	//ʱ
				Event_Logging[135] = Event_time.RTC_Min;                     //��
				Event_Logging[134] = FaultCode;                             	//���ϴ���
										
			}
			else
			{
				Event_Logging[0]++;
				FaultNum =Event_Logging[0];  
				Event_Logging[7*FaultNum] = FaultNum;	                    	//���
				Event_Logging[7*FaultNum-1] = Event_time.RTC_Year - 2000;   	//��
				Event_Logging[7*FaultNum-2] = Event_time.RTC_Mon;	        //��
				Event_Logging[7*FaultNum-3] = Event_time.RTC_Mday;	        //��
				Event_Logging[7*FaultNum-4] = Event_time.RTC_Hour;          	//ʱ
				Event_Logging[7*FaultNum-5] = Event_time.RTC_Min;           	//��
				Event_Logging[7*FaultNum-6] = FaultCode;                     	//���ϴ���
				for(i=0; i<FaultNum; i++)
				{
					Event_Logging[(i+1)*7] = FaultNum-i;						//��ŵ���		
				}
			}
			I2C_write;                                                  //д��I2C	   
			WriteEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);       //�����¼��I2C						    
			I2C_read;
 
			ReadEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);       //�����¼��I2C						    
		}
	}
}
/******************************************************************************
** ������:temp_judge 
**
** ����:�¶ȴ���
******************************************************************************/

/*void temp_judge(void)
{
	static uint8 Count =0;
	
	if(ADCFinishProcessFlag==1)
	{
		if(ModuleTemp>((Parameter[924]<<8)|Parameter[925]) )//ģ���¶ȳ�����
		{
			temp_change1++;
			if( temp_change1>10 )
			{
				SystemFailure=1;
				SystemFailureBak =0;
				temp_high1=1;
			}	
		}
		else
		{
			temp_change1=0;
		}
		if( Internaltemp>((Parameter[926]<<8)|Parameter[927]) )//�ӽ��¶ȳ�����
		{
			temp_change2++;
			if(temp_change2>10)
			{
				SystemFailure=1;
				SystemFailureBak =0;
				temp_high2=1;
			}
		}
		else
		{
			temp_change2=0;
		}
		if( DeratingJudgeFlag==1 )//�����ж��¶�ʱ��
		{
			DeratingJudgeFlag=0;
			if( canshuzhong2[0]<30 )//������С���¶������²�������
			{
				if( ModuleTemp > ((Parameter[918]<<8)|Parameter[919]) )
				{
					SystemFailure=1;
					SystemFailureBak =0;
					temp_high1=1;
				}
				if( Internaltemp > ((Parameter[922]<<8)|Parameter[923]) )
				{
					SystemFailure=1;
					SystemFailureBak =0;
					temp_high2=1;
				}
			}
			
			if( ModuleTemp>((Parameter[916]<<8)|Parameter[917]) || Internaltemp>((Parameter[920]<<8)|Parameter[921]) )//����
			{
				Count++;
				if(((Parameter[930]<<8)|Parameter[931])<=1000&&Count >20)
				{
					Count =0;
					canshuzhong2[0] =(Parameter[930]<<8)|Parameter[931];							//������ Ŀ��ֵ

					ParametersConfigflag =1;														//��֪DSP ������������
					SystemDerating =1;
				}
			}
		 	if( ModuleTemp<=((Parameter[918]<<8)|Parameter[919]) && Internaltemp<=((Parameter[922]<<8)|Parameter[923]) )//�ָ�
			{
				if((ModuleTemp==(Parameter[918]<<8)|Parameter[919])||Internaltemp<=((Parameter[922]<<8)|Parameter[923]))
				{
					SystemDerating=0;
				}
				if(((Parameter[932]<<8)|Parameter[933])<=100&&SystemDerating ==1)
				{
					if( canshuzhong2[0]<1000 )
					{
						canshuzhong2[0]=canshuzhong2[0]+ ((Parameter[932]<<8)|Parameter[933]);		//����ʱ ����
					}
					else
					{
						canshuzhong2[0]=1000;
					}
					ParametersConfigflag =1;														// ��֪DSP ������������
				}
			}
		}
	}
}*/
/******************************************************************************
** ������:FanProcess 
**
** ����:�¶ȴ���
******************************************************************************/
/*
void FanProcess(void)
{
	if(SystemClose==1)				//ϵͳ�ػ�
	{
		if(SystemCloseBak ==0)
		{
			OnOffDelayCount=0;
			SystemCloseBak =1;
		}
		if(OnOffDelayCount>29)		//�ػ���30�� �رշ���
		{
//			aout3_L;
		}
	}
	if(SystemFailure==1)			//���Ϻ� ��������ʱ��رշ��ȣ�������
	{
		if(SystemFailureBak ==0)
		{
			FaultDelayCount =0;
			SystemFailureBak =1;
		}
	 	FaultDelayCount++;
		if( FaultDelayCount > (((Parameter[888]<<8)|Parameter[889])*((Parameter[894]<<8)|Parameter[895])) )		//����Ƶ�� * ����ʱ��
		{
//			aout3_L;
//			aout4_H;
		}
	}
}
*/
/******************************************************************************
** ������:ADCProcess 
**
** ����:�¶Ȳ���ֵ����
******************************************************************************/
/*void ADCProcess(uint8 Aisle,uint16 DelayTime)
{
	uint16 i,j,temp,num;
	static uint32 Sum =0;
	static uint16 TempSampleTime=0;


	if(GetCurrentTime(t10ms,TempSampleTime)>DelayTime)		//  1000msִ��һ�β����¶�ƽ��ֵ����
 	{
		while(Aisle--)
 		{
			for (j = 0; j < GetADMax- 1; j++)
			{     
				for (i = 0; i < GetADMax - 1 - j; i++)
				{
					if(data_temp[Aisle][i] > data_temp[Aisle][i + 1])
					{
						temp = data_temp[Aisle][i];
					    data_temp[Aisle][i] = data_temp[Aisle][i + 1];
					    data_temp[Aisle][i + 1] = temp;
					}
				}
			}

			for(num =20; num<GetADMax-20;num++)
			{
				Sum= Sum+data_temp[Aisle][num];
			}
			Avg[Aisle]=(Sum+(GetADMax-40)/2)/(GetADMax-40);				//  ��������
			Sum =0;
		}
		ADCFinishProcessFlag =1;										// �����������
		TempSampleTime= GetSystemTime(t10ms);							//���¼�ʱ
	}
}*/
/******************************************************************************
** ������:ADCProcess 
**
** ����:�¶Ȳ���ֵ����
******************************************************************************/
/*void ParametersConfig(void)
{
	uint16 DelayTime=0,*pointer16;
	if(ParametersConfigflag==1)
	{
		pointer16 = canshuzhong2;
		canshuzhong2[20] = crc_chk_value(pointer16,20);	//У�� 
		LPC_SSP0->IMSC=0x04;
		
//		ARMOUT1_L;	 //ʹ������DSP�Ĳ����ܽ� 
		if( canshuzhong2[1]==0 )
		{
			led3_ON;
		}
		DelayTime= GetSystemTime(t10ms);
		while(ARMIN1!= 0)
		{
			if(GetCurrentTime(t10ms,DelayTime)>499)
			{
				dsp_data[0] =11;
				break;
			}  //�ȴ�5��
		}
		ARMOUT1_H;
	
		ParametersConfigflag=0;
	}
}*/
/******************************************************************************
** ������:Power_current_judge 
**
** ����:���������ж�
******************************************************************************/
/*void Power_current_judge(void)
{
	uint8 SystemMode =0;
	SystemMode=((((((Parameter[6]<<8)|Parameter[7])%10000)%1000)%100)%10);		
	if( SystemMode==AnalogOutputMode 
	 || SystemMode==ReactiveBakingMode 
	 || SystemMode==BackwardReactiveBakingMode 
	 || SystemMode==ActiveBakingMode
	 || SystemMode==BackwardActiveBakingMode )
	{
		SystemCurrentReachedFlag=1;
	}
	else
	{
		if( SystemCurrentReachedFlag==0 && dsp_data[37]>=((Parameter[28]<<8)|Parameter[29])*1.414 && 	//���������ж�
		dsp_data[38]>=((Parameter[28]<<8)|Parameter[29])*1.414 && 
		dsp_data[39]>=((Parameter[28]<<8)|Parameter[29])*1.414 )
  		{
			SystemCurrentReachedFlag=1;
		}
  		if( SystemCurrentReachedFlag==1 && dsp_data[37]<((Parameter[30]<<8)|Parameter[31])*1.414 && 	//�ػ������ж�
        dsp_data[38]<((Parameter[30]<<8)|Parameter[31])*1.414 && 
		dsp_data[39]<((Parameter[30]<<8)|Parameter[31])*1.414 )
  		{
			SystemCurrentReachedFlag=0;
		}
	}		
}*/
/******************************************************************************
** SPIReadySend
******************************************************************************/
void SPIReadyData(void)
{
	uint8 CurrentFrameID;

	if(SPIRecvSuccess==1)
	{
		//SPI_ReceiveProcess();						// SPI���մ���
		StatusProcess();							//�豸״̬����
		error_handle();              				//���ϴ���
		Event_handle();								//�¼�����
		if(ParameterFinishFlag ==0)					//������ȡ��ɱ�־λ
		{	
			if(NextReceFrameID<22)
			{
				NextReceFrameID++;	
			}
			else NextReceFrameID=0;	
		}
		else 
		{
			if(ManualPassiveSwitchFlagBak==1)		//�ֶ�Ͷ�б�־λ
			{
				if(NextReceFrameID<44)
				{
					NextReceFrameID++;	
				}
				else NextReceFrameID=22;
			}
			else
			{
				if(NextReceFrameID<43)
				{
					NextReceFrameID++;	
				}
				else NextReceFrameID=22;
			}
		}
		

		CurrentFrameID = SPI_CommumWithDSP();		//SPIͨѶ֡����
		if(NextReceFrameID==43)						//�ֶ�Ͷ��
		{
			SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];
			SendSpiBuf[2] =(uint16)(ManualPassiveSwitch&0xFFFF);
			SendSpiBuf[3] =(uint16)(ManualPassiveSwitch>>16);
		}
		else if(NextReceFrameID==42)				//��Դ�����ź�
		{
			SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];
			SendSpiBuf[2] =PassiveSign;
		}
		else										//ѭ����ѯ����
		{
			SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];
			SendSpiBuf[2] =SPI_CurrentStartAddress[CurrentFrameID];
		}
		SPIRecvSuccess=0;
		SPICommDelay =0;
	}
	else
	{
		if(SPICommDelay>60)
		{
			Event_time = RTCGetTime();		//��ʱ���
			Event_Code = SPIERROR;
			SPICommError =1;
			
		}
			
	}
}
/******************************************************************************
** ������ʼ
******************************************************************************/
int main()
{
 	SystemInit();                                  	//ϵͳƵ�ʳ�ʼ��(������12MHz,RTCʱ��Ƶ��32KHz,�ڲ�RCƵ��4M Hz)
 	GPIO_init();                                   	//�ܽų�ʼ��
	TIME_init(0);                                   //��ʱ��0��ʼ��
	I2c1_Init();                                   	//i2c��ʼ��
	Parameter_init();                              	//������ʼ��
	UART0_Init();								   	//UART0��ʼ��
	UART1_Init();                                  	//UART1��ʼ��
	UART3_Init();                                  	//UART1��ʼ��	
	Bluetooth_init();								//������ʼ��
//	Testtime_init();					//��ʼ��ʱ��͹��ܿ���
 	GZ_init();                                     	//��ȡ�ӻ����Ϻ͵������Ϻ������޸Ĳ��� 
	SSP0_Init();                                   	//SSP0��ʼ��
	WDTInit();										//���Ź�
/****************************��ѭ��**********************************/
	SpiComm_Enable;									//ͨѶʹ��
	SpiWrite_Enable;								//����дʹ��
	while(1)                  
 	{
		/*if(led3_flag==0){led3_ON;led3_flag=1;}
		else
		{
			led3_OFF;
			led3_flag=0;
		}
		*/
		//SPIRecvSuccess =1;
		//while(1){};
		
		//puyaotest2 =0;
		AccumEventProcess();						//�ۼ�ʱ���ദ��
		DatalogProcess();							//���ݼ�¼����
		//puyaotest2 =1;
		SPIReadyData();								//SPIͨѶ����
//		Testtime_main();								//��ʼ�ж�ʱ��
		//puyaotest2 =2;
		//ExternUartProcess();						//�ⲿͨѶ����
		ExternUartProcess(Uart1);					//�ⲿͨѶ����
		ExternBTProcess(Uart0);						//����ͨѶ����
		//puyaotest2 =3;
		CommunWithDUGSProcess(100);					//��ĻͨѶ��������ˢ��ʱ��
		//puyaotest2 =4;
		OnOffProcess();								//���ػ�����
		
		if ((ini_version_a!=ini_version_b)&&(ini_version_b!=0))
		{
			if(ini_version_a!=10000)
			{
				ini_version_flag=1;
				DUGSWriteCorrectionParameter2Flag=1;
			}			
		}			
						
		puyaotest2 =5;

		if((LPC_SSP0->MIS& 0x01)==1||(LPC_SSP0->MIS& 0x02)==1)
		{
			LPC_SSP0->ICR =3;						//SSP0��ʼ��
			SSP0_Init();							//SSP0��ʼ��
		}
		if(Uart1Reset==1)
		{
			UART1_Init();                            //UART1��ʼ��
			Uart1Reset =0;
		}
		if(Uart0Reset==1)
		{
			UART0_Init();                            //UART1��ʼ��
			Uart0Reset =0;
		}
		if(Uart3Reset==1)
		{
			UART3_Init();                            //UART1��ʼ��
			Uart3Reset =0;
		}
		PassiveReturnSignProcess();					//��Դ�ɽ�㷵���źŴ���
		WDTFeed();									//ι��
		puyaotest2 =6;
		
		//�ж��Ƿ���DSPͨѶ����ͨѶ�ͳ�ʼ������
		if((bluetooth_int==1)&&(first_int==0))
		{
			Bluetooth_init();
			first_int=1;
		}
		
		if(BluetoothConnectOK !=0)					//����ͨѶ����
		{
			led3_ON;
		}
		else
		{
			led3_OFF;
		}
		//SpiComm_Disable;
 	} 				   									    						                                           
}

/******************************************************************************
** ������: UART1_IRQHandler
**
** ���ܣ�UART1�ж�,��������ͨѶ���շ�
******************************************************************************/
void UART0_IRQHandler(void) 
{
	uint32 j,tmp;
	uint8 UartError=0;
	puyaotest =5;
	puyaotestfault++;
	//WDTFeed();									//ι��
	if(led4_flag==0){led4_ON;led4_flag=1;}
	else
	{
		led4_OFF;
		led4_flag=0;
	}
	if(Delay35Count>Delay35character)												// ����3.5���ַ�ʱ��
	{
		RecvSciNum =0;
		while (((LPC_UART0->IIR & 0x01) == 0)&&(RecvSciNum<150))					// �ж��ж��Ƿ�����Ҳ���300��
		{										 
			switch (LPC_UART0->IIR & 0x0E)											// �ж��жϱ�־λ	 
			{											  
				case 0x04:
					RecvSciNum =0;
					while(((LPC_UART0->LSR&0x01)!=0)&&(RecvSciNum<150))			//�ж�FIFO�Ƿ�Ϊ�ղ��Ҳ���300��
					{
						LPC_UART0->LCR=0x03;
						if(BluetoothATcmd==1)
						{
							RecvSciBuf[RecvSciNum] =UART0_GetByte();
	
							RecvSciNum =1;
							j=9600*Delay35character;								//�����ӳ�
							while(1)
							{
								if((LPC_UART0->LSR&0x01)==0)						//FIFOץ�պ�ȴ�һ��ʱ��
								{	
									j--;
									if(j==0)
									{
										//WDTFeed();
										break;
									}
								}
								else
								{	
									RecvSciBuf[RecvSciNum] =  UART0_GetByte();		//��FIFO��ȡ��
									RecvSciNum++;
									//RecvNewFlag = 1;
									if(RecvSciNum>150)
									{
										//WDTFeed();
										break;
									}
									j=9600;
								}
							}
							RecvBTFlag = 1;		
							
						}
						else
						{
							RecvSciBuf[RecvSciNum] =UART0_GetByte();					//ץͷ
						
							if(RecvSciBuf[0]==LocalAddr/*||RecvSciBuf[0]==0xA5*/)		//�жϵ�ַ
							{
								RecvSciNum =1;
								j=9600*Delay35character;								//�����ӳ�
								while(1)
								{
									if((LPC_UART0->LSR&0x01)==0)						//FIFOץ�պ�ȴ�һ��ʱ��
									{	
										j--;
										if(j==0)
										{
											//WDTFeed();
											break;
										}
									}
									else
									{	
										RecvSciBuf[RecvSciNum] =  UART0_GetByte();		//��FIFO��ȡ��
										RecvSciNum++;
										//RecvNewFlag = 1;
										if(RecvSciNum>150)
										{
											//WDTFeed();
											break;
										}
										j=9600;
									}
								}
								if((RecvSciBuf[1]==0x04&&RecvSciNum==8)
								||(RecvSciBuf[1]==0x03&&RecvSciNum==8)
								||(RecvSciBuf[1]==0x06&&RecvSciNum==8)
								||(RecvSciBuf[1]==0x010&&RecvSciNum>8)
								||(RecvSciBuf[1]==0x6A&&RecvSciNum==8)
								||(RecvSciBuf[1]==0x6B)
								||(RecvSciBuf[1]==0x67&&RecvSciNum==8)
								||(RecvSciBuf[1]==0x68&&RecvSciNum==8))
								{
									RecvBTFlag = 1;								//�ж�ͷ��֡��
									RecvDTUSuccessTest++;
								}
								Delay35Count =0;
							}
							else break;
						}
					}
					break;
				case 0x0C:															// �ַ���ʱ�ж� ץ��FIFO��ץ��300�����˳�	  
					RecvSciNum =0;
					while((LPC_UART0->LSR&0x01)!=0)
					{
						tmp =	LPC_UART0->LSR;
						RecvSciBuf[RecvSciNum] =  LPC_UART0->RBR;
						RecvSciNum++;
						if(BluetoothATcmd==1)
						{
							RecvBTFlag =1;
						}
						if(RecvSciNum>150)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;
				case 0x03:
					RecvSciNum =0;													// ״̬���ж� ץ��FIFO��ץ��300�����˳�	 
					while((LPC_UART0->LSR&0x01)!=0)
					{
						tmp =	LPC_UART0->LSR;
						RecvSciBuf[RecvSciNum] =  LPC_UART0->RBR;
						RecvSciNum++;
						if(BluetoothATcmd==1)
						{
							RecvBTFlag =1;
						}
						if(RecvSciNum>150)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;	
				default:
					RecvSciNum =0;
					while((LPC_UART0->LSR&0x01)!=0)
					{
						tmp =	LPC_UART0->LSR;
						RecvSciBuf[RecvSciNum] =  LPC_UART0->RBR;
						RecvSciNum++;
						if(BluetoothATcmd==1)
						{
							RecvBTFlag =1;
						}
						if(RecvSciNum>150)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;
			}
			//tmp =LPC_UART0->LSR;
		} 
		if(UartError ==1)
		{
			LPC_UART0->IER =0;
			Uart1Reset =1;
		}
		puyaotest =6;
	}
}
/******************************************************************************
** ������: UART0_IRQHandler
**
** ���ܣ�UART0�ж�,������ʾ�����ڵ��շ�
******************************************************************************/
void UART1_IRQHandler(void)//��λ��,��С����ͨѶ��
{	
	uint32 j,tmp;
	uint8 UartError=0;
	puyaotest =5;
	puyaotestfault++;
	//WDTFeed();									//ι��
	if(led4_flag==0){led4_ON;led4_flag=1;}
	else
	{
		led4_OFF;
		led4_flag=0;
	}
	if(Delay35Count>Delay35character)												// ����3.5���ַ�ʱ��
	{
		RecvSci3Num =0;
		while (((LPC_UART1->IIR & 0x01) == 0)&&(RecvSci3Num<300))					// �ж��ж��Ƿ�����Ҳ���300��
		{										 
			switch (LPC_UART1->IIR & 0x0E)											// �ж��жϱ�־λ	 
			{											  
				case 0x04:
					while(((LPC_UART1->LSR&0x01)!=0)&&(RecvSci3Num<300))			//�ж�FIFO�Ƿ�Ϊ�ղ��Ҳ���300��
					{
						LPC_UART1->LCR=0x03;
						RecvSci3Num =0;
						RecvSci3Buf[RecvSci3Num] =UART1_GetByte();					//ץͷ
					
						if(RecvSci3Buf[0]==LocalAddr/*||RecvSciBuf[0]==0xA5*/)		//�жϵ�ַ
						{
							RecvSci3Num =1;
							j=9600*Delay35character;								//�����ӳ�
							while(1)
							{
								if((LPC_UART1->LSR&0x01)==0)						//FIFOץ�պ�ȴ�һ��ʱ��
								{	
									j--;
									if(j==0)
									{
										//WDTFeed();
										break;
									}
								}
								else
								{	
									RecvSci3Buf[RecvSci3Num] =  UART1_GetByte();		//��FIFO��ȡ��
									RecvSci3Num++;
									//RecvNewFlag = 1;
									if(RecvSci3Num>299)
									{
										//WDTFeed();
										break;
									}
									j=9600;
								}
							}
							if((RecvSci3Buf[1]==0x04&&RecvSci3Num==8)
							||(RecvSci3Buf[1]==0x03&&RecvSci3Num==8)
							||(RecvSci3Buf[1]==0x06&&RecvSci3Num==8)
							||(RecvSci3Buf[1]==0x010&&RecvSci3Num>8)
							||(RecvSci3Buf[1]==0x6A&&RecvSci3Num==8)
							||(RecvSci3Buf[1]==0x6B)
							||(RecvSci3Buf[1]==0x67&&RecvSci3Num==8)
							||(RecvSci3Buf[1]==0x68&&RecvSci3Num==8))
							{
								RecvNewFlag = 1;								//�ж�ͷ��֡��
								RecvDTUSuccessTest++;
							}
							Delay35Count =0;
						}
						else break;
					}
					break;
				case 0x0C:															// �ַ���ʱ�ж� ץ��FIFO��ץ��300�����˳�	  
					while((LPC_UART1->LSR&0x01)!=0)
					{
						tmp =	LPC_UART1->LSR;
						RecvSci3Buf[RecvSci3Num] =  LPC_UART1->RBR;
						RecvSci3Num++;
						if(RecvSci3Num>299)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;
				case 0x03:															// ״̬���ж� ץ��FIFO��ץ��300�����˳�	 
					while((LPC_UART1->LSR&0x01)!=0)
					{
						tmp =	LPC_UART1->LSR;
						RecvSci3Buf[RecvSci3Num] =  LPC_UART1->RBR;
						RecvSci3Num++;
						if(RecvSci3Num>299)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;	
				default:
					while((LPC_UART1->LSR&0x01)!=0)
					{
						tmp =	LPC_UART1->LSR;
						RecvSci3Buf[RecvSci3Num] =  LPC_UART1->RBR;
						RecvSci3Num++;
						if(RecvSci3Num>299)
						{
							//WDTFeed();
							UartError =1;break;
						}
					}
					break;
			}
			//tmp =LPC_UART1->LSR;
		} 
		if(UartError ==1)
		{
			LPC_UART1->IER =0;
			Uart1Reset =1;
		}
		puyaotest =6;
	}
}
/******************************************************************************
** ������: UART3_IRQHandler
**
** ���ܣ�UART3�ж�,�ⲿͨѶ���շ�
******************************************************************************/
void UART3_IRQHandler(void) 
{
	uint32 i,tmp;
	uint8 UartError=0;

	puyaotest =7;
	sendok=0;
	
	//WDTFeed();									//ι��
	while ((LPC_UART3->IIR & 0x01) == 0)										// �ж��ж��Ƿ����
	{							
		
		switch (LPC_UART3->IIR & 0x0E)											// �ж��жϱ�־λ	 
		{		
				
			case 0x04:		
				while((LPC_UART3->LSR&0x01)!=0)
				{
					RecvDUGSNum =0; 
					RecvDUGSBuf[RecvDUGSNum] =UART3_GetByte();
						
					if(RecvDUGSBuf[0]==0x5A)
					{
						RecvDUGSNum=1;
						i=9600*Delay35character;
						while(1)
						{
							if((LPC_UART3->LSR&0x01)==0)
							{	
								i--;
								if(i==0)
								{
									//WDTFeed();
									break;
								}
							}
							else
							{	
								RecvDUGSBuf[RecvDUGSNum] =  UART3_GetByte();
								RecvDUGSNum++;
								if(RecvDUGSNum==300)
								{
									//WDTFeed();
									break;
								}
								i=9600;
							}
						}
						if(RecvDUGSNum>5)
						{
							RecvDUGSNewFlag = 1;
						}
						
					}
				}
				break;
			case 0x0C:															// �ַ���ʱ�ж� ץ��FIFO��ץ��300�����˳�	 
				while((LPC_UART3->LSR&0x01)!=0)
				{
					tmp =	LPC_UART3->LSR;
					RecvDUGSBuf[RecvDUGSNum] = LPC_UART3->RBR;
					RecvDUGSNum++;
					if(RecvDUGSNum==300)
					{
						//WDTFeed();
						UartError =1;break;
					}
				}
				break;
			case 0x03:															// ״̬���ж� ץ��FIFO��ץ��300�����˳�	 	 
				while((LPC_UART3->LSR&0x01)!=0)
				{
					tmp =	LPC_UART3->LSR;
					RecvDUGSBuf[RecvDUGSNum] = LPC_UART3->RBR;
					RecvDUGSNum++;
					if(RecvDUGSNum==300)
					{
						//WDTFeed();
						UartError =1;break;
					}
				}
				break;	
			default:
				while((LPC_UART3->LSR&0x01)!=0)
				{
					tmp =	LPC_UART3->LSR;
					RecvDUGSBuf[RecvDUGSNum] = LPC_UART3->RBR;
					RecvDUGSNum++;
					if(RecvDUGSNum==300)
					{
						//WDTFeed();
						UartError =1;break;
					}
				}
				break;
		}
		//tmp =LPC_UART0->LSR;
	}
	if(UartError ==1)
	{
		LPC_UART3->IER =0;
		Uart3Reset =1;
	}
	puyaotest =8;
}

/******************************************************************************
** ������: OnOffProcess
**
** ���ܣ����ػ�����
******************************************************************************/
void OnOffProcess(void)
{
	APFStartRunning=OnOffCommand;

	if((SystemStatus ==SystemStandby||SystemStatus ==SystemReady||SystemStatus ==SystemDerating||SystemStatus==SystemRunning||SystemStatus==SystemReseting||SystemStatus==SystemProtect||SystemStatus==SystemParameterError)
		&&APFStartRunning==PowerON
		&&AC_Switch !=PowerOFF
		&&SPIError==0
		&&SystemStatus!=SystemPfault)
	{
		if(ACSwitchFlag==0)
		{	
			if(RemotePowerOnOffFlag==1)
			{
				Event_time = RTCGetTime();		//��ʱ���
				Event_Code = RemotePowerOn;
				ACSwitchFlag =1;
				RecordEventFlag =1;
				RemotePowerOnOffFlag=0;
			}
			else
			{
				Event_time = RTCGetTime();		//��ʱ���
				Event_Code = ManualPowerOn;
				ACSwitchFlag =1;
				RecordEventFlag =1;
			}
		}
		OnOff_Enable;
		SwitchOn;
		SwitchON=1;
	}
	else
	{
		if(ACSwitchFlag==1)
		{
			if(RemotePowerOnOffFlag==1)
			{
				Event_time = RTCGetTime();		//��ʱ���
				Event_Code = RemotePowerOff;
				ACSwitchFlag =0;
				RecordEventFlag =1;
				RemotePowerOnOffFlag=0;
			}
			else
			{
				Event_time = RTCGetTime();		//��ʱ���
				Event_Code = ManualPowerOff;
				ACSwitchFlag =0;
				RecordEventFlag =1;
			}
		}
		OnOff_Disable;
		SwitchOff;
		SwitchON =0;
	}

	if((SystemStatus ==SystemReady||SystemStatus ==SystemDerating||SystemStatus==SystemRunning||SystemStatus==SystemReseting||SystemStatus==SystemProtect||SystemStatus==SystemParameterError)
		&&APFStartRunning==PowerON
		&&AC_Switch !=PowerOFF
		&&SPIError==0
//	  &&Testtime <test_time
		&&SystemStatus!=SystemPfault)
	{
		AlarmTimeProcess();//��ʱֻ��1,2,3����ʹ�ã���ʱ4��ɾ��
	}
	else OnOff2_Disable;
}
/******************************************************************************
** ������: AlarmTimeProcess
**
** ���ܣ���ʱ���ػ�����
******************************************************************************/
void AlarmTimeProcess(void)
{	
	current_time = RTCGetTime();		//��ȡʱ��
	if(AlarmTime1[0]!=0)
 	{
 		if(AlarmTime1[0]==1)//��ʱ����
 		{
 			if(AlarmTime1[1]<=AlarmTime1[3])//��ʼʱ��С�ڽ���ʱ��
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				&&((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Enable;
					if(AlarmSwitchFlag==0)
					{
						Event_Code=AlarmpPowerOn;
						Event_time = RTCGetTime();		//��ʱ���
						AlarmSwitchFlag=1;
						RecordEventFlag =1;
					}
				}
			}
			else
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				||((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Enable;
					if(AlarmSwitchFlag==0)
					{
						Event_Code=AlarmpPowerOn;
						Event_time = RTCGetTime();		//��ʱ���
						AlarmSwitchFlag=1;
						RecordEventFlag =1;
					}
				}
			}
			if(AlarmTime2[0]==1)
			{
				if(AlarmTime2[1]<=AlarmTime2[3])//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					&&((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					||((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				if(AlarmTime3[0]==1)
				{
					if(AlarmTime3[1]<=AlarmTime3[3])//��ʼʱ��С�ڽ���ʱ��
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						&&((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						||((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
//					if(AlarmTime4[0]==1)
//					{
//						if(AlarmTime4[1]<=AlarmTime4[3])//��ʼʱ��С�ڽ���ʱ��
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							&&((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Enable;
//								if(AlarmSwitchFlag==0)
//								{
//									Event_Code=AlarmpPowerOn;
//									Event_time = RTCGetTime();		//��ʱ���
//									AlarmSwitchFlag=1;
//									RecordEventFlag =1;
//								}
//							}
//						}
//						else
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							||((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Enable;
//								if(AlarmSwitchFlag==0)
//								{
//									Event_Code=AlarmpPowerOn;
//									Event_time = RTCGetTime();		//��ʱ���
//									AlarmSwitchFlag=1;
//									RecordEventFlag =1;
//								}
//							}
//						}
//					}
				}
			}
			if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
					//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
					)
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime3[1]>AlarmTime3[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
//					else if(AlarmTime4[1]>AlarmTime4[3])
//					{
//						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
//						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
//						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
//						&&(((current_time.RTC_Hour<AlarmTime4[1])&&(current_time.RTC_Hour>AlarmTime4[3]))||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4])))
//						{
//							OnOff2_Disable;
//							if(AlarmSwitchFlag==1)
//							{
//								Event_Code=AlarmpPowerOff;
//								Event_time = RTCGetTime();		//��ʱ���
//								AlarmSwitchFlag=0;
//								RecordEventFlag =1;
//							}
//						}
//					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&(((current_time.RTC_Hour<AlarmTime4[1])&&(current_time.RTC_Hour>AlarmTime4[3]))||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime3[1]>AlarmTime3[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3]))//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else//��ʱ�ػ�
			{
				if(AlarmTime1[1]<=AlarmTime1[3])//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
			}
 		}
		else if(AlarmTime1[0]==2)//��ʱ�ػ�
		{
			if(AlarmTime1[1]<=AlarmTime1[3])//��ʼʱ��С�ڽ���ʱ��
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				&&((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Disable;
					if(AlarmSwitchFlag==1)
					{
						Event_Code=AlarmpPowerOff;
						Event_time = RTCGetTime();		//��ʱ���
						AlarmSwitchFlag=0;
						RecordEventFlag =1;
					}
				}
			}
			else
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				||((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Disable;
					if(AlarmSwitchFlag==1)
					{
						Event_Code=AlarmpPowerOff;
						Event_time = RTCGetTime();		//��ʱ���
						AlarmSwitchFlag=0;
						RecordEventFlag =1;
					}
				}
			}
			if(AlarmTime2[0]==1)
			{
				if(AlarmTime2[1]<=AlarmTime2[3])//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					&&((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					||((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				if(AlarmTime3[0]==1)
				{
					if(AlarmTime3[1]<=AlarmTime3[3])//��ʼʱ��С�ڽ���ʱ��
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						&&((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						||((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
//					if(AlarmTime4[0]==1)
//					{
//						if(AlarmTime4[1]<=AlarmTime4[3])//��ʼʱ��С�ڽ���ʱ��
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							&&((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Disable;
//								if(AlarmSwitchFlag==1)
//								{
//									Event_Code=AlarmpPowerOff;
//									Event_time = RTCGetTime();		//��ʱ���
//									AlarmSwitchFlag=0;
//									RecordEventFlag =1;
//								}
//							}
//						}
//						else
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							||((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Disable;
//								if(AlarmSwitchFlag==1)
//								{
//									Event_Code=AlarmpPowerOff;
//									Event_time = RTCGetTime();		//��ʱ���
//									AlarmSwitchFlag=0;
//									RecordEventFlag =1;
//								}
//							}
//						}
//					}
				}
			}
			if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
					//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
					)
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime3[1]>AlarmTime3[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&((current_time.RTC_Hour<AlarmTime4[1])||(current_time.RTC_Hour>AlarmTime4[3])||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
//					else if(AlarmTime4[1]>AlarmTime4[3])
//					{
//						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
//						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
//						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
//						//&&(((current_time.RTC_Hour<AlarmTime4[1])&&(current_time.RTC_Hour>AlarmTime4[3]))||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
//						)
//						{
//							OnOff2_Enable;
//							if(AlarmSwitchFlag==0)
//							{
//								Event_Code=AlarmpPowerOn;
//								Event_time = RTCGetTime();		//��ʱ���
//								AlarmSwitchFlag=1;
//								RecordEventFlag =1;
//							}
//						}
//					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4]))
						//&&(((current_time.RTC_Hour<AlarmTime4[1])&&(current_time.RTC_Hour>AlarmTime4[3]))||(current_time.RTC_Hour==AlarmTime4[1]&&current_time.RTC_Min<AlarmTime4[2])||(current_time.RTC_Hour==AlarmTime4[3]&&current_time.RTC_Min>=AlarmTime4[4]))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime3[1]>AlarmTime3[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
						&&(((current_time.RTC_Hour<AlarmTime3[1])&&(current_time.RTC_Hour>AlarmTime3[3]))||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3]))//��ʼʱ��С�ڽ���ʱ��
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(AlarmTime1[1]>AlarmTime1[3])
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else if(AlarmTime2[1]>AlarmTime2[3])
					{
						if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
					else
					{
						if((((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
						&&(((current_time.RTC_Hour<AlarmTime2[1])&&(current_time.RTC_Hour>AlarmTime2[3]))||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//��ʱ���
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else
			{
				if(AlarmTime1[1]<=AlarmTime1[3])
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4])))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				else
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])&&(current_time.RTC_Hour>AlarmTime1[3]))||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//��ʱ���
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
			}
		}
	}
	else
	{
		OnOff2_Enable;
	}
}
/******************************************************************************
** ������: AccumEventProcess
**
** ���ܣ��ۼ��¼�����
******************************************************************************/
void AccumEventProcess(void)
{
	uint8 temp[76],i,j;
	//uint64	tmp64H[2],tmp64L[2],tmp64Val[2];
	//uint8   DatalogNum;
	
	local_time = RTCGetTime();					//��ȡʱ��
	
	if(local_time.RTC_Year!= EepromYear||local_time.RTC_Mon!= EepromMon||local_time.RTC_Mday!= EepromDay)	//���ڱ��
	{
		TotalEnergyBakHB =TotalEnergyHB;
		TotalEnergyBakLB =TotalEnergyLB;						//����ǰһ���ۼ�����

		TotalRunningTimeBak =TotalRunningTime;					//����ǰһ���ۼ�����ʱ��
		
		DailyEnergy =0;
		DailyRunningTime =0;
		
		EepromYear =local_time.RTC_Year;
		EepromMon  =local_time.RTC_Mon;
		EepromDay  =local_time.RTC_Mday;
		

		temp[0] =(uint8)(TotalEnergyHB>>24);
		temp[1] =(uint8)(TotalEnergyHB>>16);
		temp[2] =(uint8)(TotalEnergyHB>>8);
		temp[3] =(uint8)(TotalEnergyHB);
		temp[4] =(uint8)(TotalEnergyLB>>24);
		temp[5] =(uint8)(TotalEnergyLB>>16);
		temp[6] =(uint8)(TotalEnergyLB>>8);
		temp[7] =(uint8)(TotalEnergyLB);
		temp[8] =(uint8)(TotalRunningTime>>24);
		temp[9] =(uint8)(TotalRunningTime>>16);
		temp[10] =(uint8)(TotalRunningTime>>8);
		temp[11] =(uint8)(TotalRunningTime);
		temp[12] =(uint8)(DailyEnergy>>24);
		temp[13] =(uint8)(DailyEnergy>>16);
		temp[14] =(uint8)(DailyEnergy>>8);
		temp[15] =(uint8)(DailyEnergy);
		temp[16] =(uint8)(DailyRunningTime>>8);
		temp[17] =(uint8)(DailyRunningTime);
		temp[18] =(uint8)(EepromYear>>8);
		temp[19] =(uint8)(EepromYear);
		temp[20] =(uint8)(EepromMon>>8);
		temp[21] =(uint8)(EepromMon);
		temp[22] =(uint8)(EepromDay>>8);
		temp[23] =(uint8)(EepromDay);
		//temp[20] =(uint8)(TotalEnergyBakHB>>24);
		//temp[21] =(uint8)(TotalEnergyBakHB>>16);
		//temp[22] =(uint8)(TotalEnergyBakHB>>8);
		//temp[23] =(uint8)(TotalEnergyBakHB);
		//temp[24] =(uint8)(TotalEnergyBakLB>>24);
		//temp[25] =(uint8)(TotalEnergyBakLB>>16);
		//temp[26] =(uint8)(TotalEnergyBakLB>>8);
		//temp[27] =(uint8)(TotalEnergyBakLB);
		//temp[20] =(uint8)(TotalRunningTimeBak>>24);
		//temp[21] =(uint8)(TotalRunningTimeBak>>16);
		//temp[22] =(uint8)(TotalRunningTimeBak>>8);
		//temp[23] =(uint8)(TotalRunningTimeBak);
		I2C_write;
		WriteEeprom(EepParameterAddr+1568,temp,24);   					//����������д����ʽ������
		I2C_read;
		
		DatalogReset=1;										//////////////////���ݼ�¼����////////////////////////////////
	}
	if(SystemStatus>=SystemStandby)										//�豸���ڴ����Ժ�
	{
		//////////////////���ݼ�¼����////////////////////////////////
		if(DatalogReset==1)
		{
			DatalogReset =0;
			Datalog[0]= dsp_data[0];Datalog[1]= dsp_data[0];
			Datalog[2]= dsp_data[1];Datalog[3]= dsp_data[1];
			Datalog[4]= dsp_data[2];Datalog[5]= dsp_data[2];
			Datalog[6]= dsp_data[3];Datalog[7]= dsp_data[3];
			Datalog[8]= dsp_data[4];Datalog[9]= dsp_data[4];
			Datalog[10]= dsp_data[5];Datalog[11]= dsp_data[5];
			Datalog[12]= dsp_data[6];Datalog[13]= dsp_data[6];
			Datalog[14]= dsp_data[7];Datalog[15]= dsp_data[7];
			Datalog[16]= dsp_data[8];Datalog[17]= dsp_data[8];
			Datalog[18]= dsp_data[10];Datalog[19]= dsp_data[10];
			Datalog[20]= dsp_data[11];Datalog[21]= dsp_data[11];
			Datalog[22]= dsp_data[12];Datalog[23]= dsp_data[12];
			Datalog[24]= dsp_data[13];Datalog[25]= dsp_data[13];
			Datalog[26]= dsp_data[14];Datalog[27]= dsp_data[14];
			Datalog[28]= dsp_data[15];Datalog[29]= dsp_data[15];

			for(i=0;i<16;i++)
			{
				IntPassive.Times[i] =0;
			}
			
			DatalogMax++;
			if(DatalogMax>13)DatalogMax =13;
			DatalogStart++;
			if(DatalogStart>13)DatalogStart =DatalogStart%13;

			temp[0]= (uint8)DatalogMax;
			temp[1]= (uint8)DatalogStart;
			
			I2C_write;
			WriteEeprom(EepDatalogMaxAddr,temp,2);
			I2C_read;
		}
	}
	if(SystemStatus>=SystemReady)								//���к�ʼ��¼
	{
		SystemRunningFlag =1;
		/*if(TimeDelay>59)										//ÿ����ˢ��һ�ε����ۼ�
		{
			TimeDelay =0;
			tmp64H[0] =(uint64)(TotalEnergyHB);
			tmp64L[0] =(uint64)(TotalEnergyLB);
			tmp64Val[0] =(tmp64H[0]<<32)|tmp64L[0];
				
			tmp64H[1] =(uint64)(TotalEnergyBakHB);
			tmp64L[1] =(uint64)(TotalEnergyBakLB);
			tmp64Val[1] =(tmp64H[1]<<32)|tmp64L[1];
				
			DailyEnergy =tmp64Val[0]-tmp64Val[1];				//�����ۼƲ�����

		}*/
		if(AccumDelay>359)						//ÿ6���Ӽ�¼һ���ۼƲ�����
		{
			AccumDelay =0;
			TotalRunningTime++;
			DailyRunningTime =TotalRunningTime-TotalRunningTimeBak;			//�����ۼƲ�����

			temp[0] =(uint8)(TotalEnergyHB>>24);
			temp[1] =(uint8)(TotalEnergyHB>>16);
			temp[2] =(uint8)(TotalEnergyHB>>8);
			temp[3] =(uint8)(TotalEnergyHB);		
			temp[4] =(uint8)(TotalEnergyLB>>24);
			temp[5] =(uint8)(TotalEnergyLB>>16);
			temp[6] =(uint8)(TotalEnergyLB>>8);
			temp[7] =(uint8)(TotalEnergyLB);
			temp[8] =(uint8)(TotalRunningTime>>24);
			temp[9] =(uint8)(TotalRunningTime>>16);
			temp[10] =(uint8)(TotalRunningTime>>8);
			temp[11] =(uint8)(TotalRunningTime);
			temp[12] =(uint8)(DailyEnergy>>24);
			temp[13] =(uint8)(DailyEnergy>>16);
			temp[14] =(uint8)(DailyEnergy>>8);
			temp[15] =(uint8)(DailyEnergy);
			temp[16] =(uint8)(DailyRunningTime>>8);
			temp[17] =(uint8)(DailyRunningTime);

			I2C_write;
			WriteEeprom(EepParameterAddr+1568,temp,18);
			I2C_read;

		}
		if(DatalogDelay>600)						//ÿ10���Ӽ�¼һ�����ݼ�¼
		{
			DatalogDelay =0;
			j=0;
			for(i=0;i<30;i++)
			{
				temp[j++]=(uint8)(Datalog[i]>>8);
				temp[j++]=(uint8)(Datalog[i]);
			}
			temp[j++]=(uint8)(IntPassive.Times[0]);
			temp[j++]=(uint8)(IntPassive.Times[1]);
			temp[j++]=(uint8)(IntPassive.Times[2]);
			temp[j++]=(uint8)(IntPassive.Times[3]);
			temp[j++]=(uint8)(IntPassive.Times[4]);
			temp[j++]=(uint8)(IntPassive.Times[5]);
			temp[j++]=(uint8)(IntPassive.Times[6]);
			temp[j++]=(uint8)(IntPassive.Times[7]);
			temp[j++]=(uint8)(IntPassive.Times[8]);
			temp[j++]=(uint8)(IntPassive.Times[9]);
			temp[j++]=(uint8)(IntPassive.Times[10]);
			temp[j++]=(uint8)(IntPassive.Times[11]);
			temp[j++]=(uint8)(IntPassive.Times[12]);
			temp[j++]=(uint8)(IntPassive.Times[13]);
			temp[j++]=(uint8)(IntPassive.Times[14]);
			temp[j++]=(uint8)(IntPassive.Times[15]);

			if(DatalogStart==0)DatalogStart=1;
			I2C_write;
			WriteEeprom(DatalogStAddr[DatalogStart-1],temp,76);
			I2C_read;
		}
	}
	if( SystemStatus<SystemReady&&SystemRunningFlag==1)		//�豸�����е�������¼һ���ۼ�����
	{
		SystemRunningFlag =0;
		temp[0] =(uint8)(TotalEnergyHB>>24);
		temp[1] =(uint8)(TotalEnergyHB>>16);
		temp[2] =(uint8)(TotalEnergyHB>>8);
		temp[3] =(uint8)(TotalEnergyHB);		
		temp[4] =(uint8)(TotalEnergyLB>>24);
		temp[5] =(uint8)(TotalEnergyLB>>16);
		temp[6] =(uint8)(TotalEnergyLB>>8);
		temp[7] =(uint8)(TotalEnergyLB);
		temp[8] =(uint8)(TotalRunningTime>>24);
		temp[9] =(uint8)(TotalRunningTime>>16);
		temp[10] =(uint8)(TotalRunningTime>>8);
		temp[11] =(uint8)(TotalRunningTime);
		temp[12] =(uint8)(DailyEnergy>>24);
		temp[13] =(uint8)(DailyEnergy>>16);
		temp[14] =(uint8)(DailyEnergy>>8);
		temp[15] =(uint8)(DailyEnergy);
		temp[16] =(uint8)(DailyRunningTime>>8);
		temp[17] =(uint8)(DailyRunningTime);

		I2C_write;
		WriteEeprom(EepParameterAddr+1568,temp,18);
		I2C_read;

	}
	/*if(SystemStatus ==SystemRunning && ResetDelay>3599)
	{
		if(ResetCount<50000)
		{
			ResetCount++;
		}
		if(slave_Reset[0]<50000)
		{
			slave_Reset[0]++;
		}
		if(slave_Reset[1]<50000)
		{
			slave_Reset[1]++;
		}
		if(slave_Reset[2]<50000)
		{
			slave_Reset[2]++;
		}
		if(slave_Reset[3]<50000)
		{
			slave_Reset[3]++;
		}
		if(slave_Reset[4]<50000)
		{
			slave_Reset[4]++;
		}
		if(slave_Reset[5]<50000)
		{
			slave_Reset[5]++;
		}
		if(slave_Reset[6]<50000)
		{
			slave_Reset[6]++;
		}
		if(slave_Reset[7]<50000)
		{
			slave_Reset[7]++;
		}
		if(slave_Reset[8]<50000)
		{
			slave_Reset[8]++;
		}
		if(slave_Reset[9]<50000)
		{
			slave_Reset[9]++;
		}
		temp[0] =(uint8)(ResetCount>>8);
		temp[1] =(uint8)(ResetCount);
					
		temp[2] =(uint8)(slave_Reset[0]>>8);
		temp[3] =(uint8)(slave_Reset[0]);
		temp[4] =(uint8)(slave_Reset[1]>>8);
		temp[5] =(uint8)(slave_Reset[1]);
		temp[6] =(uint8)(slave_Reset[2]>>8);
		temp[7] =(uint8)(slave_Reset[2]);
		temp[8] =(uint8)(slave_Reset[3]>>8);
		temp[9] =(uint8)(slave_Reset[3]);
		temp[10] =(uint8)(slave_Reset[4]>>8);
		temp[11] =(uint8)(slave_Reset[4]);
		temp[12] =(uint8)(slave_Reset[5]>>8);
		temp[13] =(uint8)(slave_Reset[5]);
		temp[14] =(uint8)(slave_Reset[6]>>8);
		temp[15] =(uint8)(slave_Reset[6]);
		temp[16] =(uint8)(slave_Reset[7]>>8);
		temp[17] =(uint8)(slave_Reset[7]);
		temp[18] =(uint8)(slave_Reset[8]>>8);
		temp[19] =(uint8)(slave_Reset[8]);
		temp[20] =(uint8)(slave_Reset[9]>>8);
		temp[21] =(uint8)(slave_Reset[9]);
		I2C_write;													//д��I2C
		WriteEeprom(EepParameterAddr+848,temp,16);   			//����������д����ʽ������
		I2C_read;

		I2C_write;													//д��I2C
		WriteEeprom(EepParameterAddr+864,temp+16,6);   			//����������д����ʽ������
		I2C_read;
		DUGSReadMainParameterFlag =1;
		ResetDelay =0;
		
	}*/
}
/*uint64 BCD2HEX(uint64 bcd_data)    //BCDתΪHEX�ӳ���    
{   
    uint64 temp;   
    temp=(bcd_data/16*10 + bcd_data%16);   
    return temp;   
}*/   
uint8 HEX2BCD(uint64 Dec, uint8 *Bcd, uint8 length)     //HEXתΪBCD�ӳ���     
{   
	int i; 
	int temp; 

	for(i=length-1; i>=0; i--) 
	{ 
		temp = Dec%100; 
		Bcd[i] = ((temp/10)<<4) + ((temp%10) & 0x0F); 
		Dec /= 100; 
	} 

	return 0; 
} 


//ARMӲ�����Ϻ����˺�����Ŀ����Ӳ�������ܹ���λ
void hard_fault_handler_c(unsigned int * hardfault_args)
{
	uint32 i;
	/*static unsigned int stacked_r0; 
	static unsigned int stacked_r1;
	static unsigned int stacked_r2;
	static unsigned int stacked_r3;
	static unsigned int stacked_r12;
	static unsigned int stacked_lr;
	static unsigned int stacked_pc;
	static unsigned int stacked_psr;
	static unsigned int SHCSR;
	static unsigned char MFSR;
	static unsigned char BFSR;
	static unsigned short int UFSR;
	static unsigned int HFSR;
	static unsigned int DFSR;
	static unsigned int MMAR;
	static unsigned int BFAR;

	stacked_r0 = ((unsigned long) hardfault_args[0]);
	stacked_r1 = ((unsigned long) hardfault_args[1]);
	stacked_r2 = ((unsigned long) hardfault_args[2]);
	stacked_r3 = ((unsigned long) hardfault_args[3]);
	stacked_r12 = ((unsigned long) hardfault_args[4]);*/
	/*�쳣�жϷ���ʱ������쳣ģʽ�ض�������R14,��lr�����óɸ��쳣ģʽ��Ҫ���صĵ�ַ*/
	/*stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);
	 SHCSR = (*((volatile unsigned long *)(0xE000ED24))); //ϵͳHandler���Ƽ�״̬�Ĵ���  
	 MFSR = (*((volatile unsigned char *)(0xE000ED28))); //�洢������fault״̬�Ĵ���   
	 BFSR = (*((volatile unsigned char *)(0xE000ED29))); //����fault״̬�Ĵ���   
	 UFSR = (*((volatile unsigned short int *)(0xE000ED2A)));//�÷�fault״̬�Ĵ���    
	 HFSR = (*((volatile unsigned long *)(0xE000ED2C)));  //Ӳfault״̬�Ĵ���     
	 DFSR = (*((volatile unsigned long *)(0xE000ED30))); //����fault״̬�Ĵ���  
	 MMAR = (*((volatile unsigned long *)(0xE000ED34))); //�洢�����ַ�Ĵ���  
	 BFAR = (*((volatile unsigned long *)(0xE000ED38))); //����fault��ַ�Ĵ���  
	*/
	 while (1) {
	//OnOff_Disable;
	LPC_SSP0->IMSC = 0x00;			  		//		  ��ֹ�ж�
	SystemStatus = SystemProtect;
	MainErrorCode =31;
	alarm_time = RTCGetTime();					//��ʱ���
	error_handle();              				//���ϴ���
 	led4_ON;
	led3_ON;
 	led2_ON;
 	led1_ON;
 	for(i=60000000;i>0;i--);
 	led2_OFF;
	for(i=60000000;i>0;i--);
	led1_OFF;
	for(i=60000000;i>0;i--);
	led4_OFF;
	for(i=60000000;i>0;i--);
	led3_OFF;
	Reset();
	 };
}
void Reset(void)
{
__set_FAULTMASK(1);      // �ر������ж�
NVIC_SystemReset();			 // ִ�и�λ
}
/*****************************************************************************
** Function name:		Passive Return Sign
**
** Descriptions:		�̵������ع����ź�
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void PassiveReturnSignProcess( void )
{
	uint8 i;
	//static uint16 count1 =0,count2 =0,count3 =0,count4 =0;
	if(((1<<8)& LPC_GPIO1->FIOPIN)==0)// �ɽӵ�ajin1
	{
		PassiveSign_aisle[0]=1;
	}
	else	PassiveSign_aisle[0]=0;
	if(((1<<10)& LPC_GPIO1->FIOPIN)==0)// �ɽӵ�ajin2
	{
		PassiveSign_aisle[1]=1;
	}
	else	PassiveSign_aisle[1]=0;
	if(((1<<15)& LPC_GPIO1->FIOPIN)==0)// �ɽӵ�ajin3
	{
		PassiveSign_aisle[2]=1;
	}
	else	PassiveSign_aisle[2]=0;
	if(((1<<17)& LPC_GPIO1->FIOPIN)==0)// �ɽӵ�ajin4
	{
		PassiveSign_aisle[3]=1;
	}
	else	PassiveSign_aisle[3]=0;
	PassiveSign =0;
	for(i=0;i<4;i++)
	{		
		PassiveSign |= PassiveSign_aisle[i]<<i;
	}
}
/*****************************************************************************
** Function name:		DatalogProcess
**
** Descriptions:		ʹ�ø��ز����ݼ�¼����
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void DatalogProcess( void )
{
	int16 temp1,temp2;
	if(dsp_data[0]>Datalog[0])Datalog[0]=dsp_data[0];		//VAmax
	if(dsp_data[0]<Datalog[1])Datalog[1]=dsp_data[0];		//VAmin

	if(dsp_data[1]>Datalog[2])Datalog[2]=dsp_data[1];		//VBmax
	if(dsp_data[1]<Datalog[3])Datalog[3]=dsp_data[1];		//VBmin

	if(dsp_data[2]>Datalog[4])Datalog[4]=dsp_data[2];		//VCmax
	if(dsp_data[2]<Datalog[5])Datalog[5]=dsp_data[2];		//VCmin	

	if(dsp_data[3]>Datalog[6])Datalog[6]=dsp_data[3];		//THDVmaxA
	if(dsp_data[3]<Datalog[7])Datalog[7]=dsp_data[3];		//THDVminA

	if(dsp_data[4]>Datalog[8])Datalog[8]=dsp_data[4];		//THDVmaxB
	if(dsp_data[4]<Datalog[9])Datalog[9]=dsp_data[4];		//THDVminB

	if(dsp_data[5]>Datalog[10])Datalog[10]=dsp_data[5];		//THDVmaxC
	if(dsp_data[5]<Datalog[11])Datalog[11]=dsp_data[5];		//THDVminC

	if(dsp_data[19]>Datalog[12])Datalog[12]=dsp_data[19];		//IAmax
	if(dsp_data[19]<Datalog[13])Datalog[13]=dsp_data[19];		//IAmin
	
	if(dsp_data[20]>Datalog[14])Datalog[14]=dsp_data[20];		//IBmax
	if(dsp_data[20]<Datalog[15])Datalog[15]=dsp_data[20];		//IBmin
	
	if(dsp_data[21]>Datalog[16])Datalog[16]=dsp_data[21];		//ICmax
	if(dsp_data[21]<Datalog[17])Datalog[17]=dsp_data[21];		//ICmin

	if(dsp_data[23]>Datalog[18])Datalog[18]=dsp_data[23];	//THDImaxA
	if(dsp_data[23]<Datalog[19])Datalog[19]=dsp_data[23];	//THDIminA

	if(dsp_data[24]>Datalog[20])Datalog[20]=dsp_data[24];	//THDImaxB
	if(dsp_data[24]<Datalog[21])Datalog[21]=dsp_data[24];	//THDIminB

	if(dsp_data[25]>Datalog[22])Datalog[22]=dsp_data[25];	//THDImaxC
	if(dsp_data[25]<Datalog[23])Datalog[23]=dsp_data[25];	//THDIminC

	temp1=abs((int16)(dsp_data[26]));
	temp2=abs((int16)(Datalog[24]));
	if(temp1>temp2)Datalog[24]=dsp_data[26];	//PFmaxA
	temp1=abs((int16)(dsp_data[13]));
	temp2=abs((int16)(Datalog[25]));
	if(temp1<temp2)Datalog[25]=dsp_data[26];	//PFminA
	temp1=abs((int16)(dsp_data[27]));
	temp2=abs((int16)(Datalog[26]));
	if(temp1>temp2)Datalog[26]=dsp_data[27];	//PFmaxB
	temp1=abs((int16)(dsp_data[27]));
	temp2=abs((int16)(Datalog[27]));
	if(temp1<temp2)Datalog[27]=dsp_data[27];	//PFminB
	temp1=abs((int16)(dsp_data[28]));
	temp2=abs((int16)(Datalog[28]));
	if(temp1>temp2)Datalog[28]=dsp_data[28];	//PFmaxC
	temp1=abs((int16)(dsp_data[28]));
	temp2=abs((int16)(Datalog[29]));
	if(temp1<temp2)Datalog[29]=dsp_data[28];	//PFminC
}

/******************************************************************************
**                            End Of File
******************************************************************************/
