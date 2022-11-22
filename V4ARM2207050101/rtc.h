/*****************************************************************************
 *   rtc.h:  Header file for NXP LPC17xx Family Microprocessors
 *
 *   Copyright(C) 2009, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.05.27  ver 1.00    Prelimnary version, first Release
 *
******************************************************************************/
#ifndef __RTC_H 
#define __RTC_H

/**********************************GPIO***************************************/
#define I2C_write    	{LPC_GPIO1->FIOCLR=1<<28;} 	//I2Cд
#define I2C_read     	{LPC_GPIO1->FIOSET=1<<28;} 	//I2C��

#define OnOff_Disable	{LPC_GPIO2->FIOSET=1<<13;}	//��1����ʾ������DSP����
#define OnOff_Enable	{LPC_GPIO2->FIOCLR=1<<13;}	//��0����ʾ����DSP����

#define OnOff2_Disable	{LPC_GPIO0->FIOSET=1<<22;}	//��1����ʾ������DSP����
#define OnOff2_Enable	{LPC_GPIO0->FIOCLR=1<<22;}	//��0����ʾ����DSP����

#define	SpiComm_Disable	{LPC_GPIO2->FIOSET=1<<11;}	//��1����ʾ������DSP��ARMоƬ����SPIͨѶ
#define SpiComm_Enable	{LPC_GPIO2->FIOCLR=1<<11;}	//��0����ʾ����DSP��ARMоƬ����SPIͨѶ

#define SpiWrite_Disable {LPC_GPIO2->FIOSET=1<<12;}	//��1����ʾSPIд��ʹ�ܣ�������Ч
#define SpiWrite_Enable	{LPC_GPIO2->FIOCLR=1<<12;}	//��0����ʾSPIдʹ��

#define DSP_OnOff		((1<<13)&LPC_GPIO2->FIOPIN)	//��0����ʾDSP�Ѿ�����.��1��DSPû�п���
#define AC_Switch		((1<<25)&LPC_GPIO1->FIOPIN)	//��0����ʾDSP�Ѿ�����.��1��DSPû�п���
//#define BluetoothConnectOK		((1<<2)&LPC_GPIO2->FIOPIN)	//��0������δ����.��1������������
#define BluetoothConnectOK		((1<<1)&LPC_GPIO1->FIOPIN)	//��0������δ����.��1������������

#define led4_ON			{LPC_GPIO0->FIOCLR=1<<29;}  //LED4
#define led4_OFF		{LPC_GPIO0->FIOSET=1<<29;}  //LED4
#define led3_ON			{LPC_GPIO0->FIOCLR=1<<30;}  //LED3
#define led3_OFF		{LPC_GPIO0->FIOSET=1<<30;}  //LED3
#define	led2_ON			{LPC_GPIO3->FIOCLR=1<<26;}	//LED2
#define	led2_OFF		{LPC_GPIO3->FIOSET=1<<26;}	//LED2
#define	led1_ON			{LPC_GPIO3->FIOCLR=1<<25;}	//LED1
#define	led1_OFF		{LPC_GPIO3->FIOSET=1<<25;}	//LED1
#define uart_read3		{LPC_GPIO1->FIOCLR=1<<29;} 	//UART3����
#define uart_write3   	{LPC_GPIO1->FIOSET=1<<29;} 	//UART3����
#define uart_read0		{LPC_GPIO0->FIOCLR=1<<23;} 	//UART0����
#define uart_write0   	{LPC_GPIO0->FIOSET=1<<23;} 	//UART0����
#define uart_read1		{LPC_GPIO2->FIOCLR=1<<2;} 	//UART1����
#define uart_write1   	{LPC_GPIO2->FIOSET=1<<2;} 	//UART1����

//#define uart_write1		{LPC_GPIO0->FIOCLR=1<<23;} 	//UART0����
//#define uart_read1   	{LPC_GPIO0->FIOSET=1<<23;} 	//UART0����
//#define ARM_DSP_ON		{LPC_GPIO1->FIOCLR=1<<25;}	//ʹ��DSP
//#define ARM_DSP_OFF		{LPC_GPIO1->FIOSET=1<<25;}	//��ʹ��DSP
#define	SwitchOn		{LPC_GPIO0->FIOSET = 1<<9;}	//�����
#define	SwitchOff		{LPC_GPIO0->FIOCLR = 1<<9;}	//�����



#define aioin1			((1)&LPC_GPIO1->FIOPIN)				//aioin1


/*************************************RTC***********************************************/

#define IMSEC		0x00000001
#define IMMIN		0x00000002
#define IMHOUR		0x00000004
#define IMDOM		0x00000008
#define IMDOW		0x00000010
#define IMDOY		0x00000020
#define IMMON		0x00000040
#define IMYEAR		0x00000080

#define AMRSEC		0x00000001  /* Alarm mask for Seconds */
#define AMRMIN		0x00000002  /* Alarm mask for Minutes */
#define AMRHOUR		0x00000004  /* Alarm mask for Hours */
#define AMRDOM		0x00000008  /* Alarm mask for Day of Month */
#define AMRDOW		0x00000010  /* Alarm mask for Day of Week */
#define AMRDOY		0x00000020  /* Alarm mask for Day of Year */
#define AMRMON		0x00000040  /* Alarm mask for Month */
#define AMRYEAR		0x00000080  /* Alarm mask for Year */

#define PREINT_RTC	0x000001C8  /* Prescaler value, integer portion, 
				    PCLK = 15Mhz */
#define PREFRAC_RTC	0x000061C0  /* Prescaler value, fraction portion, 
				    PCLK = 15Mhz */
#define ILR_RTCCIF	0x01
#define ILR_RTCALF	0x02

#define CCR_CLKEN	0x01
#define CCR_CTCRST	0x02
#define CCR_CLKSRC	0x10

#define t1ms  	0
#define t10ms	1
#define t100ms	2
#define t1s		3
#define	t1us	4
#define	t10us	5
#define	t100us	6

#define	Uart3   0
#define	Uart1   0
#define	Uart0   1


/*************************************Uart***********************************************/

#define WrongFunCode 			0x01
#define	WrongDatalong 			0x02
#define	WrongStartaddr	 		0x03
#define	WrongDatalimit	 		0x04

#define StateFrameheadload 		0x00
#define StateSending			0x01
#define StateWaiting			0x02
#define StateReceving			0x03
#define StateAlarm				0x04

#define MainDevice 				0
#define Slave1Device			1
#define Slave2Device			2
#define Slave3Device			3
#define Slave4Device			4
#define Slave5Device			5
#define Slave6Device			6
#define Slave7Device			7
#define Slave8Device			8
#define Slave9Device			9
#define Slave10Device			10


#define	PowerON					0x01
#define	PowerOFF				0x00

#define WriteParameters			46	
#define ReadParameters			47
#define ClearFaults				48
#define IllegalOperating		50


#define Max						256


//#define 8Inch					0x60
//#define 43Inch					0x61

//��ѯҳ����Ϣ
#define PageStartaddr			0x0001	
#define PageDataNUM          	18
//��ҳ��Ϣ
#define UserDataStartaddr		0x0013	
#define UserDataNUM          	96

//��Ҫ����
#define MainDataStartaddr		0x0C00//0x0064				
#define MainDataNUM      		110
//ѡ�β���
#define SelectedStartaddr		0x00C8				
#define SelectedNUM      		51

//����ͼ
//#define WaveformDataStartaddr	0x00fA	
//#define WaveformDataNUM         6

//��״ͼA��
#define HistogramAStartaddr		0x1500	
#define HistogramaNUM         	52
//��״ͼ2A��
#define Histogram2AStartaddr	0x1534	
#define HistogramaNUM         	52
//��״ͼB��
#define HistogramBStartaddr		0x1568	
#define HistogramaNUM         	52
//��״ͼ2B��
#define Histogram2BStartaddr	0x159C	
#define HistogramaNUM         	52
//��״ͼC��
#define HistogramCStartaddr		0x15D0	
#define HistogramaNUM         	52
//��״ͼ2C��
#define Histogram2CStartaddr	0x1604	
#define HistogramaNUM         	52

//���η�ֵ��λ����
#define RunningDataStartaddr	0x0100				
#define RunningDataNUM      	100

//�ӻ�����
#define SlaveDataStartaddr		0x0200				
#define SlaveDataNUM      		10

//�ӻ�����2
#define SlaveDataStartaddr2		0x0264				
#define SlaveDataNUM2      		10

//�ӻ������С
#define SlaveMaxMinStartaddr	0x0300				
#define SlaveMaxMinNUM      	10

//�ӻ������С2
#define SlaveMaxMinStartaddr2	0x0364				
#define SlaveMaxMinNUM2      	10

//��Ҫ����
#define MainParameterStartaddr	0x0500				
#define MainParameterNUM      	100			//һ�ζ�ȡ���0x7F

//ARM����
#define ARMParameterStartaddr	0x0564				
#define ARMParameterNUM      	100			//������ARM�ϵĲ���
//ARM����2
#define ARMParameterStartaddr2	0x0D00				
#define ARMParameterNUM2      	100			//������ARM�ϵĲ���
//��������
#define CorrectParamStartaddr	0x0E00				
#define CorrectParamNUM      	11			//������ARM�ϵĲ���

//ʱ������
#define TimeSetStartaddr		0x0567				
#define TimeSetNUM      		5			//������ARM�ϵĲ���

//��������1
#define CorrectionStartaddr 	0x0600
#define CorrectionNUM      		100

//�ӻ�����1
#define SlaveParameterStartaddr 0x0700
#define SlaveParameterNUM      	100
//��������2
#define Correction2Startaddr 	0x0800
//#define CorrectionNUM      		100
//��������3
#define Correction3Startaddr 	0x0900
//#define CorrectionNUM      		100
//��Դ����
#define PassiveParameterStartaddr 	0x0A00
#define PassiveParameterNUM      	100
//�ֶ���Դ������ͨ��
#define ManualPassiveStartaddr 	0x0A64
#define ManualPassiveNUM      	32
//DSP��Դ������ͨ��
#define DSPPassiveStartaddr 	0x0A84
#define DSPPassiveNUM      	32
//��ʷ���ݲ�ѯ
#define DataLogStartaddr 	0x0B00
#define DataLogNUM      	46




//������Ϣ
#define FaultDataStartaddr		0x1000	
#define FaultDataNUM          	41

//�ӻ�������Ϣ
#define SlaveFaultStartaddr		0x111F	
#define FaultDataNUM          	41


//ͼ��1A�ನ��ͨ��;
#define Wave1AisleA				0
//ͼ��1B�ನ��ͨ��;
#define Wave1AisleB				1
//ͼ��1C�ನ��ͨ��;
#define Wave1AisleC				2
//ͼ��2A�ನ��ͨ��;
#define Wave2AisleA				3
//ͼ��2B�ನ��ͨ��;
#define Wave2AisleB				4
//ͼ��2C�ನ��ͨ��;
#define Wave2AisleC				5
//���ε���
#define WaveformLatticeNUM		114


/*********************************�û�������*************************************************/
#define	EepromVersion			2003204001										//1906234002			   		//Eeprom�汾
#define ARMVersion 				2210180101u//2206160101u //2003200101					//1812190101											//1806040101		   		// ARM�汾  ��1710240101
#define	DUGSVersion				1604086001					//DUGS�汾
//2210180101u���޸�04������
//2207050101u ����Ĭ���¶ȵ��裬֧�ֵ�120�ȣ���λ�����������ơ�
//2206160101  ������690NTC������֧�����µ�9000��ַ��λ��Ĭ�ϲ����޸ġ�
//2001090101�����ӵ�ѹ����������Ĭ�ϲ������ģ������汾���¡�

#define	DSP1Version				1604111001			   		//DSP1�汾
#define DSP2Version 			1603111101			   		//DSP2�汾
#define ProtocolVersion 		0001			   			//Э��汾
//#define	SerialNumber			0x0001034160405001			//���к�

#define InputStartAddr			0x0000
#define InputEndAddr			0x09A0
#define InputRegisteNum			2464

#define HoldingStartAddr		0x1000	
#define HoldingEndAddr			0x172D
#define HoldingRegisteNum		1880

#define EventStartAddr			0x2000	
#define EventEndAddr			0x23D4
#define EventRegisteNum			891

#define	LocalDevice				0x1034						//1033	APF��������//  1034	APF��������//2033	SVG��������//  2034	SVG��������
#define	PreviousDevice			0xFFFF
#define	AfterDevice				0xFFFF

#define	Table1StartAddr			0x0100
#define	Table1RegisterNum		133
#define	Table1Funcode			0x04

#define	Table2StartAddr			0x0200//0x1000
#define	Table2RegisterNum		1500//78
#define	Table2Funcode			0x04//0x03

#define	Table3StartAddr			0x0800//0x0200
#define	Table3RegisterNum		160//1500
#define	Table3Funcode			0x04//0x04

#define	Table4StartAddr			0x0900//0x1100
#define	Table4RegisterNum		160//1481
#define	Table4Funcode			0x04//0x03

#define	Table5StartAddr			0x1000//0x0800
#define	Table5RegisterNum		78//160
#define	Table5Funcode			0x03//0x04

#define	Table6StartAddr			0x1100//0x159D
#define	Table6RegisterNum		1481//300
#define	Table6Funcode			0x03//0x03

#define	Table7StartAddr			0x159D//0x0900
#define	Table7RegisterNum		300//160
#define	Table7Funcode			0x03//0x04

#define	Table8StartAddr			0x16c9//0xFFFF
#define	Table8RegisterNum		100//0xFFFF
#define	Table8Funcode			0x03//0xFFFF

#define	Table9StartAddr			0xFFFF
#define	Table9RegisterNum		0xFFFF
#define	Table9Funcode			0xFFFF

#define	Table10StartAddr		0xFFFF
#define	Table10RegisterNum		0xFFFF
#define	Table10Funcode			0xFFFF

#define	Table11StartAddr		0xFFFF
#define	Table11RegisterNum		0xFFFF
#define	Table11Funcode			0xFFFF

#define Reserved				0xFFFF


#define GetADMax					100						//��������
#define ADCAisleMax					4						//����ͨ��
/**********************************ARM״̬************************************************/
#define SystemChecking				0x00					//ϵͳ��ʼ����
#define SystemSetting				0x01					//ϵͳ������
#define SystemStandby				0x02					//ϵͳ������
#define SystemReady					0x03					//ϵͳԤ����
#define SystemDerating				0x04					//ϵͳ����
#define SystemRunning				0x05					//ϵͳ������
#define SystemReseting				0x06					//ϵͳ�ȴ�������
#define SystemProtect				0x07					//ϵͳ������
#define SystemPfault				0x08					//ϵͳ������
#define	SystemParameterError		0x09					//ϵͳ�������ô���

/*********************************��ʾ״̬**************************************************/
#define AnalogOutputMode			0x03					//ģ�����ģʽ
#define ReactiveBakingMode			0x04					//�޹�����ģʽ
#define BackwardReactiveBakingMode	0x05					//�����޹�����ģʽ
#define ActiveBakingMode			0x06					//�й�����ģʽ
#define BackwardActiveBakingMode	0x07					//�����й�����ģʽ
/*********************************�¼�����**************************************************/
#define ManualPowerOff				0x01					//�ֶ��ػ�
#define ManualPowerOn				0x02					//�ֶ�����
#define RemotePowerOff				0x03					//Զ�̹ػ�
#define RemotePowerOn				0x04					//Զ�̿���
#define	AlarmpPowerOff				0x05					//��ʱ�ػ�
#define	AlarmpPowerOn				0x06					//��ʱ����
#define	SPIERROR				0x07					//SPIͨѶ����

/******************************Eeprom��ַӳ���*********************************************/

#define EepParameterAddr		0x0000				   	//0		��ǰ��������Eeprom��ַ
#define ParameterNumMax 		1600					//��������+��Դ����+��������+ARM����+Ԥ�� +У�����

#define EepPassiveAddr			EepParameterAddr+200	//200	��ǰ��Դ����Eeprom��ַ
#define PassiveNumMax 			200						//��Դ�����������+ �汾У��	

#define EepCorrectionAddr		EepPassiveAddr+200		//400	��������Eeprom��ַ
#define CorrectionNumMax 		600						//���������������+ �汾У��	

#define EepARMParamddr			EepCorrectionAddr+600	//1000	��������Eeprom��ַ
#define ARMParamNumMax 			200						//ARM�����������+ �汾У��	

#define EepARMCorrectAddr		EepARMParamddr+200		//1200	ARM��������Eeprom��ַ
#define ARMCorrectNumMax 			22						//ARM�����������+ �汾У��

//1400 

#define EepFaultDataAddr		EepParameterAddr+1600	//1600	��������
#define FaultDataNumMax			160						//������������ 1+20*7+1

#define EepEventLogAddr			EepFaultDataAddr+160	//1760	�¼�����
#define EventLogNumMax			160						//������������ 1+20*7+1

#define EepDatalogAddr			EepEventLogAddr+160		//1920,2016,2112,2208,2304,2400,2496,2592,2688,2784,2880,2976,3072,3168,
#define EepDatalogNum			76						//13����¼+����ʷ��¼��

#define EepDatalogMaxAddr		EepDatalogAddr+160*14	//3264	��ʷ�������� DatalogMax,DatalogStart 2��

#define EepSlaveFaultAddr		0x0D00					//3328	�ӻ�����(�ӻ�1)  
#define EepSlave2FaultAddr		EepSlaveFaultAddr+104	//3432	�ӻ�����(�ӻ�2)
#define EepSlave3FaultAddr		EepSlave2FaultAddr+104  //3536	�ӻ�����(�ӻ�3)
#define EepSlave4FaultAddr		EepSlave3FaultAddr+104	//3640	�ӻ�����(�ӻ�4)
#define EepSlave5FaultAddr		EepSlave4FaultAddr+104	//3744	�ӻ�����(�ӻ�5)

// #define EepSlave6FaultAddr		EepSlave5FaultAddr+104	//3848	�ӻ�����(�ӻ�6)
// #define EepSlave7FaultAddr		EepSlave6FaultAddr+104	//3952	�ӻ�����(�ӻ�7)
// #define EepSlave8FaultAddr		EepSlave7FaultAddr+104	//4056	�ӻ�����(�ӻ�8)
// #define EepSlave9FaultAddr		EepSlave8FaultAddr+104	//4160	�ӻ�����(�ӻ�9)
// #define EepSlave10FaultAddr		EepSlave9FaultAddr+104	//4264	�ӻ�����(�ӻ�10)
// #define EepSlave11FaultAddr		EepSlave10FaultAddr+104	//4368	�ӻ�����(�ӻ�11)	
// #define EepSlave12FaultAddr		EepSlave11FaultAddr+104	//4472	�ӻ�����(�ӻ�12)
// #define EepSlave13FaultAddr		EepSlave12FaultAddr+104	//4576	�ӻ�����(�ӻ�13)
// #define EepSlave14FaultAddr		EepSlave13FaultAddr+104	//4680	�ӻ�����(�ӻ�14)
// #define EepSlave15FaultAddr		EepSlave14FaultAddr+104	//4784	�ӻ�����(�ӻ�15)
// #define EepSlave16FaultAddr		EepSlave15FaultAddr+104	//4888	�ӻ�����(�ӻ�16)


#define SingleFaultNum			104						//��̨�ӻ����� 104 = 1+7*14 +1 +4
#define SlaveFaultNumMax		1664					//�ӻ��������� 1664 = 104*16

#define EepParameterInitAddr	0x1380					//4992	Ĭ�ϲ��������ַ
#define ParameterInitNumMax 	1600					//�������������������+ �汾У��		

#define EepParameterAddrBak		0x19C0				   	//6592	�������������ַ
#define ParameterNumMaxBak 		1600					//Ĭ�ϱ�������������� +У�����



/******************************
0000h	EepParameterAddr		��ʼ��ַ	���ò��� 220+2
  .
  .
  .
  .
0400h	EepFaultDataAddr		��ʼ��ַ	�������� 981 +1
  .
  .
  .
  .
07D6h	EepFaultDataAddr		������ַ	��������
 ... 		Ԥ��
0800h	EepSlaveFaultAddr		��ʼ��ַ	�������� 981 +1
  .
  .
  .
  .
0BE8h	EepSlaveFaultAddr		������ַ	�ӻ�����
 ... 		Ԥ��
0C00h	EepParameterInitAddr	��ʼ��ַ	��������
  .
  .
  .
  .
0fffh	EepParameterInitAddr	������ַ	��������

*******************************/ 
#define Internaltemp 	Avg[0]				// �ӽ��¶�	
#define ModuleTemp		Avg[1]				// ģ���¶�
#define ReservedADC2 	Avg[2]				//Ԥ������ͨ��
#define ReservedADC3	Avg[3]				//Ԥ������ͨ��


typedef struct {
    uint32_t RTC_Sec;     /* Second value - [0,59] */
    uint32_t RTC_Min;     /* Minute value - [0,59] */
    uint32_t RTC_Hour;    /* Hour value - [0,23] */
    uint32_t RTC_Mday;    /* Day of the month value - [1,31] */
    uint32_t RTC_Mon;     /* Month value - [1,12] */
    uint32_t RTC_Year;    /* Year value - [0,4095] */
    uint32_t RTC_Wday;    /* Day of week value - [0,6] */
    uint32_t RTC_Yday;    /* Day of year value - [1,365] */
} RTCTime;

typedef struct {
    uint16 Sec;     /* Second value - [0,59] */
    uint8 Min;     /* Minute value - [0,59] */
    uint8 Hour;    /* Hour value - [0,23] */
    uint8 Mday;    /* Day of the month value - [1,31] */
    uint8 Mon;     /* Month value - [1,12] */
    uint16 Year;    /* Year value - [0,4095] */
} DUGSTime;

typedef struct {
uint16	Framehead;
uint8	Framelength;
uint8	Funcode;
uint8	RegisterAddr;
uint16	StartAddr;
uint8	Aisle;
uint8	Datalength;
uint16   Databuff[Max];
//uint8   FirstCommunSuccess;
uint16 	Failure;
uint16	Counter;
uint8	WriteDataTypes;
} Commun;

typedef struct {
uint8 No;	
uint8 Year;
uint8 Month;
uint8 Day;
uint8 Hour;
uint8 Min;
uint8 Code;
} Fault;
typedef struct {
uint8 State[32];	
uint8 Fault[32];	
uint8 Switch[32];
uint8 Ready[32];
//uint8 ReadyBak[16];
uint8 Times[32];
} Aisle;

extern uint16  TimeCount[4];

extern void RTC_IRQHandler (void);
extern void RTCInit( void );
extern void RTCStart( void );
extern void RTCStop( void );
extern void RTC_CTCReset( void );
extern void RTCSetTime( RTCTime );
extern RTCTime RTCGetTime( void );
extern void RTCSetAlarm( RTCTime );
extern void RTCSetAlarmMask( uint32_t AlarmMask );
extern uint16 GetSystemTime(uint8 SetTime);
extern uint16 GetCurrentTime(uint8 SetTime,uint16 GetStartTime);
extern int16 abs( int16 tmp);


#endif /* end __RTC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
