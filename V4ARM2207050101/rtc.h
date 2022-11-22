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
#define I2C_write    	{LPC_GPIO1->FIOCLR=1<<28;} 	//I2C写
#define I2C_read     	{LPC_GPIO1->FIOSET=1<<28;} 	//I2C读

#define OnOff_Disable	{LPC_GPIO2->FIOSET=1<<13;}	//置1：表示不允许DSP开机
#define OnOff_Enable	{LPC_GPIO2->FIOCLR=1<<13;}	//置0：表示允许DSP开机

#define OnOff2_Disable	{LPC_GPIO0->FIOSET=1<<22;}	//置1：表示不允许DSP开机
#define OnOff2_Enable	{LPC_GPIO0->FIOCLR=1<<22;}	//置0：表示允许DSP开机

#define	SpiComm_Disable	{LPC_GPIO2->FIOSET=1<<11;}	//置1：表示不允许DSP向ARM芯片发送SPI通讯
#define SpiComm_Enable	{LPC_GPIO2->FIOCLR=1<<11;}	//置0：表示允许DSP向ARM芯片发送SPI通讯

#define SpiWrite_Disable {LPC_GPIO2->FIOSET=1<<12;}	//置1：表示SPI写不使能，参数生效
#define SpiWrite_Enable	{LPC_GPIO2->FIOCLR=1<<12;}	//置0：表示SPI写使能

#define DSP_OnOff		((1<<13)&LPC_GPIO2->FIOPIN)	//置0：表示DSP已经开机.置1：DSP没有开机
#define AC_Switch		((1<<25)&LPC_GPIO1->FIOPIN)	//置0：表示DSP已经开机.置1：DSP没有开机
//#define BluetoothConnectOK		((1<<2)&LPC_GPIO2->FIOPIN)	//置0：蓝牙未连接.置1：蓝牙已连接
#define BluetoothConnectOK		((1<<1)&LPC_GPIO1->FIOPIN)	//置0：蓝牙未连接.置1：蓝牙已连接

#define led4_ON			{LPC_GPIO0->FIOCLR=1<<29;}  //LED4
#define led4_OFF		{LPC_GPIO0->FIOSET=1<<29;}  //LED4
#define led3_ON			{LPC_GPIO0->FIOCLR=1<<30;}  //LED3
#define led3_OFF		{LPC_GPIO0->FIOSET=1<<30;}  //LED3
#define	led2_ON			{LPC_GPIO3->FIOCLR=1<<26;}	//LED2
#define	led2_OFF		{LPC_GPIO3->FIOSET=1<<26;}	//LED2
#define	led1_ON			{LPC_GPIO3->FIOCLR=1<<25;}	//LED1
#define	led1_OFF		{LPC_GPIO3->FIOSET=1<<25;}	//LED1
#define uart_read3		{LPC_GPIO1->FIOCLR=1<<29;} 	//UART3发送
#define uart_write3   	{LPC_GPIO1->FIOSET=1<<29;} 	//UART3接收
#define uart_read0		{LPC_GPIO0->FIOCLR=1<<23;} 	//UART0发送
#define uart_write0   	{LPC_GPIO0->FIOSET=1<<23;} 	//UART0接收
#define uart_read1		{LPC_GPIO2->FIOCLR=1<<2;} 	//UART1发送
#define uart_write1   	{LPC_GPIO2->FIOSET=1<<2;} 	//UART1接收

//#define uart_write1		{LPC_GPIO0->FIOCLR=1<<23;} 	//UART0发送
//#define uart_read1   	{LPC_GPIO0->FIOSET=1<<23;} 	//UART0接收
//#define ARM_DSP_ON		{LPC_GPIO1->FIOCLR=1<<25;}	//使能DSP
//#define ARM_DSP_OFF		{LPC_GPIO1->FIOSET=1<<25;}	//不使能DSP
#define	SwitchOn		{LPC_GPIO0->FIOSET = 1<<9;}	//输出高
#define	SwitchOff		{LPC_GPIO0->FIOCLR = 1<<9;}	//输出低



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

//查询页面信息
#define PageStartaddr			0x0001	
#define PageDataNUM          	18
//首页信息
#define UserDataStartaddr		0x0013	
#define UserDataNUM          	96

//主要数据
#define MainDataStartaddr		0x0C00//0x0064				
#define MainDataNUM      		110
//选次参数
#define SelectedStartaddr		0x00C8				
#define SelectedNUM      		51

//波形图
//#define WaveformDataStartaddr	0x00fA	
//#define WaveformDataNUM         6

//柱状图A相
#define HistogramAStartaddr		0x1500	
#define HistogramaNUM         	52
//柱状图2A相
#define Histogram2AStartaddr	0x1534	
#define HistogramaNUM         	52
//柱状图B相
#define HistogramBStartaddr		0x1568	
#define HistogramaNUM         	52
//柱状图2B相
#define Histogram2BStartaddr	0x159C	
#define HistogramaNUM         	52
//柱状图C相
#define HistogramCStartaddr		0x15D0	
#define HistogramaNUM         	52
//柱状图2C相
#define Histogram2CStartaddr	0x1604	
#define HistogramaNUM         	52

//各次幅值相位数据
#define RunningDataStartaddr	0x0100				
#define RunningDataNUM      	100

//从机数据
#define SlaveDataStartaddr		0x0200				
#define SlaveDataNUM      		10

//从机数据2
#define SlaveDataStartaddr2		0x0264				
#define SlaveDataNUM2      		10

//从机最大最小
#define SlaveMaxMinStartaddr	0x0300				
#define SlaveMaxMinNUM      	10

//从机最大最小2
#define SlaveMaxMinStartaddr2	0x0364				
#define SlaveMaxMinNUM2      	10

//主要参数
#define MainParameterStartaddr	0x0500				
#define MainParameterNUM      	100			//一次读取最大0x7F

//ARM参数
#define ARMParameterStartaddr	0x0564				
#define ARMParameterNUM      	100			//保存在ARM上的参数
//ARM参数2
#define ARMParameterStartaddr2	0x0D00				
#define ARMParameterNUM2      	100			//保存在ARM上的参数
//修正参数
#define CorrectParamStartaddr	0x0E00				
#define CorrectParamNUM      	11			//保存在ARM上的参数

//时间设置
#define TimeSetStartaddr		0x0567				
#define TimeSetNUM      		5			//保存在ARM上的参数

//修正参数1
#define CorrectionStartaddr 	0x0600
#define CorrectionNUM      		100

//从机参数1
#define SlaveParameterStartaddr 0x0700
#define SlaveParameterNUM      	100
//修正参数2
#define Correction2Startaddr 	0x0800
//#define CorrectionNUM      		100
//修正参数3
#define Correction3Startaddr 	0x0900
//#define CorrectionNUM      		100
//无源参数
#define PassiveParameterStartaddr 	0x0A00
#define PassiveParameterNUM      	100
//手动无源控制器通道
#define ManualPassiveStartaddr 	0x0A64
#define ManualPassiveNUM      	32
//DSP无源控制器通道
#define DSPPassiveStartaddr 	0x0A84
#define DSPPassiveNUM      	32
//历史数据查询
#define DataLogStartaddr 	0x0B00
#define DataLogNUM      	46




//故障信息
#define FaultDataStartaddr		0x1000	
#define FaultDataNUM          	41

//从机故障信息
#define SlaveFaultStartaddr		0x111F	
#define FaultDataNUM          	41


//图形1A相波形通道;
#define Wave1AisleA				0
//图形1B相波形通道;
#define Wave1AisleB				1
//图形1C相波形通道;
#define Wave1AisleC				2
//图形2A相波形通道;
#define Wave2AisleA				3
//图形2B相波形通道;
#define Wave2AisleB				4
//图形2C相波形通道;
#define Wave2AisleC				5
//波形点阵
#define WaveformLatticeNUM		114


/*********************************用户配置区*************************************************/
#define	EepromVersion			2003204001										//1906234002			   		//Eeprom版本
#define ARMVersion 				2210180101u//2206160101u //2003200101					//1812190101											//1806040101		   		// ARM版本  旧1710240101
#define	DUGSVersion				1604086001					//DUGS版本
//2210180101u：修复04不发送
//2207050101u 增加默认温度电阻，支持到120度，上位机防死机机制。
//2206160101  增加了690NTC采样，支持最新的9000地址上位机默认参数修改。
//2001090101：增加电压修正参数，默认参数更改，参数版本更新。

#define	DSP1Version				1604111001			   		//DSP1版本
#define DSP2Version 			1603111101			   		//DSP2版本
#define ProtocolVersion 		0001			   			//协议版本
//#define	SerialNumber			0x0001034160405001			//序列号

#define InputStartAddr			0x0000
#define InputEndAddr			0x09A0
#define InputRegisteNum			2464

#define HoldingStartAddr		0x1000	
#define HoldingEndAddr			0x172D
#define HoldingRegisteNum		1880

#define EventStartAddr			0x2000	
#define EventEndAddr			0x23D4
#define EventRegisteNum			891

#define	LocalDevice				0x1034						//1033	APF三相三线//  1034	APF三相四线//2033	SVG三相三线//  2034	SVG三相四线
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


#define GetADMax					100						//采样次数
#define ADCAisleMax					4						//采样通道
/**********************************ARM状态************************************************/
#define SystemChecking				0x00					//系统初始化中
#define SystemSetting				0x01					//系统设置中
#define SystemStandby				0x02					//系统待机中
#define SystemReady					0x03					//系统预备中
#define SystemDerating				0x04					//系统降额
#define SystemRunning				0x05					//系统运行中
#define SystemReseting				0x06					//系统等待重启中
#define SystemProtect				0x07					//系统保护中
#define SystemPfault				0x08					//系统故障中
#define	SystemParameterError		0x09					//系统参数设置错误

/*********************************显示状态**************************************************/
#define AnalogOutputMode			0x03					//模拟输出模式
#define ReactiveBakingMode			0x04					//无功烤机模式
#define BackwardReactiveBakingMode	0x05					//反向无功烤机模式
#define ActiveBakingMode			0x06					//有功烤机模式
#define BackwardActiveBakingMode	0x07					//反向有功烤机模式
/*********************************事件类型**************************************************/
#define ManualPowerOff				0x01					//手动关机
#define ManualPowerOn				0x02					//手动开机
#define RemotePowerOff				0x03					//远程关机
#define RemotePowerOn				0x04					//远程开机
#define	AlarmpPowerOff				0x05					//定时关机
#define	AlarmpPowerOn				0x06					//定时开机
#define	SPIERROR				0x07					//SPI通讯故障

/******************************Eeprom地址映射表*********************************************/

#define EepParameterAddr		0x0000				   	//0		当前保护参数Eeprom地址
#define ParameterNumMax 		1600					//主机参数+无源参数+修正参数+ARM参数+预留 +校验变量

#define EepPassiveAddr			EepParameterAddr+200	//200	当前无源参数Eeprom地址
#define PassiveNumMax 			200						//无源参数最大数量+ 版本校验	

#define EepCorrectionAddr		EepPassiveAddr+200		//400	修正参数Eeprom地址
#define CorrectionNumMax 		600						//修正参数最大数量+ 版本校验	

#define EepARMParamddr			EepCorrectionAddr+600	//1000	修正参数Eeprom地址
#define ARMParamNumMax 			200						//ARM参数最大数量+ 版本校验	

#define EepARMCorrectAddr		EepARMParamddr+200		//1200	ARM修正参数Eeprom地址
#define ARMCorrectNumMax 			22						//ARM参数最大数量+ 版本校验

//1400 

#define EepFaultDataAddr		EepParameterAddr+1600	//1600	故障数据
#define FaultDataNumMax			160						//故障数据总数 1+20*7+1

#define EepEventLogAddr			EepFaultDataAddr+160	//1760	事件数据
#define EventLogNumMax			160						//故障数据总数 1+20*7+1

#define EepDatalogAddr			EepEventLogAddr+160		//1920,2016,2112,2208,2304,2400,2496,2592,2688,2784,2880,2976,3072,3168,
#define EepDatalogNum			76						//13条记录+总历史记录数

#define EepDatalogMaxAddr		EepDatalogAddr+160*14	//3264	历史数据总数 DatalogMax,DatalogStart 2个

#define EepSlaveFaultAddr		0x0D00					//3328	从机故障(从机1)  
#define EepSlave2FaultAddr		EepSlaveFaultAddr+104	//3432	从机故障(从机2)
#define EepSlave3FaultAddr		EepSlave2FaultAddr+104  //3536	从机故障(从机3)
#define EepSlave4FaultAddr		EepSlave3FaultAddr+104	//3640	从机故障(从机4)
#define EepSlave5FaultAddr		EepSlave4FaultAddr+104	//3744	从机故障(从机5)

// #define EepSlave6FaultAddr		EepSlave5FaultAddr+104	//3848	从机故障(从机6)
// #define EepSlave7FaultAddr		EepSlave6FaultAddr+104	//3952	从机故障(从机7)
// #define EepSlave8FaultAddr		EepSlave7FaultAddr+104	//4056	从机故障(从机8)
// #define EepSlave9FaultAddr		EepSlave8FaultAddr+104	//4160	从机故障(从机9)
// #define EepSlave10FaultAddr		EepSlave9FaultAddr+104	//4264	从机故障(从机10)
// #define EepSlave11FaultAddr		EepSlave10FaultAddr+104	//4368	从机故障(从机11)	
// #define EepSlave12FaultAddr		EepSlave11FaultAddr+104	//4472	从机故障(从机12)
// #define EepSlave13FaultAddr		EepSlave12FaultAddr+104	//4576	从机故障(从机13)
// #define EepSlave14FaultAddr		EepSlave13FaultAddr+104	//4680	从机故障(从机14)
// #define EepSlave15FaultAddr		EepSlave14FaultAddr+104	//4784	从机故障(从机15)
// #define EepSlave16FaultAddr		EepSlave15FaultAddr+104	//4888	从机故障(从机16)


#define SingleFaultNum			104						//单台从机故障 104 = 1+7*14 +1 +4
#define SlaveFaultNumMax		1664					//从机故障总数 1664 = 104*16

#define EepParameterInitAddr	0x1380					//4992	默认参数保存地址
#define ParameterInitNumMax 	1600					//出厂保护参数最大数量+ 版本校验		

#define EepParameterAddrBak		0x19C0				   	//6592	出厂参数保存地址
#define ParameterNumMaxBak 		1600					//默认保护参数最大数量 +校验变量



/******************************
0000h	EepParameterAddr		起始地址	常用参数 220+2
  .
  .
  .
  .
0400h	EepFaultDataAddr		起始地址	故障数据 981 +1
  .
  .
  .
  .
07D6h	EepFaultDataAddr		结束地址	主机故障
 ... 		预留
0800h	EepSlaveFaultAddr		起始地址	故障数据 981 +1
  .
  .
  .
  .
0BE8h	EepSlaveFaultAddr		结束地址	从机故障
 ... 		预留
0C00h	EepParameterInitAddr	起始地址	出厂参数
  .
  .
  .
  .
0fffh	EepParameterInitAddr	结束地址	出厂参数

*******************************/ 
#define Internaltemp 	Avg[0]				// 钣金温度	
#define ModuleTemp		Avg[1]				// 模块温度
#define ReservedADC2 	Avg[2]				//预留采样通道
#define ReservedADC3	Avg[3]				//预留采样通道


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
