/*****************************************************************************
 *   mian.c:  单机ARM程序
 *   History
 *   2017.10.24  ver 1710240101
 *	 外设:	ARM硬件配置包含 RTC，I2C，SSP0，Uart0，Uart1，Uart3等外设处理。
 	 函数:	本文件函数功能包含 管脚初始化，定时器初始化，I2C初始化，I2C读写，参数初始化，蓝牙初始化，
 	 		故障记录初始化，故障处理，事件处理，SPI通讯处理，开关机处理，定时开关机处理，累计事件处理，
 	 		Hex转BCD码，硬件故障处理，无源返回信号处理，数据累计处理
 	 中断:	本文件中中断函数包含:定时中断，	用于处理增加计数器
 	 							 SSP中断，	用于处理DSP接收数据
 	 							 Uart0中断，用于处理蓝牙数据接收
 	 							 Uart1中断，用于处理屏幕数据接收
 	 							 Uart3中断，用于处理对外485数据接收
 	 							 硬件故障中断，用于ARM出现硬件故障后的复位
 	 							 看门狗中断	用于ARM触发看门狗后的复位
*****************************************************************************/
/*****定义头文件*****/
#include "Lpc17xx.h"
#include "math.h"
#include "type.h"
#include "ssp.c" 
#include "IAP.c"
#include "IAP.h"
#include "uart.c"
#include "rtc.c"
#include "WDT.c"

#define I2C_FEQ 400000								//I2C频率



extern void	delay(uint32_t dly);//延迟函数，用于程序延迟，非确定时间
extern uint16 crc_chk(uint8 *buf,uint32 length);//CRC16计算
extern void I2c1_Init(void);//I2C初始化
extern void I2c1_Recv(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num);//I2C读取函数
extern void I2c1_Send(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num);//I2C写入函数
extern void WriteEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//I2C写入封装函数(起始地址，目标指针，读取数量)每次写8个byte，起始地址需要取8的整数倍
extern void ReadEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//I2C读取封装函数(起始地址，目标指针，读取数量)
extern void Parameter_init(void);//参数初始化，首次上电参数初始化
extern void GZ_init(void);//故障初始化，首次上电故障清0
extern void GPIO_init(void);//IO管脚初始化
extern void Bluetooth_init(void);//蓝牙初始化
extern uint32_t TIME_init(uint8 time_num);//定时器初始化(定时器外设标号)，目前初始化为1ms中断
extern void error_handle(void); //故障处理，记录主机故障
extern void Event_handle(void); //事件记录，记录事件信息包括，开关机，定时开关机，远程开关机
extern void SPIReadyData(void); //SPI通讯函数，准备SPI下一帧返回数据装载
extern void OnOffProcess(void); //开关机处理
extern void AlarmTimeProcess(void);//定时开关机处理
extern void AccumEventProcess(void);//累计事件处理，主要累计数据累加，运行数据记录
extern uint8 HEX2BCD(uint64 Dec, uint8 *Bcd, uint8 length);     //HEX转为BCD子程序  
extern void PassiveReturnSignProcess(void);//无源投切返回信号处理，M3江苏标设备使用
extern void Reset(void);//ARM系统复位函数
extern void DatalogProcess(void);//数据记录处理
extern void TIMER0_IRQHandler (void) ;//定时器中断，数据器分别累计1ms，10ms，100ms和1s
 void UART0_IRQHandler(void) ;//Uart0中断，用于蓝牙模块通讯，波特率9600
extern void UART1_IRQHandler(void); //Uart1中断，用于DGUS屏幕通讯，波特率115200
extern void UART3_IRQHandler(void) ;//Uart3中断，用于DTU通讯，波特率可设
extern void hard_fault_handler_c(unsigned int * hardfault_args);//ARM硬件故障中断，用于突发性硬件故障复位

//extern void ARM_proess(void);
/************************激活码**********************************/
/**模块**/
extern uint16 Testtime_time1;
extern uint16 test_time;								//测试时间设置
extern uint16 time_flag;							//写i2c标志符
extern uint16 sendok;									//发送成功标志符


/**********************用户配置参数******************************/
__align(4) uint8 Parameter[ParameterNumMax],ParameterBak[ParameterInitNumMax];//存储类修改,修改最高级别对象的字节边界,保证数据对象是相应对齐
extern uint32_t SystemFrequency;										//系统频率
uint8 	led2_flag,led3_flag,led4_flag;									//Led闪烁标志位
uint8 	gz_dan[160];													//故障
uint8 	SlaveFault[5][100];											//从机故障
uint8 	Event_Logging[160];												//事件记录
uint8   DR_LOG[800];
uint8   capa_bak;

double  pclock;															//系统时钟
uint8	RemoteEnable;													//远程使能
uint16	AlarmTime1[5],AlarmTime2[5],AlarmTime3[5],AlarmTime4[5];		//定时开关机变量
uint8	APFStartRunning =0;												//开关机状态
extern Aisle	IntPassive;												//无源通道
uint16	TimeDelay=0,AccumDelay=0,ResetDelay =0,DatalogDelay;			//延迟类数据
uint32	moni_delay =0;													//模拟
uint32	ProjectNo;														//项目号
uint16	ProductionNo;													//生产号
uint16	VolOnOffEnable,CurOnOffEnable;									//电压开关机，电流开关机
//主互感器安装位置，主互感器A相方向，主互感器B相方向,主互感器C相方向,主CT相序,输出互感器安装位置，输出互感器A相方向，输出互感器B相方向,输出互感器C相方向,输出互感器相序,
uint16	MainCTLocation,MainCTDirectionA,MainCTDirectionB,MainCTDirectionC,MainCTPhase,OutCTDirectionA,OutCTDirectionB,OutCTDirectionC,OutCTPhase;
uint16	Position[15],Group[15],Capacitance[15];							//仓位变量
uint16  PassiveChannel[32],PassiveValue[32],PassiveCom;					//无源变量
uint16	CorrectParam[11];												//修正变量
uint32	IntPassiveState,IntPassiveFault,IntPassiveSwitch;				//无源状态，无源故障，无源开关
uint16	ActiveBal,ReactiveBal,HarmonicBal,ApparentBal;					//有功不平衡，无功不平衡，谐波不平衡，视在不平衡,视在不平衡备份
uint16	SetupMode;														//仓位配置
uint64	SerialNumber,tmp64;												//序列号
uint8	Event_Code,RecordEventFlag,ACSwitchFlag,AlarmSwitchFlag;		//事件代码，事件标志位，面板旋转开关标志位，定时开关机标志位
uint16	Datalog[38],DatalogMax,DatalogStart;							//数据记录，数据记录最大值，数据记录起始位置
volatile uint8 DatalogReset;											//数据变量复位

/***********************Timer时钟变量区*************************/
uint8  	time_1us,time_10us,time_100us,time_1ms,time_10ms,time_100ms,time_1s;
uint8	time_SpiDelay;
uint16  TimeCount[4];
RTCTime local_time, alarm_time, current_time,Event_time,DR_time;   				//RTC结构体
/***********************Uart0变量区*****************************/
extern volatile uint8		RecvBTFlag;                               	// 串口接收数据标志位
extern uint8        		RecvSciBuf[150];                         	// 串口接收数据缓存
extern uint32      			RecvSciNum;                                 // 串口接收数据个数
//extern uint16		 		RecvSuccessTest;							// 串口接收成功次数
//extern uint16 		 	SciSuccessNUM;								// 串口接收成功次数
extern	uint16				SciWaitingCount;							// 串口等待计时
extern	uint8 				BluetoothTestOK;							// 蓝牙测试成功
uint8 						BluetoothTestOK;							// 蓝牙测试成功
uint8						BluetoothChangeNameOK;						// 蓝牙设备名称修改
uint8						BluetoothATcmd;								// 蓝牙AT指令状态

uint8		bluetooth_int;	//判断是否需要初始化
uint16 count_swj;
uint8   first_int;				//第一次初始化
/***********************Uart3变量区*****************************/
extern volatile uint8		RecvNewFlag;                               	// 串口接收数据标志位
extern uint8        		RecvSci3Buf[150];                         	// 串口接收数据缓存
extern uint32      			RecvSci3Num;                                // 串口接收数据个数
//extern uint16		 		RecvSuccessTest3;							// 串口接收成功次数
//extern uint16 		 	SciSuccessNUM3;								// 串口接收成功次数
extern	uint16				SciWaitingCount3;							// 串口等待计时
uint8						Uart3Reset,Uart1Reset,Uart0Reset;			// 串口复位标志位
/********************** *SSP变量区*************************************************/
volatile uint8				SPIRecvSuccess;                             // SPI接收数据标志位
uint8						SPICommDelay;								// SPI通讯延迟
uint8						NextReceFrameID;							//下次接收命令帧ID
uint8						SPICommError;								//SPI通讯错误
uint16						SPICommCount;								//SPI通讯计数

uint16	ReadCount;					//参数读取计数

//								 主机参数地址	各次修正地址	各次修正地址	各次修正地址	  无源参数		用户参数地址
//									0				1				2				3				4				5
uint16	SPI_NextStartAddress[44]={	10000,			10100,			10200,			10300,			10400,			11000,
//								从机0参数地址	从机1参数地址	从机2参数地址	从机3参数地址	从机4参数地址
//									6				7				8				9				10
									12000,			12100,			12200,			12300,			12400,
//								从机5参数地址	从机6参数地址	从机7参数地址	从机8参数地址	从机9参数地址
//									11				12				13				14				15
									12500,			12600,			12700,			12800,			12900,
//								从机10参数地址	从机11参数地址	从机12参数地址	从机13参数地址	从机14参数地址	60000参数地址
//									16				17				18				19				20				21
									13000,			13100,			13200,			13300,			13400,			60000,
//								主机数据地址	A相系统侧地址	B相系统侧地址	C相系统侧地址	A相负载侧地址
//									22				23				24				25				26
									0,				100,			200,			300,			400,
//								B相负载侧地址	C相负载侧地址	A相输出侧地址	B相输出侧地址	C相输出侧地址
//									27				28				29				30				31
									500,			600,			700,			800,			900,
//								A相电压地址		B相电压地址		C相电压地址		A相电容柜地址	B相电容柜地址
//									32				33				34				35				36
									1000,			1100,			1200,			1300,			1400,
//								C相电容柜地址	从机基本数据	从机最大最小数据 从机11-16数据	从机11-16数据	无源返回信号	手动无源投切
//									37				38				39				40				41			42				43
									1500,			2000,			2100,			2200,			2300,		9100,			9000};


//								 	无效地址		主机参数地址	各次修正地址	各次修正地址	各次修正地址	 无源参数		用户参数地址
//										0				1				2				3				4			  	5				6
uint16	SPI_CurrentStartAddress[22]={	0xFFFF,			10000,			10100,			10200,			10300,			10400,			11000,
//									从机0参数地址	从机1参数地址	从机2参数地址	从机3参数地址	从机4参数地址
//										7				8				9				10				11
										12000,			12100,			12200,			12300,			12400,
//									从机5参数地址	从机6参数地址	从机7参数地址	从机8参数地址	从机9参数地址
//										12				13				14				15				16
										12500,			12600,			12700,			12800,			12900,
//									从机10参数地址	从机11参数地址	从机12参数地址	从机13参数地址	从机14参数地址	      60000参数
//										17				18				19				20				21				22     
										13000,			13100,			13200,			13300,			13400,			60000};
/******************************************************************************
** 函数名: delay
**
** 功能：延时
**		  
** 用法：delay(要延时的时间)
******************************************************************************/
void delay(uint32_t dly) 
{
  uint32_t	  i_delay;
  for(; dly>0; dly--)
  for(i_delay=0; i_delay<100000; i_delay++); 
}
/******************************************************************************
** 函数名: Get_temp
**
** 功能：温度计算
**		  
** 用法：Get_temp(采样值)
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
//AD 初始化
/*void AD_ini(void)
{
  LPC_SC->PCONP	|=	(1<<12);              
  LPC_ADC->ADCR	=	15 |                  	
   		            (6<< 8) |              
                  (1<<21);               						 
}
*/
/************************************************************************
** 函数名: crc_chk
**
** 功能：计算CRC，用于校验
**			
** 用法：crc_chk(要计算的数,长度)
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
** 函数名: I2c1_Init 
**
** 功能：I2C的初始化
************************************************************************/
void I2c1_Init(void)
{
	
 	LPC_I2C2->I2SCLH = (pclock/I2C_FEQ+1)/2;	//I2C1时钟是400KHz
 	LPC_I2C2->I2SCLL = (pclock/I2C_FEQ)/2;
 	LPC_I2C2->I2CONCLR = 0x2C;              	//清除AAC，SIC，STAC位
 	LPC_I2C2->I2CONSET = 0x40;              	//使能I2C功能，主模式
}

/************************************************************************
** 函数名: I2c1_Recv 
**
** 功能：I2C的读取
**
** 用法：I2c1_Recv(I2C地址,入口地址,读取的数组,保存的个数)
************************************************************************/
void I2c1_Recv(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num)
{ 
 	uint32_t	var_i2c;
 	LPC_I2C2->I2CONCLR = (1<<2)|(1<<3)|(1<<5);	//清AA//清SI//清STA          
 	for(var_i2c=10000; var_i2c>0; var_i2c--);           
 	LPC_I2C2->I2CONSET = 0x60;                	//起始信号，选择为主模式         
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
 	LPC_I2C2->I2CONSET = (1<<5);              	//重复起始条件
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
** 函数名: I2c1_Send 
**
** 功能：I2C的保存
**
** 用法：I2c1_Send(I2C地址，入口地址，保存数组，保存的个数)
************************************************************************/
void I2c1_Send(uint8_t sla,uint32_t suba,uint8_t *pointeri2c,uint32_t num)
{ 
 	uint32_t	var_i2c;  
 	LPC_I2C2->I2CONCLR = (1<<2)|(1<<3)|(1<<5);	//清AA//清SI//清STA    
 	LPC_I2C2->I2CONSET = 0x60;                  //起始信号，选择为主模式
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
 	LPC_I2C2->I2CONCLR = (1<<3)|(1<<5);         //结束总线                
 	for(var_i2c=0; var_i2c<10000; var_i2c++);
}
/************************************************************************
** 函数名: WriteEeprom 
**
** 功能：Eeprom的保存
**
** 用法：WriteEeprom(地址，保存数组，个数)
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
	   		I2c1_Send(0xa0,EepromAddr,pointer8,8);   				//将出厂参数写到正式参数中
	   		for(delaytime=0x80000; delaytime>0; delaytime--);
	  	}
		if(j>0)
		{
			pointer8 =pointer;
			pointer8 += counter<<3;
			EepromAddr =addr;
			EepromAddr += counter<<3;
			I2c1_Send(0xa0,EepromAddr,pointer8,j);   				//将出厂参数写到正式参数中
	   		for(delaytime=0x80000; delaytime>0; delaytime--);
		}
	}
	else
	{
		I2c1_Send(0xa0,addr,pointer,num);   				//将出厂参数写到正式参数中
		for(delaytime=0x80000; delaytime>0; delaytime--);
	}
	
	//LPC_UART0->IER = 1;
	//LPC_UART1->IER = 1; 
	//LPC_UART3->IER = 1; 
	LPC_SSP0->IMSC = 0x04;
}
/************************************************************************
** 函数名: ReadEeprom 
**
** 功能：Eeprom的保存
**
** 用法：WriteEeprom(地址，保存数组，个数)
************************************************************************/
void ReadEeprom(uint32_t addr,uint8_t *pointer,uint32_t num)
{ 
	I2c1_Recv(0xa0,addr,pointer,num);				//读取24C32内的出厂参数
}
/******************************************************************************
** 函数名:Parameter_init 
**
** 功能：参数初始化
******************************************************************************/
void Parameter_init(void)
{
	//uint8	temp;
 	uint16	i,j,tmp[100],tmp16;
	uint32 	tmp32;
	uint64	tmp64H,tmp64L,tmp64Val;
/****************************************************初始化全局变量*******************************************************/		
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
	ReadEeprom(0x0000,Eep_parameter,ParameterInitNumMax);		//读取24C32内的出厂参数
	*/


/****************************************************初始化主机出厂参数***************************************************/	
 	ReadEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);		//读取24C32内的出厂参数

	tmp32 = ((Eep_parameter[ParameterInitNumMax-4]<<24)
			|(Eep_parameter[ParameterInitNumMax-3]<<16)
			|(Eep_parameter[ParameterInitNumMax-2]<<8)
			|(Eep_parameter[ParameterInitNumMax-1]));
	if(tmp32!=EepromVersion)											//判断出厂参数版本是否更新
 	{
 		i=0;
	  	Eep_parameter[0]=0;		Eep_parameter[1]=230;							//接入系统的额定相电压	
	  	Eep_parameter[2]=0;		Eep_parameter[3]=0;							//接入系统的额定频率	
	  	Eep_parameter[4]=0;		Eep_parameter[5]=0;							//电压支撑响应速度			   			  
		Eep_parameter[6]=0;		Eep_parameter[7]=2;								//系统电压的相序
		Eep_parameter[8]=0;		Eep_parameter[9] =0; 							//系统的接线方式			   
		Eep_parameter[10]=0;	Eep_parameter[11] =1;   				  		//设备的基本工作模式			   	
		Eep_parameter[12]=0;	Eep_parameter[13] =1;  				 			//是否启动有功不平衡抑制		   
		Eep_parameter[14]=0;	Eep_parameter[15] =0;   						//是否启动慢响应   
		Eep_parameter[16]=0;	Eep_parameter[17] =0;   						//是否具备不同容量混用功能     	      
		Eep_parameter[18]=0;	Eep_parameter[19] =0;							//是否启动备机功能	   
		Eep_parameter[20]=0;	Eep_parameter[21] =0;							//主电流互感器的安装位置			   
		Eep_parameter[22]=0;	Eep_parameter[23] =200;    					//主电流互感器的一次额定电流	   
		Eep_parameter[24]=0;	Eep_parameter[25] =1; 							//主电流互感器的二次额定电流		  
		Eep_parameter[26]=0;	Eep_parameter[27] =0;							//主电流互感器的安装数量	   
		Eep_parameter[28]=0;	Eep_parameter[29] =16;    						//LCL静态电流	   
		Eep_parameter[30]=0;	Eep_parameter[31] =0;   						//输出电流互感器的安装位置
		Eep_parameter[32]=0;	Eep_parameter[33] =0;    						//模拟输出变化速率	   
		Eep_parameter[34]=0;	Eep_parameter[35] =0xF0;     					//智能电容通讯波特率 2400
		Eep_parameter[36]=0;	Eep_parameter[37] =200;     					//输出电流互感器的一次额定电流    
		Eep_parameter[38]=0;	Eep_parameter[39] =1;							//输出电流互感器的二次额定电流   
		Eep_parameter[40]=0;	Eep_parameter[41] =0;							//优先级工作模式
		Eep_parameter[42]=0x02;	Eep_parameter[43] =0x58; 						//总输出电流峰值限制
		Eep_parameter[44]=0x02;	Eep_parameter[45] =0x58; 						//基波无功峰值电流限制
		Eep_parameter[46]=0;	Eep_parameter[47] =200; 						//总谐波峰值电流限制
		Eep_parameter[48]=0x02;	Eep_parameter[49] =0x58; 						//不平衡电流峰值限制
		Eep_parameter[50]=0x03;	Eep_parameter[51] =0xE8; 						//目标功率因素
		Eep_parameter[52]=0;	Eep_parameter[53] =20;							//设备开机允许电流
		Eep_parameter[54]=0;	Eep_parameter[55] =10;							//设备关机允许电流
		Eep_parameter[56]=0;	Eep_parameter[57] =10;							//开机允许电流延时
		Eep_parameter[58]=0;	Eep_parameter[59] =30;  						//关机允许电流延时
		Eep_parameter[60]=0;	Eep_parameter[61] =1;							//设备供选择了多少频段的待补偿电流
		Eep_parameter[62]=0;	Eep_parameter[63] =1; 							//选中的第一个补偿频段
		for(i=0;i<18;i++)
	  	{
	  		Eep_parameter[i*2+64]=0;Eep_parameter[i*2+65]=0;					//被选中的第2~19个频段
	  	}
	  	Eep_parameter[100]=0;	Eep_parameter[101]=0; 							//电压支撑模式
		Eep_parameter[102]=0;	Eep_parameter[103]=230;							//电压合格上限
		Eep_parameter[104]=0;	Eep_parameter[105]=210; 						//电压合格下限
		Eep_parameter[106]=0x03;Eep_parameter[107]=0x52;						//中线电流限制850
		Eep_parameter[108]=0;	Eep_parameter[109]=1;							//主电流互感器的综合系数1
		Eep_parameter[110]=0x0E;Eep_parameter[111]=0x4F;						//主电流互感器的综合系数2 3663
		Eep_parameter[112]=0;	Eep_parameter[113]=240; 						//电压启动上限
		Eep_parameter[114]=0;	Eep_parameter[115]=200; 						//电压启动下限         
		Eep_parameter[116]=0;	Eep_parameter[117]=1;  							//输出电流互感器的综合系数1          
		Eep_parameter[118]=0x0E;Eep_parameter[119]=0x4F; 						//输出电流互感器的综合系数2         
		Eep_parameter[120]=0x27;Eep_parameter[121]=0x10; 						//A相主电流互感器的幅值修正          
		Eep_parameter[122]=0x27;Eep_parameter[123]=0x10;						//B相主电流互感器的幅值修正          
		Eep_parameter[124]=0x27;Eep_parameter[125]=0x10; 						//C相主电流互感器的幅值修正        
		Eep_parameter[126]=0; Eep_parameter[127]=120;  						//运行状态下的故障重启时间间隔    
		Eep_parameter[128]=0;	Eep_parameter[129]=30;							//运行状态下的保护重启时间间隔         
		Eep_parameter[130]=0;	Eep_parameter[131]=1; 							//开机保护延迟      
		Eep_parameter[132]=0x27;Eep_parameter[133]=0x10; 						//A相输出电流互感器的幅值修正       
		Eep_parameter[134]=0x27;Eep_parameter[135]=0x10; 						//B相输出电流互感器的幅值修正        
		Eep_parameter[136]=0x27;Eep_parameter[137]=0x10; 						//C相输出电流互感器的幅值修正
		Eep_parameter[138]=0;	Eep_parameter[139]=0;   						//主电流互感器的相位修正
		Eep_parameter[140]=0;	Eep_parameter[141]=0;  							//无源电容类型
		Eep_parameter[142]=0;	Eep_parameter[143]=0;						  	//输出电流互感器的相位修正
		Eep_parameter[144]=0;	Eep_parameter[145]=0;							//是否启用无源补偿
		Eep_parameter[146]=0;	Eep_parameter[147]=0;							//开关机条件判断
		Eep_parameter[148]=0;	Eep_parameter[149]=0;							//变压器容量
		Eep_parameter[150]=0;Eep_parameter[151]=2;						//开机等待延时
		Eep_parameter[152]=0x13;Eep_parameter[153]=0x88;						//0状态等待通讯延时计数
		Eep_parameter[154]=0x06;Eep_parameter[155]=0x40;						//非0状态等待通讯延时计数
		Eep_parameter[156]=0;	Eep_parameter[157]=2;							//是否启用负载侧的修正
		Eep_parameter[158]=0x1F;Eep_parameter[159]=0x40;						//系统侧的补偿反馈系数
		Eep_parameter[160]=0x3;Eep_parameter[161]=0xE8;						//负载侧的补偿反馈系数
		Eep_parameter[162]=0x27;Eep_parameter[163]=0x10;						//不平衡系统侧的补偿反馈系数 10000
		Eep_parameter[164]=0x1;Eep_parameter[165]=0x2C;						//不平衡负载侧的补偿反馈系数  300
		Eep_parameter[166]=0;	Eep_parameter[167]=0;							//目标电流的相位修正1
		Eep_parameter[168]=0;	Eep_parameter[169]=0;							//目标电流的相位修正2	
		Eep_parameter[170]=0;Eep_parameter[171]=10;						//断电重启允许时间间隔
		Eep_parameter[172]=0;	Eep_parameter[173]=0;							//测试模式
		Eep_parameter[174]=0;Eep_parameter[175]=10;						//保护或故障重启的等待时间
		Eep_parameter[176]=0;	Eep_parameter[177]=1;							//备用
		Eep_parameter[178]=0;	Eep_parameter[179]=14;							//备用
		Eep_parameter[180]=0;	Eep_parameter[181]=0;							//升压变压器变比
		Eep_parameter[182]=0;	Eep_parameter[183]=0;							//升压变相位差
		Eep_parameter[184]=0x01;	Eep_parameter[185]=0xf4;							//慢跟随响应速度
		Eep_parameter[186]=0;	Eep_parameter[187]=10;							//备机电流维持时间
		Eep_parameter[188]=0;	Eep_parameter[189]=10;							//备机电流阈值1
		Eep_parameter[190]=0;	Eep_parameter[191]=20;							//备机电流阈值2
		Eep_parameter[192]=0;	Eep_parameter[193]=30;							//备机电流阈值3
		Eep_parameter[194]=0;	Eep_parameter[195]=40;							//备机电流阈值4
		Eep_parameter[196]=3;	Eep_parameter[197]=0xE8;							// 10单元的使能
		Eep_parameter[198]=0;	Eep_parameter[199]=1;							//从机数量
		
		//无功控制
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
		Eep_parameter[300]= 0;	Eep_parameter[301]= 5; //10450#无源参数
		Eep_parameter[302]= 0;	Eep_parameter[303]= 1;
		Eep_parameter[304]= 0;	Eep_parameter[305]= 2;	
		
		for(i=0;i<94;i++)
		{
			Eep_parameter[306+i]= 0;
		}
		
		Eep_parameter[306]= 0;	Eep_parameter[307]= 5;	
		Eep_parameter[310]= 0;	Eep_parameter[311]= 10;		//（有功）10455#
		Eep_parameter[312]= 0;	Eep_parameter[313]= 10;		//（WU功）10456#
		Eep_parameter[314]= 0;	Eep_parameter[315]= 0;		//2
		Eep_parameter[316]= 0;	Eep_parameter[317]= 10;		//3
		Eep_parameter[318]= 0;	Eep_parameter[319]= 0;		//4
		Eep_parameter[320]= 0;	Eep_parameter[321]= 10;		//5
		Eep_parameter[322]= 0;	Eep_parameter[323]= 0;		//6
		Eep_parameter[324]= 0;	Eep_parameter[325]= 10;		//7
		Eep_parameter[326]= 0;	Eep_parameter[327]= 0;		//（8
		Eep_parameter[328]= 0;	Eep_parameter[329]= 10;		//（9
		Eep_parameter[330]= 0;	Eep_parameter[331]= 0;		//10
		Eep_parameter[332]= 0;	Eep_parameter[333]= 10;		//11
		Eep_parameter[334]= 0;	Eep_parameter[335]= 0;		//（12
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
	
/****************************************************初始化修正出厂参数***************************************************/
		for(i=0; i<50; i++)	
		{
		 	Eep_parameter[2*i+400] =0x27;Eep_parameter[2*i+401] =0x10;			//负载侧A相基波修正参数
			Eep_parameter[2*i+500] =0;Eep_parameter[2*i+501] =0;
			Eep_parameter[2*i+600] =0x27;Eep_parameter[2*i+601] =0x10;			//负载侧B相基波修正参数
			Eep_parameter[2*i+700] =0;Eep_parameter[2*i+701] =0;
			Eep_parameter[2*i+800] =0x27;Eep_parameter[2*i+801] =0x10;			//负载侧C相基波修正参数
			Eep_parameter[2*i+900] =0;Eep_parameter[2*i+901] =0;
		}
		
		Eep_parameter[500] =0x27;Eep_parameter[501] =0x10;//基波相角改为不平衡幅值修正
		Eep_parameter[700] =0x27;Eep_parameter[701] =0x10;
		Eep_parameter[900] =0x27;Eep_parameter[901] =0x10;
		//30K的默认修正参数
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
	
/****************************************************初始化ARM出厂参数***************************************************/			
		for(i=0;i<600;i++)
		{
			Eep_parameter[i+1000] =0;
		}
		Eep_parameter[1000]=0;	Eep_parameter[1001]=1;							//通讯地址	
		Eep_parameter[1002]=0;	Eep_parameter[1003]=3;							//通讯波特率
		Eep_parameter[1004]=0;	Eep_parameter[1005]=1;							//通讯优先级 0:本地优先  1:远程优先
		Eep_parameter[1006]=0;	Eep_parameter[1007]=1;							//上电默认开机状态 0:关机 1:开机
		Eep_parameter[1008]=0;	Eep_parameter[1009]=0;							//定时1	使能
		Eep_parameter[1010]=0;	Eep_parameter[1011]=0;							//定时1	时
		Eep_parameter[1012]=0;	Eep_parameter[1013]=0;							//定时1	分
		Eep_parameter[1014]=0;	Eep_parameter[1015]=0;							//定时1	时
		Eep_parameter[1016]=0;	Eep_parameter[1017]=0;							//定时1	分
		Eep_parameter[1018]=0;	Eep_parameter[1019]=0;							//定时2	使能
		Eep_parameter[1020]=0;	Eep_parameter[1021]=0;							//定时2	时
		Eep_parameter[1022]=0;	Eep_parameter[1023]=0;							//定时2	分
		Eep_parameter[1024]=0;	Eep_parameter[1025]=0;							//定时2	时
		Eep_parameter[1026]=0;	Eep_parameter[1027]=0;							//定时2	分
		Eep_parameter[1028]=0;	Eep_parameter[1029]=0;							//定时3	使能
		Eep_parameter[1030]=0;	Eep_parameter[1031]=0;							//定时3	时
		Eep_parameter[1032]=0;	Eep_parameter[1033]=0;							//定时3	分
		Eep_parameter[1034]=0;	Eep_parameter[1035]=0;							//定时3	时
		Eep_parameter[1036]=0;	Eep_parameter[1037]=0;							//定时3	分
		Eep_parameter[1038]=0;	Eep_parameter[1039]=0;							//定时4 	使能
		Eep_parameter[1040]=0;	Eep_parameter[1041]=0;							//定时4	时
		Eep_parameter[1042]=0;	Eep_parameter[1043]=0;							//定时4	分
		Eep_parameter[1044]=0;	Eep_parameter[1045]=0;							//定时4	时
		Eep_parameter[1046]=0;	Eep_parameter[1047]=0;							//定时4	分
		Eep_parameter[1048]=0;	Eep_parameter[1049]=1;							//有功不平衡
		Eep_parameter[1050]=0;	Eep_parameter[1051]=0;							//无功不平衡
		Eep_parameter[1052]=0;	Eep_parameter[1053]=0;							//谐波不平衡
		Eep_parameter[1054]=0;	Eep_parameter[1055]=0;							//视在不平衡
		Eep_parameter[1056]=0;	Eep_parameter[1057]=1;							//直控选择
		Eep_parameter[1058]=0x27;Eep_parameter[1059]=0x10;							//修正1
		Eep_parameter[1060]=0x27;Eep_parameter[1061]=0x10;							//修正2
		Eep_parameter[1062]=0x27;Eep_parameter[1063]=0x10;							//修正3
		Eep_parameter[1064]=0x27;Eep_parameter[1065]=0x10;							//修正4
		Eep_parameter[1066]=0x27;Eep_parameter[1067]=0x10;							//修正5
		Eep_parameter[1068]=0x27;Eep_parameter[1069]=0x10;							//修正6
		
		Eep_parameter[1188]=0x27;Eep_parameter[1189]=0x10;							//修正7
		Eep_parameter[1191]=1;
		Eep_parameter[1192]=0;Eep_parameter[1193]=0x1E;							//修正容量默认30
		Eep_parameter[1194]=0;//修正模式为0，手动
		Eep_parameter[1195]=100;//修正比例为100
		
		Eep_parameter[1196]=0x5A;Eep_parameter[1197]=0x3C;							//初始版本A
		Eep_parameter[1198]=0x27;Eep_parameter[1199]=0x10;							//修正8	电压畸变率
		
		Eep_parameter[1070]=0;	Eep_parameter[1071]=0;							//项目号上
		Eep_parameter[1072]=0;	Eep_parameter[1073]=0;							//项目号下
		Eep_parameter[1074]=0;	Eep_parameter[1075]=0;							//生产序号
		
		tmp32 =EepromVersion;													//参数版本
		Eep_parameter[ParameterInitNumMax-4]=(uint8)(tmp32>>24);
		Eep_parameter[ParameterInitNumMax-3]=(uint8)(tmp32>>16);				//Eeprom 出厂版本高位
		Eep_parameter[ParameterInitNumMax-2]=(uint8)(tmp32>>8);
		Eep_parameter[ParameterInitNumMax-1]=(uint8)(tmp32);					//Eeprom 出厂版本低位
		
		I2C_write;  
		WriteEeprom(EepParameterAddr,Eep_parameter,ParameterInitNumMax); 
		I2C_read;
		//I2C_write;                           									//写入I2C
		//WriteEeprom(EepParameterInitAddr,Eep_parameter,2);	//写出厂参数到I2C	
	  //I2C_read;

		delay(500);
		
		I2C_write;                           									//写入I2C
		WriteEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);	//写出厂参数到I2C	
	  	I2C_read;
		//ReadEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);		//读取24C32内的出厂参数
	}	
/****************************************************初始化当前参数***************************************************/ 	
 	ReadEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);        		//读取24C32内的当前设备参数

	if(Init_parameter[ParameterNumMax-4]!=0xA5
	&&Init_parameter[ParameterNumMax-3]!=0xA5
	&&Init_parameter[ParameterNumMax-2]!=0xA5
	&&Init_parameter[ParameterNumMax-1]!=0xA5)	//判断是否写过
 	{
	  	for(i=0;i<1600;i++)
		{
			Init_parameter[i] =Eep_parameter[i];								//恢复出厂	
		}
		
		Init_parameter[1568]=0;	Init_parameter[1569]=0;							//累计节能量 HH
		Init_parameter[1570]=0;	Init_parameter[1571]=0;							//累计节能量 H
		Init_parameter[1572]=0;	Init_parameter[1573]=0;							//累计节能量 L
		Init_parameter[1574]=0;	Init_parameter[1575]=0;							//累计节能量 LL
		Init_parameter[1576]=0;	Init_parameter[1577]=0;							//累计运行时间H
		Init_parameter[1578]=0;	Init_parameter[1579]=0;							//累计运行时间L
		Init_parameter[1580]=0;	Init_parameter[1581]=0;							//今日节能量 H
		Init_parameter[1582]=0;	Init_parameter[1583]=0;							//今日节能量 L
		Init_parameter[1584]=0;	Init_parameter[1585]=0;							//当天运行时间H
		Init_parameter[1586]=0;	Init_parameter[1587]=0;							//记录年
		Init_parameter[1588]=0;	Init_parameter[1589]=0;							//记录月
		Init_parameter[1590]=0;	Init_parameter[1291]=0;							//记录日
		Init_parameter[1592]=0;	Init_parameter[1593]=0;							//预留
		Init_parameter[1594]=0;	Init_parameter[1595]=0;							//预留

		Init_parameter[ParameterNumMax-4] =0xA5;
		Init_parameter[ParameterNumMax-3] =0xA5;
		Init_parameter[ParameterNumMax-2] =0xA5;
		Init_parameter[ParameterNumMax-1] =0xA5;
		I2C_write;
		WriteEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);   					//将出厂参数写到正式参数中
		I2C_read;																			//读取I2C

		delay(500);

		I2C_write;
		WriteEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);   					//将出厂参数写到正式参数中
		I2C_read;
		
 	}
/*********************************************************************************************************************/	
	for(i=0;i<100;i++)
	{
		Passive_parameter[i] =(Init_parameter[2*i+200]<<8)|Init_parameter[2*i+201];						//初始化无源参数组
	}
	for(i=0;i<100;i++)
	{	
		main_parameter[i]=(Init_parameter[2*i]<<8)|Init_parameter[2*i+1];								//初始化主参数
	}
	if(main_parameter[72]==2)																			//手动投切
	{
		ManualPassiveSwitchFlag=1;
	}
	else 
	{
		ManualPassiveSwitchFlag =0;
	}
	/*for(i=0;i<10;i++)
	{
		slave_Enable[i]= main_parameter[i+89];															//从机使能
	}*/
	// for(i=0;i<50;i++)
	// {
	// 	Selected_parameter[i] =0;																		//选次参数
	// }
	// for(i=0;i<19;i++)																	//将选择要补偿的频段打开
	// {
	// 	j=main_parameter[31+i];
	// 	if(j>0)Selected_parameter[j-1]=1;
	// }
	for(i=0; i<100; i++)	
	{
		load_correctionA[i] =	(Init_parameter[2*i+400]<<8)|Init_parameter[2*i+401];
		load_correctionB[i] =	(Init_parameter[2*i+600]<<8)|Init_parameter[2*i+601];
		load_correctionC[i] =	(Init_parameter[2*i+800]<<8)|Init_parameter[2*i+801];							//初始化修正参数

	}
	
	LocalAddr =((Init_parameter[1000]<<8)|Init_parameter[1001]);													//ARM 参数组
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

	// AlarmTime1[0]=(Init_parameter[1008]<<8|Init_parameter[1009]);//定时1	使能
	// AlarmTime1[1]=(Init_parameter[1010]<<8|Init_parameter[1011]);//定时1	时
	// AlarmTime1[2]=(Init_parameter[1012]<<8|Init_parameter[1013]);//定时1	分
	// AlarmTime1[3]=(Init_parameter[1014]<<8|Init_parameter[1015]);//定时1	使能
	// AlarmTime1[4]=(Init_parameter[1016]<<8|Init_parameter[1017]);//定时1	时
	
	// AlarmTime2[0]=(Init_parameter[1018]<<8|Init_parameter[1019]);//定时2	使能
	// AlarmTime2[1]=(Init_parameter[1020]<<8|Init_parameter[1021]);//定时2	时
	// AlarmTime2[2]=(Init_parameter[1022]<<8|Init_parameter[1023]);//定时2	分
	// AlarmTime2[3]=(Init_parameter[1024]<<8|Init_parameter[1025]);//定时2	时					
	// AlarmTime2[4]=(Init_parameter[1026]<<8|Init_parameter[1027]);//定时2	分
	
	// AlarmTime3[0]=(Init_parameter[1028]<<8|Init_parameter[1029]);//定时3	时
	// AlarmTime3[1]=(Init_parameter[1030]<<8|Init_parameter[1031]);//定时3	分	
	// AlarmTime3[2]=(Init_parameter[1032]<<8|Init_parameter[1033]);//定时3 使能
	// AlarmTime3[3]=(Init_parameter[1034]<<8|Init_parameter[1035]);//定时3	时
	// AlarmTime3[4]=(Init_parameter[1036]<<8|Init_parameter[1037]);//定时3	分	

	ntc_type=Init_parameter[1038];
	//不使用
//	AlarmTime4[0]=(Init_parameter[1038]<<8|Init_parameter[1039]);//定时4 使能
//	AlarmTime4[1]=(Init_parameter[1040]<<8|Init_parameter[1041]);//定时4	时
//	AlarmTime4[2]=(Init_parameter[1042]<<8|Init_parameter[1043]);//定时4	分
//	AlarmTime4[3]=(Init_parameter[1044]<<8|Init_parameter[1045]);//定时4	时
//	AlarmTime4[4]=(Init_parameter[1046]<<8|Init_parameter[1047]);//定时4	分
	
	// ActiveBal =(Init_parameter[1048]<<8|Init_parameter[1049]);//有功不平衡
	// ReactiveBal=(Init_parameter[1050]<<8|Init_parameter[1051]);//无功不平衡
	// HarmonicBal=(Init_parameter[1052]<<8|Init_parameter[1053]);//谐波不平衡
	// ApparentBal=(Init_parameter[1054]<<8|Init_parameter[1055]);//视在不平衡
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
	
	//智能投切
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
	
//ARM参数初始化	

	for(i=0;i<100;i++) tmp[i]=0;//清零

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
	// tmp[43] =0;						//电压开机使能//058F
	// CurOnOffEnable =main_parameter[73]/10;
	// tmp[44] =CurOnOffEnable;		//电流开机使能//0590

	// MainCTPhase =main_parameter[10]/10000;tmp16 =main_parameter[10]%10000;
	// MainCTDirectionC=tmp16/1000;tmp16 =main_parameter[10]%1000;
	// MainCTDirectionB=tmp16/100;tmp16 =main_parameter[10]%100;
	// MainCTDirectionA=tmp16/10;tmp16 =main_parameter[10]%10;
	// MainCTLocation  =tmp16;
	
	// tmp[45] =MainCTLocation;		//主互感器位置//0591
	// tmp[46] =MainCTDirectionA;		//主互感器方向A//0592
	// tmp[47] =MainCTDirectionB;		//主互感器方向B//0593
	// tmp[48] =MainCTDirectionC;		//主互感器方向C//0594
	// tmp[49] =MainCTPhase; 			//主互感器相序//0595
	
	// OutCTPhase =main_parameter[15]/10000;tmp16 =main_parameter[15]%10000;
	// OutCTDirectionC=tmp16/1000;tmp16 =main_parameter[15]%1000;
	// OutCTDirectionB=tmp16/100;tmp16 =main_parameter[15]%100;
	// OutCTDirectionA=tmp16/10;tmp16 =main_parameter[15]%10;
	
	// tmp[50] =OutCTPhase; 			//辅助互感器相序//0596
	
	// tmp[51] =Position[0];			//仓位2	
	// tmp[52] =Group[0];				//组2
	// tmp[53] =Capacitance[0];		//容值2

	// tmp[54] =Position[1];			//仓位3	
	// tmp[55] =Group[1];				//组3
	// tmp[56] =Capacitance[1];		//容值3

	// tmp[57] =Position[2];			//仓位4	
	// tmp[58] =Group[2];				//组4
	// tmp[59] =Capacitance[2];		//容值4

	// tmp[60] =Position[3];			//仓位5	
	// tmp[61] =Group[3];				//组5
	// tmp[62] =Capacitance[3];		//容值5

	// tmp[63] =Position[4];			//仓位6	
	// tmp[64] =Group[4];				//组6
	// tmp[65] =Capacitance[4];		//容值6

	// tmp[66] =Position[5];			//仓位7	
	// tmp[67] =Group[5];				//组7
	// tmp[68] =Capacitance[5];		//容值7

	// tmp[69] =Position[6];			//仓位8	
	// tmp[70] =Group[6];				//组8
	// tmp[71] =Capacitance[6];		//容值8

	// tmp[72] =Position[7];			//仓位9	
	// tmp[73] =Group[7];				//组9
	// tmp[74] =Capacitance[7];		//容值9

	// tmp[75] =Position[8];			//仓位10	
	// tmp[76] =Group[8];				//组10
	// tmp[77] =Capacitance[8];		//容值10

	// tmp[78] =OutCTDirectionA;		//输出互感器方向A
	// tmp[79] =OutCTDirectionB;		//输出互感器方向B
	// tmp[80] =OutCTDirectionC;		//输出互感器方向C

	// tmp[81] =Position[9];			//仓位11	
	// tmp[82] =Group[9];				//组11
	// tmp[83] =Capacitance[9];		//容值11

	// tmp[84] =Position[10];			//仓位12	
	// tmp[85] =Group[10];				//组12
	// tmp[86] =Capacitance[10];		//容值12

	// tmp[87] =Position[11];			//仓位13	
	// tmp[88] =Group[11];				//组13
	// tmp[89] =Capacitance[11];		//容值13

	// tmp[90] =Position[12];			//仓位14	
	// tmp[91] =Group[12];				//组14
	// tmp[92] =Capacitance[12];		//容值14

	// tmp[93] =Position[13];			//仓位15	
	// tmp[94] =Group[13];				//组15
	// tmp[95] =Capacitance[13];		//容值15

	// tmp[96] =Position[14];			//仓位16
	// tmp[97] =Group[14];				//组16
	// tmp[98] =Capacitance[14];		//组16
	// tmp[99] =0;						//预留
	
	for(i=0; i<100; i++)	
	{
		ARM_param[i] =	tmp[i];//ARM参数组
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
** 函数名: GZ_init
**
** 功能：读取故障和谐波自行修正参数
******************************************************************************/
void GZ_init(void)
{
	uint8 tmp[1664];
	uint16 i=0;
 	ReadEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);                        //读取故障记录
 	
 	if(gz_dan[159]!=0)                                                   		//第一次使用，记录清0
 	{	  															   
  		for(i=0; i<FaultDataNumMax; i++) {gz_dan[i] = 0;}						//先清零
  		I2C_write;                                                           	//写入I2C	   				  
   		WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);		    		//保存记录到I2C		    
  		I2C_read; 		    
 	}

	ReadEeprom(EepSlaveFaultAddr,tmp,SlaveFaultNumMax);                        //读取故障记录

	if(tmp[1663] !=0)
	{
		for(i=0; i<SlaveFaultNumMax; i++) {tmp[i] = 0;}						//先清零
  		I2C_write;                                                           	//写入I2C	   				  
   		WriteEeprom(EepSlaveFaultAddr,tmp,SlaveFaultNumMax);		    		//保存记录到I2C		    
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
		DR_LOG[i]=tmp[i+520];//获取电容记录
	}

	ReadEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);                        //读取故障记录

	if(Event_Logging[159] !=0)
	{
		for(i=0; i<EventLogNumMax; i++) {Event_Logging[i] = 0;}						//先清零
  		I2C_write;                                                           	//写入I2C	   				  
   		WriteEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);		    		//保存记录到I2C		    
  		I2C_read; 
	}
	
	ReadEeprom(EepDatalogMaxAddr,tmp,2);
	if(tmp[0]>13||tmp[1]>13)	
	{
		DatalogMax=0;
		DatalogStart=0;
		tmp[0]=(uint8)DatalogMax;
		tmp[1]=(uint8)DatalogStart;
		
		I2C_write;                                                           	//写入I2C	   				  
   		WriteEeprom(EepDatalogMaxAddr,tmp,2);		    		//保存记录到I2C		    
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
** 函数名:GPIO_init 
**
** 功能:管脚初始化
******************************************************************************/
void GPIO_init(void)
{
 	/********管脚初始化*************
 	P2.8用于温度开关
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
	LPC_PINCON->PINMODE_OD0 = 0xC00;		//I2C2改为开漏
	//LPC_PINCON->PINMODE3 = 0xf300;		//由上拉电阻改为下拉电阻	
	LPC_GPIO0->FIODIR = 0x62E00200; 	//0110 0010 1110 0000 1000 0010 0000 0000 
	LPC_GPIO1->FIODIR = 0x30000013;		//0011 0000 0000 0000 0000 0000 0001 0011
	LPC_GPIO2->FIODIR = 0x00003804; 	//0000 0000 0000 0000 0011 1000 0000 0100
	LPC_GPIO3->FIODIR = 0x06000000; 	//0000 0110 0000 0000 0000 0000 0000 0000
	LPC_GPIO4->FIODIR = 0;
	LPC_GPIO1->FIOSET = 1<<28;			//awp
	//LPC_GPIO0->FIOSET = 1<<15;		//awp
	LPC_GPIO0->FIOSET = 1<<22;			// ARM的开关机使能管脚：
										//置0：表示允许DSP开机，允许进入状态5
										//置1：表示不允许DSP开机，DSP进入状态3
	

	LPC_GPIO0->FIOSET = 1<<13;			// ARM的大开关机使能管脚：
										//置0：表示允许DSP开机，允许进入状态3
										//置1：表示不允许DSP开机，DSP进入状态2
	
	LPC_GPIO2->FIOSET = 1<<11;			// ARM的SPI通讯使能管脚
										//置0: 允许SPI通讯
										//置1：不允许SPI通讯
										
	LPC_GPIO2->FIOSET = 1<<12;			//ARM的SPI写操作使能
										//置1：表示SPI写不使能
										//置0: 表示SPI写使能
	LPC_GPIO2->FIOSET = 1<<13;			//上电 开机管脚为高不开机
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
	//LPC_GPIO0->FIOSET = 1<<25;			//输出高
	LPC_GPIO0->FIOSET = 1<<9;			//输出高
	
	
	LPC_GPIO1->FIOSET = 1<<4;			//蓝牙Reset 上拉
	LPC_GPIO1->FIOCLR = 1<<0;			//蓝牙Key 下拉-AT模式
	
	
}
/******************************************************************************
** 函数名:Bluetooth_init 
**
** 功能:蓝牙模块初始化
******************************************************************************/
void Bluetooth_init(void)
{
	uint8 i,j,*SendPtr,SendData,ATcmd[26];
	uint64 temp64;
	LPC_GPIO1->FIOCLR = 1<<0;			//蓝牙Key 下拉-AT模式
/*****************************测试蓝牙硬件**********************************/	
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
/*****************************蓝牙设备名字修改**********************************/
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
					LPC_GPIO1->FIOSET = 1<<0;			//蓝牙Key 上拉-透传模式
					break;
				}
			}
		}
	}
		
	
}

/******************************************************************************
** 函数名:TIME_init 
**
** 功能:定时器初始化
******************************************************************************/
uint32_t TIME_init(uint8 time_num)
{
  	pclock=SystemFrequency/4; 		//内部RC时钟频率为4M Hz
 	if(time_num == 0)
	{
	 	LPC_TIM0->TC = 0;   	    //定时器0初始化
 		LPC_TIM0->PR = 0;   		//1分频
 		LPC_TIM0->MCR = 0x3;       	//定时器比较产生中断标志位，TC计数器清零
 		//LPC_TIM0->MR0 = pclock*0.5;	//主频是96Mhz Fpclk的频率是24Mhz  
 		LPC_TIM0->MR0 = pclock/1000-1; //  ((1000000/1000)-1+1)/24000000 =1ms
 		LPC_TIM0->IR = 0xff;	    // reset all interrrupts 
 		//LPC_TIM0->TCR = 0;         	//不使能定时器计数0
 		NVIC_EnableIRQ(TIMER0_IRQn);
 		LPC_TIM0->TCR =1 ;		    //使能定时器计数0
	  return (TRUE);
	}
  else if(time_num == 1)
	{
	 	LPC_TIM1->TC = 0;   	    //定时器1初始化
 		LPC_TIM1->PR = 0;   		//1分频
 		LPC_TIM1->MCR = 0x3;       	//定时器比较产生中断标志位，TC计数器清零
 		LPC_TIM1->MR0 = pclock*2.5;//*0.5;	//主频是96Mhz Fpclk的频率是24Mhz  
 		LPC_TIM1->IR = 0xff;	    // reset all interrrupts
		NVIC_EnableIRQ(TIMER1_IRQn); 
 		LPC_TIM1->TCR = 0;         	//不使能定时器计数1
	  return (TRUE);
	}
	else if(time_num == 2)
	{ 
		LPC_TIM2->TC = 0;   	    //定时器0初始化
 		LPC_TIM2->PR = 0;   		//1分频
 		LPC_TIM2->MCR = 0x3;       	//定时器比较产生中断标志位，TC计数器清零
 		LPC_TIM2->MR0 = pclock*1.0;	//主频是96Mhz Fpclk的频率是24Mhz  
 		LPC_TIM2->IR = 0xff;	    // reset all interrrupts 
		NVIC_EnableIRQ(TIMER2_IRQn); 
 		LPC_TIM2->TCR = 0;         	//不使能定时器计数0
	  return (TRUE);
	}
	else if(time_num == 3)
	{
	 	LPC_TIM3->TC = 0;   	    //定时器0初始化
 		LPC_TIM3->PR = 0;   		//1分频
 		LPC_TIM3->MCR = 0x3;       	//定时器比较产生中断标志位，TC计数器清零
 		LPC_TIM3->MR0 = pclock*1.0;	//主频是96Mhz Fpclk的频率是24Mhz  
 		LPC_TIM3->IR = 0xff;	    // reset all interrrupts 
		NVIC_EnableIRQ(TIMER3_IRQn); 
 		LPC_TIM3->TCR = 0;         	//不使能定时器计数0
	  return (TRUE);	
	}
 	return (FALSE);
}
/******************************************************************************
** 函数名:TIMER0_IRQHandler 
**
** 功能:定时器
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
		if(Testtime_function==1)Testtime_time1++;							//如果功能开启就递加
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
					if(xiuzhen_mode==2)//APF修正
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
					else if(xiuzhen_mode==1)//svg修正
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
					else//手动修正
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
				if(xiuzhen_time>40)//强制的手动修正
				{
					if(xiuzhen_mode==2)//APF修正
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
					else if(xiuzhen_mode==1)//svg修正
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
					else//手动修正
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
** 函数名:error_handle 
**
** 功能:故障处理
******************************************************************************/
void error_handle(void)
{
	uint16 i,j,FaultNum,FaultCode;
	uint8 Tmp_Fault[104];

	if(SystemStatus !=SystemProtect && SystemStatus !=SystemPfault && SystemStatus !=SystemParameterError)	//故障是否存在
	{
		SystemStatusBak =SystemStatus;	//状态备份 
		//led3_OFF;
	}
	else
	{
		//led3_ON;
		if(SystemStatus==SystemParameterError)	//参数错误
		{
			SystemStatusBak =SystemStatus;		//状态备份 
		}
		if(SystemStatusBak!=SystemProtect &&SystemStatusBak!=SystemPfault&&SystemStatusBak!=SystemParameterError)//判断上次是否为故障等状态
		{
			/*if(SystemStatus==SystemProtect)
			{
				if(ResetCount>0)
				{
					ResetCount--;
					temp[0] =(uint8)(ResetCount>>8);
					temp[1] =(uint8)(ResetCount);
					I2C_write;													//写入I2C
					WriteEeprom(EepParameterAddr+1048,temp,2);   			//将出厂参数写到正式参数中
					I2C_read;
					DUGSReadMainParameterFlag =1;
				}
			}*/
			SystemStatusBak =SystemStatus;
			if(MainErrorCode !=0)	
			{
				FaultNum =gz_dan[0];
				FaultCode =MainErrorCode;
				if(FaultNum==20)		   									//主设备最大记录140条故障
				{ 
					for(i=1; i<141; i++)
					{
						if(i%7!=0) gz_dan[i] = gz_dan[7+i];					//往前覆盖	
					} 	   
					gz_dan[140] = 1;	                                  	//序号
					gz_dan[139] = alarm_time.RTC_Year - 2000;				//年
					gz_dan[138] = alarm_time.RTC_Mon;	                    //月
					gz_dan[137] = alarm_time.RTC_Mday;	                //日
					gz_dan[136] = alarm_time.RTC_Hour;                   	//时
					gz_dan[135] = alarm_time.RTC_Min;                     //分
					gz_dan[134] = FaultCode;                             	//故障代码
										
				}
				else
				{
					gz_dan[0]++;
					FaultNum =gz_dan[0];  
					gz_dan[7*FaultNum] = FaultNum;	                    	//序号
					gz_dan[7*FaultNum-1] = alarm_time.RTC_Year - 2000;   	//年
					gz_dan[7*FaultNum-2] = alarm_time.RTC_Mon;	        //月
					gz_dan[7*FaultNum-3] = alarm_time.RTC_Mday;	        //日
					gz_dan[7*FaultNum-4] = alarm_time.RTC_Hour;          	//时
					gz_dan[7*FaultNum-5] = alarm_time.RTC_Min;           	//分
					gz_dan[7*FaultNum-6] = FaultCode;                     	//故障代码
					for(i=0; i<FaultNum; i++)
					{
						gz_dan[(i+1)*7] = FaultNum-i;						//序号调整		
					}
				}
				I2C_write;                                                  //写入I2C	   
				WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);       //保存记录到I2C						    
				I2C_read;
			}
		}
	}
	
	for(j=0;j<main_parameter[99];j++)
	{
		//if(slave_Enable[j]!=22) continue;
		
		if( SlaveFaultFlag[j]!=SystemProtect && SlaveFaultFlag[j] !=SystemPfault && SlaveFaultFlag[j] !=SystemParameterError)//判断是否存在故障
		{
			SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//没有故障就记录当前状态备份
		}
		else//存在故障
		{
			if(SlaveFaultFlag[j]==SystemParameterError)//判断是否是参数错误故障
			{
				SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//备份参数错误故障
			}
			if(SlaveFaultFlagBak[j]!=SystemProtect &&SlaveFaultFlagBak[j]!=SystemPfault&&SlaveFaultFlagBak[j]!=SystemParameterError)//如果上次备份的状态不是故障状态
			{
				/*if(SlaveFaultFlag[j]==SystemPfault)
				{
					if(slave_Reset[j]>0)
					{
						slave_Reset[j]--;
						temp[0] =(uint8)(slave_Reset[j]>>8);
						temp[1] =(uint8)(slave_Reset[j]);
		
						I2C_write;													//写入I2C
						WriteEeprom(EepParameterAddr+1050+2*j,temp,2);   			//将出厂参数写到正式参数中
						I2C_read;

						DUGSReadMainParameterFlag =1;
					}
				}*/
				SlaveFaultFlagBak[j] =SlaveFaultFlag[j];//备份当前状态
				if(SlaveErrorCode[j]	>	4)		//如果当前备份的状态大于4
				{
					if(SlaveErrorCode[j]	!=	0)		//从机故障代码budengyu 0才记录
					{
						FaultNum =SlaveFault[j][0];//获取上次故障记录次数
						FaultCode =SlaveErrorCode[j];//获取当前故障代码
						
						if(FaultNum==14)												//从机最大记录14条故障
						{
							for(i=1; i<99; i++)
							{
								if(i%7!=0) SlaveFault[j][i] = SlaveFault[j][7+i];		//往前覆盖	
							} 	   
							SlaveFault[j][98] = 1;	                                  	//序号
							SlaveFault[j][97] = alarm_time.RTC_Year - 2000;			//年
							SlaveFault[j][96] = alarm_time.RTC_Mon;	                //月
							SlaveFault[j][95] = alarm_time.RTC_Mday;	                //日
							SlaveFault[j][94] = alarm_time.RTC_Hour;                 //时
							SlaveFault[j][93] = alarm_time.RTC_Min;                  //分
							SlaveFault[j][92] = FaultCode;                            //故障代码
													
						}
						else
						{
							SlaveFault[j][0]++;  
							FaultNum =SlaveFault[j][0];
							SlaveFault[j][7*FaultNum] = FaultNum;	                    	//序号
							SlaveFault[j][7*FaultNum-1] = alarm_time.RTC_Year - 2000; 	//年
							SlaveFault[j][7*FaultNum-2] = alarm_time.RTC_Mon;	        //月
							SlaveFault[j][7*FaultNum-3] = alarm_time.RTC_Mday;	        //日
							SlaveFault[j][7*FaultNum-4] = alarm_time.RTC_Hour;          	//时
							SlaveFault[j][7*FaultNum-5] = alarm_time.RTC_Min;           	//分
							SlaveFault[j][7*FaultNum-6] = FaultCode;                     	//故障代码
							for(i=0; i<SlaveFault[j][0]; i++)
							{
								SlaveFault[j][(i+1)*7] = SlaveFault[j][0]-i;//前面故障记录序号减1
							}
						}
						for(i=0;i<100;i++)
						{
							Tmp_Fault[i]=SlaveFault[j][i];
						}
						I2C_write;                                                   		//写入I2C	   
						WriteEeprom(EepSlaveFaultAddr+j*SingleFaultNum,Tmp_Fault,SingleFaultNum);    	//保存记录到I2C						    
						I2C_read;

						//I2C_write;                                                   		//写入I2C	   
						ReadEeprom(EepSlaveFaultAddr+j*SingleFaultNum,Tmp_Fault,SingleFaultNum);    	//保存记录到I2C		

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
** 函数名:Event_handle 
**
** 功能:故障处理
******************************************************************************/
void Event_handle(void)
{
	uint16 i,FaultNum,FaultCode;
	//uint8 /*Tmp_Fault[104],*/temp[2];
	uint8 fault_flag;

	//判断电容的故障地址
	fault_flag=0;
	if(capa[3]>0)//存在故障
	{
		if(capa_bak!=capa[3])//这次故障与上次故障不一样
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
			if(FaultNum==20)		   									//主设备最大记录140条故障
			{ 
				for(i=1; i<141; i++)
				{
					if(i%7!=0) Event_Logging[i] = Event_Logging[7+i];					//往前覆盖	
				}
				Event_Logging[140] = 1;	                                  	//序号
				Event_Logging[139] = Event_time.RTC_Year - 2000;				//年
				Event_Logging[138] = Event_time.RTC_Mon;	                    //月
				Event_Logging[137] = Event_time.RTC_Mday;	                //日
				Event_Logging[136] = Event_time.RTC_Hour;                   	//时
				Event_Logging[135] = Event_time.RTC_Min;                     //分
				Event_Logging[134] = FaultCode;                             	//故障代码
										
			}
			else
			{
				Event_Logging[0]++;
				FaultNum =Event_Logging[0];  
				Event_Logging[7*FaultNum] = FaultNum;	                    	//序号
				Event_Logging[7*FaultNum-1] = Event_time.RTC_Year - 2000;   	//年
				Event_Logging[7*FaultNum-2] = Event_time.RTC_Mon;	        //月
				Event_Logging[7*FaultNum-3] = Event_time.RTC_Mday;	        //日
				Event_Logging[7*FaultNum-4] = Event_time.RTC_Hour;          	//时
				Event_Logging[7*FaultNum-5] = Event_time.RTC_Min;           	//分
				Event_Logging[7*FaultNum-6] = FaultCode;                     	//故障代码
				for(i=0; i<FaultNum; i++)
				{
					Event_Logging[(i+1)*7] = FaultNum-i;						//序号调整		
				}
			}
			I2C_write;                                                  //写入I2C	   
			WriteEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);       //保存记录到I2C						    
			I2C_read;
 
			ReadEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);       //保存记录到I2C						    
		}
	}
}
/******************************************************************************
** 函数名:temp_judge 
**
** 功能:温度处理
******************************************************************************/

/*void temp_judge(void)
{
	static uint8 Count =0;
	
	if(ADCFinishProcessFlag==1)
	{
		if(ModuleTemp>((Parameter[924]<<8)|Parameter[925]) )//模块温度超上限
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
		if( Internaltemp>((Parameter[926]<<8)|Parameter[927]) )//钣金温度超上限
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
		if( DeratingJudgeFlag==1 )//到达判断温度时间
		{
			DeratingJudgeFlag=0;
			if( canshuzhong2[0]<30 )//电流很小，温度依旧下不，报错
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
			
			if( ModuleTemp>((Parameter[916]<<8)|Parameter[917]) || Internaltemp>((Parameter[920]<<8)|Parameter[921]) )//降容
			{
				Count++;
				if(((Parameter[930]<<8)|Parameter[931])<=1000&&Count >20)
				{
					Count =0;
					canshuzhong2[0] =(Parameter[930]<<8)|Parameter[931];							//降容至 目标值

					ParametersConfigflag =1;														//告知DSP 参数已做更改
					SystemDerating =1;
				}
			}
		 	if( ModuleTemp<=((Parameter[918]<<8)|Parameter[919]) && Internaltemp<=((Parameter[922]<<8)|Parameter[923]) )//恢复
			{
				if((ModuleTemp==(Parameter[918]<<8)|Parameter[919])||Internaltemp<=((Parameter[922]<<8)|Parameter[923]))
				{
					SystemDerating=0;
				}
				if(((Parameter[932]<<8)|Parameter[933])<=100&&SystemDerating ==1)
				{
					if( canshuzhong2[0]<1000 )
					{
						canshuzhong2[0]=canshuzhong2[0]+ ((Parameter[932]<<8)|Parameter[933]);		//升容时 步长
					}
					else
					{
						canshuzhong2[0]=1000;
					}
					ParametersConfigflag =1;														// 告知DSP 参数已做更改
				}
			}
		}
	}
}*/
/******************************************************************************
** 函数名:FanProcess 
**
** 功能:温度处理
******************************************************************************/
/*
void FanProcess(void)
{
	if(SystemClose==1)				//系统关机
	{
		if(SystemCloseBak ==0)
		{
			OnOffDelayCount=0;
			SystemCloseBak =1;
		}
		if(OnOffDelayCount>29)		//关机后30秒 关闭风扇
		{
//			aout3_L;
		}
	}
	if(SystemFailure==1)			//故障后 超过重启时间关闭风扇，并重启
	{
		if(SystemFailureBak ==0)
		{
			FaultDelayCount =0;
			SystemFailureBak =1;
		}
	 	FaultDelayCount++;
		if( FaultDelayCount > (((Parameter[888]<<8)|Parameter[889])*((Parameter[894]<<8)|Parameter[895])) )		//故障频率 * 重启时间
		{
//			aout3_L;
//			aout4_H;
		}
	}
}
*/
/******************************************************************************
** 函数名:ADCProcess 
**
** 功能:温度采样值计算
******************************************************************************/
/*void ADCProcess(uint8 Aisle,uint16 DelayTime)
{
	uint16 i,j,temp,num;
	static uint32 Sum =0;
	static uint16 TempSampleTime=0;


	if(GetCurrentTime(t10ms,TempSampleTime)>DelayTime)		//  1000ms执行一次采样温度平均值计算
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
			Avg[Aisle]=(Sum+(GetADMax-40)/2)/(GetADMax-40);				//  四舍五入
			Sum =0;
		}
		ADCFinishProcessFlag =1;										// 采样处理完成
		TempSampleTime= GetSystemTime(t10ms);							//重新计时
	}
}*/
/******************************************************************************
** 函数名:ADCProcess 
**
** 功能:温度采样值计算
******************************************************************************/
/*void ParametersConfig(void)
{
	uint16 DelayTime=0,*pointer16;
	if(ParametersConfigflag==1)
	{
		pointer16 = canshuzhong2;
		canshuzhong2[20] = crc_chk_value(pointer16,20);	//校验 
		LPC_SSP0->IMSC=0x04;
		
//		ARMOUT1_L;	 //使能连接DSP的参数管脚 
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
			}  //等待5秒
		}
		ARMOUT1_H;
	
		ParametersConfigflag=0;
	}
}*/
/******************************************************************************
** 函数名:Power_current_judge 
**
** 功能:开机电流判断
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
		if( SystemCurrentReachedFlag==0 && dsp_data[37]>=((Parameter[28]<<8)|Parameter[29])*1.414 && 	//开机电流判断
		dsp_data[38]>=((Parameter[28]<<8)|Parameter[29])*1.414 && 
		dsp_data[39]>=((Parameter[28]<<8)|Parameter[29])*1.414 )
  		{
			SystemCurrentReachedFlag=1;
		}
  		if( SystemCurrentReachedFlag==1 && dsp_data[37]<((Parameter[30]<<8)|Parameter[31])*1.414 && 	//关机电流判断
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
		//SPI_ReceiveProcess();						// SPI接收处理
		StatusProcess();							//设备状态处理
		error_handle();              				//故障处理
		Event_handle();								//事件处理
		if(ParameterFinishFlag ==0)					//参数读取完成标志位
		{	
			if(NextReceFrameID<22)
			{
				NextReceFrameID++;	
			}
			else NextReceFrameID=0;	
		}
		else 
		{
			if(ManualPassiveSwitchFlagBak==1)		//手动投切标志位
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
		

		CurrentFrameID = SPI_CommumWithDSP();		//SPI通讯帧处理
		if(NextReceFrameID==43)						//手动投切
		{
			SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];
			SendSpiBuf[2] =(uint16)(ManualPassiveSwitch&0xFFFF);
			SendSpiBuf[3] =(uint16)(ManualPassiveSwitch>>16);
		}
		else if(NextReceFrameID==42)				//无源返回信号
		{
			SendSpiBuf[1] =SPI_NextStartAddress[NextReceFrameID];
			SendSpiBuf[2] =PassiveSign;
		}
		else										//循环查询数据
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
			Event_time = RTCGetTime();		//打时间搓
			Event_Code = SPIERROR;
			SPICommError =1;
			
		}
			
	}
}
/******************************************************************************
** 主程序开始
******************************************************************************/
int main()
{
 	SystemInit();                                  	//系统频率初始化(主晶振12MHz,RTC时钟频率32KHz,内部RC频率4M Hz)
 	GPIO_init();                                   	//管脚初始化
	TIME_init(0);                                   //定时器0初始化
	I2c1_Init();                                   	//i2c初始化
	Parameter_init();                              	//参数初始化
	UART0_Init();								   	//UART0初始化
	UART1_Init();                                  	//UART1初始化
	UART3_Init();                                  	//UART1初始化	
	Bluetooth_init();								//蓝牙初始化
//	Testtime_init();					//初始化时间和功能开关
 	GZ_init();                                     	//读取从机故障和单机故障和自行修改参数 
	SSP0_Init();                                   	//SSP0初始化
	WDTInit();										//看门狗
/****************************主循环**********************************/
	SpiComm_Enable;									//通讯使能
	SpiWrite_Enable;								//参数写使能
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
		AccumEventProcess();						//累计时间类处理
		DatalogProcess();							//数据记录处理
		//puyaotest2 =1;
		SPIReadyData();								//SPI通讯处理
//		Testtime_main();								//开始判断时间
		//puyaotest2 =2;
		//ExternUartProcess();						//外部通讯处理
		ExternUartProcess(Uart1);					//外部通讯处理
		ExternBTProcess(Uart0);						//蓝牙通讯处理
		//puyaotest2 =3;
		CommunWithDUGSProcess(100);					//屏幕通讯处理，数据刷新时间
		//puyaotest2 =4;
		OnOffProcess();								//开关机处理
		
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
			LPC_SSP0->ICR =3;						//SSP0初始化
			SSP0_Init();							//SSP0初始化
		}
		if(Uart1Reset==1)
		{
			UART1_Init();                            //UART1初始化
			Uart1Reset =0;
		}
		if(Uart0Reset==1)
		{
			UART0_Init();                            //UART1初始化
			Uart0Reset =0;
		}
		if(Uart3Reset==1)
		{
			UART3_Init();                            //UART1初始化
			Uart3Reset =0;
		}
		PassiveReturnSignProcess();					//无源干结点返回信号处理
		WDTFeed();									//喂狗
		puyaotest2 =6;
		
		//判断是否与DSP通讯，有通讯就初始化蓝牙
		if((bluetooth_int==1)&&(first_int==0))
		{
			Bluetooth_init();
			first_int=1;
		}
		
		if(BluetoothConnectOK !=0)					//蓝牙通讯连接
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
** 函数名: UART1_IRQHandler
**
** 功能：UART1中断,处理蓝牙通讯的收发
******************************************************************************/
void UART0_IRQHandler(void) 
{
	uint32 j,tmp;
	uint8 UartError=0;
	puyaotest =5;
	puyaotestfault++;
	//WDTFeed();									//喂狗
	if(led4_flag==0){led4_ON;led4_flag=1;}
	else
	{
		led4_OFF;
		led4_flag=0;
	}
	if(Delay35Count>Delay35character)												// 大于3.5个字符时间
	{
		RecvSciNum =0;
		while (((LPC_UART0->IIR & 0x01) == 0)&&(RecvSciNum<150))					// 判断中断是否挂起并且不满300数
		{										 
			switch (LPC_UART0->IIR & 0x0E)											// 判断中断标志位	 
			{											  
				case 0x04:
					RecvSciNum =0;
					while(((LPC_UART0->LSR&0x01)!=0)&&(RecvSciNum<150))			//判断FIFO是否为空并且不满300数
					{
						LPC_UART0->LCR=0x03;
						if(BluetoothATcmd==1)
						{
							RecvSciBuf[RecvSciNum] =UART0_GetByte();
	
							RecvSciNum =1;
							j=9600*Delay35character;								//设置延迟
							while(1)
							{
								if((LPC_UART0->LSR&0x01)==0)						//FIFO抓空后等待一段时间
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
									RecvSciBuf[RecvSciNum] =  UART0_GetByte();		//从FIFO中取数
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
							RecvSciBuf[RecvSciNum] =UART0_GetByte();					//抓头
						
							if(RecvSciBuf[0]==LocalAddr/*||RecvSciBuf[0]==0xA5*/)		//判断地址
							{
								RecvSciNum =1;
								j=9600*Delay35character;								//设置延迟
								while(1)
								{
									if((LPC_UART0->LSR&0x01)==0)						//FIFO抓空后等待一段时间
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
										RecvSciBuf[RecvSciNum] =  UART0_GetByte();		//从FIFO中取数
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
									RecvBTFlag = 1;								//判断头和帧长
									RecvDTUSuccessTest++;
								}
								Delay35Count =0;
							}
							else break;
						}
					}
					break;
				case 0x0C:															// 字符超时中断 抓空FIFO或抓满300数就退出	  
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
					RecvSciNum =0;													// 状态线中断 抓空FIFO或抓满300数就退出	 
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
** 函数名: UART0_IRQHandler
**
** 功能：UART0中断,处理显示屏串口的收发
******************************************************************************/
void UART1_IRQHandler(void)//上位机,大小屏的通讯口
{	
	uint32 j,tmp;
	uint8 UartError=0;
	puyaotest =5;
	puyaotestfault++;
	//WDTFeed();									//喂狗
	if(led4_flag==0){led4_ON;led4_flag=1;}
	else
	{
		led4_OFF;
		led4_flag=0;
	}
	if(Delay35Count>Delay35character)												// 大于3.5个字符时间
	{
		RecvSci3Num =0;
		while (((LPC_UART1->IIR & 0x01) == 0)&&(RecvSci3Num<300))					// 判断中断是否挂起并且不满300数
		{										 
			switch (LPC_UART1->IIR & 0x0E)											// 判断中断标志位	 
			{											  
				case 0x04:
					while(((LPC_UART1->LSR&0x01)!=0)&&(RecvSci3Num<300))			//判断FIFO是否为空并且不满300数
					{
						LPC_UART1->LCR=0x03;
						RecvSci3Num =0;
						RecvSci3Buf[RecvSci3Num] =UART1_GetByte();					//抓头
					
						if(RecvSci3Buf[0]==LocalAddr/*||RecvSciBuf[0]==0xA5*/)		//判断地址
						{
							RecvSci3Num =1;
							j=9600*Delay35character;								//设置延迟
							while(1)
							{
								if((LPC_UART1->LSR&0x01)==0)						//FIFO抓空后等待一段时间
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
									RecvSci3Buf[RecvSci3Num] =  UART1_GetByte();		//从FIFO中取数
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
								RecvNewFlag = 1;								//判断头和帧长
								RecvDTUSuccessTest++;
							}
							Delay35Count =0;
						}
						else break;
					}
					break;
				case 0x0C:															// 字符超时中断 抓空FIFO或抓满300数就退出	  
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
				case 0x03:															// 状态线中断 抓空FIFO或抓满300数就退出	 
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
** 函数名: UART3_IRQHandler
**
** 功能：UART3中断,外部通讯的收发
******************************************************************************/
void UART3_IRQHandler(void) 
{
	uint32 i,tmp;
	uint8 UartError=0;

	puyaotest =7;
	sendok=0;
	
	//WDTFeed();									//喂狗
	while ((LPC_UART3->IIR & 0x01) == 0)										// 判断中断是否挂起
	{							
		
		switch (LPC_UART3->IIR & 0x0E)											// 判断中断标志位	 
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
			case 0x0C:															// 字符超时中断 抓空FIFO或抓满300数就退出	 
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
			case 0x03:															// 状态线中断 抓空FIFO或抓满300数就退出	 	 
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
** 函数名: OnOffProcess
**
** 功能：开关机处理
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
				Event_time = RTCGetTime();		//打时间搓
				Event_Code = RemotePowerOn;
				ACSwitchFlag =1;
				RecordEventFlag =1;
				RemotePowerOnOffFlag=0;
			}
			else
			{
				Event_time = RTCGetTime();		//打时间搓
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
				Event_time = RTCGetTime();		//打时间搓
				Event_Code = RemotePowerOff;
				ACSwitchFlag =0;
				RecordEventFlag =1;
				RemotePowerOnOffFlag=0;
			}
			else
			{
				Event_time = RTCGetTime();		//打时间搓
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
		AlarmTimeProcess();//定时只有1,2,3可以使用，定时4已删除
	}
	else OnOff2_Disable;
}
/******************************************************************************
** 函数名: AlarmTimeProcess
**
** 功能：定时开关机处理
******************************************************************************/
void AlarmTimeProcess(void)
{	
	current_time = RTCGetTime();		//读取时间
	if(AlarmTime1[0]!=0)
 	{
 		if(AlarmTime1[0]==1)//定时开机
 		{
 			if(AlarmTime1[1]<=AlarmTime1[3])//起始时间小于结束时间
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				&&((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Enable;
					if(AlarmSwitchFlag==0)
					{
						Event_Code=AlarmpPowerOn;
						Event_time = RTCGetTime();		//打时间搓
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
						Event_time = RTCGetTime();		//打时间搓
						AlarmSwitchFlag=1;
						RecordEventFlag =1;
					}
				}
			}
			if(AlarmTime2[0]==1)
			{
				if(AlarmTime2[1]<=AlarmTime2[3])//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					&&((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
							AlarmSwitchFlag=1;
							RecordEventFlag =1;
						}
					}
				}
				if(AlarmTime3[0]==1)
				{
					if(AlarmTime3[1]<=AlarmTime3[3])//起始时间小于结束时间
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						&&((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Enable;
							if(AlarmSwitchFlag==0)
							{
								Event_Code=AlarmpPowerOn;
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
//					if(AlarmTime4[0]==1)
//					{
//						if(AlarmTime4[1]<=AlarmTime4[3])//起始时间小于结束时间
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							&&((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Enable;
//								if(AlarmSwitchFlag==0)
//								{
//									Event_Code=AlarmpPowerOn;
//									Event_time = RTCGetTime();		//打时间搓
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
//									Event_time = RTCGetTime();		//打时间搓
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
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//起始时间小于结束时间
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
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
//								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3]))//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else//定时关机
			{
				if(AlarmTime1[1]<=AlarmTime1[3])//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4])))
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
			}
 		}
		else if(AlarmTime1[0]==2)//定时关机
		{
			if(AlarmTime1[1]<=AlarmTime1[3])//起始时间小于结束时间
			{
				if(((current_time.RTC_Hour>AlarmTime1[1])||((current_time.RTC_Hour==AlarmTime1[1])&&(current_time.RTC_Min>=AlarmTime1[2])))
				&&((current_time.RTC_Hour<AlarmTime1[3])||((current_time.RTC_Hour==AlarmTime1[3])&&(current_time.RTC_Min<AlarmTime1[4])))
				)
				{
					OnOff2_Disable;
					if(AlarmSwitchFlag==1)
					{
						Event_Code=AlarmpPowerOff;
						Event_time = RTCGetTime();		//打时间搓
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
						Event_time = RTCGetTime();		//打时间搓
						AlarmSwitchFlag=0;
						RecordEventFlag =1;
					}
				}
			}
			if(AlarmTime2[0]==1)
			{
				if(AlarmTime2[1]<=AlarmTime2[3])//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour>AlarmTime2[1])||((current_time.RTC_Hour==AlarmTime2[1])&&(current_time.RTC_Min>=AlarmTime2[2])))
					&&((current_time.RTC_Hour<AlarmTime2[3])||((current_time.RTC_Hour==AlarmTime2[3])&&(current_time.RTC_Min<AlarmTime2[4])))
					)
					{
						OnOff2_Disable;
						if(AlarmSwitchFlag==1)
						{
							Event_Code=AlarmpPowerOff;
							Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
							AlarmSwitchFlag=0;
							RecordEventFlag =1;
						}
					}
				}
				if(AlarmTime3[0]==1)
				{
					if(AlarmTime3[1]<=AlarmTime3[3])//起始时间小于结束时间
					{
						if(((current_time.RTC_Hour>AlarmTime3[1])||((current_time.RTC_Hour==AlarmTime3[1])&&(current_time.RTC_Min>=AlarmTime3[2])))
						&&((current_time.RTC_Hour<AlarmTime3[3])||((current_time.RTC_Hour==AlarmTime3[3])&&(current_time.RTC_Min<AlarmTime3[4])))
						)
						{
							OnOff2_Disable;
							if(AlarmSwitchFlag==1)
							{
								Event_Code=AlarmpPowerOff;
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=0;
								RecordEventFlag =1;
							}
						}
					}
//					if(AlarmTime4[0]==1)
//					{
//						if(AlarmTime4[1]<=AlarmTime4[3])//起始时间小于结束时间
//						{
//							if(((current_time.RTC_Hour>AlarmTime4[1])||((current_time.RTC_Hour==AlarmTime4[1])&&(current_time.RTC_Min>=AlarmTime4[2])))
//							&&((current_time.RTC_Hour<AlarmTime4[3])||((current_time.RTC_Hour==AlarmTime4[3])&&(current_time.RTC_Min<AlarmTime4[4])))
//							)
//							{
//								OnOff2_Disable;
//								if(AlarmSwitchFlag==1)
//								{
//									Event_Code=AlarmpPowerOff;
//									Event_time = RTCGetTime();		//打时间搓
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
//									Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
//								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1&&AlarmTime3[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3])&&(AlarmTime3[1]<=AlarmTime3[3]))//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4]))
					&&((current_time.RTC_Hour<AlarmTime3[1])||(current_time.RTC_Hour>AlarmTime3[3])||(current_time.RTC_Hour==AlarmTime3[1]&&current_time.RTC_Min<AlarmTime3[2])||(current_time.RTC_Hour==AlarmTime3[3]&&current_time.RTC_Min>=AlarmTime3[4])))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
								AlarmSwitchFlag=1;
								RecordEventFlag =1;
							}
						}
					}
				}
			}
			else if(AlarmTime2[0]==1)
			{
				if((AlarmTime1[1]<=AlarmTime1[3])&&(AlarmTime2[1]<=AlarmTime2[3]))//起始时间小于结束时间
				{
					if(((current_time.RTC_Hour<AlarmTime1[1])||(current_time.RTC_Hour>AlarmTime1[3])||(current_time.RTC_Hour==AlarmTime1[1]&&current_time.RTC_Min<AlarmTime1[2])||(current_time.RTC_Hour==AlarmTime1[3]&&current_time.RTC_Min>=AlarmTime1[4]))
					&&((current_time.RTC_Hour<AlarmTime2[1])||(current_time.RTC_Hour>AlarmTime2[3])||(current_time.RTC_Hour==AlarmTime2[1]&&current_time.RTC_Min<AlarmTime2[2])||(current_time.RTC_Hour==AlarmTime2[3]&&current_time.RTC_Min>=AlarmTime2[4])))
					{
						OnOff2_Enable;
						if(AlarmSwitchFlag==0)
						{
							Event_Code=AlarmpPowerOn;
							Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
								Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
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
							Event_time = RTCGetTime();		//打时间搓
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
** 函数名: AccumEventProcess
**
** 功能：累计事件处理
******************************************************************************/
void AccumEventProcess(void)
{
	uint8 temp[76],i,j;
	//uint64	tmp64H[2],tmp64L[2],tmp64Val[2];
	//uint8   DatalogNum;
	
	local_time = RTCGetTime();					//读取时间
	
	if(local_time.RTC_Year!= EepromYear||local_time.RTC_Mon!= EepromMon||local_time.RTC_Mday!= EepromDay)	//日期变更
	{
		TotalEnergyBakHB =TotalEnergyHB;
		TotalEnergyBakLB =TotalEnergyLB;						//备份前一天累计数据

		TotalRunningTimeBak =TotalRunningTime;					//备份前一天累计运行时间
		
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
		WriteEeprom(EepParameterAddr+1568,temp,24);   					//将出厂参数写到正式参数中
		I2C_read;
		
		DatalogReset=1;										//////////////////数据记录重置////////////////////////////////
	}
	if(SystemStatus>=SystemStandby)										//设备处于待机以后
	{
		//////////////////数据记录重置////////////////////////////////
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
	if(SystemStatus>=SystemReady)								//运行后开始记录
	{
		SystemRunningFlag =1;
		/*if(TimeDelay>59)										//每分钟刷新一次当天累计
		{
			TimeDelay =0;
			tmp64H[0] =(uint64)(TotalEnergyHB);
			tmp64L[0] =(uint64)(TotalEnergyLB);
			tmp64Val[0] =(tmp64H[0]<<32)|tmp64L[0];
				
			tmp64H[1] =(uint64)(TotalEnergyBakHB);
			tmp64L[1] =(uint64)(TotalEnergyBakLB);
			tmp64Val[1] =(tmp64H[1]<<32)|tmp64L[1];
				
			DailyEnergy =tmp64Val[0]-tmp64Val[1];				//当天累计补偿量

		}*/
		if(AccumDelay>359)						//每6分钟记录一次累计补偿量
		{
			AccumDelay =0;
			TotalRunningTime++;
			DailyRunningTime =TotalRunningTime-TotalRunningTimeBak;			//当天累计补偿量

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
		if(DatalogDelay>600)						//每10分钟记录一次数据记录
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
	if( SystemStatus<SystemReady&&SystemRunningFlag==1)		//设备从运行到待机记录一次累计数据
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
		I2C_write;													//写入I2C
		WriteEeprom(EepParameterAddr+848,temp,16);   			//将出厂参数写到正式参数中
		I2C_read;

		I2C_write;													//写入I2C
		WriteEeprom(EepParameterAddr+864,temp+16,6);   			//将出厂参数写到正式参数中
		I2C_read;
		DUGSReadMainParameterFlag =1;
		ResetDelay =0;
		
	}*/
}
/*uint64 BCD2HEX(uint64 bcd_data)    //BCD转为HEX子程序    
{   
    uint64 temp;   
    temp=(bcd_data/16*10 + bcd_data%16);   
    return temp;   
}*/   
uint8 HEX2BCD(uint64 Dec, uint8 *Bcd, uint8 length)     //HEX转为BCD子程序     
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


//ARM硬件故障后进入此函数，目的是硬件故障能够复位
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
	/*异常中断发生时，这个异常模式特定的物理R14,即lr被设置成该异常模式将要返回的地址*/
	/*stacked_lr = ((unsigned long) hardfault_args[5]);
	stacked_pc = ((unsigned long) hardfault_args[6]);
	stacked_psr = ((unsigned long) hardfault_args[7]);
	 SHCSR = (*((volatile unsigned long *)(0xE000ED24))); //系统Handler控制及状态寄存器  
	 MFSR = (*((volatile unsigned char *)(0xE000ED28))); //存储器管理fault状态寄存器   
	 BFSR = (*((volatile unsigned char *)(0xE000ED29))); //总线fault状态寄存器   
	 UFSR = (*((volatile unsigned short int *)(0xE000ED2A)));//用法fault状态寄存器    
	 HFSR = (*((volatile unsigned long *)(0xE000ED2C)));  //硬fault状态寄存器     
	 DFSR = (*((volatile unsigned long *)(0xE000ED30))); //调试fault状态寄存器  
	 MMAR = (*((volatile unsigned long *)(0xE000ED34))); //存储管理地址寄存器  
	 BFAR = (*((volatile unsigned long *)(0xE000ED38))); //总线fault地址寄存器  
	*/
	 while (1) {
	//OnOff_Disable;
	LPC_SSP0->IMSC = 0x00;			  		//		  禁止中断
	SystemStatus = SystemProtect;
	MainErrorCode =31;
	alarm_time = RTCGetTime();					//打时间搓
	error_handle();              				//故障处理
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
__set_FAULTMASK(1);      // 关闭所有中断
NVIC_SystemReset();			 // 执行复位
}
/*****************************************************************************
** Function name:		Passive Return Sign
**
** Descriptions:		继电器返回故障信号
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void PassiveReturnSignProcess( void )
{
	uint8 i;
	//static uint16 count1 =0,count2 =0,count3 =0,count4 =0;
	if(((1<<8)& LPC_GPIO1->FIOPIN)==0)// 干接点ajin1
	{
		PassiveSign_aisle[0]=1;
	}
	else	PassiveSign_aisle[0]=0;
	if(((1<<10)& LPC_GPIO1->FIOPIN)==0)// 干接点ajin2
	{
		PassiveSign_aisle[1]=1;
	}
	else	PassiveSign_aisle[1]=0;
	if(((1<<15)& LPC_GPIO1->FIOPIN)==0)// 干接点ajin3
	{
		PassiveSign_aisle[2]=1;
	}
	else	PassiveSign_aisle[2]=0;
	if(((1<<17)& LPC_GPIO1->FIOPIN)==0)// 干接点ajin4
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
** Descriptions:		使用负载侧数据记录数据
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
