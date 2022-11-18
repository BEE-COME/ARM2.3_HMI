uint16 Testtime;		          	//体验时间
uint16 Testtime_function;       //功能开关
uint16 Testtime_word;           //体验功能密码
uint16 Testtime_time1;					//1级时间减速
/*****************************************************************************
** Function name:		Testtime_main
**
** Descriptions:		判断体验时间
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_main(void)
{
   Testtime_read();						    			//读密码
	if(Testtime_function==1)							//判断功能是否开启
	{
		if(Testtime==20)										//判断时间是否到达
		{
			delay(50);
			ChangePage(0xB0);									//屏幕锁定
			delay(50);
			ChangePage(0xB0);									//屏幕锁定
			delay(50);
			ChangePage(0xB0);									//屏幕锁定
			while(Testtime_word !=1000)					//循环密码
			{
				Testtime_read();									//读密码
				delay(50);
			}
			delay(50);
			DUGSInit();												//屏幕初始化
			Testtime_close();									//关闭功能，时间清零，密码清零
			delay(50);
			DUGSInit();												//屏幕初始化
			Testtime_close();									//关闭功能，时间清零，密码清零
			delay(50);
			DUGSInit();												//屏幕初始化
			Testtime_close();									//关闭功能，时间清零，密码清零
		}
		else
			Testtime_add();										//时间递加
	}
}
/*****************************************************************************
** Function name:		Testtime_init
**
** Descriptions:		初始化时间和功能开关
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_init(void)
{
	uint8 Eeptempbuff[4];
	
	ReadEeprom(0x1196, Eeptempbuff,4);						//读取eeprom里的时间和功能开关
  Testtime         =Eeptempbuff[0]<<8|Eeptempbuff[1];
  Testtime_function=Eeptempbuff[2]<<8|Eeptempbuff[3];
}
/*****************************************************************************
** Function name:		Testtime_save
**
** Descriptions:		时间和功能开关保存
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_save(void)
{
	uint8 Eeptempbuff[4];
	
	Eeptempbuff[0]=(uint8)(Testtime<<8);									//将时间和功能开关写入eeprom
	Eeptempbuff[1]=(uint8)(Testtime);
	Eeptempbuff[2]=(uint8)(Testtime_function<<8);
	Eeptempbuff[3]=(uint8)(Testtime_function);
	
	I2C_write;
	WriteEeprom(0x1196, Eeptempbuff,4);	
  I2C_read;
}
/*****************************************************************************
** Function name:		Testtime_add
**
** Descriptions:		时间增加
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_add(void)
{
	Testtime_time1++;
	if(Testtime_time1==50)												//一级时间减速，如需要，可多级减速
	{ 
		Testtime_time1=0;
	  Testtime++;
		Testtime_save();														//保存时间和功能开关
	}
}

/*****************************************************************************
** Function name:		Testtime_read
**
** Descriptions:		读取密码，功能码
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_read(void)										//读取密码和功能码
{
		uint8  RecvDatalengh;
		uint16 i,RecvDatabuff[15];
		uint32 RecvFramehead;
	CommunWithDUGS.Framehead=0x0103;									//帧头
	CommunWithDUGS.Framelength =1;										//数据长度
	CommunWithDUGS.Funcode= 0x83;											//协议功能码
	CommunWithDUGS.StartAddr=0x0998;										//地址
	CommunWithDUGS.Datalength= 4;												//查询数据长度
		SendDatatoDUGS();																	//发送数据
		delay(10);																				//延时等待
		RecvFramehead = (uint32)((RecvDUGSBuf[3]<<16)|(RecvDUGSBuf[4]<<8)|RecvDUGSBuf[5]);								//拼合接受帧头
		RecvDatalengh = RecvDUGSBuf[6];																																			//接受数据长度
		for(i=0;i<RecvDatalengh;i++)
		{
			RecvDatabuff[i] =(uint16)((RecvDUGSBuf[7+(i<<1)]<<8)|RecvDUGSBuf[8+(i<<1)]);												//拼合16位数据内容
		}
		if(RecvFramehead ==0x830998)									//判断地址是否正确
		{	
			 Testtime_word     =RecvDatabuff[0];									//获得密码
			 Testtime_function =RecvDatabuff[1];								//获得功能开关
		}
}
/*****************************************************************************
** Function name:		Testtime_close
**
** Descriptions:		时间清零，功能关闭，密码清零
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_close(void)											//清零
{		
	CommunWithDUGS.Framehead=0x0103;
	CommunWithDUGS.Framelength =1;
	CommunWithDUGS.Funcode= 0x82;
	CommunWithDUGS.StartAddr=0x0998;
	CommunWithDUGS.Framelength=4;
	CommunWithDUGS.Databuff[0]=0x0000;										//密码清零
	CommunWithDUGS.Databuff[1]=0x0000;									//功能码清零
		SendDatatoDUGS();
	Testtime=0;																				//累计时间计数清零
}
/******************************************************************************
**                            End Of File
******************************************************************************/