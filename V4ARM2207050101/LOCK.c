uint16 Testtime;		          	//����ʱ��
uint16 Testtime_function;       //���ܿ���
uint16 Testtime_word;           //���鹦������
uint16 Testtime_time1;					//1��ʱ�����
/*****************************************************************************
** Function name:		Testtime_main
**
** Descriptions:		�ж�����ʱ��
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_main(void)
{
   Testtime_read();						    			//������
	if(Testtime_function==1)							//�жϹ����Ƿ���
	{
		if(Testtime==20)										//�ж�ʱ���Ƿ񵽴�
		{
			delay(50);
			ChangePage(0xB0);									//��Ļ����
			delay(50);
			ChangePage(0xB0);									//��Ļ����
			delay(50);
			ChangePage(0xB0);									//��Ļ����
			while(Testtime_word !=1000)					//ѭ������
			{
				Testtime_read();									//������
				delay(50);
			}
			delay(50);
			DUGSInit();												//��Ļ��ʼ��
			Testtime_close();									//�رչ��ܣ�ʱ�����㣬��������
			delay(50);
			DUGSInit();												//��Ļ��ʼ��
			Testtime_close();									//�رչ��ܣ�ʱ�����㣬��������
			delay(50);
			DUGSInit();												//��Ļ��ʼ��
			Testtime_close();									//�رչ��ܣ�ʱ�����㣬��������
		}
		else
			Testtime_add();										//ʱ��ݼ�
	}
}
/*****************************************************************************
** Function name:		Testtime_init
**
** Descriptions:		��ʼ��ʱ��͹��ܿ���
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_init(void)
{
	uint8 Eeptempbuff[4];
	
	ReadEeprom(0x1196, Eeptempbuff,4);						//��ȡeeprom���ʱ��͹��ܿ���
  Testtime         =Eeptempbuff[0]<<8|Eeptempbuff[1];
  Testtime_function=Eeptempbuff[2]<<8|Eeptempbuff[3];
}
/*****************************************************************************
** Function name:		Testtime_save
**
** Descriptions:		ʱ��͹��ܿ��ر���
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_save(void)
{
	uint8 Eeptempbuff[4];
	
	Eeptempbuff[0]=(uint8)(Testtime<<8);									//��ʱ��͹��ܿ���д��eeprom
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
** Descriptions:		ʱ������
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_add(void)
{
	Testtime_time1++;
	if(Testtime_time1==50)												//һ��ʱ����٣�����Ҫ���ɶ༶����
	{ 
		Testtime_time1=0;
	  Testtime++;
		Testtime_save();														//����ʱ��͹��ܿ���
	}
}

/*****************************************************************************
** Function name:		Testtime_read
**
** Descriptions:		��ȡ���룬������
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_read(void)										//��ȡ����͹�����
{
		uint8  RecvDatalengh;
		uint16 i,RecvDatabuff[15];
		uint32 RecvFramehead;
	CommunWithDUGS.Framehead=0x0103;									//֡ͷ
	CommunWithDUGS.Framelength =1;										//���ݳ���
	CommunWithDUGS.Funcode= 0x83;											//Э�鹦����
	CommunWithDUGS.StartAddr=0x0998;										//��ַ
	CommunWithDUGS.Datalength= 4;												//��ѯ���ݳ���
		SendDatatoDUGS();																	//��������
		delay(10);																				//��ʱ�ȴ�
		RecvFramehead = (uint32)((RecvDUGSBuf[3]<<16)|(RecvDUGSBuf[4]<<8)|RecvDUGSBuf[5]);								//ƴ�Ͻ���֡ͷ
		RecvDatalengh = RecvDUGSBuf[6];																																			//�������ݳ���
		for(i=0;i<RecvDatalengh;i++)
		{
			RecvDatabuff[i] =(uint16)((RecvDUGSBuf[7+(i<<1)]<<8)|RecvDUGSBuf[8+(i<<1)]);												//ƴ��16λ��������
		}
		if(RecvFramehead ==0x830998)									//�жϵ�ַ�Ƿ���ȷ
		{	
			 Testtime_word     =RecvDatabuff[0];									//�������
			 Testtime_function =RecvDatabuff[1];								//��ù��ܿ���
		}
}
/*****************************************************************************
** Function name:		Testtime_close
**
** Descriptions:		ʱ�����㣬���ܹرգ���������
**
** parameters:			None
** Returned value:		None
** 
*****************************************************************************/
void Testtime_close(void)											//����
{		
	CommunWithDUGS.Framehead=0x0103;
	CommunWithDUGS.Framelength =1;
	CommunWithDUGS.Funcode= 0x82;
	CommunWithDUGS.StartAddr=0x0998;
	CommunWithDUGS.Framelength=4;
	CommunWithDUGS.Databuff[0]=0x0000;										//��������
	CommunWithDUGS.Databuff[1]=0x0000;									//����������
		SendDatatoDUGS();
	Testtime=0;																				//�ۼ�ʱ���������
}
/******************************************************************************
**                            End Of File
******************************************************************************/