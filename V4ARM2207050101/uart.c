/*****************************************************************************
 *   Uart.c:  ����ARM����
 *   History
 *   2017.10.24  ver 1710240101
 *	 ����:	ARMӲ�����ð��� RTC��I2C��SSP0��Uart0��Uart1��Uart3�����账��
 	 ����:	���ļ��������ܰ��� ��Ļ��������485����������������������Ҫ����ĻͨѶ״̬������������֡װ�غ�����
 	 ����485Э������������������ݷ���׼��������
 	 �ж�:	���ļ����жϺ�������:��ʱ�жϣ�SSP�жϣ�Uart0�жϣ�Uart1�жϣ�Uart3�жϣ�Ӳ�������жϣ����Ź��ж�
*****************************************************************************/

#include "lpc17xx.h"
#include "type.h"
#include "IAP.h"
#include "rtc.h"
void ARM_proess(void);

//uint16  swj_param[100];//��ַ9000-9200������Ĭ�ϲ���
/**����**/
uint16 Testtime;		          	//����ʱ��
uint16 Testtime_function;       //���ܿ���
uint16 Testtime_word;           //���鹦������
uint16 Testtime_time1;					//1��ʱ�����
uint16 Readfunction;            //��ȡ�������־λ
uint16 DUGSReadfunction;				//��Ļ��ʱ��͹���
uint16 w=0,Page,word=1000;			//Ĭ������1000
uint16 test_time=2160;								//����ʱ������
uint16 time_flag=0;							//дi2c��־��
uint16 sendok;									//���ͳɹ���־��

/**��������**/
uint16 xiuzhen_dianliujibian;//��������������
uint16 xiuzhen_dianyajibian;//��������������
uint16 xiuzhen_fuzhi;//2-50�η�ֵ����
uint16 xiuzhen_pf;//pf����
uint16 xiuzhen_cos;//cos������
uint16 xiuzhen_shuchu;//���������Чֵ����
uint16 xiuzhen_wugong;//�޹���������
uint16 xiuzhen_jibian,xiuzhen_rongliang;//���书������,�������������������
uint8  xiuzhen_mode,xiuzhen_bili;//����ģʽ��0:�ֶ���1��SVG��2��APF���������������ﵽ�����������

uint16 xiuzhen_time=0;//���ú���Чʱ�䣬Ĭ��30S

uint16 xiuzhen_dianliujibian2;//��������������2
uint16 xiuzhen_dianyajibian2;//��������������2
uint16 xiuzhen_fuzhi2;//2-50�η�ֵ����2
uint16 xiuzhen_pf2;//pf����2
uint16 xiuzhen_cos2;//cos������2
uint16 xiuzhen_shuchu2;//���������Чֵ����2
uint16 xiuzhen_wugong2;//�޹���������2
uint16 xiuzhen_jibian2;//���书������2

uint16 ini_version_a,ini_version_b,ini_version_flag;//Ĭ�ϳ�ʼ�汾��DSP��ʼ�汾����ʼ�汾���±�־��

uint32 zongxiebo_A,zongxiebo_B,zongxiebo_C;//��г������A,B,C

uint16 lanya_flag=0;						//�����ֶ�Ͷ�б�־
extern void SPIReadyData(void); //SPIͨѶ������׼��SPI��һ֡��������װ��

extern	void RecvDataProcess(uint8 uart);//����485���������մ����������봮�ں�
extern	void ReadyData(void);//׼�����ⷢ������
extern	uint16	crc_chk(uint8 *buf,uint32 length);//CRC16����
extern	uint8 FaultProcess(uint8 FaultPageNo,uint8 row,uint8 type);//���͸���Ļ�Ĺ��Ϻ�������
extern	void RestoreProcess(void);//�߼��������������������ݣ��ָ��������ص����ݣ�ϵͳ��ʼ����ǿ���������������
extern	void ChangePage(uint16 PageID);//��ת��Ļҳ��
extern	void DUGSInit(void);//��Ļ��ʼ��
extern	void DUGSProcess(void);//��Ļҳ�洦��
extern	void RecvDataFromDUGS(void);//������Ļ�������
extern	void SendDatatoDUGS(void);//������ĻͨѶ
extern	void SendDataload(uint32 Framehead);//��Ļ��������װ��
extern	uint32 Frameheadload(void);//֡ͷװ��
extern	void CommunWithDUGSProcess(uint16 Delaytime);//����ĻͨѶ״̬������
extern	void ExternUartProcess(uint8 Uart);//����485���ڴ�����
extern	void ExternBTProcess(uint8 Uart);//�����������ڴ�����
extern	void ChangeStatus(uint16 Status);//�ı���Ļ��ʾ״̬
extern void ReadEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//��ȡI2C����
extern void WriteEeprom(uint32_t addr,uint8_t *pointer,uint32_t num);//д��I2C����
extern void delay(uint32_t dly);//�ӳ�
//extern uint64 BCD2HEX(uint64 hex_data);    //HEXתΪBCD�ӳ���  
extern uint8 HEX2BCD(uint64 Dec, uint8 *Bcd, uint8 length);     //HEXתΪBCD�ӳ���  
extern void SaveARMParaProcess(void);//����ARM���������
extern void CabinProcess(void);//���ܲ�λ���ú���
extern void ChannelProcess(void);//ֱͨ��Դ���ú���
extern void Bluetooth_init(void);	


extern uint16 Testtime_function;
uint16 ApparentBal2;//���ڲ�ƽ�ⱸ��
extern uint16 count_swj;

extern	double	pclock;
/*extern	uint16   xuliezongshu,xishu;
extern	uint8	dj_correction[50];
extern	uint8 	var_DecPo1,var_DecPo2,var_DecPo3,var_DecPo4;
extern	uint16 	dsp_data_OCt[300],dsp_data_OCf[300],dsp_data_LCt[300],dsp_data_L_Ot[300],dsp_data_L_Of[300],dsp_data_Vt[300];
extern	uint8 	*pointer8,dev_mode,password_pass;
extern	uint16 	data_temp[ADCAisleMax][GetADMax],Avg[ADCAisleMax];		//�¶Ȳ�������ֵ��ƽ��ֵ
extern	uint16	i_suanfa_before[50];*/
extern	uint8 	led2_flag;
extern	uint16	LocalAddr;	 										// ���ڵ�ַ
extern	uint16 	UART_BPS;     			                        	// ���ڲ�����
extern	uint8 	RemoteEnable; 										// Զ��ʹ��
extern	uint16	ReadCount;
extern	uint32	moni_delay;
extern	uint32	ProjectNo;											//��Ŀ��
extern	uint16	ProductionNo;										//������
extern	uint64	SerialNumber;										//���к�
//extern	uint16	OnOffCount;
extern	uint16	puyaotest;
extern	uint16 AlarmTime1[5],AlarmTime2[5],AlarmTime3[5],AlarmTime4[5];
extern  RTCTime local_time, alarm_time, current_time,Event_time;   //RTC�ṹ��
extern	uint16	PassiveSign_aisle[16];
extern	uint16  PassiveChannel[32],PassiveValue[32],PassiveCom;
extern	uint16  SetupMode;
extern	uint16	CorrectParam[11];
uint16  ARM_param[100];

uint16	LocalAddr; 										// ���ڵ�ַ
uint16 	UART_BPS;     			                        // ���ڲ�����
uint16	OnOffStatus;									// Ĭ�Ͽ��ػ�״̬
uint16	puyaotest,puyaotest2,puyaotest3,puyaotest4,puyaotesttmp[10],puyaotestfault;
uint16 CT_MAIN1,CT_MAIN2;//����������������װλ��
uint8 SPIError;

uint8 	xiazai,duqv;

volatile uint8		RecvNewFlag;                               	 // ���ڽ������ݱ�־λ
volatile uint8		RecvBTFlag;                               	 // ���ڽ������ݱ�־λ
volatile uint8 		RecvDUGSNewFlag;							// DUGS�������ݱ�־λ
//*****************************************�����־λ*****************************************//
volatile uint8		RunningInfoWindowFlag;						//������Ϣ����
volatile uint8		WaveformWindowFlag;							//���ν���
volatile uint8		HistogramWindowFlag;						//��״ͼ����
volatile uint8		HistoryFaultWindowFlag;						//���ϼ�¼����
volatile uint8		SlaveFaultWindowFlag;						//�ӻ����ϼ�¼����
volatile uint8		DebugWindowFlag;							//�������ݽ���
volatile uint8		HarmonicWindowFlag;							//г�����ݽ���
volatile uint8		SlaveWindowFlag;							//�ӻ����ݽ���
volatile uint8		EventLogWindowFlag;							//�¼���¼����
volatile uint8		PassiveWindowFlag;							//��ԴͶ�н���
volatile uint8		DataLogWindowFlag;							//��ʷ���ݼ�¼����
volatile uint8		WindowbakFlag;								//��������

volatile uint8		FirstEnterWindowFlag;						//�״ν������
volatile uint8		FirstCommunSuccess;							//�״�ͨѶ�ɹ�

volatile uint8		DUGSSystemParametersInitFlag;				//ϵͳ������ʼ��
volatile uint8		ReadFaultRecordFinishFlag;					//���϶�ȡ���
volatile uint8		BackupFinishFlag;							//�����������
volatile uint8		ReStoreBackupSuccess;						//��ȡ���ݳɹ�
volatile uint8		ReStoreBackupFailure;						//��ȡ����ʧ��
volatile uint8		ReStoreFactorySuccess;						//�ָ������ɹ�
volatile uint8		ReStoreFactoryFailure;						//�ָ�����ʧ��
volatile uint8		ClearFaultSuccess;							//��������ɹ�
volatile uint8		SystemResetSuccess;							//ǿ�������ɹ�
volatile uint8		SystemInitSuccess;							//ϵͳ��ʼ���ɹ�

volatile uint8		IllegalOperation;							//�Ƿ�����
volatile uint8		ParameterFinishFlag;						//����д��ɹ�
volatile uint8		ManualPassiveSwitchFlag,ManualPassiveSwitchFlagBak;		//�ֶ���ԴͶ��
volatile uint8		PassiveReturnFlag,PassiveReturnFlagBak;		//��Դ�����ź�
volatile uint8		OnekeyClean;								//һ�����
volatile uint8		ParameterError;								//��������



//*****************************************************��������־λ**********************************************************************//
//						��Ҫ������־λ					ѡ�β�����־λ				�û�������־λ					���Բ�����־λ		
volatile uint8		ReadMainParameterFlag,			ReadSelectedtimesFlag,		ReadUserParameterFlag,/*ReadDebugParameterFlag,*/
//						У׼1������־λ					У׼2������־λ				У׼3������־λ					��Դ������־λ
					ReadCorrectionParameterFlag,	ReadCorrectionParameter2Flag,ReadCorrectionParameter3Flag,	ReadPassiveParameterFlag,
//						�ӻ�1������־λ 				�ӻ�2������־λ 			�ӻ�3������־λ 				�ӻ�4������־λ 
					ReadSlaveParameterFlag,	  		ReadSlaveParameter2Flag, 	ReadSlaveParameter3Flag,		ReadSlaveParameter4Flag,
//						�ӻ�5������־λ 				�ӻ�6������־λ 			�ӻ�7������־λ 				�ӻ�8������־λ 
					ReadSlaveParameter5Flag,	 	ReadSlaveParameter6Flag, 	ReadSlaveParameter7Flag,		ReadSlaveParameter8Flag,
//						�ӻ�9������־λ 				�ӻ�10������־λ			�ӻ�11������־λ 				�ӻ�12������־λ
					ReadSlaveParameter9Flag,	  	ReadSlaveParameter10Flag, 	ReadSlaveParameter11Flag,	  	ReadSlaveParameter12Flag, 
//						�ӻ�13������־λ 				�ӻ�14������־λ			�ӻ�15������־λ				�ӻ�16������־λ
					ReadSlaveParameter13Flag,	  	ReadSlaveParameter14Flag, 	ReadSlaveParameter15Flag,	  	ReadSlaveParameter16Flag, 
//						г��������־λ
					ReadHrmonicParameterFlag;
//*****************************************************д������־λ**********************************************************************//
//						��Ҫ������־λ					ѡ�β�����־λ				�û�������־λ			���Բ�����־λ		
volatile uint8		WriteMainParameterFlag,			WriteSelectedtimesFlag,		  WriteUserParameterFlag,/*WriteDebugParameterFlag,*/
//						У׼1������־λ					У׼2������־λ				У׼3������־λ					��Դ������־λ
					WriteCorrectionParameterFlag, 	WriteCorrectionParameter2Flag,WriteCorrectionParameter3Flag,	WritePassiveParameterFlag,
//						�ӻ�1������־λ					�ӻ�2������־λ				�ӻ�3������־λ					�ӻ�4������־λ	
					WriteSlaveParameterFlag,	 	WriteSlaveParameter2Flag,		WriteSlaveParameter3Flag,		WriteSlaveParameter4Flag,
//						�ӻ�5������־λ 				�ӻ�6������־λ 			�ӻ�7������־λ 				�ӻ�8������־λ 
					WriteSlaveParameter5Flag,	  	WriteSlaveParameter6Flag, 	WriteSlaveParameter7Flag,		WriteSlaveParameter8Flag,
//						�ӻ�9������־λ 				�ӻ�10������־λ 			�ӻ�11������־λ 				�ӻ�12������־λ 
					WriteSlaveParameter9Flag,	 	WriteSlaveParameter10Flag, 	WriteSlaveParameter11Flag,		WriteSlaveParameter12Flag,
//						�ӻ�13������־λ 				�ӻ�14������־λ 			�ӻ�15������־λ 				�ӻ�16������־λ 
					WriteSlaveParameter13Flag,	  	WriteSlaveParameter14Flag, 	WriteSlaveParameter15Flag,		WriteSlaveParameter16Flag,WriteSpcialParameterFlag,
//						г��������־λ
					WriteHrmonicParameterFlag;
volatile uint8		DUGSReadMainParameterFlag,			DUGSReadSelectedtimesFlag,		DUGSReadCorrectionParameterFlag,	DUGSReadCorrectionParameter2Flag,	DUGSReadARMCorrectFlag,DUGSReadParameterFlag,
					DUGSReadCorrectionParameter3Flag,	DUGSReadSlaveParameterFlag,		DUGSReadSlaveDataFlag,				DUGSReadSlaveMaxMinFlag,			DUGSReadPassiveParameterFlag;

volatile uint8		DUGSWriteMainParameterFlag,			DUGSWriteARMParameterFlag,			DUGSWriteARMCorrectFlag,		DUGSWriteSelectedtimesFlag,		DUGSWriteCorrectionParameterFlag,DUGSWriteParameterFlag,
					DUGSWriteCorrectionParameter2Flag,	DUGSWriteCorrectionParameter3Flag,	DUGSWriteSlaveParameterFlag,	DUGSWritePassiveParameterFlag;
volatile uint8		DUGSTimesetFlag;					
volatile uint8		DUGSInitFlag,DUGSInitFinishFlag,RecoverProcessFlag,RestoreProcessFlag,SetPageIDFlag;
volatile uint8		DataRetrans;								// �����ط�
volatile uint8		NoNeedWaitingFlag;							//����Ҫ�ȴ��ظ���־λ
volatile uint8		RemotePowerOnOffFlag,OnOffChangeFlag;

volatile uint8 DUGSRead666Flag,DUGSWrite666Flag;

uint16				Remote_Password;


uint8        		RecvSciBuf[150];         					// ����ͨѶ��������
uint32      		RecvSciNum;                                  // ����ͨѶ��������

uint8        		RecvSci3Buf[150];         					// �ⲿ����ͨѶ��������
uint32      		RecvSci3Num;                                  // �ⲿ���ڽ�������

uint8				RecvDUGSBuf[300];							 // DUGS�������ݻ���
uint32      		RecvDUGSNum;                                  // DUGS�������ݸ���

uint16		 		RecvDTUSuccessTest;							 // DTU���ճɹ�����
uint16		 		RecvDGUSSuccessTest;						 // DGUS���ճɹ�����
uint16		 		SendDTUSuccessTest;							 // DTU���ͳɹ�����
uint16		 		SendDGUSSuccessTest;						 // DGUS���ͳɹ�����

//uint16 		 		SciSuccessNUM;								// ���ڽ��ճɹ�����
uint8				Delay35character;
uint8				Delay35Count;
uint16				SpiOverTimeCount;
uint16				SciWaitingCount;
uint16				SciWaitingCount3;								// ���ڵȴ���ʱ

uint32 				CurrentFramehead;
uint32 				NextFramehead;


uint16				InputRegister[2600],HoldingRegister[1950];

uint16				ScreenType;									//��Ļ����
uint16				LogoType;									//Logo����
uint16 				CurrentPageID,LastPageID;					//��ǰҳ�棬�ϴ�ҳ��
uint16 				RestoreCommand;								//�ָ���������
uint16				LocalOnOff,OnOffCommand,RemoteOnOff,SwitchON;//���ػ�����
uint16				SaveOrReadFlag;								//��д��־λ
uint16				RecoverCommand;								//�߼���������
uint16				Password;									//����
uint16				BuzzerEnable;								//������ʹ��
uint16				WaveformAccuracy;							//���ξ���
uint16				WaveformID;									//����ͼ1
uint16				Waveform2ID;								//����ͼ2
uint16				HistogramID;								//��״ͼ
uint16				HistogramPageID;							//��״ͼҳ��
uint16				HistogramSetting;							//��״ͼ��ʾ
uint16				FaultRecordID;								//���Ͻ���ID
uint16				SlaveID,SlaveIDbak;							//�ӻ�����ID
uint16				CorrectionID,CorrectionIDbak;				//��������ID
uint16				PrimevalID,PrimevalIDbak;					//���ν���ID



uint16				APFStatus,CommunStatus,SlaveStatus[16];										//�豸����״̬��ͨѶ״̬
uint16				MainErrorCode,SlaveErrorCode[16];

uint16				DispMode,DispModeBak;														//�豸��ʾ����״̬
uint32				DailyEnergy,TotalRunningTime,TotalRunningTimeBak;							//���������,�ۼ�����ʱ��
uint16				DailyRunningTime;															//��������ʱ��
uint32				TotalEnergyHB,TotalEnergyLB,TotalEnergyBakHB,TotalEnergyBakLB;				//�ۼƽ�����

uint16				EepromYear,EepromMon,EepromDay,ResetCount;									//��¼����ʱ��,�ϵ���������

uint32				Co2HB;																		//������̼������
uint32				Co2LB;																		//������̼������


static uint8		FirstDispMode=0;
uint8				SystemStatus=0,SystemStatusBak=0,SlaveFaultFlag[16],SlaveFaultFlagBak[16],SystemRunningFlag =0;

static Commun		CommunWithDUGS;
static Fault 		FaultCode;
static Aisle		IntPassive;

uint16				InterCounter;
uint8 				BoostMode;



extern uint8		gz_dan[160];//����
extern uint8 		SlaveFault[16][100];//�ӻ�����
extern uint8 		Event_Logging[160];//�¼���¼

extern uint8		screen_mode;
//extern uint16 		OnOffDelayCount,FaultDelayCount,StartDelayCount,WorkTimeCount,BoostTimeCount;
extern uint8		Event_Code,RecordEventFlag,ACSwitchFlag,AlarmSwitchFlag;
extern uint16		VolOnOffEnable,CurOnOffEnable;
extern uint16		MainCTLocation,MainCTDirectionA,MainCTDirectionB,MainCTDirectionC,MainCTPhase,OutCTDirectionA,OutCTDirectionB,OutCTDirectionC,OutCTPhase/*,AidCTLocation,AidCTDirection,AidCTPhase*/;
extern uint16		Position[15],Group[15],Capacitance[15];
extern uint16		ActiveBal,ReactiveBal,HarmonicBal,ApparentBal;
extern uint16		IntPassive_aisle[32];
extern uint32 		IntPassiveState,IntPassiveFault,IntPassiveSwitch;
extern uint16		HighVolA,HighVolB,HighVolC;

uint16	DatalogStAddr[13]={1920,2016,2112,2208,2304,2400,2496,2592,2688,2784,2880,2976,3072};
extern uint16	Datalog[38],DatalogMax,DatalogStart;


void ClrRevFIFO(void)
{ 
	volatile uint32	i1,temp=0;
	for(i1=0; i1<8; i1++)
	{
		if((LPC_UART0->LSR&0x01)==0x01) temp=LPC_UART0->RBR;
        else break; 
    }
}

void ClrRevFIFO1(void)
{ 
	volatile uint32	i1,temp=0;
	for(i1=0; i1<8; i1++)
	{
     	if((LPC_UART1->LSR&0x01)==0x01) temp=LPC_UART1->RBR;
  		else break; 
 	}
}
void ClrRevFIFO3(void)
{ 
	volatile uint32	i1,temp=0;
	for(i1=0; i1<8; i1++)
	{
     	if((LPC_UART3->LSR&0x01)==0x01) temp=LPC_UART3->RBR;
  		else break; 
 	}
}


void UART0_Init(void)
{
 	uint16	Fdiv;
	//uint32 	Uart1Bps;
 	/*��PCONP������UART*/
 	LPC_SC->PCONP|=1<<3;         //ʹ��UART1
 	//LPC_UART1->IER =0;
 	/*��ʼ��֡��ʽ*/
 	LPC_UART0->LCR=0x83;		  //�շ�8λ�ַ�����  ʹ�ܷ��ʳ���������  1 ��ֹͣλ  ��ֹ��ż������У��
	/*switch(UART_BPS)
	{
		case 0:
			Uart1Bps = 9600;			//9600
			Delay35character =1;		// 4
			break;
		case 1:
			Uart1Bps = 19200;
			Delay35character =2;
			break;
		case 2:
			Uart1Bps = 38400;
			Delay35character =1;
			break;
		case 3:
			Uart1Bps = 9600;
			Delay35character =1;
			break;
		default :
			Uart1Bps = 9600;
			Delay35character =4;
			break;
	}*/
	Fdiv=(pclock/16)/9600; 	
	Delay35character =4;
	LPC_UART0->DLM=Fdiv/256;	  // �����ʷ�Ƶ���Ĵ���
 	LPC_UART0->DLL=Fdiv&255;	  //
 	//LPC_UART0->FDR = 0x45;//0x45;
 	LPC_UART0->LCR=0x03;		  //
 	//����FIFO							
 	LPC_UART0->FCR=0x87;         //ʹ��FIFO,������FIFO�������Ϊ8
 	//ʹ��uart0�ж�
	LPC_UART0->IER =1;
	NVIC_EnableIRQ(UART0_IRQn);
	//LocalAddr=Parameter[899];
 	ClrRevFIFO();
	uart_read0;
}

void UART1_Init(void)
{
 	
 	uint16	Fdiv;
	uint32 	UART1Bps;
 	/*��PCONP������UART*/
 	LPC_SC->PCONP|=1<<4;         //ʹ��UART1
 	//LPC_UART1->IER =0;
 	/*��ʼ��֡��ʽ*/
 	LPC_UART1->LCR=0x83;		  //�շ�8λ�ַ�����  ʹ�ܷ��ʳ���������  1 ��ֹͣλ  ��ֹ��ż������У��
	switch(UART_BPS)
	{
		case 4:
			UART1Bps = 1200;			//1200
			Delay35character =4;		// 4
			break;
		case 5:
			UART1Bps = 2400;
			Delay35character =4;
			break;		
		case 0:
			UART1Bps = 9600;			//9600
			Delay35character =4;		// 4
			break;
		case 1:
			UART1Bps = 19200;
			Delay35character =2;
			break;
		case 2:
			UART1Bps = 38400;
			Delay35character =1;
			break;
		case 3:
			UART1Bps = 115200;
			Delay35character =1;
			break;
		default :
			UART1Bps = 9600;
			Delay35character =4;
			break;
	}
	Fdiv=(pclock/16)/UART1Bps; 	
	LPC_UART1->DLM=Fdiv/256;	  // �����ʷ�Ƶ���Ĵ���
 	LPC_UART1->DLL=Fdiv&255;	  //
 	//LPC_UART1->FDR = 0x45;//0x45;
 	LPC_UART1->LCR=0x03;		  //
 	//����FIFO							
 	LPC_UART1->FCR=0x87;         //ʹ��FIFO,������FIFO�������Ϊ8
 	//ʹ��uart0�ж�
	LPC_UART1->IER =1;
	NVIC_EnableIRQ(UART1_IRQn);
	//LocalAddr=Parameter[899];
 	ClrRevFIFO();
	uart_read1;
}
void UART3_Init(void)
{

	uint16	Fdiv;
 	/*��PCONP������UART*/
 	LPC_SC->PCONP|=1<<25;        //ʹ��UART0
 	LPC_UART3->IER =0;
 	/*��ʼ��֡��ʽ*/
 	LPC_UART3->LCR=0x83;		  //�շ�8λ�ַ�����  ʹ�ܷ��ʳ���������  1 ��ֹͣλ  ��ֹ��ż������У��  
	Fdiv=(pclock/16)/115200;			//115200
	LPC_UART3->DLM=Fdiv/256;	  // �����ʷ�Ƶ���Ĵ���
 	LPC_UART3->DLL=Fdiv&255;	  //
 	//LPC_UART3->FDR = 0x45;//0x45;
 	LPC_UART3->LCR=0x03;		  //
 	//����FIFO
 	LPC_UART3->FCR=0x7;         //ʹ��FIFO,������FIFO�������Ϊ8
 	//ʹ��UART3�ж� 
 	//LPC_UART3->IER = 0x01;
 	NVIC_EnableIRQ(UART3_IRQn);
 	LPC_UART3->IER =1;
 	ClrRevFIFO3();
	uart_read3;
	
}

uint8_t UART0_GetByte(void)
{
 	uint8	rcv_dat;
 	uint32	n;
 	for(n=0; n<500;n++)//000; n++)
 	{
  		if((LPC_UART0->LSR&0x01)!=0){n=600000;rcv_dat=LPC_UART0->RBR;}
 	}
 	return(rcv_dat);
}

void UART0_SendByte(uint32_t adat)
{
 	uint32	m;
 	LPC_UART0->THR=adat;
 	for(m=0; m<5000000; m++)
 	{
  		if((LPC_UART0->LSR&0x40)!=0) m=6000000;
 	}
}

uint8_t UART1_GetByte(void)
{
 	uint8	rcv_dat;
 	uint32	n;
 	for(n=0; n<500;n++)//000; n++)
 	{
  		if((LPC_UART1->LSR&0x01)!=0)
			{
				rcv_dat=LPC_UART1->RBR;
				n=600000;
			}
 	}
 	return(rcv_dat);
}

void UART1_SendByte(uint32_t adat)
{
 	uint32	m;
	LPC_UART1->THR=adat;
	for(m=0; m<5000000; m++)
 	{
  		if((LPC_UART1->LSR&0x40)!=0) m=6000000;
 	}
}
uint8_t UART3_GetByte(void)
{
 	uint8	rcv_dat;
 	uint32	n;
 	for(n=0; n<500;n++)//000; n++)
 	{
  		if((LPC_UART3->LSR&0x01)!=0){n=600000;rcv_dat=LPC_UART3->RBR;}
 	}
 	return(rcv_dat);
}

void UART3_SendByte(uint32_t adat)
{
 	uint32	m;
 	LPC_UART3->THR=adat;
 	for(m=0; m<5000000; m++)
 	{
  		if((LPC_UART3->LSR&0x40)!=0) m=6000000;
 	}
}

/******************************************************************************
** ������: RecvDataProcess
** ��Ҫ����:���ݶ���485ͨѶЭ�飬��ȡ���ݺͲ��������ݵ�ַ�Ͳ�����ַ����Э����Ӧ��
	���ݳ����ݲ������ơ�
** ���ܣ�������λ��
******************************************************************************/
void RecvDataProcess(uint8 uart)
{
	uint8 ReceStartAddrH,ReceStartAddrL,ReceFunCode,SendBuff[1000],TmpDataHbyte,TmpDataLbyte,SendData,Eeptempbuff[200],temp[50],tmp8[1664];
	uint16 ReceDataLengh,ReceStartAddr,ReceRegisterLengh,ultmp,TmpDataWord;
	uint16 Crc_data,SendCount=0,i=0,j=0,k=0,l=0,tmp16;
	uint8 *SendPtr,ExceptionCode =0/*WrongFunCodeFlag=0,WrongStartaddrFlag=0,WrongDatalongFlag=0,WrongDatalimitFlag=0*/;
	uint8 addr,Tmp_DataLog[76],swj_ok=0;
	
	if(uart==Uart0)//����
	{
		ReceFunCode =RecvSciBuf[1];							// Function code
		
		ReceStartAddrH =RecvSciBuf[2];
		ReceStartAddrL =RecvSciBuf[3];

		ReceStartAddr =((ReceStartAddrH<<8)|ReceStartAddrL);   // Starting address

		ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
	}
	else//�ⲿͨѶ
	{
		ReceFunCode =RecvSci3Buf[1];							// Function code
		
		ReceStartAddrH =RecvSci3Buf[2];
		ReceStartAddrL =RecvSci3Buf[3];

		ReceStartAddr =((ReceStartAddrH<<8)|ReceStartAddrL);   // Starting address

		ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
	}
	//puyaotest3=1;
	
		//�ж���ǰ�Ĺ����룬03��Ҫ500ms��06��0x10��Ҫ1000ms�������ѯ
	
	if(ReceFunCode==3)
	{
		if(count_swj>4)
		{
			swj_ok=1;
			count_swj=0;
		}
		else swj_ok=0;
	}
	else if(ReceFunCode==4)
	{
		swj_ok=1;
	}
	else if ((ReceFunCode==6)||(ReceFunCode==16))
	{
		if(count_swj>18)
		{
			swj_ok=1;
			count_swj=0;
		}
		else swj_ok=0;
	}
	
	if(ReceStartAddr>=0x9000)swj_ok=1;

	if(swj_ok==1)//���ʱ������ͼ������²�ѯ
	{
		switch(ReceFunCode)
		{
		case 0x03://��ȡ����
			if(ReceStartAddr>0xFFF && ReceStartAddr<=HoldingEndAddr)    //0-700  Running information
			{
				//ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				
				if((ReceDataLengh<=HoldingRegisteNum)&&(ReceStartAddr+ReceDataLengh <=(HoldingRegisteNum+HoldingStartAddr)))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x1000;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(HoldingRegister[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(HoldingRegister[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x1FFF)&&(ReceStartAddr<=0x2009))    //0-700  Running information
			{
				ReceStartAddr= ReceStartAddr-0x2000;
				if(uart == Uart0)
				{
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=10)&&(ReceStartAddr+ReceDataLengh <=0x000A))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(Factory_parameter[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(Factory_parameter[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x8FFF)&&(ReceStartAddr<=0x9063))    //9000��������
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9064))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9000;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(main_parameter[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(main_parameter[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}		
			else if((ReceStartAddr>0x90FF)&&(ReceStartAddr<=0x9163))    //9100�ӻ�����
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9164))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9100;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[0][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[0][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x91FF)&&(ReceStartAddr<=0x9263))    //9200ARMcans1
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9264))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9200;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(ARM_param[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(ARM_param[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x92FF)&&(ReceStartAddr<=0x9363))    //9300����1
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9364))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9300;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(load_correctionA[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(load_correctionA[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x93FF)&&(ReceStartAddr<=0x9463))    //9400����2
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9464))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9400;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(load_correctionB[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(load_correctionB[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x94FF)&&(ReceStartAddr<=0x9563))    //9500����3
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9564))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9500;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(load_correctionC[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(load_correctionC[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x95FF)&&(ReceStartAddr<=0x9663))    //9600�ӻ�2
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9664))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9600;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[1][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[1][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x96FF)&&(ReceStartAddr<=0x9763))    //9700�ӻ�3
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9764))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9700;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[2][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[2][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x97FF)&&(ReceStartAddr<=0x9863))    //9800�ӻ�4
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9864))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9800;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[3][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[3][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x98FF)&&(ReceStartAddr<=0x9963))    //9900�ӻ�5
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9964))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9900;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[4][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[4][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x99FF)&&(ReceStartAddr<=0x9A63))    //9A00�ӻ�6
			{
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x9A64))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x9A00;
					i=0;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(slave_parameter[5][ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(slave_parameter[5][ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else
			{
				ExceptionCode =WrongStartaddr;
			}
			break;
		case 0x04://��ȡ����
			if(ReceStartAddr<=InputEndAddr)  //   protect parameters
			{
				//ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]); //  Lengh of Data
				
				if((ReceDataLengh<=InputRegisteNum)&&(ReceStartAddr+ReceDataLengh <=(InputRegisteNum+InputStartAddr))) // Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					for(j=0;j<(ReceDataLengh);j++)
					{
						 SendBuff[3+i] =(uint8)(InputRegister[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(InputRegister[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =ReceDataLengh +3;
					//puyaotest3=2;
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x1FFF && ReceStartAddr<=EventEndAddr)    //��������
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=EventRegisteNum)&&(ReceStartAddr+ReceDataLengh <=(EventRegisteNum+EventStartAddr)))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x2000;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =gz_dan[ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x2FFF && ReceStartAddr<=0x3064)    //�ӻ�1����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3065))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3000;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[0][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x30FF && ReceStartAddr<=0x3164)    //�ӻ�2����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3165))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3100;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[1][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x31FF && ReceStartAddr<=0x3264)    //�ӻ�3����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3265))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3200;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[2][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x32FF && ReceStartAddr<=0x3364)    //�ӻ�4����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3365))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3300;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[3][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x33FF && ReceStartAddr<=0x3464)    //�ӻ�5����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3465))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3400;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[4][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x34FF && ReceStartAddr<=0x3564)    //�ӻ�6����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3565))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3500;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[5][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x35FF && ReceStartAddr<=0x3664)    //�ӻ�7����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3665))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3600;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[6][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x36FF && ReceStartAddr<=0x3764)    //�ӻ�8����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3765))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3700;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[7][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x37FF && ReceStartAddr<=0x3864)    //�ӻ�9����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3865))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3800;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[8][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x38FF && ReceStartAddr<=0x3964)    //�ӻ�10����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3965))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3900;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[9][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x39FF && ReceStartAddr<=0x3A64)    //�ӻ�11����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3A65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3A00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[10][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3AFF && ReceStartAddr<=0x3B64)    //�ӻ�12����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3B65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3B00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[11][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3BFF && ReceStartAddr<=0x3C64)    //�ӻ�13����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3C65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3C00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[12][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3CFF && ReceStartAddr<=0x3D64)    //�ӻ�14����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3D65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3D00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[13][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3DFF && ReceStartAddr<=0x3E64)    //�ӻ�15����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3E65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3E00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[14][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3EFF && ReceStartAddr<=0x3F64)    //�ӻ�16����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3F65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3F00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[15][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else
			{
				ExceptionCode =WrongStartaddr;
			}
			break;
		case 0x06://д����
			if(ReceStartAddr>0xFFF &&ReceStartAddr<=0x10BF)//�ж���ʼ��ַ
			{
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				
				if(ReceStartAddr==0x1000)
				{
					if(TmpDataWord==0xAAAA)
					{
						OnOffCommand =0x01;
						RemotePowerOnOffFlag =1;
					}
					else if(TmpDataWord==0x5555)
					{	
						OnOffCommand =0x00;
						RemotePowerOnOffFlag =1;
					}
					else
					{
						ExceptionCode =WrongDatalimit;	//ָ�����
					}
						
				}
				else if(ReceStartAddr==0x1001||ReceStartAddr==0x1002||ReceStartAddr==0x1003)
				{
					ExceptionCode =WrongDatalimit;						//ָ�����
				}
				else if(ReceStartAddr==0x1004)
				{	
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Year = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart(); 
				}
				else if(ReceStartAddr==0x1005)
				{	
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Mon = TmpDataHbyte;
					local_time.RTC_Mday = TmpDataLbyte;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart(); 
				}
				else if(ReceStartAddr==0x1006)
				{	
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Hour= TmpDataHbyte;
					local_time.RTC_Min= TmpDataLbyte;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart(); 
				}
				else if(ReceStartAddr==0x1007)
				{	
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Sec = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart(); 
				}
				else if(ReceStartAddr==0x1008)
				{
					main_parameter[0]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1; 
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1009)
				{
					main_parameter[1]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100A)
				{
					main_parameter[2]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100B)
				{
					main_parameter[3]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100C)
				{
					main_parameter[4]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100D)
				{
					main_parameter[5]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100E)
				{
					main_parameter[6]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x100F)
				{
					main_parameter[7]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1010)
				{
					main_parameter[8]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1011)
				{
					main_parameter[9]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1012)
				{
					main_parameter[10]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1013)
				{
					main_parameter[11]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1014)
				{
					main_parameter[12]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1015)
				{
					main_parameter[13]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1016)
				{
					main_parameter[14]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1017)
				{
					main_parameter[15]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1018)
				{
					main_parameter[16]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x1019)
				{
					main_parameter[17]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr==0x101A)
				{
					main_parameter[20]= TmpDataWord;

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;													//��ȡI2C
				}
				else if(ReceStartAddr>=0x101B &&ReceStartAddr<=0x104C)
				{
					Selected_parameter[ReceStartAddr-0x101B] =TmpDataWord;

					j=0;
					for(i=0;i<50;i++)
					{
						if(Selected_parameter[i]==1)
						{
							main_parameter[31+j] =i+1;
							j++;
						}
						if(j>18)break;
					}
					main_parameter[30] =j;
					for(i=0;i<19-j;i++)
					{
						main_parameter[31+j+i] =0;
					}

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
					
				}
				else if(ReceStartAddr==0x104D)
				{
					LocalAddr 	 =TmpDataWord;
					UART1_Init();
					SaveARMParaProcess();
				}
				else if(ReceStartAddr==0x104E)
				{
					UART_BPS 	 =TmpDataWord;
					UART1_Init();
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x104F)
				{
					RemoteEnable 	 =TmpDataWord;
					UART1_Init();
					SaveARMParaProcess();	
				}
				
				else if(ReceStartAddr==0x1050)
				{
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Year = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart();              //����RTC	
				}
				else if(ReceStartAddr==0x1051)
				{
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Mon = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart();              //����RTC	
				}
				else if(ReceStartAddr==0x1052)
				{
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Mday = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart();              //����RTC	
				}
				else if(ReceStartAddr==0x1053)
				{
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Hour = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart();              //����RTC	
				}
				else if(ReceStartAddr==0x1054)
				{
					RTCStop();RTCInit();     //��ʼ��
					local_time.RTC_Min = TmpDataWord;
     				RTCSetTime( local_time );//����ʱ��
     				RTCStart();              //����RTC	
				}
				else if(ReceStartAddr==0x1055)
				{
					OnOffStatus 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1056)
				{
					AlarmTime1[0] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1057)
				{
					AlarmTime1[1] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1058)
				{
					AlarmTime1[2] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1059)
				{
					AlarmTime1[3] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105A)
				{
					AlarmTime1[4] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105B)
				{
					AlarmTime2[0] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105C)
				{
					AlarmTime2[1] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105D)
				{
					AlarmTime2[2] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105E)
				{
					AlarmTime2[3] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x105F)
				{
					AlarmTime2[4] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1060)
				{
					AlarmTime3[0] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1061)
				{
					AlarmTime3[1] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1062)
				{
					AlarmTime3[2] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1063)
				{
					AlarmTime3[3] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1064)
				{
					AlarmTime3[4] 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1065)
				{
					ntc_type 	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1066)
				{
//					AlarmTime4[1] 	 =TmpDataWord;
//					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1067)
				{
//					AlarmTime4[2] 	 =TmpDataWord;
//					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1068)
				{
//					AlarmTime4[3] 	 =TmpDataWord;
//					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1069)
				{
//					AlarmTime4[4] 	 =TmpDataWord;
//					SaveARMParaProcess();	
				}
				/*else if(ReceStartAddr==0x106A)
				{
					ResetCount	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x106B)
				{
					slave_Reset[0]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x106C)
				{
					slave_Reset[1]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x106D)
				{
					slave_Reset[2]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x106E)
				{
					slave_Reset[3]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x106F)
				{
					slave_Reset[4]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1070)
				{
					slave_Reset[5]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1071)
				{
					slave_Reset[6]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1072)
				{
					slave_Reset[7]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1073)
				{
					slave_Reset[8]	 =TmpDataWord;
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1074)
				{
					slave_Reset[9]	 =TmpDataWord;
					SaveARMParaProcess();	
				}*/
				else if(ReceStartAddr==0x1075)
				{
					tmp16=(uint16)(ProjectNo);
					ProjectNo=(uint32)((TmpDataWord<<16)|tmp16);//��Ŀ��
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1076)
				{
					tmp16=(uint16)(ProjectNo>>16);
					ProjectNo=(uint32)((tmp16<<16)|TmpDataWord);//��Ŀ��
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1077)
				{
					ProductionNo	=TmpDataWord;//��Ŀ��
					SaveARMParaProcess();	
				}
				else if(ReceStartAddr==0x1078 || ReceStartAddr==0x1079)
				{
					if(ReceStartAddr==0x1078)VolOnOffEnable	=TmpDataWord;
					else if(ReceStartAddr==0x1079)CurOnOffEnable=TmpDataWord;
					
					main_parameter[73] =VolOnOffEnable+CurOnOffEnable*10;
					SaveARMParaProcess();
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
				}
				else if(ReceStartAddr==0x107A || ReceStartAddr==0x107B || ReceStartAddr==0x107C||ReceStartAddr==0x107D || ReceStartAddr==0x107E)
				{
					if(ReceStartAddr==0x107A)MainCTLocation	=TmpDataWord;
					else if(ReceStartAddr==0x107B)MainCTDirectionA	=TmpDataWord;
					else if(ReceStartAddr==0x107C)MainCTDirectionB  =TmpDataWord;
					else if(ReceStartAddr==0x107D)MainCTDirectionC  =TmpDataWord;
					else if(ReceStartAddr==0x107E)MainCTPhase	=TmpDataWord;
					
					main_parameter[10] =MainCTLocation+MainCTDirectionA*10+MainCTDirectionB*100+MainCTDirectionB*1000+MainCTPhase*10000;
					SaveARMParaProcess();
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
				}
				
				else if(ReceStartAddr==0x10BC || ReceStartAddr==0x10BD || ReceStartAddr==0x10BE || ReceStartAddr==0x107F)
				{
					if(ReceStartAddr==0x10BC)OutCTDirectionA	=TmpDataWord;
					else if(ReceStartAddr==0x10BD)OutCTDirectionB	=TmpDataWord;
					else if(ReceStartAddr==0x10BE)OutCTDirectionC	=TmpDataWord;
					else if(ReceStartAddr==0x107F)OutCTPhase	=TmpDataWord;
					//main_parameter[14] =OutCTDirectionA*10+OutCTDirectionB*100+OutCTDirectionC*1000+OutCTPhase*10000;	
					main_parameter[15] =OutCTDirectionA*10+OutCTDirectionB*100+OutCTDirectionC*1000+OutCTPhase*10000;
					SaveARMParaProcess();
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
				}
				else if(ReceStartAddr==0x10BF)									//Զ�����ѹ��ܿ���
					Testtime_function =TmpDataWord;
				else if(ReceStartAddr>=0x1080 &&ReceStartAddr<=0x1094)
				{
					if(ReceStartAddr==0x1080)Position[0] =TmpDataWord;
					else if(ReceStartAddr==0x1081)Group[0] =TmpDataWord;
					else if(ReceStartAddr==0x1082)Capacitance[0] =TmpDataWord;
					else if(ReceStartAddr==0x1083)Position[1] =TmpDataWord;
					else if(ReceStartAddr==0x1084)Group[1] =TmpDataWord;
					else if(ReceStartAddr==0x1085)Capacitance[1] =TmpDataWord;
					else if(ReceStartAddr==0x1086)Position[2] =TmpDataWord;
					else if(ReceStartAddr==0x1087)Group[2] =TmpDataWord;
					else if(ReceStartAddr==0x1088)Capacitance[2] =TmpDataWord;
					else if(ReceStartAddr==0x1089)Position[3] =TmpDataWord;
					else if(ReceStartAddr==0x108A)Group[3] =TmpDataWord;
					else if(ReceStartAddr==0x108B)Capacitance[3] =TmpDataWord;
					else if(ReceStartAddr==0x108C)Position[4] =TmpDataWord;
					else if(ReceStartAddr==0x108D)Group[4] =TmpDataWord;
					else if(ReceStartAddr==0x108E)Capacitance[4] =TmpDataWord;
					else if(ReceStartAddr==0x108F)Position[5] =TmpDataWord;
					else if(ReceStartAddr==0x1090)Group[5] =TmpDataWord;
					else if(ReceStartAddr==0x1091)Capacitance[5] =TmpDataWord;
					else if(ReceStartAddr==0x1092)Position[6] =TmpDataWord;
					else if(ReceStartAddr==0x1093)Group[6] =TmpDataWord;
					else if(ReceStartAddr==0x1094)Capacitance[6] =TmpDataWord;
					else if(ReceStartAddr==0x1095)Position[7] =TmpDataWord;
					else if(ReceStartAddr==0x1096)Group[7] =TmpDataWord;
					else if(ReceStartAddr==0x1097)Capacitance[7] =TmpDataWord;
					else if(ReceStartAddr==0x1098)Position[8] =TmpDataWord;
					else if(ReceStartAddr==0x1099)Group[8] =TmpDataWord;
					else if(ReceStartAddr==0x109A)Capacitance[8] =TmpDataWord;
					
					CabinProcess();
					SaveARMParaProcess();
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;													//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
					j=0;				
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(Passive_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(Passive_parameter[i]);
					}
					I2C_write;															//д��I2C
					WriteEeprom(EepPassiveAddr,Eeptempbuff,200);   				//����������д����ʽ������
					I2C_read;
				}
				else if(ReceStartAddr>=0x109B &&ReceStartAddr<0x10BB )
				{
					if(TmpDataWord !=1)TmpDataWord=0;
					
					if(ReceStartAddr==0x109B)ManualPassive_aisle[0] =TmpDataWord;
					else if(ReceStartAddr==0x109C)ManualPassive_aisle[1] =TmpDataWord;
					else if(ReceStartAddr==0x109D)ManualPassive_aisle[2] =TmpDataWord;
					else if(ReceStartAddr==0x109E)ManualPassive_aisle[3] =TmpDataWord;
					else if(ReceStartAddr==0x109F)ManualPassive_aisle[4] =TmpDataWord;
					else if(ReceStartAddr==0x10A0)ManualPassive_aisle[5] =TmpDataWord;
					else if(ReceStartAddr==0x10A1)ManualPassive_aisle[6] =TmpDataWord;
					else if(ReceStartAddr==0x10A2)ManualPassive_aisle[7] =TmpDataWord;
					else if(ReceStartAddr==0x10A3)ManualPassive_aisle[8] =TmpDataWord;
					else if(ReceStartAddr==0x10A4)ManualPassive_aisle[9] =TmpDataWord;
					else if(ReceStartAddr==0x10A5)ManualPassive_aisle[10] =TmpDataWord;
					else if(ReceStartAddr==0x10A6)ManualPassive_aisle[11] =TmpDataWord;
					else if(ReceStartAddr==0x10A7)ManualPassive_aisle[12] =TmpDataWord;
					else if(ReceStartAddr==0x10A8)ManualPassive_aisle[13] =TmpDataWord;
					else if(ReceStartAddr==0x10A9)ManualPassive_aisle[14] =TmpDataWord;
					else if(ReceStartAddr==0x10AA)ManualPassive_aisle[15] =TmpDataWord;
					else if(ReceStartAddr==0x10AB)ManualPassive_aisle[16] =TmpDataWord;
					else if(ReceStartAddr==0x10AC)ManualPassive_aisle[17] =TmpDataWord;
					else if(ReceStartAddr==0x10AD)ManualPassive_aisle[18] =TmpDataWord;
					else if(ReceStartAddr==0x10AE)ManualPassive_aisle[19] =TmpDataWord;
					else if(ReceStartAddr==0x10AF)ManualPassive_aisle[20] =TmpDataWord;
					else if(ReceStartAddr==0x10B0)ManualPassive_aisle[21] =TmpDataWord;
					else if(ReceStartAddr==0x10B1)ManualPassive_aisle[22] =TmpDataWord;
					else if(ReceStartAddr==0x10B2)ManualPassive_aisle[23] =TmpDataWord;
					else if(ReceStartAddr==0x10B3)ManualPassive_aisle[24] =TmpDataWord;
					else if(ReceStartAddr==0x10B4)ManualPassive_aisle[25] =TmpDataWord;
					else if(ReceStartAddr==0x10B5)ManualPassive_aisle[26] =TmpDataWord;
					else if(ReceStartAddr==0x10B6)ManualPassive_aisle[27] =TmpDataWord;
					else if(ReceStartAddr==0x10B7)ManualPassive_aisle[28] =TmpDataWord;
					else if(ReceStartAddr==0x10B8)ManualPassive_aisle[29] =TmpDataWord;
					else if(ReceStartAddr==0x10B9)ManualPassive_aisle[30] =TmpDataWord;
					else if(ReceStartAddr==0x10BA)ManualPassive_aisle[31] =TmpDataWord;
//					ManualPassiveSwitch =0;
//					for(i=0;i<32;i++)
//					{		
//						ManualPassiveSwitch |= ManualPassive_aisle[i]<<i;
//					}
						lanya_flag=1;
				}
				else if(ReceStartAddr==0x10BB )
				{
					main_parameter[70]= TmpDataWord;			//Ӧ��Ϊ70

					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
						
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
						
					I2C_write;													//д��I2C
					WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
					I2C_read;
					
				}
			}
			else if(ReceStartAddr>0x10BF &&ReceStartAddr<0x10FF )
			{
				ExceptionCode =WrongDatalimit;						//ָ�����
			}
			else if(ReceStartAddr ==0x10FF)
			{
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				Remote_Password = TmpDataWord;
			}
			else if(ReceStartAddr>=0x1100 &&ReceStartAddr<=0x1158 )	//159B
			{
				if(Remote_Password==7777)
				{
					if(uart==Uart0)
					{
						TmpDataHbyte = RecvSciBuf[4];
						TmpDataLbyte = RecvSciBuf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}
					else
					{	
						TmpDataHbyte = RecvSci3Buf[4];
						TmpDataLbyte = RecvSci3Buf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}

					if(ReceStartAddr==0x1100)
					{
						main_parameter[18]= TmpDataWord;
					}
					else if(ReceStartAddr==0x1101)
					{
						main_parameter[19]= TmpDataWord;
					}
					else if(ReceStartAddr==0x1102)
					{
						main_parameter[21]= TmpDataWord;
					}
					else if(ReceStartAddr==0x1133)					//�������ܻ�ֱ��ģʽ
					{
						SetupMode= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1151)					//��������������
					{
						xiuzhen_dianliujibian= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1152)					//2-50�η�ֵ����
					{
						xiuzhen_fuzhi= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1153)					//pf����
					{
						xiuzhen_pf= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1154)					//cos������
					{
						xiuzhen_cos= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1155)					//���������Чֵ����
					{
						xiuzhen_shuchu= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1156)					//�޹���������
					{
						xiuzhen_wugong= TmpDataWord;
						SaveARMParaProcess();
					}
					else if(ReceStartAddr==0x1157)					//���书������
					{
						xiuzhen_jibian= TmpDataWord;
						SaveARMParaProcess();
					}			
					else if(ReceStartAddr==0x1158)					//��ѹ����������
					{
						xiuzhen_dianyajibian= TmpDataWord;
						SaveARMParaProcess();
					}
					else 
					{
						if(ReceStartAddr==0x110B&&TmpDataWord>18);
						
						else{
						
						main_parameter[ReceStartAddr-0x10ED]= TmpDataWord;
					
						j=main_parameter[30];
						for(i=0;i<19-j;i++)
						{
							main_parameter[31+j+i]=0;
						}
				  	
			   	}
				 }
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
					j=0;
					
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(main_parameter[i]);
					}
					
					I2C_write;															//д��I2C
				   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   					//����������д����ʽ������
					I2C_read;
				}
				else
				{
					ExceptionCode =WrongDatalimit;										//ָ�����
				}
			}
			else if(ReceStartAddr>=0x1159 &&ReceStartAddr<=0x11B4 )
			{
				ExceptionCode =WrongDatalimit;											//ָ�����
			}
			else if(ReceStartAddr>=0x11B5 &&ReceStartAddr<=0x159C )
			{
				if(Remote_Password==8888)
				{
					if(uart==Uart0)
					{
						TmpDataHbyte = RecvSciBuf[4];
						TmpDataLbyte = RecvSciBuf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}
					else
					{	
						TmpDataHbyte = RecvSci3Buf[4];
						TmpDataLbyte = RecvSci3Buf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}
					
					if(ReceStartAddr>=0x11B5 &&ReceStartAddr<=0x1218 )
					{
						slave_parameter[0][ReceStartAddr-0x11B5]= TmpDataWord;
						WriteSlaveParameterFlag =1;
					}
					else if(ReceStartAddr>=0x1219 &&ReceStartAddr<=0x127C )
					{
						slave_parameter[1][ReceStartAddr-0x1219]= TmpDataWord;
						WriteSlaveParameter2Flag =1;
					}
					else if(ReceStartAddr>=0x127D &&ReceStartAddr<=0x12E0 )
					{
						slave_parameter[2][ReceStartAddr-0x127D]= TmpDataWord;
						WriteSlaveParameter3Flag =1;
					}
					else if(ReceStartAddr>=0x12E1 &&ReceStartAddr<=0x1344 )
					{
						slave_parameter[3][ReceStartAddr-0x12E1]= TmpDataWord;
						WriteSlaveParameter4Flag =1;
					}
					else if(ReceStartAddr>=0x1345 &&ReceStartAddr<=0x13A8 )
					{
						slave_parameter[4][ReceStartAddr-0x1345]= TmpDataWord;
						WriteSlaveParameter5Flag =1;
					}
					else if(ReceStartAddr>=0x13A9 &&ReceStartAddr<=0x140C )
					{
						slave_parameter[5][ReceStartAddr-0x13A9]= TmpDataWord;
						WriteSlaveParameter6Flag =1;
					}
					else if(ReceStartAddr>=0x140D &&ReceStartAddr<=0x1470 )
					{
						slave_parameter[6][ReceStartAddr-0x140D]= TmpDataWord;
						WriteSlaveParameter7Flag =1;
					}
					else if(ReceStartAddr>=0x1471 &&ReceStartAddr<=0x14D4 )
					{
						slave_parameter[7][ReceStartAddr-0x1471]= TmpDataWord;
						WriteSlaveParameter8Flag =1;
					}
					else if(ReceStartAddr>=0x14D5 &&ReceStartAddr<=0x127C )
					{
						slave_parameter[8][ReceStartAddr-0x14D5]= TmpDataWord;
						WriteSlaveParameter9Flag =1;
					}
					else if(ReceStartAddr>=0x1539 &&ReceStartAddr<=0x159C )
					{
						slave_parameter[9][ReceStartAddr-0x1539]= TmpDataWord;
						WriteSlaveParameter10Flag =1;
					}
				}
				else
				{
					ExceptionCode =WrongDatalimit;											//ָ�����
				}
			}
			else if(ReceStartAddr>=0x159D &&ReceStartAddr<=0x16C8 )							//��������
			{
				if(Remote_Password==9999)
				{
					if(uart==Uart0)
					{
						TmpDataHbyte = RecvSciBuf[4];
						TmpDataLbyte = RecvSciBuf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}
					else
					{	
						TmpDataHbyte = RecvSci3Buf[4];
						TmpDataLbyte = RecvSci3Buf[5];
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					}
					
					if(ReceStartAddr>=0x159D &&ReceStartAddr<=0x1600 )						//�������� A
					{
						load_correctionA[ReceStartAddr-0x159D]= TmpDataWord;

						j=0;				
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
						}
						
						j=200;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
						}	
						j=400;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
						}

						I2C_write;
					   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C
					}
					else if(ReceStartAddr>=0x1601 &&ReceStartAddr<=0x1664 )					//�������� B
					{
						load_correctionB[ReceStartAddr-0x1601]= TmpDataWord;

						j=0;				
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
						}
						
						j=200;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
						}	
						j=400;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
						}

						I2C_write;
					   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C
					}
					else if(ReceStartAddr>=0x1665 &&ReceStartAddr<=0x16C8 )					//�������� C
					{
						load_correctionC[ReceStartAddr-0x1665]= TmpDataWord;

						j=0;				
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
						}
						
						j=200;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
						}	
						j=400;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
						}

						I2C_write;
					   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C
					}
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag =1;
					WriteCorrectionParameter2Flag =1;
					WriteCorrectionParameter3Flag =1;
					WritePassiveParameterFlag =1;
				}
				else
				{
					ExceptionCode =WrongDatalimit;											//ָ�����
				}
			}
			else if(ReceStartAddr>=0x16C9 &&ReceStartAddr<0x172D )							//��Դ����
			{
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				
				Passive_parameter[ReceStartAddr-0x16C9]= TmpDataWord;

				j=0;				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(Passive_parameter[i]>>8);
					Eeptempbuff[j++] =(uint8)(Passive_parameter[i]);
				}
					
				I2C_write;															//д��I2C
				WriteEeprom(EepPassiveAddr,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if((ReceStartAddr>0x2010)&&(ReceStartAddr<=0x2016))   //�߼�����
			{
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				
				RestoreCommand=0;
				
				if (ReceStartAddr==0x2010)
				{
					if (TmpDataWord==6666)
					RestoreCommand=1;
				}
				else if (ReceStartAddr==0x2011)
				{
					if (TmpDataWord==6666)
					RestoreCommand=2;
				}
				else if (ReceStartAddr==0x2012)
				{
					if (TmpDataWord==6666)
					RestoreCommand=3;
				}
				else if (ReceStartAddr==0x2013)
				{
					if (TmpDataWord==6666)
					RestoreCommand=4;
				}
				else if (ReceStartAddr==0x2014)
				{
					if (TmpDataWord==6666)
					RestoreCommand=5;
				}
				else if (ReceStartAddr==0x2015)
				{
					if (TmpDataWord==6666)
					RestoreCommand=6;
				}
				else if (ReceStartAddr==0x2016)
				{
					if (TmpDataWord==6666)
					OnekeyClean=1;
				}
									
				
			}
			else if((ReceStartAddr>0x1FFF)&&(ReceStartAddr<=0x2009))	 //0-700  Running information
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x2000;
				if(ReceStartAddr==0x0000)
				{
					FactoryEnable =TmpDataWord;
					if(FactoryEnable !=0x0001)
					{
						FactoryEnable =0x00;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0005&&FactoryEnable==0x0001 )
				{
					LocalAddr =TmpDataWord;
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0006&&FactoryEnable==0x0001 )
				{
					
					UART_BPS =TmpDataWord;
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0007&&FactoryEnable==0x0001 )
				{
					TotalEnergyHB =0; 
					TotalEnergyLB =0;
					TotalRunningTime =0;
					
					DailyEnergy =0;
					DailyRunningTime =0;
						
					EepromYear =2015;
					EepromMon =1;
					EepromDay =1;
			
					TotalEnergyBakHB =0;
					TotalEnergyBakLB =0;
					TotalRunningTimeBak =0;
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
					I2C_write;
					WriteEeprom(EepParameterAddr+1568,temp,24);
					I2C_read;
			
			
					SystemInitSuccess =1;
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0008&&FactoryEnable==0x0001 )
				{
					ChangeStatus(ClearFaults);
					for(i=0;i<FaultDataNumMax;i++){gz_dan[i]=0;}											//���ϼ�¼����
					I2C_write;//aWP
					WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);
					I2C_read;//aWP
					for(i=0;i<10;i++)
					{
						for(j=0;j<100;j++)
						{
							SlaveFault[i][j]=0; 															//�ӻ���������
						}
					}
					for(i=0;i<SlaveFaultNumMax;i++)
					{
						tmp8[i] =0;
					}
					I2C_write;//aWP
					WriteEeprom(EepSlaveFaultAddr,tmp8,SlaveFaultNumMax);
					I2C_read;//aWP
					for(i=0;i<EventLogNumMax;i++)
					{
						tmp8[i] =0;
					}
					I2C_write;//aWP
					WriteEeprom(EepEventLogAddr,tmp8,EventLogNumMax);
					I2C_read;//aWP
					ClearFaultSuccess =1;	
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0009&&FactoryEnable==0x0001 )
				{
					//if(SystemStatus==SystemPfault)															//ϵͳ����
					{
						/*for(i=0;i<main_parameter[99];i++)
						{
							slave_Enable[i] =22;
							main_parameter[89+i] =slave_Enable[i];
						}
						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[i++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[i++] =(uint8)(main_parameter[i]);
						}
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr,Eeptempbuff,200);				//����������д����ʽ������
						I2C_read;	
						WriteMainParameterFlag =1;
			
						ResetCount=50000;
						slave_Reset[0] =50000;
						slave_Reset[1] =50000;
						slave_Reset[2] =50000;
						slave_Reset[3] =50000;
						slave_Reset[4] =50000;
						slave_Reset[5] =50000;
						slave_Reset[6] =50000;
						slave_Reset[7] =50000;
						slave_Reset[8] =50000;
						slave_Reset[9] =50000;
									
									
						Init_parameter[1048] =(uint8)(ResetCount>>8);
						Init_parameter[1049] =(uint8)(ResetCount);
												
						Init_parameter[1050] =(uint8)(slave_Reset[0]>>8);
						Init_parameter[1051] =(uint8)(slave_Reset[0]);
						Init_parameter[1052] =(uint8)(slave_Reset[1]>>8);
						Init_parameter[1053] =(uint8)(slave_Reset[1]);
						Init_parameter[1054] =(uint8)(slave_Reset[2]>>8);
						Init_parameter[1055] =(uint8)(slave_Reset[2]);
						Init_parameter[1056] =(uint8)(slave_Reset[3]>>8);
						Init_parameter[1057] =(uint8)(slave_Reset[3]);
						Init_parameter[1058] =(uint8)(slave_Reset[4]>>8);
						Init_parameter[1059] =(uint8)(slave_Reset[4]);
						Init_parameter[1060] =(uint8)(slave_Reset[5]>>8);
						Init_parameter[1061] =(uint8)(slave_Reset[5]);
						Init_parameter[1062] =(uint8)(slave_Reset[6]>>8);
						Init_parameter[1063] =(uint8)(slave_Reset[6]);
						Init_parameter[1064] =(uint8)(slave_Reset[7]>>8);
						Init_parameter[1065] =(uint8)(slave_Reset[7]);
						Init_parameter[1066] =(uint8)(slave_Reset[8]>>8);
						Init_parameter[1067] =(uint8)(slave_Reset[8]);
						Init_parameter[1068] =(uint8)(slave_Reset[9]>>8);
						Init_parameter[1069] =(uint8)(slave_Reset[9]);
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr+1048,Init_parameter+1048,8);			//����������д����ʽ������
						I2C_read;	
						
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr+1056,Init_parameter+1056,14);			//����������д����ʽ������
						I2C_read;

						//Reset();
						DUGSReadMainParameterFlag=1;
						SystemResetSuccess =1;
						IllegalOperation =0;*/
									 
						SpiComm_Disable;										//	ϵͳǿ������
						SystemResetSuccess =1;
					}
					/*else 
					{
						SpiComm_Disable;
					}*/
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if((ReceStartAddr>0x8FFF)&&(ReceStartAddr<=0x9063))	 //9000-9063 ��������
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9000;
				
				main_parameter[ReceStartAddr]= TmpDataWord;
				WriteMainParameterFlag=1;
				j=0;
				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
					Eeptempbuff[j++] =(uint8)(main_parameter[i]);
				}
				
				I2C_write;													//д��I2C
					WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
				I2C_read;			
				
			}	
			else if((ReceStartAddr>0x90FF)&&(ReceStartAddr<=0x9163))	 //9100-9163 �ӻ�����
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9100;				
				slave_parameter[0][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameterFlag =1;				
			}		
			else if((ReceStartAddr>0x91FF)&&(ReceStartAddr<=0x9263))	 //9200-9263 ��Դ����
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9200;				
				ARM_param[ReceStartAddr]= TmpDataWord;

				ARM_proess();//����ARM����
			}			
			else if((ReceStartAddr>0x92FF)&&(ReceStartAddr<=0x9363))	 //9300 ��������1
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
					
				ReceStartAddr=ReceStartAddr-0x9300;				
				load_correctionA[ReceStartAddr]= TmpDataWord;

				j=0;				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
					Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
				}
					
				I2C_write;															//д��I2C
				WriteEeprom(EepCorrectionAddr,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;					
			}
			else if((ReceStartAddr>0x93FF)&&(ReceStartAddr<=0x9463))	 //9400 ��������2
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
					
				ReceStartAddr=ReceStartAddr-0x9400;				
				load_correctionB[ReceStartAddr]= TmpDataWord;

				j=0;				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
					Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
				}
					
				I2C_write;															//д��I2C
				WriteEeprom(EepCorrectionAddr+200,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;					
			}
			else if((ReceStartAddr>0x94FF)&&(ReceStartAddr<=0x9563))	 //9500 ��������3
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
					
				ReceStartAddr=ReceStartAddr-0x9500;				
				load_correctionC[ReceStartAddr]= TmpDataWord;

				j=0;				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
					Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
				}
					
				I2C_write;															//д��I2C
				WriteEeprom(EepCorrectionAddr+400,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;					
			}
			else if((ReceStartAddr>0x95FF)&&(ReceStartAddr<=0x9663))	 //9600 �ӻ�����2
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9600;				
				slave_parameter[1][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameter2Flag =1;				
			}
			else if((ReceStartAddr>0x96FF)&&(ReceStartAddr<=0x9763))	 //9700 �ӻ�����3
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9700;				
				slave_parameter[2][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameter3Flag =1;				
			}
			else if((ReceStartAddr>0x97FF)&&(ReceStartAddr<=0x9863))	 //9800 �ӻ�����4
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9800;				
				slave_parameter[3][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameter4Flag =1;				
			}
			else if((ReceStartAddr>0x98FF)&&(ReceStartAddr<=0x9963))	 //9900 �ӻ�����5
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9900;				
				slave_parameter[4][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameter5Flag =1;				
			}
			else if((ReceStartAddr>0x99FF)&&(ReceStartAddr<=0x9a63))	 //9a00 �ӻ�����6
			{	
				if(uart==Uart0)
				{
					TmpDataHbyte = RecvSciBuf[4];
					TmpDataLbyte = RecvSciBuf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				else
				{	
					TmpDataHbyte = RecvSci3Buf[4];
					TmpDataLbyte = RecvSci3Buf[5];
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
				}
				ReceStartAddr=ReceStartAddr-0x9a00;				
				slave_parameter[5][ReceStartAddr]= TmpDataWord;
				WriteSlaveParameter6Flag =1;				
			}
			else
			{
				ExceptionCode =WrongStartaddr;
			}
			DUGSReadMainParameterFlag =1;
			DUGSReadSelectedtimesFlag =1;
			DUGSReadCorrectionParameterFlag =1;
			DUGSReadCorrectionParameter2Flag =1;
			DUGSReadCorrectionParameter3Flag =1;
			
			SendBuff[0] = LocalAddr;
			SendBuff[1] = ReceFunCode;

			if(uart==Uart0)
			{
				SendBuff[2] = RecvSciBuf[2];
				SendBuff[3] = RecvSciBuf[3];
				SendBuff[4] = RecvSciBuf[4];
				SendBuff[5] = RecvSciBuf[5];
			}
			else
			{
				SendBuff[2] = RecvSci3Buf[2];
				SendBuff[3] = RecvSci3Buf[3];
				SendBuff[4] = RecvSci3Buf[4];
				SendBuff[5] = RecvSci3Buf[5];
			}
				
			SendCount =6;

			break;
		case 0x10://����д�������
			if(ReceStartAddr>0xFFF &&ReceStartAddr<0x1100)
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;
					if(ultmp==0x1000)
					{
						if(TmpDataWord==0xAAAA)
						{
							OnOffCommand =0x01;
							RemotePowerOnOffFlag =1;
						}
						else if(TmpDataWord==0x5555)
						{	
							OnOffCommand =0x00;
							RemotePowerOnOffFlag =1;
						}
						else
						{
							ExceptionCode =WrongDatalimit;						//ָ�����
						}
					}
					else if(ultmp==0x1001||ultmp==0x1002||ultmp==0x1003)
					{
						ExceptionCode =WrongDatalimit;						//ָ�����
					}
					else if(ultmp==0x1004)
					{	
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Year = TmpDataWord;
     					RTCSetTime( local_time );//����ʱ��
     					RTCStart(); 
					}
					else if(ultmp==0x1005)
					{	
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Mon = TmpDataHbyte;
						local_time.RTC_Mday = TmpDataLbyte;
     					RTCSetTime( local_time );//����ʱ��
     					RTCStart(); 
					}
					else if(ultmp==0x1006)
					{	
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Hour= TmpDataHbyte;
						local_time.RTC_Min= TmpDataLbyte;
     					RTCSetTime( local_time );//����ʱ��
     					RTCStart(); 
					}
					else if(ultmp==0x1007)
					{	
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Sec = TmpDataWord;
     					RTCSetTime( local_time );//����ʱ��
     					RTCStart(); 
					}
					else if(ultmp==0x1008)
					{
						main_parameter[0]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
					
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
					
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1009)
					{
						main_parameter[1]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100A)
					{
						main_parameter[2]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100B)
					{
						main_parameter[3]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100C)
					{
						main_parameter[4]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100D)
					{
						main_parameter[5]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100E)
					{
						main_parameter[6]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x100F)
					{
						main_parameter[7]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1010)
					{
						main_parameter[8]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1011)
					{
						main_parameter[9]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1012)
					{
						main_parameter[10]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1013)
					{
						main_parameter[11]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1014)
					{
						main_parameter[12]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1015)
					{
						main_parameter[13]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1016)
					{
						main_parameter[14]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1017)
					{
						main_parameter[15]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1018)
					{
						main_parameter[16]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x1019)
					{
						main_parameter[17]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp==0x101A)
					{
						main_parameter[20]= TmpDataWord;

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
					else if(ultmp>=0x101B &&ultmp<=0x104C)
					{
						Selected_parameter[ultmp-0x101B] =TmpDataWord;

						j=0;
						for(i=0;i<50;i++)
						{
							if(Selected_parameter[i]==1)
							{
								main_parameter[31+j] =i+1;
								j++;
							}
							if(j>18)break;
						}
						main_parameter[30] =j;
						for(i=0;i<19-j;i++)
						{
							main_parameter[31+j+i] =0;
						}

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
						
					}
					else if(ultmp==0x104D)
					{
						LocalAddr 	 =TmpDataWord;
						UART1_Init();
						SaveARMParaProcess();
					}
					else if(ultmp==0x104E)
					{
						UART_BPS 	 =TmpDataWord;
						UART1_Init();
						SaveARMParaProcess();	
					}
					else if(ultmp==0x104F)
					{
						RemoteEnable 	 =TmpDataWord;
						UART1_Init();
						SaveARMParaProcess();	
					}
					
					else if(ultmp==0x1050)
					{
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Year = TmpDataWord;
	     				RTCSetTime( local_time );//����ʱ��
	     				RTCStart();              //����RTC	
					}
					else if(ultmp==0x1051)
					{
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Mon = TmpDataWord;
	     				RTCSetTime( local_time );//����ʱ��
	     				RTCStart();              //����RTC	
					}
					else if(ultmp==0x1052)
					{
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Mday = TmpDataWord;
	     				RTCSetTime( local_time );//����ʱ��
	     				RTCStart();              //����RTC	
					}
					else if(ultmp==0x1053)
					{
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Hour = TmpDataWord;
	     				RTCSetTime( local_time );//����ʱ��
	     				RTCStart();              //����RTC	
					}
					else if(ultmp==0x1054)
					{
						RTCStop();RTCInit();     //��ʼ��
						local_time.RTC_Min = TmpDataWord;
	     				RTCSetTime( local_time );//����ʱ��
	     				RTCStart();              //����RTC	
					}
					else if(ultmp==0x1055)
					{
						OnOffStatus 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1056)
					{
						AlarmTime1[0] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1057)
					{
						AlarmTime1[1] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1058)
					{
						AlarmTime1[2] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1059)
					{
						AlarmTime1[3] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105A)
					{
						AlarmTime1[4] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105B)
					{
						AlarmTime2[0] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105C)
					{
						AlarmTime2[1] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105D)
					{
						AlarmTime2[2] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105E)
					{
						AlarmTime2[3] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x105F)
					{
						AlarmTime2[4] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1060)
					{
						AlarmTime3[0] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1061)
					{
						AlarmTime3[1] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1062)
					{
						AlarmTime3[2] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1063)
					{
						AlarmTime3[3] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1064)
					{
						AlarmTime3[4] 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1065)
					{
						ntc_type 	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1066)
					{
//						AlarmTime4[1] 	 =TmpDataWord;
//						SaveARMParaProcess();	
					}
					else if(ultmp==0x1067)
					{
//						AlarmTime4[2] 	 =TmpDataWord;
//						SaveARMParaProcess();	
					}
					else if(ultmp==0x1068)
					{
//						AlarmTime4[3] 	 =TmpDataWord;
//						SaveARMParaProcess();	
					}
					else if(ultmp==0x1069)
					{
//						AlarmTime4[4] 	 =TmpDataWord;
//						SaveARMParaProcess();	
					}
					/*else if(ultmp==0x106A)
					{
						ResetCount	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x106B)
					{
						slave_Reset[0]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x106C)
					{
						slave_Reset[1]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x106D)
					{
						slave_Reset[2]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x106E)
					{
						slave_Reset[3]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x106F)
					{
						slave_Reset[4]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1070)
					{
						slave_Reset[5]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ReceStartAddr==0x1071)
					{
						slave_Reset[6]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1072)
					{
						slave_Reset[7]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1073)
					{
						slave_Reset[8]	 =TmpDataWord;
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1074)
					{
						slave_Reset[9]	 =TmpDataWord;
						SaveARMParaProcess();	
					}*/
					else if(ultmp==0x1075)
					{
						tmp16=(uint16)(ProjectNo);
						ProjectNo=(uint32)((TmpDataWord<<16)|tmp16);//��Ŀ��
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1076)
					{
						tmp16=(uint16)(ProjectNo>>16);
						ProjectNo=(uint32)((tmp16<<16)|TmpDataWord);//��Ŀ��
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1077)
					{
						ProductionNo	=TmpDataWord;//��Ŀ��
						SaveARMParaProcess();	
					}
					else if(ultmp==0x1078 || ultmp==0x1079)
					{
						if(ultmp==0x1078)VolOnOffEnable	=TmpDataWord;
						else if(ultmp==0x1079)CurOnOffEnable=TmpDataWord;
						
						main_parameter[73] =VolOnOffEnable+CurOnOffEnable*10;
						SaveARMParaProcess();
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
					}
					else if(ultmp==0x107A || ultmp==0x107B || ultmp==0x107C||ultmp==0x107D || ultmp==0x107E)
					{
						if(ultmp==0x107A)MainCTLocation	=TmpDataWord;
						else if(ultmp==0x107B)MainCTDirectionA	=TmpDataWord;
						else if(ultmp==0x107C)MainCTDirectionB  =TmpDataWord;
						else if(ultmp==0x107D)MainCTDirectionC  =TmpDataWord;
						else if(ultmp==0x107E)MainCTPhase	=TmpDataWord;
						
						main_parameter[10] =MainCTLocation+MainCTDirectionA*10+MainCTDirectionB*100+MainCTDirectionB*1000+MainCTPhase*10000;
						SaveARMParaProcess();
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
					}
					
					else if(ultmp==0x10AC || ultmp==0x10AD || ultmp==0x10AE || ultmp==0x107F)
					{
						if(ultmp==0x10AC)OutCTDirectionA	=TmpDataWord;
						else if(ultmp==0x10AD)OutCTDirectionB	=TmpDataWord;
						else if(ultmp==0x10AE)OutCTDirectionC	=TmpDataWord;
						else if(ultmp==0x107F)OutCTPhase	=TmpDataWord;
						main_parameter[15] =OutCTDirectionA*10+OutCTDirectionB*100+OutCTDirectionC*1000+OutCTPhase*10000;
						SaveARMParaProcess();
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
					}
					else if(ultmp>=0x1080 &&ultmp<=0x1094)
					{
						if(ultmp==0x1080)Position[0] =TmpDataWord;
						else if(ultmp==0x1081)Group[0] =TmpDataWord;
						else if(ultmp==0x1082)Capacitance[0] =TmpDataWord;
						else if(ultmp==0x1083)Position[1] =TmpDataWord;
						else if(ultmp==0x1084)Group[1] =TmpDataWord;
						else if(ultmp==0x1085)Capacitance[1] =TmpDataWord;
						else if(ultmp==0x1086)Position[2] =TmpDataWord;
						else if(ultmp==0x1087)Group[2] =TmpDataWord;
						else if(ultmp==0x1088)Capacitance[2] =TmpDataWord;
						else if(ultmp==0x1089)Position[3] =TmpDataWord;
						else if(ultmp==0x108A)Group[3] =TmpDataWord;
						else if(ultmp==0x108B)Capacitance[3] =TmpDataWord;
						else if(ultmp==0x108C)Position[4] =TmpDataWord;
						else if(ultmp==0x108D)Group[4] =TmpDataWord;
						else if(ultmp==0x108E)Capacitance[4] =TmpDataWord;
						else if(ultmp==0x108F)Position[5] =TmpDataWord;
						else if(ultmp==0x1090)Group[5] =TmpDataWord;
						else if(ultmp==0x1091)Capacitance[5] =TmpDataWord;
						else if(ultmp==0x1092)Position[6] =TmpDataWord;
						else if(ultmp==0x1093)Group[6] =TmpDataWord;
						else if(ultmp==0x1094)Capacitance[6] =TmpDataWord;
						else if(ultmp==0x1095)Position[7] =TmpDataWord;
						else if(ultmp==0x1096)Group[7] =TmpDataWord;
						else if(ultmp==0x1097)Capacitance[7] =TmpDataWord;
						else if(ultmp==0x1098)Position[8] =TmpDataWord;
						else if(ultmp==0x1099)Group[8] =TmpDataWord;
						else if(ultmp==0x109A)Capacitance[8] =TmpDataWord;
						CabinProcess();
						SaveARMParaProcess();
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
						j=0;				
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(Passive_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(Passive_parameter[i]);
						}
						I2C_write;															//д��I2C
						WriteEeprom(EepPassiveAddr,Eeptempbuff,200);   				//����������д����ʽ������
						I2C_read;
					}
					else if(ultmp>=0x109B &&ultmp<0x10BB )
					{
						if(TmpDataWord !=0)TmpDataWord=1;
						
						if(ultmp==0x109B)ManualPassive_aisle[0] =TmpDataWord;
						else if(ultmp==0x109C)ManualPassive_aisle[1] =TmpDataWord;
						else if(ultmp==0x109D)ManualPassive_aisle[2] =TmpDataWord;
						else if(ultmp==0x109E)ManualPassive_aisle[3] =TmpDataWord;
						else if(ultmp==0x109F)ManualPassive_aisle[4] =TmpDataWord;
						else if(ultmp==0x10A0)ManualPassive_aisle[5] =TmpDataWord;
						else if(ultmp==0x10A1)ManualPassive_aisle[6] =TmpDataWord;
						else if(ultmp==0x10A2)ManualPassive_aisle[7] =TmpDataWord;
						else if(ultmp==0x10A3)ManualPassive_aisle[8] =TmpDataWord;
						else if(ultmp==0x10A4)ManualPassive_aisle[9] =TmpDataWord;
						else if(ultmp==0x10A5)ManualPassive_aisle[10] =TmpDataWord;
						else if(ultmp==0x10A6)ManualPassive_aisle[11] =TmpDataWord;
						else if(ultmp==0x10A7)ManualPassive_aisle[12] =TmpDataWord;
						else if(ultmp==0x10A8)ManualPassive_aisle[13] =TmpDataWord;
						else if(ultmp==0x10A9)ManualPassive_aisle[14] =TmpDataWord;
						else if(ultmp==0x10AA)ManualPassive_aisle[15] =TmpDataWord;
						else if(ultmp==0x10AB)ManualPassive_aisle[16] =TmpDataWord;
						else if(ultmp==0x10AC)ManualPassive_aisle[17] =TmpDataWord;
						else if(ultmp==0x10AD)ManualPassive_aisle[18] =TmpDataWord;
						else if(ultmp==0x10AE)ManualPassive_aisle[19] =TmpDataWord;
						else if(ultmp==0x10AF)ManualPassive_aisle[20] =TmpDataWord;
						else if(ultmp==0x10B0)ManualPassive_aisle[21] =TmpDataWord;
						else if(ultmp==0x10B1)ManualPassive_aisle[22] =TmpDataWord;
						else if(ultmp==0x10B2)ManualPassive_aisle[23] =TmpDataWord;
						else if(ultmp==0x10B3)ManualPassive_aisle[24] =TmpDataWord;
						else if(ultmp==0x10B4)ManualPassive_aisle[25] =TmpDataWord;
						else if(ultmp==0x10B5)ManualPassive_aisle[26] =TmpDataWord;
						else if(ultmp==0x10B6)ManualPassive_aisle[27] =TmpDataWord;
						else if(ultmp==0x10B7)ManualPassive_aisle[28] =TmpDataWord;
						else if(ultmp==0x10B8)ManualPassive_aisle[29] =TmpDataWord;
						else if(ultmp==0x10B9)ManualPassive_aisle[30] =TmpDataWord;
						else if(ultmp==0x10BA)ManualPassive_aisle[31] =TmpDataWord;
						ManualPassiveSwitch =0;
						for(i=0;i<32;i++)
						{		
							ManualPassiveSwitch |= ManualPassive_aisle[i]<<i;
						}
					}
					else if(ultmp==0x10BB )
					{
						main_parameter[70]= TmpDataWord;//�ĳ�70

						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
						j=0;
							
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
							
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;
						
					}
					else if(ultmp ==0x10FF)
					{
						Remote_Password = TmpDataWord;
					}
				
				}
			}
			else if(ReceStartAddr>=0x1100 &&ReceStartAddr<=0x1150 )	//159B
			{
				if(Remote_Password==7777)
				{
					if(uart== Uart0)
					{
						ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
						ReceDataLengh = RecvSciBuf[6];
					}
					else
					{
						ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
						ReceDataLengh = RecvSci3Buf[6];
					}

					k=0;
					for(l=0;l<ReceRegisterLengh;l++)
					{
						if(uart== Uart0)
						{
							TmpDataHbyte = RecvSciBuf[7+k];
							TmpDataLbyte = RecvSciBuf[8+k];
						}
						else
						{
							TmpDataHbyte = RecvSci3Buf[7+k];
							TmpDataLbyte = RecvSci3Buf[8+k];
						}
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
						k =k+2;
						ultmp =ReceStartAddr+l;	
						if(ultmp==0x1100)
						{
							main_parameter[18]= TmpDataWord;
						}
						else if(ultmp==0x1101)
						{
							main_parameter[19]= TmpDataWord;
						}
						else if(ultmp==0x1102)
						{
							main_parameter[21]= TmpDataWord;
						}
						else 
						{
							main_parameter[ultmp-0x10ED]= TmpDataWord;
						  j=0;
						if(main_parameter[0x001E]>18)
								main_parameter[0x001E]=0;
					     j=main_parameter[0x001E];
						for(i=0;i<19-j;i++)
						{
							main_parameter[0x001F+j+i]=0;
						}							
						}
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						j=0;
						
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;															//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   					//����������д����ʽ������
						I2C_read;
					}
				}
				else
				{
					ExceptionCode =WrongDatalimit;										//ָ�����
				}
			}
			else if(ReceStartAddr>=0x11B5 &&ReceStartAddr<=0x159C )
			{
				if(Remote_Password==8888)
				{
					
					if(uart== Uart0)
					{
						ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
						ReceDataLengh = RecvSciBuf[6];
					}
					else
					{
						ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
						ReceDataLengh = RecvSci3Buf[6];
					}

					k=0;
					for(l=0;l<ReceRegisterLengh;l++)
					{
						if(uart== Uart0)
						{
							TmpDataHbyte = RecvSciBuf[7+k];
							TmpDataLbyte = RecvSciBuf[8+k];
						}
						else
						{
							TmpDataHbyte = RecvSci3Buf[7+k];
							TmpDataLbyte = RecvSci3Buf[8+k];
						}
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
						k =k+2;
						ultmp =ReceStartAddr+l;				
						if(ultmp>=0x11B5 &&ultmp<=0x1218 )
						{
							slave_parameter[0][ultmp-0x11B5]= TmpDataWord;
							WriteSlaveParameterFlag =1;
						}
						else if(ultmp>=0x1219 &&ultmp<=0x127C )
						{
							slave_parameter[1][ultmp-0x1219]= TmpDataWord;
							WriteSlaveParameter2Flag =1;
						}
						else if(ultmp>=0x127D &&ultmp<=0x12E0 )
						{
							slave_parameter[2][ultmp-0x127D]= TmpDataWord;
							WriteSlaveParameter3Flag =1;
						}
						else if(ultmp>=0x12E1 &&ultmp<=0x1344 )
						{
							slave_parameter[3][ultmp-0x12E1]= TmpDataWord;
							WriteSlaveParameter4Flag =1;
						}
						else if(ultmp>=0x1345 &&ultmp<=0x13A8 )
						{
							slave_parameter[4][ultmp-0x1345]= TmpDataWord;
							WriteSlaveParameter5Flag =1;
						}
						else if(ultmp>=0x13A9 &&ultmp<=0x140C )
						{
							slave_parameter[5][ultmp-0x13A9]= TmpDataWord;
							WriteSlaveParameter6Flag =1;
						}
						else if(ultmp>=0x140D &&ultmp<=0x1470 )
						{
							slave_parameter[6][ultmp-0x140D]= TmpDataWord;
							WriteSlaveParameter7Flag =1;
						}
						else if(ultmp>=0x1471 &&ultmp<=0x14D4 )
						{
							slave_parameter[7][ultmp-0x1471]= TmpDataWord;
							WriteSlaveParameter8Flag =1;
						}
						else if(ultmp>=0x14D5 &&ultmp<=0x127C )
						{
							slave_parameter[8][ultmp-0x14D5]= TmpDataWord;
							WriteSlaveParameter9Flag =1;
						}
						else if(ultmp>=0x1539 &&ultmp<=0x159C )
						{
							slave_parameter[9][ultmp-0x1539]= TmpDataWord;
							WriteSlaveParameter10Flag =1;
						}
					}
				}
				else
				{
					ExceptionCode =WrongDatalimit;											//ָ�����
				}
			}
			else if(ReceStartAddr>=0x159D &&ReceStartAddr<=0x16C8 )							//��������
			{
				if(Remote_Password==9999)
				{
					if(uart== Uart0)
					{
						ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
						ReceDataLengh = RecvSciBuf[6];
					}
					else
					{
						ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
						ReceDataLengh = RecvSci3Buf[6];
					}

					k=0;
					for(l=0;l<ReceRegisterLengh;l++)
					{
						if(uart== Uart0)
						{
							TmpDataHbyte = RecvSciBuf[7+k];
							TmpDataLbyte = RecvSciBuf[8+k];
						}
						else
						{
							TmpDataHbyte = RecvSci3Buf[7+k];
							TmpDataLbyte = RecvSci3Buf[8+k];
						}
						TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
						k =k+2;
						ultmp =ReceStartAddr+l;	
					
						if(ultmp>=0x159D &&ultmp<=0x1600 )						//�������� A
						{
							load_correctionA[ultmp-0x159D]= TmpDataWord;

							j=0;				
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
							}
							
							j=200;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
							}	
							j=400;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
							}

							I2C_write;
						   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
							I2C_read;														//��ȡI2C
						}
						else if(ultmp>=0x1601 &&ultmp<=0x1664 )					//�������� B
						{
							load_correctionB[ultmp-0x1601]= TmpDataWord;

							j=0;				
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
							}
							
							j=200;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
							}	
							j=400;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
							}

							I2C_write;
						   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
							I2C_read;														//��ȡI2C
						}
						else if(ultmp>=0x1665 &&ultmp<=0x16C8 )					//�������� C
						{
							load_correctionC[ultmp-0x1665]= TmpDataWord;

							j=0;				
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
							}
							
							j=200;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
							}	
							j=400;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
							}

							I2C_write;
						   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,600);  				//����������д����ʽ������
							I2C_read;														//��ȡI2C
						}
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag =1;
						WriteCorrectionParameter2Flag =1;
						WriteCorrectionParameter3Flag =1;
					}
				}
				else
				{
					ExceptionCode =WrongDatalimit;											//ָ�����
				}
			}
			else if(ReceStartAddr>=0x16C9 &&ReceStartAddr<=0x172B )							//��Դ����
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					Passive_parameter[ultmp-0x16C9]= TmpDataWord;

					j=0;				
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(Passive_parameter[i]>>8);
						Eeptempbuff[j++] =(uint8)(Passive_parameter[i]);
					}
				}
				I2C_write;															//д��I2C
				WriteEeprom(EepPassiveAddr,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x8FFF &&ReceStartAddr<=0x9063 )							//9000��������
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					main_parameter[ultmp-0x9000]= TmpDataWord;
					
				}
				
				j=0;				
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
					Eeptempbuff[j++] =(uint8)(main_parameter[i]);
				}
				I2C_write;															//д��I2C
				WriteEeprom(EepParameterAddr,Eeptempbuff,200);   				//����������д����ʽ������
				I2C_read;
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x90FF &&ReceStartAddr<=0x9163 )							//9100�ӻ�����
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[0][ultmp-0x9100]= TmpDataWord;					
				}
				WriteSlaveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x91FF &&ReceStartAddr<=0x9263 )							//9200ARM����
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					ARM_param[ultmp-0x9200]= TmpDataWord;					
				}
				ARM_proess();
			}
			else if(ReceStartAddr>=0x92FF &&ReceStartAddr<=0x9363 )							//9300��������1
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}			
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					load_correctionA[ultmp-0x9300]= TmpDataWord;								
				}
				j=0;				
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
						Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
					}		
				I2C_write;
					WriteEeprom(EepCorrectionAddr,Eeptempbuff,200);  				//����������д����ʽ������
				I2C_read;														//��ȡI2C
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x93FF &&ReceStartAddr<=0x9463 )							//9400��������2
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}			
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					load_correctionB[ultmp-0x9400]= TmpDataWord;								
				}
				j=0;				
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
						Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
					}		
				I2C_write;
					WriteEeprom(EepCorrectionAddr+200,Eeptempbuff,200);  				//����������д����ʽ������
				I2C_read;														//��ȡI2C
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x94FF &&ReceStartAddr<=0x9563 )							//9500��������3
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}			
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					load_correctionC[ultmp-0x9500]= TmpDataWord;								
				}
				
				j=0;				
					for(i=0;i<100;i++)
					{
						Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
						Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
					}		
				I2C_write;
					WriteEeprom(EepCorrectionAddr+400,Eeptempbuff,200);  				//����������д����ʽ������
				I2C_read;														//��ȡI2C
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag =1;
				WriteCorrectionParameter2Flag =1;
				WriteCorrectionParameter3Flag =1;
				WritePassiveParameterFlag =1;
			}
			else if(ReceStartAddr>=0x95FF &&ReceStartAddr<=0x9663 )							//9600�ӻ�����2
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[1][ultmp-0x9600]= TmpDataWord;
					WriteSlaveParameter2Flag =1;
				}
			}
			else if(ReceStartAddr>=0x96FF &&ReceStartAddr<=0x9763 )							//9700�ӻ�����3
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[2][ultmp-0x9700]= TmpDataWord;
					WriteSlaveParameter3Flag =1;
				}
			}
			else if(ReceStartAddr>=0x97FF &&ReceStartAddr<=0x9863 )							//9800�ӻ�����4
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[3][ultmp-0x9800]= TmpDataWord;
					WriteSlaveParameter3Flag =1;
				}
			}
			else if(ReceStartAddr>=0x98FF &&ReceStartAddr<=0x9963 )							//9900�ӻ�����5
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[4][ultmp-0x9900]= TmpDataWord;
					WriteSlaveParameter3Flag =1;
				}
			}
			else if(ReceStartAddr>=0x99FF &&ReceStartAddr<=0x9A63 )							//9A00�ӻ�����6
			{
				if(uart== Uart0)
				{
					ReceRegisterLengh  =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					ReceDataLengh = RecvSciBuf[6];
				}
				else
				{
					ReceRegisterLengh  =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					ReceDataLengh = RecvSci3Buf[6];
				}

				k=0;
				for(l=0;l<ReceRegisterLengh;l++)
				{
					if(uart== Uart0)
					{
						TmpDataHbyte = RecvSciBuf[7+k];
						TmpDataLbyte = RecvSciBuf[8+k];
					}
					else
					{
						TmpDataHbyte = RecvSci3Buf[7+k];
						TmpDataLbyte = RecvSci3Buf[8+k];
					}
					TmpDataWord = (uint16)((TmpDataHbyte<<8)|TmpDataLbyte);
					k =k+2;
					ultmp =ReceStartAddr+l;	
					slave_parameter[5][ultmp-0x9A00]= TmpDataWord;
					WriteSlaveParameter3Flag =1;
				}
			}
			else
			{
				ExceptionCode =WrongStartaddr;
			}
			SendBuff[0] = LocalAddr;
			SendBuff[1] = ReceFunCode;
			if(uart==Uart0)
			{
				SendBuff[2] = RecvSciBuf[2];
				SendBuff[3] = RecvSciBuf[3];
				SendBuff[4] = RecvSciBuf[4];
				SendBuff[5] = RecvSciBuf[5];
			}
			else
			{
				SendBuff[2] = RecvSci3Buf[2];
				SendBuff[3] = RecvSci3Buf[3];
				SendBuff[4] = RecvSci3Buf[4];
				SendBuff[5] = RecvSci3Buf[5];
			}	
			
			SendCount =6;
				
			DUGSReadMainParameterFlag =1;
			DUGSReadSelectedtimesFlag =1;
			DUGSReadCorrectionParameterFlag =1;
			DUGSReadCorrectionParameter2Flag =1;
			DUGSReadCorrectionParameter3Flag =1;
			break;
		case 0x6A:
			if(ReceStartAddr>0x1FFF && ReceStartAddr<=EventEndAddr)    //��������
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=EventRegisteNum)&&(ReceStartAddr+ReceDataLengh <=(EventRegisteNum+EventStartAddr)))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x2000;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =gz_dan[ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x2FFF && ReceStartAddr<=0x3064)    //�ӻ�1����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3065))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3000;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[0][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x30FF && ReceStartAddr<=0x3164)    //�ӻ�2����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3165))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3100;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[1][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x31FF && ReceStartAddr<=0x3264)    //�ӻ�3����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3265))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3200;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[2][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x32FF && ReceStartAddr<=0x3364)    //�ӻ�4����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3365))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3300;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[3][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x33FF && ReceStartAddr<=0x3464)    //�ӻ�5����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3465))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3400;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[4][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x34FF && ReceStartAddr<=0x3564)    //�ӻ�6����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3565))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3500;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[5][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x35FF && ReceStartAddr<=0x3664)    //�ӻ�7����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3665))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3600;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[6][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x36FF && ReceStartAddr<=0x3764)    //�ӻ�8����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3765))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3700;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[7][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x37FF && ReceStartAddr<=0x3864)    //�ӻ�9����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3865))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3800;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[8][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x38FF && ReceStartAddr<=0x3964)    //�ӻ�10����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3965))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3900;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[9][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x39FF && ReceStartAddr<=0x3A64)    //�ӻ�11����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3A65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3A00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[10][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3AFF && ReceStartAddr<=0x3B64)    //�ӻ�12����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3B65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3B00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[11][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3BFF && ReceStartAddr<=0x3C64)    //�ӻ�13����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3C65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3C00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[12][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3CFF && ReceStartAddr<=0x3D64)    //�ӻ�14����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3D65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3D00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[13][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3DFF && ReceStartAddr<=0x3E64)    //�ӻ�15����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3E65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3E00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[14][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else if(ReceStartAddr>0x3EFF && ReceStartAddr<=0x3F64)    //�ӻ�15����
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=100)&&(ReceStartAddr+ReceDataLengh <=0x3F65))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					ReceStartAddr = ReceStartAddr-0x3F00;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+i] =0x00;
						i++;
						SendBuff[3+i] =SlaveFault[15][ReceStartAddr+j];
						i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			else
			{
				ExceptionCode =WrongStartaddr;
			}
			break;
		case 0x6B:
			if(ReceStartAddr>0 && ReceStartAddr<=0xFFF)    //���ݼ�¼
			{
				if(uart== Uart0)
				{	
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=38))   //Starting address + Lengh of Data
				{
					if(ReceStartAddr>13)
						ReceStartAddr=ReceStartAddr%13;
					if(ReceStartAddr==0)ReceStartAddr =13;
					if(DatalogMax<13)
					{
						if(ReceStartAddr>DatalogMax) 
						{
							addr =13-(ReceStartAddr-DatalogMax);
						}
						else addr =DatalogMax-HistogramID;
							
						ReadEeprom(DatalogStAddr[addr],Tmp_DataLog,EepDatalogNum);
					}
					else if(DatalogMax==13)
					{
						if(ReceStartAddr<=DatalogStart)
						{
							addr =DatalogStart-ReceStartAddr;
						}
						else if(ReceStartAddr>DatalogStart)
						{
							addr =13-(ReceStartAddr-DatalogStart);
						}

						ReadEeprom(DatalogStAddr[addr],Tmp_DataLog,EepDatalogNum);
					}
					
					ReceDataLengh = ReceDataLengh <<1;
					//ReceStartAddr = ReceStartAddr-0x4000;
					for(j=0;j<ReceDataLengh;j++)
					{
						SendBuff[3+j] =Tmp_DataLog[j];

					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}
			break;
		case 0x67:
			if(ReceStartAddr<=0x0009)    //0-700  Running information
			{
				if(uart == Uart0)
				{
					ReceDataLengh =((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
				}
				else
				{
					ReceDataLengh =((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
				}
				
				if((ReceDataLengh<=10)&&(ReceStartAddr+ReceDataLengh <=0x000A))   //Starting address + Lengh of Data
				{
					ReceDataLengh = ReceDataLengh <<1;
					for(j=0;j<ReceDataLengh;j++)
					{
						 SendBuff[3+i] =(uint8)(Factory_parameter[ReceStartAddr+j]>>8);
						 i++;
						 SendBuff[3+i] =(uint8)(Factory_parameter[ReceStartAddr+j]);
						 i++;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = ReceDataLengh;
					
					SendCount =SendBuff[2] +3;
					
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}

			else
			{
				ExceptionCode =WrongStartaddr;
			}
			break;
		case 0x68:
			if(ReceStartAddr<=0x0009)    //0-700  Running information
			{	
				if(ReceStartAddr==0x0000)
				{
					if(uart == Uart0)
					{
						FactoryEnable=((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					}
					else
					{
						FactoryEnable=((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					}
					if(FactoryEnable !=0x0001)
					{
						FactoryEnable =0x00;
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0005&&FactoryEnable==0x0001 )
				{
					if(uart == Uart0)
					{
						LocalAddr=((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					}
					else
					{
						LocalAddr=((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0006&&FactoryEnable==0x0001 )
				{
					if(uart == Uart0)
					{
						UART_BPS=((RecvSciBuf[4]<<8)|RecvSciBuf[5]);
					}
					else
					{
						UART_BPS=((RecvSci3Buf[4]<<8)|RecvSci3Buf[5]);
					}
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0007&&FactoryEnable==0x0001 )
				{
					TotalEnergyHB =0; 
					TotalEnergyLB =0;
					TotalRunningTime =0;
					
					DailyEnergy =0;
					DailyRunningTime =0;
					
					EepromYear =2015;
					EepromMon =1;
					EepromDay =1;

					TotalEnergyBakHB =0;
					TotalEnergyBakLB =0;
					TotalRunningTimeBak =0;
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
					I2C_write;
					WriteEeprom(EepParameterAddr+1568,temp,24);
					I2C_read;

					SystemInitSuccess =1;
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0008&&FactoryEnable==0x0001 )
				{
					ChangeStatus(ClearFaults);
					for(i=0;i<FaultDataNumMax;i++){gz_dan[i]=0;}											//���ϼ�¼����
					I2C_write;//aWP
		  			WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);
		  			I2C_read;//aWP
					for(i=0;i<10;i++)
					{
						for(j=0;j<100;j++)
						{
							SlaveFault[i][j]=0;																//�ӻ���������
						}
					}
					for(i=0;i<SlaveFaultNumMax;i++)
					{
						tmp8[i] =0;
					}
		  			I2C_write;//aWP
		  			WriteEeprom(EepSlaveFaultAddr,tmp8,SlaveFaultNumMax);
		  			I2C_read;//aWP
					for(i=0;i<EventLogNumMax;i++)
					{
						tmp8[i] =0;
					}
		  			I2C_write;//aWP
		  			WriteEeprom(EepEventLogAddr,tmp8,EventLogNumMax);
		  			I2C_read;//aWP
		  			ClearFaultSuccess =1;	
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else if(ReceStartAddr==0x0009&&FactoryEnable==0x0001 )
				{
					//if(SystemStatus==SystemPfault)															//ϵͳ����
					{
						/*for(i=0;i<main_parameter[99];i++)
						{
							slave_Enable[i] =22;
							main_parameter[89+i] =slave_Enable[i];
						}
						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[i++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[i++] =(uint8)(main_parameter[i]);
						}
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;	
						WriteMainParameterFlag =1;

						ResetCount=50000;
						slave_Reset[0] =50000;
						slave_Reset[1] =50000;
						slave_Reset[2] =50000;
						slave_Reset[3] =50000;
						slave_Reset[4] =50000;
						slave_Reset[5] =50000;
						slave_Reset[6] =50000;
						slave_Reset[7] =50000;
						slave_Reset[8] =50000;
						slave_Reset[9] =50000;
						
						
						Init_parameter[1048] =(uint8)(ResetCount>>8);
						Init_parameter[1049] =(uint8)(ResetCount);
									
						Init_parameter[1050] =(uint8)(slave_Reset[0]>>8);
						Init_parameter[1051] =(uint8)(slave_Reset[0]);
						Init_parameter[1052] =(uint8)(slave_Reset[1]>>8);
						Init_parameter[1053] =(uint8)(slave_Reset[1]);
						Init_parameter[1054] =(uint8)(slave_Reset[2]>>8);
						Init_parameter[1055] =(uint8)(slave_Reset[2]);
						Init_parameter[1056] =(uint8)(slave_Reset[3]>>8);
						Init_parameter[1057] =(uint8)(slave_Reset[3]);
						Init_parameter[1058] =(uint8)(slave_Reset[4]>>8);
						Init_parameter[1059] =(uint8)(slave_Reset[4]);
						Init_parameter[1060] =(uint8)(slave_Reset[5]>>8);
						Init_parameter[1061] =(uint8)(slave_Reset[5]);
						Init_parameter[1062] =(uint8)(slave_Reset[6]>>8);
						Init_parameter[1063] =(uint8)(slave_Reset[6]);
						Init_parameter[1064] =(uint8)(slave_Reset[7]>>8);
						Init_parameter[1065] =(uint8)(slave_Reset[7]);
						Init_parameter[1066] =(uint8)(slave_Reset[8]>>8);
						Init_parameter[1067] =(uint8)(slave_Reset[8]);
						Init_parameter[1068] =(uint8)(slave_Reset[9]>>8);
						Init_parameter[1069] =(uint8)(slave_Reset[9]);
						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr+1048,Init_parameter+1048,8);   			//����������д����ʽ������
						I2C_read;	

						I2C_write;													//д��I2C
						WriteEeprom(EepParameterAddr+1056,Init_parameter+1056,14);			//����������д����ʽ������
						I2C_read;
						//Reset();
						DUGSReadMainParameterFlag=1;
						SystemResetSuccess =1;
						IllegalOperation =0;*/
						 
						SpiComm_Disable;										//  ϵͳǿ������
					}
					/*else 
					{
						SpiComm_Disable;
					}*/
					SendBuff[0] = LocalAddr;
					SendBuff[1] = ReceFunCode;
					SendBuff[2] = 0x02;
					SendBuff[3] = 0xAA;
					SendBuff[4] = 0xAA;
					SendCount =SendBuff[2] +3;
				}
				else
				{
					ExceptionCode =WrongDatalong;
				}
			}

			else
			{
				ExceptionCode =WrongStartaddr;
			}
			break;
		default:
			ExceptionCode =WrongFunCode;
			break;

	}
	if(ExceptionCode !=0)
	{
		SendBuff[0] = LocalAddr;
		SendBuff[1] = ReceFunCode+0x80;
		SendBuff[2] = ExceptionCode;
		SendCount =3;
	}
	Crc_data = crc_chk(SendBuff,SendCount);
	SendBuff[SendCount] = Crc_data & 0xff;
	SendBuff[SendCount+1] = Crc_data >> 8;
	SendCount = SendCount+2;
	//puyaotest3 =3;
	if(uart==Uart0)
	{
		uart_write0;  //UART0 send
		SendPtr =SendBuff;
		for(i=SendCount;i>0;i--)
		{
			SendData=*SendPtr;UART0_SendByte(SendData);SendPtr++;
		}
		uart_read0;  //UART0 receive
	}
	else
	{
		uart_write1;  //UART0 send
		SendPtr =SendBuff;
		for(i=SendCount;i>0;i--)
		{
			SendData=*SendPtr;UART1_SendByte(SendData);SendPtr++;
		}
		uart_read1;  //UART0 receive
	}
	
	if (RestoreCommand!=0)				RestoreProcess();	
		
	}


	//SendDGUSSuccessTest++;
	//puyaotest3 =4;
}
/******************************************************************************
** ������: UARTRecvProcess
**
** ���ܣ�������λ��
******************************************************************************/
void ReadyData(void)
{
	//static uint8 tempmoni  =0;
	uint8  tmp8[8];
	uint16 i,tmp16,tmp17,tmp18;
	uint32 tmp32,tmp31;	
	int32 tmp15;
	//uint64 tmp64;
/****************************Table0********************************************/
	InputRegister[0] = LocalDevice;
	InputRegister[1] = PreviousDevice;
	InputRegister[2] = AfterDevice;
	InputRegister[3] = ProtocolVersion;
	tmp32 =DSP1Version;
	InputRegister[4] = 0x00;
	InputRegister[5] = 0x00;
	InputRegister[6] = (uint16)(tmp32>>16);
	InputRegister[7] = (uint16)(tmp32&0xFFFF);
	tmp32 =DSP2Version;
	InputRegister[8] = 0x00;
	InputRegister[9] = 0x00;
	InputRegister[10] = (uint16)(tmp32>>16);
	InputRegister[11] = (uint16)(tmp32&0xFFFF);
	tmp32 =ARMVersion;
	InputRegister[12] = 0x00;
	InputRegister[13] = 0x00;
	InputRegister[14] = (uint16)(tmp32>>16);
	InputRegister[15] = (uint16)(tmp32&0xFFFF);
	tmp32 =DUGSVersion;
	InputRegister[16] = 0x00;
	InputRegister[17] = 0x00;
	InputRegister[18] = (uint16)(tmp32>>16);
	InputRegister[19] = (uint16)(tmp32&0xFFFF);

	//tmp32 =DSPFlashVersion;
	InputRegister[20] = 0xFFFF;
	InputRegister[21] = 0xFFFF;
	InputRegister[22] = 0xFFFF;
	InputRegister[23] = 0xFFFF;
	
	tmp32 =EepromVersion;
	InputRegister[24] = 0x00;
	InputRegister[25] = 0x00;
	InputRegister[26] = (uint16)(tmp32>>16);
	InputRegister[27] = (uint16)(tmp32&0xFFFF);

	HEX2BCD(SerialNumber,tmp8,8);
	InputRegister[28] = (uint16)(tmp8[0]<<8|tmp8[1]);
	InputRegister[29] = (uint16)(tmp8[2]<<8|tmp8[3]);
	InputRegister[30] = (uint16)(tmp8[4]<<8|tmp8[5]);
	InputRegister[31] = (uint16)(tmp8[6]<<8|tmp8[7]);
	tmp8[0]=' ';
	tmp8[1]=' ';
	InputRegister[32] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]=' ';
	tmp8[1]=' ';
	InputRegister[33] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]=' ';
	tmp8[1]=' ';
	InputRegister[34] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]=' ';
	tmp8[1]='A';
	InputRegister[35] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='C';
	tmp8[1]='T';
	InputRegister[36] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='S';
	tmp8[1]='i';
	InputRegister[37] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='n';
	tmp8[1]='e';
	InputRegister[38] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='-';
	tmp8[1]='M';
	InputRegister[39] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='O';
	tmp8[1]='-';
	InputRegister[40] = (uint16)(tmp8[0]<<16|tmp8[1]);
	tmp8[0]='6';
	tmp8[1]='0';
	InputRegister[41] = (uint16)(tmp8[0]<<16|tmp8[1]);

	InputRegister[42] = Table1StartAddr;
	InputRegister[43] = Table1RegisterNum;
	InputRegister[44] = Table1Funcode;
	
	InputRegister[45] = Table2StartAddr;
	InputRegister[46] = Table2RegisterNum;
	InputRegister[47] = Table2Funcode;
	
	InputRegister[48] = Table3StartAddr;
	InputRegister[49] = Table3RegisterNum;
	InputRegister[50] = Table3Funcode;
	
	InputRegister[51] = Table4StartAddr;
	InputRegister[52] = Table4RegisterNum;
	InputRegister[53] = Table4Funcode;
	
	InputRegister[54] = Table5StartAddr;
	InputRegister[55] = Table5RegisterNum;
	InputRegister[56] = Table5Funcode;
	
	InputRegister[57] = Table6StartAddr;
	InputRegister[58] = Table6RegisterNum;
	InputRegister[59] = Table6Funcode;
	
	InputRegister[60] = Table7StartAddr;
	InputRegister[61] = Table7RegisterNum;
	InputRegister[62] = Table7Funcode;
	
	InputRegister[63] = Table8StartAddr;
	InputRegister[64] = Table8RegisterNum;
	InputRegister[65] = Table8Funcode;
	
	InputRegister[66] = Table9StartAddr;
	InputRegister[67] = Table9RegisterNum;
	InputRegister[68] = Table9Funcode;
	
	InputRegister[69] = Table10StartAddr;
	InputRegister[70] = Table10RegisterNum;
	InputRegister[71] = Table10Funcode;
	
	InputRegister[72] = Table11StartAddr;
	InputRegister[73] = Table11RegisterNum;
	InputRegister[74] = Table11Funcode;

 	for(i=75;i<256;i++)
	{
		InputRegister[i] = Reserved;
	}
/****************************Table1********************************************/	
	InputRegister[256] = main_parameter[99];
	InputRegister[257] = (uint16)(TotalEnergyHB>>16);
	InputRegister[258] = (uint16)(TotalEnergyHB&0xFFFF);
 	InputRegister[259] = (uint16)(TotalEnergyLB>>16);
	InputRegister[260] = (uint16)(TotalEnergyLB&0xFFFF);
	InputRegister[261] = (uint16)(TotalRunningTime>>16);
	InputRegister[262] = (uint16)(TotalRunningTime&0xFFFF);
	tmp32 =ARMVersion;	//ARM�汾��
	InputRegister[263] = (uint16)(tmp32>>16);
	InputRegister[264] = (uint16)(tmp32);
	InputRegister[265] = dsp_data[98];		//DSP�汾�Ÿ�16λ
	InputRegister[266] = dsp_data[99];		//DSP�汾�ŵ�16λ
	InputRegister[267] = SystemStatus;
	InputRegister[268] = MainErrorCode;
	InputRegister[269] = SlaveStatus[0];//
	InputRegister[270] = SlaveErrorCode[0];//
	InputRegister[271] = SlaveStatus[1];//
	InputRegister[272] = SlaveErrorCode[1];//
	InputRegister[273] = SlaveStatus[2];//
	InputRegister[274] = SlaveErrorCode[2];//
	InputRegister[275] = SlaveStatus[3];//
	InputRegister[276] = SlaveErrorCode[3];//
	InputRegister[277] = SlaveStatus[4];//
	InputRegister[278] = SlaveErrorCode[4];//
	InputRegister[279] = SlaveStatus[5];//
	InputRegister[280] = SlaveErrorCode[5];//
	InputRegister[281] = SlaveStatus[6];//
	InputRegister[282] = SlaveErrorCode[6];//
	InputRegister[283] = SlaveStatus[7];//
	InputRegister[284] = SlaveErrorCode[7];//
	InputRegister[285] = SlaveStatus[8];//
	InputRegister[286] = SlaveErrorCode[8];//
	InputRegister[287] = SlaveStatus[9];//
	InputRegister[288] = SlaveErrorCode[9];//
	for(i=0;i<100;i++)
	{
		InputRegister[i+289] = dsp_data[i];
	}
	
	tmp31=dsp_data[10]*xiuzhen_dianliujibian2/10000;
	InputRegister[299] =(uint16)(tmp31);//����������
	tmp31=dsp_data[11]*xiuzhen_dianliujibian2/10000;
	InputRegister[300] =(uint16)(tmp31);//����������
	tmp31=dsp_data[12]*xiuzhen_dianliujibian2/10000;
	InputRegister[301] =(uint16)(tmp31);//����������
	
	tmp31=dsp_data[3]*xiuzhen_dianyajibian2/10000;
	InputRegister[292] =(uint16)(tmp31);//��ѹ������
	tmp31=dsp_data[4]*xiuzhen_dianyajibian2/10000;
	InputRegister[293] =(uint16)(tmp31);//��ѹ������
	tmp31=dsp_data[5]*xiuzhen_dianyajibian2/10000;
	InputRegister[294] =(uint16)(tmp31);//��ѹ������
	
	if(dsp_data[42]>32768)//�ж�A�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[42];
	else
		tmp16=dsp_data[42];	
	if(dsp_data[45]>32768)//�ж�A���书���Ƿ�Ϊ��
		tmp17=65536-dsp_data[45];
	else
		tmp17=dsp_data[45];	
	if(dsp_data[39]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[39];
	else
		tmp18=dsp_data[39];		
	tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)+(tmp17*10000/xiuzhen_jibian2)*(tmp17*10000/xiuzhen_jibian2)				
		));
		if(dsp_data[42]>32768) tmp15=-1*tmp15;
		InputRegister[302]=(uint16)(tmp15);//PF
	
	
	if(dsp_data[43]>32768)//�ж�B�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[43];
	else
		tmp16=dsp_data[43];	
	if(dsp_data[46]>32768)//�ж�B���书���Ƿ�Ϊ��
		tmp17=65536-dsp_data[46];
	else
		tmp17=dsp_data[46];
	if(dsp_data[40]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[40];
	else
		tmp18=dsp_data[40];		
	tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)+(tmp17*10000/xiuzhen_jibian2)*(tmp17*10000/xiuzhen_jibian2)				
		));
	if(dsp_data[43]>32768) tmp15=-1*tmp15;
	InputRegister[303]=(uint16)(tmp15);//PF
	
	
	if(dsp_data[44]>32768)//�ж�C�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[44];
	else
		tmp16=dsp_data[44];	
	if(dsp_data[47]>32768)//�ж�C���书���Ƿ�Ϊ��
		tmp17=65536-dsp_data[47];
	else
		tmp17=dsp_data[47];
	if(dsp_data[41]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[41];
	else
		tmp18=dsp_data[41];		
	tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)+(tmp17*10000/xiuzhen_jibian2)*(tmp17*10000/xiuzhen_jibian2)				
		));
	if(dsp_data[44]>32768) tmp15=-1*tmp15;
	InputRegister[304]=(uint16)(tmp15);//PF
	
	
	
	/*
	InputRegister[302] =(uint16)(dsp_data[13]*xiuzhen_pf2/10000);//pf
	
	InputRegister[303] =(uint16)(dsp_data[14]*xiuzhen_pf2/10000);
	
	InputRegister[304] =(uint16)(dsp_data[15]*xiuzhen_pf2/10000);
	*/
	if(dsp_data[42]>32768)//�ж�A�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[42];
	else
		tmp16=dsp_data[42];	
	if(dsp_data[39]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[39];
	else
		tmp18=dsp_data[39];
	 tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)));
		if(dsp_data[42]>32768) tmp15=-1*tmp15;
		InputRegister[305]=(uint16)(tmp15);//cos��
			
	if(dsp_data[43]>32768)//�ж�B�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[43];
	else
		tmp16=dsp_data[43];	
	if(dsp_data[40]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[40];
	else
		tmp18=dsp_data[40];
	 tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)));
	if(dsp_data[43]>32768) tmp15=-1*tmp15;
	InputRegister[306]=(uint16)(tmp15);//cos��
			
	if(dsp_data[44]>32768)//�ж�C�޹������Ƿ�Ϊ��
		tmp16=65536-dsp_data[44];
	else
		tmp16=dsp_data[44];	
	if(dsp_data[41]>32768)//�ж�A�й������Ƿ�Ϊ��
		tmp18=65536-dsp_data[41];
	else
		tmp18=dsp_data[41];
	 tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16*10000/xiuzhen_pf2)*(tmp16*10000/xiuzhen_pf2)));
	if(dsp_data[44]>32768) tmp15=-1*tmp15;
	InputRegister[307]=(uint16)(tmp15);//cos��
			
	tmp31=dsp_data[32]*xiuzhen_shuchu2/10000;
	InputRegister[321]=(uint16)(tmp31);//�������
	tmp31=dsp_data[33]*xiuzhen_shuchu2/10000;
	InputRegister[322]=(uint16)(tmp31);//�������
	tmp31=dsp_data[34]*xiuzhen_shuchu2/10000;
	InputRegister[323]=(uint16)(tmp31);//�������
	
	tmp31=dsp_data[42]*10000/xiuzhen_wugong2;
	InputRegister[331]=(uint16)(tmp31);//�޹�����
	tmp31=dsp_data[43]*10000/xiuzhen_wugong2;
	InputRegister[332]=(uint16)(tmp31);//�޹�����
	tmp31=dsp_data[44]*10000/xiuzhen_wugong2;
	InputRegister[333]=(uint16)(tmp31);//�޹�����
	
	tmp31=dsp_data[45]*10000/xiuzhen_jibian2;
	InputRegister[334]=(uint16)(tmp31);//���书��
	tmp31=dsp_data[46]*10000/xiuzhen_jibian2;
	InputRegister[335]=(uint16)(tmp31);//���书��
	tmp31=dsp_data[47]*10000/xiuzhen_jibian2;
	InputRegister[336]=(uint16)(tmp31);//���书��
	
	InputRegister[362] =Ubalance;
	InputRegister[363] =dsp_data[60];
	InputRegister[364] = SlaveStatus[10];
	InputRegister[365] = SlaveErrorCode[10];//
	InputRegister[366] = SlaveStatus[11];
	InputRegister[367] = SlaveErrorCode[11];//
	InputRegister[368] = SlaveStatus[12];
	InputRegister[369] = SlaveErrorCode[12];//
	InputRegister[370] = SlaveStatus[13];
	InputRegister[371] = SlaveErrorCode[13];//
	InputRegister[372] = SlaveStatus[14];
	InputRegister[373] = SlaveErrorCode[14];//
	InputRegister[374] = SlaveStatus[15];
	InputRegister[375] = SlaveErrorCode[15];//
	InputRegister[376] = CTSyscurA;
	InputRegister[377] = CTSyscurB;
	InputRegister[378] = CTSyscurC;
	InputRegister[379] = CTOutcurA;
	InputRegister[380] = CTOutcurB;
	InputRegister[381] = CTOutcurC;
	InputRegister[382] = HighVolA;
	InputRegister[381] = HighVolB;
	InputRegister[384] = HighVolC;
	InputRegister[385] =dsp_data[90];
	InputRegister[386] =dsp_data[94];
	InputRegister[387] =dsp_data[91];
	InputRegister[388] =dsp_data[95];
	InputRegister[389] =dsp_data[92];
	InputRegister[390] =dsp_data[96];
/****************************Table2********************************************/
InputRegister[512]=system_currentA[0];
InputRegister[612]=system_currentB[0];
InputRegister[712]=system_currentC[0];
for(i=1;i<50;i++)
	{
		InputRegister[i+512] =system_currentA[i];
		if(Selected_parameter[i]==1)
		InputRegister[i+512] =(uint16)(system_currentA[i]*10000/xiuzhen_fuzhi2);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+50] =system_phaseA[i];
	}
	for(i=1;i<50;i++)
	{
		InputRegister[i+512+100] =system_currentB[i];
		if(Selected_parameter[i]==1)
		InputRegister[i+512+100] =(uint16)(system_currentB[i]*10000/xiuzhen_fuzhi2);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+150] =system_phaseB[i];
	}
	for(i=1;i<50;i++)
	{
		InputRegister[i+512+200] =system_currentC[i];
		if(Selected_parameter[i]==1)
		InputRegister[i+512+200] =(uint16)(system_currentC[i]*xiuzhen_fuzhi2/10000);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+250] =system_phaseC[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+300] =load_currentA[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+350] =load_phaseA[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+400] =load_currentB[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+450] =load_phaseB[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+500] =load_currentC[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+550] =load_phaseC[i];
	}
	for(i=0;i<50;i++)
	{
		
		InputRegister[i+512+600] =out_currentA[i];
		if(Selected_parameter[i]==1)
		InputRegister[i+512+600] =	(uint16)(out_currentA[i]*xiuzhen_fuzhi2/10000);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+650] =out_phaseA[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+700] =out_currentB[i];
		if(Selected_parameter[i]==1)
			InputRegister[i+512+700] =	(uint16)(out_currentB[i]*xiuzhen_fuzhi2/10000);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+750] =out_phaseB[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+800] =out_currentC[i];
		if(Selected_parameter[i]==1)
			InputRegister[i+512+800] =	(uint16)(out_currentC[i]*xiuzhen_fuzhi2/10000);
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+850] =out_currentC[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+900] =votageA[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+950] =votage_phaseA[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+1000] =votageB[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+1050] =votage_phaseB[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+1100] =votageC[i];
	}
	for(i=0;i<50;i++)
	{
		InputRegister[i+512+1150] =votage_phaseC[i];
	}
	for(i=0;i<300;i++)
	{
		InputRegister[i+512+1200] =capa[i];
	}
	InputRegister[2012] =Reserved;
	InputRegister[2013] =Reserved;
	InputRegister[2014] =Reserved;
	InputRegister[2015] =Reserved;
	InputRegister[2016] =Reserved;
	InputRegister[2017] =Reserved;
	InputRegister[2018] =Reserved;
	InputRegister[2019] =Reserved;
	InputRegister[2020] =Reserved;
	InputRegister[2021] =Reserved;
	InputRegister[2022] =Reserved;
	InputRegister[2023] =Reserved;
	InputRegister[2024] =Reserved;
	InputRegister[2025] =Reserved;
	InputRegister[2026] =Reserved;
	InputRegister[2027] =Reserved;
	InputRegister[2028] =Reserved;
	InputRegister[2029] =Reserved;
	InputRegister[2030] =Reserved;
	InputRegister[2031] =Reserved;
	InputRegister[2032] =Reserved;
	InputRegister[2033] =Reserved;
	InputRegister[2034] =Reserved;
	InputRegister[2035] =Reserved;
	InputRegister[2036] =Reserved;
	InputRegister[2037] =Reserved;
	InputRegister[2038] =Reserved;
	InputRegister[2039] =Reserved;
	InputRegister[2040] =Reserved;
	InputRegister[2041] =Reserved;
	InputRegister[2042] =Reserved;
	InputRegister[2043] =Reserved;
	InputRegister[2044] =Reserved;
	InputRegister[2045] =Reserved;
	InputRegister[2046] =Reserved;
	InputRegister[2047] =Reserved;
/****************************Table3********************************************/
	InputRegister[2048] =slave_data[0][0];
	InputRegister[2049] =slave_data[0][1];
	InputRegister[2050] =slave_data[0][2];
	InputRegister[2051] =slave_data[0][3];
	InputRegister[2052] =slave_data[0][4];
	InputRegister[2053] =slave_data[0][5];
	InputRegister[2054] =slave_data[0][6];
	InputRegister[2055] =slave_data[0][7];
	InputRegister[2056] =slave_data[0][8];
	InputRegister[2057] =slave_data[0][9];
	InputRegister[2058] =Reserved;
	InputRegister[2059] =Reserved;
	InputRegister[2060] =Reserved;
	InputRegister[2061] =Reserved;
	InputRegister[2062] =Reserved;
	InputRegister[2063] =Reserved;

	InputRegister[2064] =slave_data[1][0];
	InputRegister[2065] =slave_data[1][1];
	InputRegister[2066] =slave_data[1][2];
	InputRegister[2067] =slave_data[1][3];
	InputRegister[2068] =slave_data[1][4];
	InputRegister[2069] =slave_data[1][5];
	InputRegister[2070] =slave_data[1][6];
	InputRegister[2071] =slave_data[1][7];
	InputRegister[2072] =slave_data[1][8];
	InputRegister[2073] =slave_data[1][9];
	InputRegister[2074] =Reserved;
	InputRegister[2075] =Reserved;
	InputRegister[2076] =Reserved;
	InputRegister[2077] =Reserved;
	InputRegister[2078] =Reserved;
	InputRegister[2079] =Reserved;

	InputRegister[2080] =slave_data[2][0];
	InputRegister[2081] =slave_data[2][1];
	InputRegister[2082] =slave_data[2][2];
	InputRegister[2083] =slave_data[2][3];
	InputRegister[2084] =slave_data[2][4];
	InputRegister[2085] =slave_data[2][5];
	InputRegister[2086] =slave_data[2][6];
	InputRegister[2087] =slave_data[2][7];
	InputRegister[2088] =slave_data[2][8];
	InputRegister[2089] =slave_data[2][9];
	InputRegister[2090] =Reserved;
	InputRegister[2091] =Reserved;
	InputRegister[2092] =Reserved;
	InputRegister[2093] =Reserved;
	InputRegister[2094] =Reserved;
	InputRegister[2095] =Reserved;

	InputRegister[2096] =slave_data[3][0];
	InputRegister[2097] =slave_data[3][1];
	InputRegister[2098] =slave_data[3][2];
	InputRegister[2099] =slave_data[3][3];
	InputRegister[2100] =slave_data[3][4];
	InputRegister[2101] =slave_data[3][5];
	InputRegister[2102] =slave_data[3][6];
	InputRegister[2103] =slave_data[3][7];
	InputRegister[2104] =slave_data[3][8];
	InputRegister[2105] =slave_data[3][9];
	InputRegister[2106] =Reserved;
	InputRegister[2107] =Reserved;
	InputRegister[2108] =Reserved;
	InputRegister[2109] =Reserved;
	InputRegister[2110] =Reserved;
	InputRegister[2111] =Reserved;

	InputRegister[2112] =slave_data[4][0];
	InputRegister[2113] =slave_data[4][1];
	InputRegister[2114] =slave_data[4][2];
	InputRegister[2115] =slave_data[4][3];
	InputRegister[2116] =slave_data[4][4];
	InputRegister[2117] =slave_data[4][5];
	InputRegister[2118] =slave_data[4][6];
	InputRegister[2119] =slave_data[4][7];
	InputRegister[2120] =slave_data[4][8];
	InputRegister[2121] =slave_data[4][9];
	InputRegister[2122] =Reserved;
	InputRegister[2123] =Reserved;
	InputRegister[2124] =Reserved;
	InputRegister[2125] =Reserved;
	InputRegister[2126] =Reserved;
	InputRegister[2127] =Reserved;

	InputRegister[2128] =slave_data[5][0];
	InputRegister[2129] =slave_data[5][1];
	InputRegister[2130] =slave_data[5][2];
	InputRegister[2131] =slave_data[5][3];
	InputRegister[2132] =slave_data[5][4];
	InputRegister[2133] =slave_data[5][5];
	InputRegister[2134] =slave_data[5][6];
	InputRegister[2135] =slave_data[5][7];
	InputRegister[2136] =slave_data[5][8];
	InputRegister[2137] =slave_data[5][9];
	InputRegister[2138] =Reserved;
	InputRegister[2139] =Reserved;
	InputRegister[2140] =Reserved;
	InputRegister[2141] =Reserved;
	InputRegister[2142] =Reserved;
	InputRegister[2143] =Reserved;

	InputRegister[2144] =slave_data[6][0];
	InputRegister[2145] =slave_data[6][1];
	InputRegister[2146] =slave_data[6][2];
	InputRegister[2147] =slave_data[6][3];
	InputRegister[2148] =slave_data[6][4];
	InputRegister[2149] =slave_data[6][5];
	InputRegister[2150] =slave_data[6][6];
	InputRegister[2151] =slave_data[6][7];
	InputRegister[2152] =slave_data[6][8];
	InputRegister[2153] =slave_data[6][9];
	InputRegister[2154] =Reserved;
	InputRegister[2155] =Reserved;
	InputRegister[2156] =Reserved;
	InputRegister[2157] =Reserved;
	InputRegister[2158] =Reserved;
	InputRegister[2159] =Reserved;

	InputRegister[2160] =slave_data[7][0];
	InputRegister[2161] =slave_data[7][1];
	InputRegister[2162] =slave_data[7][2];
	InputRegister[2163] =slave_data[7][3];
	InputRegister[2164] =slave_data[7][4];
	InputRegister[2165] =slave_data[7][5];
	InputRegister[2166] =slave_data[7][6];
	InputRegister[2167] =slave_data[7][7];
	InputRegister[2168] =slave_data[7][8];
	InputRegister[2169] =slave_data[7][9];
	InputRegister[2170] =Reserved;
	InputRegister[2171] =Reserved;
	InputRegister[2172] =Reserved;
	InputRegister[2173] =Reserved;
	InputRegister[2174] =Reserved;
	InputRegister[2175] =Reserved;

	InputRegister[2176] =slave_data[8][0];
	InputRegister[2177] =slave_data[8][1];
	InputRegister[2178] =slave_data[8][2];
	InputRegister[2179] =slave_data[8][3];
	InputRegister[2180] =slave_data[8][4];
	InputRegister[2181] =slave_data[8][5];
	InputRegister[2182] =slave_data[8][6];
	InputRegister[2183] =slave_data[8][7];
	InputRegister[2184] =slave_data[8][8];
	InputRegister[2185] =slave_data[8][9];
	InputRegister[2186] =Reserved;
	InputRegister[2187] =Reserved;
	InputRegister[2188] =Reserved;
	InputRegister[2189] =Reserved;
	InputRegister[2190] =Reserved;
	InputRegister[2191] =Reserved;

	InputRegister[2192] =slave_data[9][0];
	InputRegister[2193] =slave_data[9][1];
	InputRegister[2194] =slave_data[9][2];
	InputRegister[2195] =slave_data[9][3];
	InputRegister[2196] =slave_data[9][4];
	InputRegister[2197] =slave_data[9][5];
	InputRegister[2198] =slave_data[9][6];
	InputRegister[2199] =slave_data[9][7];
	InputRegister[2200] =slave_data[9][8];
	InputRegister[2201] =slave_data[9][9];
	InputRegister[2202] =Reserved;
	InputRegister[2203] =Reserved;
	InputRegister[2204] =Reserved;
	InputRegister[2205] =Reserved;
	InputRegister[2206] =Reserved;
	InputRegister[2207] =Reserved;
	
	InputRegister[2208] =slave_data[10][0];
	InputRegister[2209] =slave_data[10][1];
	InputRegister[2210] =slave_data[10][2];
	InputRegister[2211] =slave_data[10][3];
	InputRegister[2212] =slave_data[10][4];
	InputRegister[2213] =slave_data[10][5];
	InputRegister[2214] =slave_data[10][6];
	InputRegister[2215] =slave_data[10][7];
	InputRegister[2216] =slave_data[10][8];
	InputRegister[2217] =slave_data[10][9];
	InputRegister[2218] =Reserved;
	InputRegister[2219] =Reserved;
	InputRegister[2220] =Reserved;
	InputRegister[2221] =Reserved;
	InputRegister[2222] =Reserved;
	InputRegister[2223] =Reserved;
	
	InputRegister[2224] =slave_data[11][0];
	InputRegister[2225] =slave_data[11][1];
	InputRegister[2226] =slave_data[11][2];
	InputRegister[2227] =slave_data[11][3];
	InputRegister[2228] =slave_data[11][4];
	InputRegister[2229] =slave_data[11][5];
	InputRegister[2230] =slave_data[11][6];
	InputRegister[2231] =slave_data[11][7];
	InputRegister[2232] =slave_data[11][8];
	InputRegister[2233] =slave_data[11][9];
	InputRegister[2234] =Reserved;
	InputRegister[2235] =Reserved;
	InputRegister[2236] =Reserved;
	InputRegister[2237] =Reserved;
	InputRegister[2238] =Reserved;
	InputRegister[2239] =Reserved;
	
	InputRegister[2240] =slave_data[12][0];
	InputRegister[2241] =slave_data[12][1];
	InputRegister[2242] =slave_data[12][2];
	InputRegister[2243] =slave_data[12][3];
	InputRegister[2244] =slave_data[12][4];
	InputRegister[2245] =slave_data[12][5];
	InputRegister[2246] =slave_data[12][6];
	InputRegister[2247] =slave_data[12][7];
	InputRegister[2248] =slave_data[12][8];
	InputRegister[2249] =slave_data[12][9];
	InputRegister[2250] =Reserved;
	InputRegister[2251] =Reserved;
	InputRegister[2252] =Reserved;
	InputRegister[2253] =Reserved;
	InputRegister[2254] =Reserved;
	InputRegister[2255] =Reserved;

	InputRegister[2256] =slave_data[13][0];
	InputRegister[2257] =slave_data[13][1];
	InputRegister[2258] =slave_data[13][2];
	InputRegister[2259] =slave_data[13][3];
	InputRegister[2260] =slave_data[13][4];
	InputRegister[2261] =slave_data[13][5];
	InputRegister[2262] =slave_data[13][6];
	InputRegister[2263] =slave_data[13][7];
	InputRegister[2264] =slave_data[13][8];
	InputRegister[2265] =slave_data[13][9];
	InputRegister[2266] =Reserved;
	InputRegister[2267] =Reserved;
	InputRegister[2268] =Reserved;
	InputRegister[2269] =Reserved;
	InputRegister[2270] =Reserved;
	InputRegister[2271] =Reserved;

	InputRegister[2272] =slave_data[14][0];
	InputRegister[2273] =slave_data[14][1];
	InputRegister[2274] =slave_data[14][2];
	InputRegister[2275] =slave_data[14][3];
	InputRegister[2276] =slave_data[14][4];
	InputRegister[2277] =slave_data[14][5];
	InputRegister[2278] =slave_data[14][6];
	InputRegister[2279] =slave_data[14][7];
	InputRegister[2280] =slave_data[14][8];
	InputRegister[2281] =slave_data[14][9];
	InputRegister[2282] =Reserved;
	InputRegister[2283] =Reserved;
	InputRegister[2284] =Reserved;
	InputRegister[2285] =Reserved;
	InputRegister[2286] =Reserved;
	InputRegister[2287] =Reserved;

	InputRegister[2288] =slave_data[15][0];
	InputRegister[2289] =slave_data[15][1];
	InputRegister[2290] =slave_data[15][2];
	InputRegister[2291] =slave_data[15][3];
	InputRegister[2292] =slave_data[15][4];
	InputRegister[2293] =slave_data[15][5];
	InputRegister[2294] =slave_data[15][6];
	InputRegister[2295] =slave_data[15][7];
	InputRegister[2296] =slave_data[15][8];
	InputRegister[2297] =slave_data[15][9];
	InputRegister[2298] =Reserved;
	InputRegister[2299] =Reserved;
	InputRegister[2300] =Reserved;
	InputRegister[2301] =Reserved;
	InputRegister[2302] =Reserved;
	InputRegister[2303] =Reserved;
	
/****************************Table4********************************************/
	InputRegister[2304] =slave_Maxmin[0][0];
	InputRegister[2305] =slave_Maxmin[0][1];
	InputRegister[2306] =slave_Maxmin[0][2];
	InputRegister[2307] =slave_Maxmin[0][3];
	InputRegister[2308] =slave_Maxmin[0][4];
	InputRegister[2309] =slave_Maxmin[0][5];
	InputRegister[2310] =slave_Maxmin[0][6];
	InputRegister[2311] =slave_Maxmin[0][7];
	InputRegister[2312] =slave_Maxmin[0][8];
	InputRegister[2313] =slave_Maxmin[0][9];
	InputRegister[2314] =Reserved;
	InputRegister[2315] =Reserved;
	InputRegister[2316] =Reserved;
	InputRegister[2317] =Reserved;
	InputRegister[2318] =Reserved;
	InputRegister[2319] =Reserved;

	InputRegister[2320] =slave_Maxmin[1][0];
	InputRegister[2321] =slave_Maxmin[1][1];
	InputRegister[2322] =slave_Maxmin[1][2];
	InputRegister[2323] =slave_Maxmin[1][3];
	InputRegister[2324] =slave_Maxmin[1][4];
	InputRegister[2325] =slave_Maxmin[1][5];
	InputRegister[2326] =slave_Maxmin[1][6];
	InputRegister[2327] =slave_Maxmin[1][7];
	InputRegister[2328] =slave_Maxmin[1][8];
	InputRegister[2329] =slave_Maxmin[1][9];
	InputRegister[2330] =Reserved;
	InputRegister[2331] =Reserved;
	InputRegister[2332] =Reserved;
	InputRegister[2333] =Reserved;
	InputRegister[2334] =Reserved;
	InputRegister[2335] =Reserved;

	InputRegister[2336] =slave_Maxmin[2][0];
	InputRegister[2337] =slave_Maxmin[2][1];
	InputRegister[2338] =slave_Maxmin[2][2];
	InputRegister[2339] =slave_Maxmin[2][3];
	InputRegister[2340] =slave_Maxmin[2][4];
	InputRegister[2341] =slave_Maxmin[2][5];
	InputRegister[2342] =slave_Maxmin[2][6];
	InputRegister[2343] =slave_Maxmin[2][7];
	InputRegister[2344] =slave_Maxmin[2][8];
	InputRegister[2345] =slave_Maxmin[2][9];
	InputRegister[2346] =Reserved;
	InputRegister[2347] =Reserved;
	InputRegister[2348] =Reserved;
	InputRegister[2349] =Reserved;
	InputRegister[2350] =Reserved;
	InputRegister[2351] =Reserved;

	InputRegister[2352] =slave_Maxmin[3][0];
	InputRegister[2353] =slave_Maxmin[3][1];
	InputRegister[2354] =slave_Maxmin[3][2];
	InputRegister[2355] =slave_Maxmin[3][3];
	InputRegister[2356] =slave_Maxmin[3][4];
	InputRegister[2357] =slave_Maxmin[3][5];
	InputRegister[2358] =slave_Maxmin[3][6];
	InputRegister[2359] =slave_Maxmin[3][7];
	InputRegister[2360] =slave_Maxmin[3][8];
	InputRegister[2361] =slave_Maxmin[3][9];
	InputRegister[2362] =Reserved;
	InputRegister[2363] =Reserved;
	InputRegister[2364] =Reserved;
	InputRegister[2365] =Reserved;
	InputRegister[2366] =Reserved;
	InputRegister[2367] =Reserved;

	InputRegister[2368] =slave_Maxmin[4][0];
	InputRegister[2369] =slave_Maxmin[4][1];
	InputRegister[2370] =slave_Maxmin[4][2];
	InputRegister[2371] =slave_Maxmin[4][3];
	InputRegister[2372] =slave_Maxmin[4][4];
	InputRegister[2373] =slave_Maxmin[4][5];
	InputRegister[2374] =slave_Maxmin[4][6];
	InputRegister[2375] =slave_Maxmin[4][7];
	InputRegister[2376] =slave_Maxmin[4][8];
	InputRegister[2377] =slave_Maxmin[4][9];
	InputRegister[2378] =Reserved;
	InputRegister[2379] =Reserved;
	InputRegister[2380] =Reserved;
	InputRegister[2381] =Reserved;
	InputRegister[2382] =Reserved;
	InputRegister[2383] =Reserved;

	InputRegister[2384] =slave_Maxmin[5][0];
	InputRegister[2385] =slave_Maxmin[5][1];
	InputRegister[2386] =slave_Maxmin[5][2];
	InputRegister[2387] =slave_Maxmin[5][3];
	InputRegister[2388] =slave_Maxmin[5][4];
	InputRegister[2389] =slave_Maxmin[5][5];
	InputRegister[2390] =slave_Maxmin[5][6];
	InputRegister[2391] =slave_Maxmin[5][7];
	InputRegister[2392] =slave_Maxmin[5][8];
	InputRegister[2393] =slave_Maxmin[5][9];
	InputRegister[2394] =Reserved;
	InputRegister[2395] =Reserved;
	InputRegister[2396] =Reserved;
	InputRegister[2397] =Reserved;
	InputRegister[2398] =Reserved;
	InputRegister[2399] =Reserved;

	InputRegister[2400] =slave_Maxmin[6][0];
	InputRegister[2401] =slave_Maxmin[6][1];
	InputRegister[2402] =slave_Maxmin[6][2];
	InputRegister[2403] =slave_Maxmin[6][3];
	InputRegister[2404] =slave_Maxmin[6][4];
	InputRegister[2405] =slave_Maxmin[6][5];
	InputRegister[2406] =slave_Maxmin[6][6];
	InputRegister[2407] =slave_Maxmin[6][7];
	InputRegister[2408] =slave_Maxmin[6][8];
	InputRegister[2409] =slave_Maxmin[6][9];
	InputRegister[2410] =Reserved;
	InputRegister[2411] =Reserved;
	InputRegister[2412] =Reserved;
	InputRegister[2413] =Reserved;
	InputRegister[2414] =Reserved;
	InputRegister[2415] =Reserved;

	InputRegister[2416] =slave_Maxmin[7][0];
	InputRegister[2417] =slave_Maxmin[7][1];
	InputRegister[2418] =slave_Maxmin[7][2];
	InputRegister[2419] =slave_Maxmin[7][3];
	InputRegister[2420] =slave_Maxmin[7][4];
	InputRegister[2421] =slave_Maxmin[7][5];
	InputRegister[2422] =slave_Maxmin[7][6];
	InputRegister[2423] =slave_Maxmin[7][7];
	InputRegister[2424] =slave_Maxmin[7][8];
	InputRegister[2425] =slave_Maxmin[7][9];
	InputRegister[2426] =Reserved;
	InputRegister[2427] =Reserved;
	InputRegister[2428] =Reserved;
	InputRegister[2429] =Reserved;
	InputRegister[2430] =Reserved;
	InputRegister[2431] =Reserved;

	InputRegister[2432] =slave_Maxmin[8][0];
	InputRegister[2433] =slave_Maxmin[8][1];
	InputRegister[2434] =slave_Maxmin[8][2];
	InputRegister[2435] =slave_Maxmin[8][3];
	InputRegister[2436] =slave_Maxmin[8][4];
	InputRegister[2437] =slave_Maxmin[8][5];
	InputRegister[2438] =slave_Maxmin[8][6];
	InputRegister[2439] =slave_Maxmin[8][7];
	InputRegister[2440] =slave_Maxmin[8][8];
	InputRegister[2441] =slave_Maxmin[8][9];
	InputRegister[2442] =Reserved;
	InputRegister[2443] =Reserved;
	InputRegister[2444] =Reserved;
	InputRegister[2445] =Reserved;
	InputRegister[2446] =Reserved;
	InputRegister[2447] =Reserved;

	InputRegister[2448] =slave_Maxmin[9][0];
	InputRegister[2449] =slave_Maxmin[9][1];
	InputRegister[2450] =slave_Maxmin[9][2];
	InputRegister[2451] =slave_Maxmin[9][3];
	InputRegister[2452] =slave_Maxmin[9][4];
	InputRegister[2453] =slave_Maxmin[9][5];
	InputRegister[2454] =slave_Maxmin[9][6];
	InputRegister[2455] =slave_Maxmin[9][7];
	InputRegister[2456] =slave_Maxmin[9][8];
	InputRegister[2457] =slave_Maxmin[9][9];
	InputRegister[2458] =Reserved;
	InputRegister[2459] =Reserved;
	InputRegister[2460] =Reserved;
	InputRegister[2461] =Reserved;
	InputRegister[2462] =Reserved;
	InputRegister[2463] =Reserved;

	InputRegister[2464] =slave_Maxmin[10][0];
	InputRegister[2465] =slave_Maxmin[10][1];
	InputRegister[2466] =slave_Maxmin[10][2];
	InputRegister[2467] =slave_Maxmin[10][3];
	InputRegister[2468] =slave_Maxmin[10][4];
	InputRegister[2469] =slave_Maxmin[10][5];
	InputRegister[2470] =slave_Maxmin[10][6];
	InputRegister[2471] =slave_Maxmin[10][7];
	InputRegister[2472] =slave_Maxmin[10][8];
	InputRegister[2473] =slave_Maxmin[10][9];
	InputRegister[2474] =Reserved;
	InputRegister[2475] =Reserved;
	InputRegister[2476] =Reserved;
	InputRegister[2477] =Reserved;
	InputRegister[2478] =Reserved;
	InputRegister[2479] =Reserved;
	
	InputRegister[2480] =slave_Maxmin[11][0];
	InputRegister[2481] =slave_Maxmin[11][1];
	InputRegister[2482] =slave_Maxmin[11][2];
	InputRegister[2483] =slave_Maxmin[11][3];
	InputRegister[2484] =slave_Maxmin[11][4];
	InputRegister[2485] =slave_Maxmin[11][5];
	InputRegister[2486] =slave_Maxmin[11][6];
	InputRegister[2487] =slave_Maxmin[11][7];
	InputRegister[2488] =slave_Maxmin[11][8];
	InputRegister[2489] =slave_Maxmin[11][9];
	InputRegister[2490] =Reserved;
	InputRegister[2491] =Reserved;
	InputRegister[2492] =Reserved;
	InputRegister[2493] =Reserved;
	InputRegister[2494] =Reserved;
	InputRegister[2495] =Reserved;

	InputRegister[2496] =slave_Maxmin[12][0];
	InputRegister[2497] =slave_Maxmin[12][1];
	InputRegister[2498] =slave_Maxmin[12][2];
	InputRegister[2499] =slave_Maxmin[12][3];
	InputRegister[2500] =slave_Maxmin[12][4];
	InputRegister[2501] =slave_Maxmin[12][5];
	InputRegister[2502] =slave_Maxmin[12][6];
	InputRegister[2503] =slave_Maxmin[12][7];
	InputRegister[2504] =slave_Maxmin[12][8];
	InputRegister[2505] =slave_Maxmin[12][9];
	InputRegister[2506] =Reserved;
	InputRegister[2507] =Reserved;
	InputRegister[2508] =Reserved;
	InputRegister[2509] =Reserved;
	InputRegister[2510] =Reserved;
	InputRegister[2511] =Reserved;
	
	InputRegister[2512] =slave_Maxmin[13][0];
	InputRegister[2513] =slave_Maxmin[13][1];
	InputRegister[2514] =slave_Maxmin[13][2];
	InputRegister[2515] =slave_Maxmin[13][3];
	InputRegister[2516] =slave_Maxmin[13][4];
	InputRegister[2517] =slave_Maxmin[13][5];
	InputRegister[2518] =slave_Maxmin[13][6];
	InputRegister[2519] =slave_Maxmin[13][7];
	InputRegister[2520] =slave_Maxmin[13][8];
	InputRegister[2521] =slave_Maxmin[13][9];
	InputRegister[2522] =Reserved;
	InputRegister[2523] =Reserved;
	InputRegister[2524] =Reserved;
	InputRegister[2525] =Reserved;
	InputRegister[2526] =Reserved;
	InputRegister[2527] =Reserved;
	
	InputRegister[2528] =slave_Maxmin[14][0];
	InputRegister[2529] =slave_Maxmin[14][1];
	InputRegister[2530] =slave_Maxmin[14][2];
	InputRegister[2531] =slave_Maxmin[14][3];
	InputRegister[2532] =slave_Maxmin[14][4];
	InputRegister[2533] =slave_Maxmin[14][5];
	InputRegister[2534] =slave_Maxmin[14][6];
	InputRegister[2535] =slave_Maxmin[14][7];
	InputRegister[2536] =slave_Maxmin[14][8];
	InputRegister[2537] =slave_Maxmin[14][9];
	InputRegister[2538] =Reserved;
	InputRegister[2539] =Reserved;
	InputRegister[2540] =Reserved;
	InputRegister[2541] =Reserved;
	InputRegister[2542] =Reserved;
	InputRegister[2543] =Reserved;

	InputRegister[2544] =slave_Maxmin[15][0];
	InputRegister[2545] =slave_Maxmin[15][1];
	InputRegister[2546] =slave_Maxmin[15][2];
	InputRegister[2547] =slave_Maxmin[15][3];
	InputRegister[2548] =slave_Maxmin[15][4];
	InputRegister[2549] =slave_Maxmin[15][5];
	InputRegister[2550] =slave_Maxmin[15][6];
	InputRegister[2551] =slave_Maxmin[15][7];
	InputRegister[2552] =slave_Maxmin[15][8];
	InputRegister[2553] =slave_Maxmin[15][9];
	InputRegister[2554] =Reserved;
	InputRegister[2555] =Reserved;
	InputRegister[2556] =Reserved;
	InputRegister[2557] =Reserved;
	InputRegister[2558] =Reserved;
	InputRegister[2559] =Reserved;
/****************************Table5 �û�������********************************************/	
	for(i=0;i<255;i++)
	{
		HoldingRegister[i] =0xFFFF;
	}

	if(OnOffCommand==1)
	{
		HoldingRegister[0] =0xAAAA;
	}
	else HoldingRegister[0] =0x5555;
	HoldingRegister[1] =0x8000; 
	HoldingRegister[2] =0x8000;
	HoldingRegister[3] =0x8000;
	HoldingRegister[4] =(uint16)local_time.RTC_Year;
	tmp8[0] =(uint16)local_time.RTC_Mon;
	tmp8[1] =(uint16)local_time.RTC_Mday;
	HoldingRegister[5] =(uint16)((tmp8[0]<<8)|tmp8[1]);
	tmp8[0] =(uint16)local_time.RTC_Hour;
	tmp8[1] =(uint16)local_time.RTC_Min;
	HoldingRegister[6] =(uint16)((tmp8[0]<<8)|tmp8[1]);
	HoldingRegister[7] =(uint16)local_time.RTC_Sec;;
	HoldingRegister[8] =main_parameter[0];
	HoldingRegister[9] =main_parameter[1];
	HoldingRegister[10]=dsp_data[93];			//�ܶ����
	HoldingRegister[11]=main_parameter[3];
	HoldingRegister[12]=main_parameter[4];
	HoldingRegister[13]=main_parameter[5];
	HoldingRegister[14]=main_parameter[6];
	HoldingRegister[15]=main_parameter[7];
	HoldingRegister[16]=main_parameter[8];
	HoldingRegister[17]=main_parameter[9];
	HoldingRegister[18]=main_parameter[10];
	HoldingRegister[19]=main_parameter[11];
	HoldingRegister[20]=main_parameter[12];
	HoldingRegister[21]=main_parameter[13];
	HoldingRegister[22]=main_parameter[14];
	HoldingRegister[23]=main_parameter[15];
	HoldingRegister[24]=main_parameter[16];
	HoldingRegister[25]=main_parameter[17]*10;	//���ܵ��ݲ�����
	HoldingRegister[26]=main_parameter[20];
	for(i=0;i<50;i++)
	{
		HoldingRegister[27+i]=Selected_parameter[i];//ѡ�β���
	}
	//HoldingRegister[77]=Remote_Password;
/****************************Table6 ARM ������********************************************/
	HoldingRegister[77] = LocalAddr;
	HoldingRegister[78] = UART_BPS;
	HoldingRegister[79] =RemoteEnable;
	HoldingRegister[80] =local_time.RTC_Year;
	HoldingRegister[81] =local_time.RTC_Mon;
	HoldingRegister[82] =local_time.RTC_Mday;
	HoldingRegister[83] =local_time.RTC_Hour;
	HoldingRegister[84] =local_time.RTC_Min;
	
	HoldingRegister[85] =OnOffStatus;
		
	HoldingRegister[86] =AlarmTime1[0];
	HoldingRegister[87] =AlarmTime1[1];
	HoldingRegister[88] =AlarmTime1[2];			
	HoldingRegister[89] =AlarmTime1[3];
	HoldingRegister[90] =AlarmTime1[4];
			
	HoldingRegister[91] =AlarmTime2[0];
	HoldingRegister[92] =AlarmTime2[1];
	HoldingRegister[93] =AlarmTime2[2];
	HoldingRegister[94] =AlarmTime2[3];
	HoldingRegister[95] =AlarmTime2[4];
			
	HoldingRegister[96] =AlarmTime3[0];
	HoldingRegister[97] =AlarmTime3[1];
	HoldingRegister[98] =AlarmTime3[2];
	HoldingRegister[99] =AlarmTime3[3];
	HoldingRegister[100] =AlarmTime3[4];
		
	HoldingRegister[101] =ntc_type;//AlarmTime4[0];
	HoldingRegister[102] =0;//AlarmTime4[1];
	HoldingRegister[103] =0;//AlarmTime4[2];
	HoldingRegister[104] =0;//AlarmTime4[3];
	HoldingRegister[105] =0;//AlarmTime4[4];
			
	HoldingRegister[106] =0xFFFF;
	HoldingRegister[107] =0xFFFF;
	HoldingRegister[108] =0xFFFF;
	HoldingRegister[109] =0xFFFF;
	HoldingRegister[110] =0xFFFF;
	HoldingRegister[111] =0xFFFF;
	HoldingRegister[112] =0xFFFF;
	HoldingRegister[113] =0xFFFF;
	HoldingRegister[114] =0xFFFF;
	HoldingRegister[115] =0xFFFF;
	HoldingRegister[116] =0xFFFF;

	HoldingRegister[117] =(uint16)(ProjectNo>>16);
	HoldingRegister[118] =(uint16)ProjectNo;
	HoldingRegister[119] =ProductionNo;

	HoldingRegister[120] =VolOnOffEnable;		//��ѹ����ʹ��//058F
	HoldingRegister[121] =CurOnOffEnable;		//��������ʹ��//0590

	HoldingRegister[122] =MainCTLocation;		//��������λ��//0591
	HoldingRegister[123] =MainCTDirectionA;		//������������//0592
	HoldingRegister[124] =MainCTDirectionB;		//������������//0593

	HoldingRegister[125] =MainCTDirectionC;		//����������λ��//0594
	HoldingRegister[126] =MainCTPhase; 			//��������������//0595
	HoldingRegister[127] =OutCTPhase; 			//��������������//0596
			
	HoldingRegister[128] =Position[0];			//��λ2	
	HoldingRegister[129] =Group[0];				//��2
	HoldingRegister[130] =Capacitance[0];		//��ֵ2

	HoldingRegister[131] =Position[1];			//��λ3	
	HoldingRegister[132] =Group[1];				//��3
	HoldingRegister[133] =Capacitance[1];		//��ֵ3

	HoldingRegister[134] =Position[2];			//��λ4	
	HoldingRegister[135] =Group[2];				//��4
	HoldingRegister[136] =Capacitance[2];		//��ֵ4

	HoldingRegister[137] =Position[3];			//��λ5	
	HoldingRegister[138] =Group[3];				//��5
	HoldingRegister[139] =Capacitance[3];		//��ֵ5

	HoldingRegister[140] =Position[4];			//��λ6	
	HoldingRegister[141] =Group[4];				//��6
	HoldingRegister[142] =Capacitance[4];		//��ֵ6

	HoldingRegister[143] =Position[5];			//��λ7	
	HoldingRegister[144] =Group[5];				//��7
	HoldingRegister[145] =Capacitance[5];		//��ֵ7

	HoldingRegister[146] =Position[6];			//��λ8	
	HoldingRegister[147] =Group[6];				//��8
	HoldingRegister[148] =Capacitance[6];		//��ֵ8

	HoldingRegister[149] =Position[7];			//��λ9	
	HoldingRegister[150] =Group[7];				//��9
	HoldingRegister[151] =Capacitance[7];		//��ֵ9

	HoldingRegister[152] =Position[8];			//��λ10	
	HoldingRegister[153] =Group[8];				//��10
	HoldingRegister[154] =Capacitance[8];		//��ֵ10

	
	for(i=0;i<32;i++)
	{
		HoldingRegister[155+i]=ManualPassive_aisle[i];	//�ֶ���Դͨ��
	}
	HoldingRegister[187] =main_parameter[70];		//��Դʹ��					�Ѹ���70
	HoldingRegister[188] =OutCTDirectionA;		//�������������A//
	HoldingRegister[189] =OutCTDirectionB;		//�������������B//
	HoldingRegister[190] =OutCTDirectionC;		//���������λ��C//
	HoldingRegister[191] =Testtime_function;						//���鹦�ܿ���
	HoldingRegister[192] =Testtime;						//������ʱ��
	HoldingRegister[193] =test_time;						//��������ʱ��
/****************************Table7 �߼�������********************************************/
	HoldingRegister[255]=Remote_Password;
	HoldingRegister[256]=main_parameter[18];
	HoldingRegister[257]=main_parameter[19];
	HoldingRegister[258]=main_parameter[21];
	for(i=0;i<78;i++)
	{
		HoldingRegister[259+i]=main_parameter[22+i];
	}
	HoldingRegister[307]=SetupMode;		//ֱ��ģʽ
	HoldingRegister[337]=xiuzhen_dianliujibian;//��������������
	HoldingRegister[338]=xiuzhen_fuzhi;//2-50�η�ֵ����2
	HoldingRegister[339]=xiuzhen_pf;//pf����
	HoldingRegister[340]=xiuzhen_cos;//cos������
	HoldingRegister[341]=xiuzhen_shuchu;//���������Чֵ����
	HoldingRegister[342]=xiuzhen_wugong;//�޹���������
	HoldingRegister[343]=xiuzhen_jibian;//���书������
	HoldingRegister[344]=xiuzhen_dianyajibian;//��������������
		
//	for(i=0;i<100;i++)
//	{
//		swj_param[i]=main_parameter[i];
//	}
	
	
	for(i=0;i<100;i++)
	{
		HoldingRegister[344+i] =Reserved;
	}
/****************************Table8 �ӻ�������********************************************/
	for(i=0;i<100;i++)
	{
		HoldingRegister[437+i] =slave_parameter[0][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[537+i] =slave_parameter[1][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[637+i] =slave_parameter[2][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[737+i] =slave_parameter[3][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[837+i] =slave_parameter[4][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[937+i] =slave_parameter[5][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1037+i] =slave_parameter[6][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1137+i] =slave_parameter[7][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1237+i] =slave_parameter[8][i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1337+i] =slave_parameter[9][i];
	}
/****************************Table9 ��������********************************************/
	for(i=0;i<100;i++)
	{
		HoldingRegister[1437+i] =load_correctionA[i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1537+i] =load_correctionB[i];
	}
	for(i=0;i<100;i++)
	{
		HoldingRegister[1637+i] =load_correctionC[i];
	}
/****************************Table10 ��Դ������********************************************/
	for(i=0;i<100;i++)
	{
		HoldingRegister[1737+i] =Passive_parameter[i];
	}
/****************************Table11********************************************/

	Factory_parameter[0] =FactoryEnable;
	HEX2BCD(SerialNumber,tmp8,8);
	Factory_parameter[1] = (uint16)(tmp8[0]<<8|tmp8[1]);
	Factory_parameter[2] = (uint16)(tmp8[2]<<8|tmp8[3]);
	Factory_parameter[3] = (uint16)(tmp8[4]<<8|tmp8[5]);
	Factory_parameter[4] = (uint16)(tmp8[6]<<8|tmp8[7]);

	Factory_parameter[5] =LocalAddr;
	Factory_parameter[6] =UART_BPS;

	Factory_parameter[7] =0x00;
	Factory_parameter[8] =0x00;
	Factory_parameter[9] =0x00;

/*********************************ģ�����***************************************/
/*	if(tempmoni==0)
	{	
		if(moni_delay>1)
		{
			tempmoni =1;
			moni_delay =0;
		}
		for(i=256;i<2464;i++)
		{
			InputRegister[i] = i+1;
		}
		InputRegister[256] =4;
		InputRegister[267] =2;
		InputRegister[268] =8;
		InputRegister[269] =2;
		InputRegister[270] =0;
		InputRegister[271] =2;
		InputRegister[272] =0;
		InputRegister[273] =2;
		InputRegister[274] =0;
		InputRegister[275] =2;
		InputRegister[276] =0;
		for(i=0;i<1637;i++)
		{
			HoldingRegister[i] = 0x1000+i-1;
		}
	}
	else
	{
		if(moni_delay>1)
		{
			tempmoni =0;
			moni_delay =0;
		}
		for(i=256;i<2464;i++)
		{
			InputRegister[i] = i-1;
		}
		InputRegister[256] =4;
		InputRegister[267] =3;
		InputRegister[268] =0;
		InputRegister[269] =3;
		InputRegister[270] =6;
		InputRegister[271] =3;
		InputRegister[272] =8;
		InputRegister[273] =4;
		InputRegister[274] =253;
		InputRegister[275] =2;
		InputRegister[276] =245;
		for(i=0;i<1637;i++)
		{
			HoldingRegister[i] = 0x1000+i+1;
		}
	}*/
}
/******************************************************************************
** ������: ExternBTProcess
**
** ���ܣ�������ĻͨѶ
******************************************************************************/
void ExternBTProcess(uint8 Uart)
{
	uint16 Crc_data;

	if(RecvBTFlag==1)		//�������ձ�־λ
	{
		
		{
			Crc_data = crc_chk(RecvSciBuf,RecvSciNum-2);//����У��
			if(RecvSciBuf[RecvSciNum-1]==(uint8)(Crc_data>>8)&&RecvSciBuf[RecvSciNum-2]==(uint8)(Crc_data&0xFF))
			{
				RecvDataProcess(Uart);					//ͨѶ����
				SciWaitingCount =0;
				RecvBTFlag =0;
				if(RecvDTUSuccessTest<0xFFFF)RecvDTUSuccessTest++;
			}
			else 
			{
				Crc_data = crc_chk(RecvSciBuf,RecvSciNum-3);			//ȥ�����һ���ֽڣ�DTU��෢һ���ֽ�
				if(RecvSciBuf[RecvSciNum-2]==(uint8)(Crc_data>>8)&&RecvSciBuf[RecvSciNum-3]==(uint8)(Crc_data&0xFF))
				{
					RecvDataProcess(Uart);
					SciWaitingCount =0;
					RecvBTFlag =0;
					if(RecvDTUSuccessTest<0xFFFF)RecvDTUSuccessTest++;
				}
				else 
				{
					RecvSciNum =0;
					RecvBTFlag =0;
				}
			}
		}
	}
	if(SciWaitingCount>60)	//����60����ղ������ݣ����ڸ�λ
	{
		UART0_Init();
			
		SciWaitingCount=0;
	}
}


/******************************************************************************
** ������: CommuWithDUGS
**
** ���ܣ�������ĻͨѶ
******************************************************************************/

void ExternUartProcess(uint8 Uart)
{
	uint16 Crc_data;

	ReadyData();				//����485�����б����

	if(RecvNewFlag==1)			//������ձ�־λ
	{
		
		{
			Crc_data = crc_chk(RecvSci3Buf,RecvSci3Num-2);//����У��
			if(RecvSci3Buf[RecvSci3Num-1]==(uint8)(Crc_data>>8)&&RecvSci3Buf[RecvSci3Num-2]==(uint8)(Crc_data&0xFF))
			{
				RecvDataProcess(Uart);						//ͨѶ����
				SciWaitingCount3 =0;
				RecvNewFlag =0;
				if(RecvDTUSuccessTest<0xFFFF)RecvDTUSuccessTest++;
			}
			else 
			{
				Crc_data = crc_chk(RecvSci3Buf,RecvSci3Num-3);			//ȥ�����һ���ֽڣ�DTU��෢һ���ֽ�
				if(RecvSci3Buf[RecvSci3Num-2]==(uint8)(Crc_data>>8)&&RecvSci3Buf[RecvSci3Num-3]==(uint8)(Crc_data&0xFF))
				{
					RecvDataProcess(Uart);
					SciWaitingCount3 =0;
					RecvNewFlag =0;
					if(RecvDTUSuccessTest<0xFFFF)RecvDTUSuccessTest++;
				}
				else 
				{
					RecvSci3Num =0;
					RecvNewFlag =0;
				}
			}
		}
	}
	if(SciWaitingCount3>60)			//����ͨѶ��ʱ��ÿ60����ղ����������ݣ����ڸ�λ
	{
		UART1_Init();
		SciWaitingCount3=0;
	}
}

/******************************************************************************
** ������: CommuWithDUGS
** ��Ҫ����:ͨ��״̬������ĻͨѶ�����������ΪͨѶ�����ͬʱҲ�ǳ�ʱʱ�䡣
	1.	Frameheadload   ֡ͷװ�أ�ͨ����ǰ��������֡ͷ��׼��һ��֡����֡ͷ
	2.	SendDataload	����֡װ�أ�ͨ������֡ͷװ����������
	3.	SendDatatoDUGS	���ݷ��ͣ����ݷ��͹����룬��������
	4.	RecvDataFromDUGS���ݽ��մ������ݽ���֡ͷ�͵�ַ�ж����ݴ�ŵ�ַ
** ���ܣ�������ĻͨѶ
******************************************************************************/
void CommunWithDUGSProcess(uint16 Delaytime)
{
	uint32 i;
	uint16 /*CurrentTime,*/Crc_data;
	static uint16 StartTime=0,WaitingTime =0;
	static uint8 CommunState = 0;
	static uint8 FirstCommWithDUGS =0;
	
	if(FirstCommWithDUGS==0)		//�״�ͨѶ
	{
		StartTime =GetSystemTime(t1ms);//��¼��ǰ��  1ms��ʱ����ʾֵ
		CommunState=0;
		FirstCommWithDUGS =1;
	}
	puyaotest3=1;
	switch(CommunState)				//״̬���ж�		0��װ֡ͷ��1�����ͣ�2���ȴ���3�����ܣ�4������
	{
		case StateFrameheadload:	//��������
			puyaotest3=1;
			if(GetCurrentTime(t1ms,StartTime)>Delaytime)			//����ϴ���ʾֵ��ȥ��ǰ��ʾֵ���Ƿ�����ӳ�ʱ��
			{
				i=Frameheadload();	//֡ͷװ��
				//puyaotest3=5;
				SendDataload(i);	//����֡װ��			
				SendDatatoDUGS();	//���ݷ���
				StartTime =GetSystemTime(t1ms);
				if(NoNeedWaitingFlag ==1)	//����Ҫ�ȴ����ر�־λ
				{
					CommunState =StateFrameheadload;
					NoNeedWaitingFlag =0;
				}
				else 
				{
					CommunState =StateWaiting;
					WaitingTime =GetSystemTime(t1ms);
				}
			}
			break;
		case StateWaiting:			//���յȴ�
			//puyaotest3=2;
			if(RecvDUGSNewFlag==1)
			{
				if(RecvDUGSNum<6)							//���մ��� �ط�
				{
					DataRetrans=1;
					RecvDUGSNum =0;					
					CommunWithDUGS.Failure++;
					RecvDUGSNewFlag =0;
					CommunState =StateFrameheadload;
				}
				else
				{
					Crc_data = crc_chk(RecvDUGSBuf+3,RecvDUGSNum-5);
					if(RecvDUGSBuf[RecvDUGSNum-1]==(uint8)(Crc_data>>8)&&RecvDUGSBuf[RecvDUGSNum-2]==(uint8)(Crc_data&0xFF))
					{
						//puyaotest4 =1;
						CommunState =StateReceving;
						CommunWithDUGS.Failure=0;
						if(FirstCommunSuccess==0)
						{
							FirstCommunSuccess =1;
						}
						InterCounter=0;
						DataRetrans =0;
						RecvDUGSNewFlag =0;
						RecvDGUSSuccessTest++;
					}
					else
					{
						//puyaotest4 =2;
						if(CommunWithDUGS.Failure<600)
						{
							DataRetrans=1;
							CommunWithDUGS.Failure++;
							RecvDUGSNewFlag =0;
							CommunState =StateFrameheadload;
						}
						else CommunState =StateAlarm;
					}
				}
			}
			else
			{
				//puyaotest4 =3;
				if(GetCurrentTime(t1ms,WaitingTime)>Delaytime)
				{
					if(InterCounter<1000)
					{
						DataRetrans=1;
						InterCounter++;
						CommunState =StateFrameheadload;
					}
					else 
						CommunState =StateAlarm;	
				}
			}
			//puyaotest4 =4;
			break;
		case StateReceving:		//���մ���
			//puyaotest3=3;
			RecvDataFromDUGS();
			CommunState =StateFrameheadload;
			break;
		case StateAlarm:		//ͨѶʧ�ܴ���
			puyaotest3=4;
			if(CommunWithDUGS.Failure>599) //����У�����600�Σ����ڸ�λ
			{
				UART3_Init();
				FirstCommunSuccess =0;
				CommunState= StateFrameheadload;
				CommunWithDUGS.Failure =0;
				
			}
			else CommunState =StateWaiting;			
			if(InterCounter>999) 			//��������ʧ�ܳ���1000�Σ�ͨѶ��λ
			{
				DataRetrans =0;
				FirstCommunSuccess =0;
				CommunState= StateFrameheadload;
				InterCounter =0;
			}
			else CommunState =StateWaiting;
				
			break;
	}

	
}
/******************************************************************************
** ������: Frameheadload
**
** ���ܣ�֡ͷװ��
******************************************************************************/
uint32 Frameheadload(void)
{
	static uint8 count=0;
	CommunWithDUGS.Framehead=0x5AA5;
	if(DataRetrans==1){return CurrentFramehead;}//�����ط�
	if(FirstCommunSuccess ==0)//�״�֡ͷװ��
	{
		
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0X2001;					//0x0001

		NextFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);				//�¸�֡ͷ00830001
	}
	else
	{
		//if(DUGSInitFinishFlag==1)
		{
/*******************************************���豸��������*******************************************/	
			if(NextFramehead==0x832001&&FirstCommunSuccess ==1)												//�����һ��ͨѶ�ɹ��������¸�֡ͷΪ830001
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X20C8;	//8200c8    дѡ��
				
				FirstCommunSuccess =0x55;
				NoNeedWaitingFlag =1;
			}

			else if(NextFramehead==0x8220C8)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1500;//820500			 д��������
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821500)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1564;//830564			 дarm����1
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821564)															
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1D00;		//820D00   дarm����2
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821D00)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1E00;//	820e00   д��������
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821E00)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1600;			//820600		д��������1
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821600)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1800;		//820800			д��������2
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821800)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1900;	//820900   	д��������3
				
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821900)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1A00;			//820a00  д��Դ����

				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821A00)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1700;

				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821700)
			{
/*******************************************��ʵʱ��Ϣ*******************************************/
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X2013;//��������
												
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x822013)
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1C00;//dsp����
								
				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821C00)
			{

				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1100;//Ƶ������

				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821100)//
			{
				CommunWithDUGS.Funcode= 0x82;
				CommunWithDUGS.StartAddr= 0X1200;//�ӻ�����

				NoNeedWaitingFlag =1;
			}
			else if(NextFramehead==0x821200)
			{
				if(ManualPassiveSwitchFlag==1)	//�����ֶ���ԴͶ��
				{
					ManualPassiveSwitchFlagBak=1;
					CommunWithDUGS.Funcode= 0x83;
					CommunWithDUGS.StartAddr= 0X1A64;
				}
				else
				{
					CommunWithDUGS.Funcode= 0x83;
					CommunWithDUGS.StartAddr= 0X2001;


					DUGSSystemParametersInitFlag= 1;	//ϵͳ������ʼ�����
				}
			}
			else if(NextFramehead==0x831A64)
			{
				CommunWithDUGS.Funcode= 0x83;
				CommunWithDUGS.StartAddr= 0X2001;


				DUGSSystemParametersInitFlag= 1;
			}
			else
			{
				if(count>20)//����20�ζ�ȡ�ӻ���������Ŀ���ǻ�ȡ�ӻ��汾�ţ������ȡ����������
				{
					CommunWithDUGS.Funcode= 0x82;
					CommunWithDUGS.StartAddr= 0X2013;	

					NoNeedWaitingFlag =1;
				}
				else
				{
					CommunWithDUGS.Funcode= 0x82;
					CommunWithDUGS.StartAddr= 0X1700;	

					NoNeedWaitingFlag =1;
					count++;
				}
			}
			NextFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);
		}
	}
	


	if(DUGSInitFlag==1)//��Ļ��ʼ��
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x2001;	

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	

		DUGSInitFlag =0;
		NoNeedWaitingFlag =1;
	}
	else if(BackupFinishFlag||ReStoreBackupSuccess||ReStoreFactorySuccess||ClearFaultSuccess||SystemResetSuccess||IllegalOperation||SystemInitSuccess)
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x2003;	

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	

		BackupFinishFlag =0;
		ReStoreBackupSuccess =0;
		ReStoreFactorySuccess =0;
		ClearFaultSuccess =0;
		SystemResetSuccess =0;
		IllegalOperation =0;
		SystemInitSuccess =0;
		NoNeedWaitingFlag =1;
	}
	else if(OnOffChangeFlag==1)//�ֶ����ػ�
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x2004;	
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	

		OnOffChangeFlag =0;
		RecoverProcessFlag =1;
		NoNeedWaitingFlag =1;
	}
	else if(RecoverProcessFlag==1)//�����д��־λ
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x2005;	
		
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	

		RecoverProcessFlag =0;
		NoNeedWaitingFlag =1;
		w=0;
	}
	else if((OnekeyClean ==1)||(lanya_flag==1))/*(ManualPassiveSwitchFlag==0&&ManualPassiveSwitchFlagBak ==1)*///��Դ�����е�һ���г�
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1A64;	

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		NoNeedWaitingFlag =1;
	}	
	else if(HistoryFaultWindowFlag==1||EventLogWindowFlag==1)	//�����������Ͻ��棬�¼���¼����
	{				
		if(CurrentFramehead==0x822013)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= FaultDataStartaddr;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x821000)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1029;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x821029)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1052;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x821052)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x107B;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x82107B)
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x10a4;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x8210a4)
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x2001;

			NoNeedWaitingFlag =0;
		}
		else
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2013;
			
			NoNeedWaitingFlag =1;
		}		
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
	}
	else if(DataLogWindowFlag==1)	//���ݼ�¼����
	{
		if(CurrentFramehead==0x822013)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1b00;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x821B00)
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x2001;

			NoNeedWaitingFlag =0;
		}
		else
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2013;

			NoNeedWaitingFlag =1;
		}
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
	}
	else if(SlaveFaultWindowFlag==1)	//�ӻ����Ͻ���
	{
		if(CurrentFramehead==0x822013)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x211F;
			
			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x82211F)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2148;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x822148)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2171;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x822171)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x219A;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x82219A)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x21C3;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x8221C3)
		{	
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x21EC;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x8221EC)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2215;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x822215)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x223E;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x82223E)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2267;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x822267)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2290;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x822290)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x22B9;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x8222B9)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x22E2;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x8222E2)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x230B;

			NoNeedWaitingFlag =1;
		}
		else if(CurrentFramehead==0x82230B)
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2334;

			NoNeedWaitingFlag =1;
		}		
		else if(CurrentFramehead==0x822334)
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x2001;

			NoNeedWaitingFlag =0;
		}
		else
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2013;
			
			NoNeedWaitingFlag =1;
		}
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
	}
	else if(DUGSReadSlaveDataFlag==1)//��ȡ�ӻ�����
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1200;

		DUGSReadSlaveDataFlag =0;
		NoNeedWaitingFlag =1;
	}
	else if(DUGSReadParameterFlag==1)	//��ȡ8001-8007����
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1A00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadParameterFlag =0;

		NoNeedWaitingFlag =1;
	}	
	else if(DUGSWriteParameterFlag==1)//����8001-8007����
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1A00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteParameterFlag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSReadSlaveMaxMinFlag==1)//��ȡ�ӻ��߼�����
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1300;

		DUGSReadSlaveMaxMinFlag =0;
		NoNeedWaitingFlag =1;
	}
	else if(DUGSReadSelectedtimesFlag==1)//��ȡѡ�β���
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x20c8;
		
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadSelectedtimesFlag =0;

		NoNeedWaitingFlag =1;
	}
	else if(DUGSWriteSelectedtimesFlag==1)//����ѡ�β���
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x20c8;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteSelectedtimesFlag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSReadMainParameterFlag==1)//��ȡ��������
	{
		if(CurrentFramehead==0x821500)//�ȶ�ARM�������ٶ�����������Ŀ���Ƿ�ֹ����������ARM��������
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1564;
			
			RecoverProcessFlag =1;
			DUGSReadMainParameterFlag =0;
		}
		else
		{
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1500;
		}
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		NoNeedWaitingFlag =1;
	}
	else if(DUGSWriteMainParameterFlag==1)//������������
	{
		if(CurrentFramehead==0x831564)
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1500;			//��д��������

			RecoverProcessFlag =1;
			DUGSWriteMainParameterFlag =0;
		}
		else
		{
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1564;			//��дARM����
		}
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		NoNeedWaitingFlag =0;
	}
	else if(DUGSTimesetFlag==1)		//����ʱ��
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1567;	

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSTimesetFlag =0;
		
		NoNeedWaitingFlag =0;
	}
	else if(DUGSReadCorrectionParameterFlag==1)	//��ȡ��������1
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1600;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadCorrectionParameterFlag =0;

		NoNeedWaitingFlag =1;
	}
	else if(DUGSReadCorrectionParameter2Flag==1)//��ȡ��������2
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1800;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadCorrectionParameter2Flag =0;

		NoNeedWaitingFlag =1;
	}
	else if(DUGSReadCorrectionParameter3Flag==1)//��ȡ��������3
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1900;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadCorrectionParameter3Flag =0;

		NoNeedWaitingFlag =1;
	}


	else if(DUGSWriteCorrectionParameterFlag==1)//������������1
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1600;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteCorrectionParameterFlag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSWriteCorrectionParameter2Flag==1)//������������2
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1800;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteCorrectionParameter2Flag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSWriteCorrectionParameter3Flag==1)//������������3
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1900;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteCorrectionParameter3Flag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSReadPassiveParameterFlag==1)	//��ȡ��Դ����
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1a00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadPassiveParameterFlag =0;
		NoNeedWaitingFlag =1;
	}
	else if(DUGSWritePassiveParameterFlag==1)//������Դ����
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1a00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWritePassiveParameterFlag =0;
		NoNeedWaitingFlag =0;
	}

	else if(DUGSReadSlaveParameterFlag==1)//��ȡ�ӻ�����
	{
		
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1700;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);
		RecoverProcessFlag =1;
		if(ReadCount>1)DUGSReadSlaveParameterFlag =0;//���Ӷ�ȡ�ӻ�����ʱ�䣬��֤�ӻ������ܹ���ȡ

		NoNeedWaitingFlag =1;
	}
	else if(DUGSWriteSlaveParameterFlag==1)//����ӻ�����
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1700;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteSlaveParameterFlag =0;
		NoNeedWaitingFlag =0;
	}
	else if(DUGSReadARMCorrectFlag==1)//��ȡARM��������
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1e00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSReadARMCorrectFlag =0;
		NoNeedWaitingFlag =1;
	}
	else if(DUGSWriteARMCorrectFlag==1)//����ARM��������
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1e00;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWriteARMCorrectFlag =0;
		NoNeedWaitingFlag =0;
	}
	else if (DUGSRead666Flag==1)//��ȡ666����
	{
		CommunWithDUGS.Funcode= 0x82;
		CommunWithDUGS.StartAddr= 0x1AAA;
		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSRead666Flag=0;
		NoNeedWaitingFlag =1;
	}
	else if (DUGSWrite666Flag==1)//д��666����
	{
		CommunWithDUGS.Funcode= 0x83;
		CommunWithDUGS.StartAddr= 0x1AAA;

		CurrentFramehead =(uint32)((CommunWithDUGS.Funcode <<16)|CommunWithDUGS.StartAddr);	
		RecoverProcessFlag =1;
		DUGSWrite666Flag =0;
		NoNeedWaitingFlag =0;
	}
	else CurrentFramehead =NextFramehead;
		
	return(CurrentFramehead);
}
void SendDataload(uint32 Framehead)
{
	uint16 i,j,SendDataBuff,WaveformData[50],WaveformPhase[50],current_phaseA,current_phaseB,current_phaseC,current_Max;
	uint16	var_histogram,tmp16,tmp17,tmp18;
	uint8  var_DecPo1,addr,Tmp_DataLog[76];
	//static uint16 FaultRecordPage=1;
	//int16 SignTmp,CurrentA,CurrentB;
	uint32 Tmp32,tmp31;
	int32 tmp15,itmp14;
	float 	Sum;
	//static uint8 Currentchange=0;
	switch(Framehead)
	{
		case 0x832001:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x20001;
			CommunWithDUGS.Datalength= PageDataNUM;
			break;
		case 0x822001:
			CommunWithDUGS.Framelength =35;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2001;

			CommunWithDUGS.Databuff[0]=ScreenType;				//��Ļ����
			CommunWithDUGS.Databuff[1]=LogoType;				//Logo����
			CommunWithDUGS.Databuff[2]=RestoreCommand;			//�ָ���������
			CommunWithDUGS.Databuff[3]=OnOffCommand;			//���ػ�����
			CommunWithDUGS.Databuff[4]=SaveOrReadFlag;			//��ȡ�����Ӧ��
			CommunWithDUGS.Databuff[5]=ManualPassiveSwitchFlag;	//��Դ�ֶ�Ͷ�в���
			CommunWithDUGS.Databuff[6]=Password;				//����
			CommunWithDUGS.Databuff[7]=HistogramSetting;		//����ͼ��ʾ
			CommunWithDUGS.Databuff[8]=WaveformAccuracy;		//���ξ���
			CommunWithDUGS.Databuff[9]=WaveformID;				//����ͼID
			CommunWithDUGS.Databuff[10]=Waveform2ID;			//����ͼ2ID
			CommunWithDUGS.Databuff[11]=HistogramID;			//��״ͼID
			CommunWithDUGS.Databuff[12]=HistogramPageID;		//��״ͼ��ҳID
			CommunWithDUGS.Databuff[13]=SlaveID;				//�ӻ�ҳ��ID
			CommunWithDUGS.Databuff[14]=CorrectionID;			//��������ҳ��
			CommunWithDUGS.Databuff[15]=PrimevalID;				//��������ID
			CommunWithDUGS.Databuff[16]=CurrentPageID;			//ҳ��ID
			CommunWithDUGS.Databuff[17]=FaultRecordID;			//����ID	
			CommunWithDUGS.Databuff[18]=DispMode;				//�豸״̬
			CommunWithDUGS.Databuff[19]=SwitchON;			//���ػ�״̬//ͨѶ״̬
			CommunWithDUGS.Databuff[20]=local_time.RTC_Year;
			CommunWithDUGS.Databuff[21]=local_time.RTC_Mon;
			CommunWithDUGS.Databuff[22]=local_time.RTC_Mday;
			CommunWithDUGS.Databuff[23]=local_time.RTC_Hour;
			CommunWithDUGS.Databuff[24]=local_time.RTC_Min;
			CommunWithDUGS.Databuff[25]=(uint16)(DailyEnergy>>16);
			CommunWithDUGS.Databuff[26]=(uint16)DailyEnergy;
			CommunWithDUGS.Databuff[27]=(uint16)(TotalEnergyHB>>16);
			CommunWithDUGS.Databuff[28]=(uint16)(TotalEnergyHB);
			CommunWithDUGS.Databuff[29]=(uint16)(TotalEnergyLB>>16);
			CommunWithDUGS.Databuff[30]=(uint16)TotalEnergyLB;
			CommunWithDUGS.Databuff[31]=(uint16)(TotalRunningTime>>16);
			CommunWithDUGS.Databuff[32]=(uint16)(TotalRunningTime);
			CommunWithDUGS.Databuff[33]=DailyRunningTime;
			CommunWithDUGS.Databuff[34]=ResetCount;
			
			DUGSInitFinishFlag =1;
			break;
		case 0x822013:
			CommunWithDUGS.Framelength =UserDataNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2013;
			
			CommunWithDUGS.Databuff[0]=DispMode;					//�豸״̬
			CommunWithDUGS.Databuff[1]=SwitchON;				//���ػ�״̬//ͨѶ״̬
			CommunWithDUGS.Databuff[2]=local_time.RTC_Year;
			CommunWithDUGS.Databuff[3]=local_time.RTC_Mon;
			CommunWithDUGS.Databuff[4]=local_time.RTC_Mday;
			CommunWithDUGS.Databuff[5]=local_time.RTC_Hour;
			CommunWithDUGS.Databuff[6]=local_time.RTC_Min;
			CommunWithDUGS.Databuff[7]=(uint16)(DailyEnergy>>16);
			CommunWithDUGS.Databuff[8]=(uint16)DailyEnergy;
			CommunWithDUGS.Databuff[9]=(uint16)(TotalEnergyHB>>16);
			CommunWithDUGS.Databuff[10]=(uint16)(TotalEnergyHB);
			CommunWithDUGS.Databuff[11]=(uint16)(TotalEnergyLB>>16);
			CommunWithDUGS.Databuff[12]=(uint16)TotalEnergyLB;
			CommunWithDUGS.Databuff[13]=(uint16)(TotalRunningTime>>16);
			CommunWithDUGS.Databuff[14]=(uint16)(TotalRunningTime);
			CommunWithDUGS.Databuff[15]=DailyRunningTime;
			CommunWithDUGS.Databuff[16]=ResetCount;
			Tmp32 =ARMVersion;
			CommunWithDUGS.Databuff[17]=(uint16)(Tmp32>>16);
			CommunWithDUGS.Databuff[18]=(uint16)(Tmp32);
			//Tmp32 =DSP1Version;
			CommunWithDUGS.Databuff[19] =dsp_data[36]+dsp_data[37]+dsp_data[38];	//ϵͳ�����ڹ���
			CommunWithDUGS.Databuff[20] =dsp_data[39]+dsp_data[40]+dsp_data[41];	//ϵͳ���й�����
			Tmp32 =DSP2Version;
			CommunWithDUGS.Databuff[21]=(uint16)(Tmp32>>16);
			CommunWithDUGS.Databuff[22]=(uint16)(Tmp32);
			Tmp32 =EepromVersion;
			CommunWithDUGS.Databuff[23]=(uint16)(Tmp32>>16);
			CommunWithDUGS.Databuff[24]=(uint16)(Tmp32);

			CommunWithDUGS.Databuff[25]=capa[1];//���ߵ��ݵ�ַ��
			CommunWithDUGS.Databuff[26]=capa[5];//���ߵ��ݵ�ַ��
			CommunWithDUGS.Databuff[27]=capa[6];//���ߵ��ݵ�ַ��
			CommunWithDUGS.Databuff[28]=capa[7];//���ߵ��ݵ�ַ��
			CommunWithDUGS.Databuff[29]=capa[8];//���ߵ��ݵ�ַ��
			CommunWithDUGS.Databuff[30]=capa[9];//���ߵ��ݵ�ַ��

			if(HistogramID<=20)
			{
				if(HistogramID>0)tmp16=(HistogramID-1)*12;
				else tmp16=0;
				CommunWithDUGS.Databuff[31]=capa[20+tmp16];//��ַ
				CommunWithDUGS.Databuff[32]=capa[21+tmp16];//����
				CommunWithDUGS.Databuff[33]=capa[22+tmp16];//����
				CommunWithDUGS.Databuff[34]=capa[23+tmp16];//״̬

				CommunWithDUGS.Databuff[35]=capa[24+tmp16];//
				CommunWithDUGS.Databuff[36]=capa[25+tmp16];
				CommunWithDUGS.Databuff[37]=capa[26+tmp16];
				CommunWithDUGS.Databuff[38]=capa[27+tmp16];

				CommunWithDUGS.Databuff[39]=capa[28+tmp16];//
				CommunWithDUGS.Databuff[40]=capa[29+tmp16];
				CommunWithDUGS.Databuff[41]=capa[30+tmp16];
				CommunWithDUGS.Databuff[42]=capa[31+tmp16];
			}
			

			CommunWithDUGS.Databuff[52] =SlaveStatus[0];
			CommunWithDUGS.Databuff[53] =SlaveErrorCode[0];
			CommunWithDUGS.Databuff[54] =SlaveStatus[1];
			CommunWithDUGS.Databuff[55] =SlaveErrorCode[1];
			CommunWithDUGS.Databuff[56] =SlaveStatus[2];
			CommunWithDUGS.Databuff[57] =SlaveErrorCode[2];
			CommunWithDUGS.Databuff[58] =SlaveStatus[3];
			CommunWithDUGS.Databuff[59] =SlaveErrorCode[3];
			CommunWithDUGS.Databuff[60] =SlaveStatus[4];
			CommunWithDUGS.Databuff[61] =SlaveErrorCode[4];
			CommunWithDUGS.Databuff[62] =SlaveStatus[5];
			CommunWithDUGS.Databuff[63] =SlaveErrorCode[5];
			CommunWithDUGS.Databuff[64] =SlaveStatus[6];
			CommunWithDUGS.Databuff[65] =SlaveErrorCode[6];
			CommunWithDUGS.Databuff[66] =SlaveStatus[7];
			CommunWithDUGS.Databuff[67] =SlaveErrorCode[7];
			CommunWithDUGS.Databuff[68] =SlaveStatus[8];
			CommunWithDUGS.Databuff[69] =SlaveErrorCode[8];
			CommunWithDUGS.Databuff[70] =SlaveStatus[9];
			CommunWithDUGS.Databuff[71] =SlaveErrorCode[9];

			CommunWithDUGS.Databuff[72] =SlaveStatus[10];
			CommunWithDUGS.Databuff[73] =SlaveErrorCode[10];
			CommunWithDUGS.Databuff[74] =SlaveStatus[11];
			CommunWithDUGS.Databuff[75] =SlaveErrorCode[11];
			CommunWithDUGS.Databuff[76] =SlaveStatus[12];
			CommunWithDUGS.Databuff[77] =SlaveErrorCode[12];
			CommunWithDUGS.Databuff[78] =SlaveStatus[13];
			CommunWithDUGS.Databuff[79] =SlaveErrorCode[13];
			CommunWithDUGS.Databuff[80] =SlaveStatus[14];
			CommunWithDUGS.Databuff[81] =SlaveErrorCode[14];
			CommunWithDUGS.Databuff[82] =SlaveStatus[15];
			CommunWithDUGS.Databuff[83] =SlaveErrorCode[15];
			CommunWithDUGS.Databuff[84] =0;
			CommunWithDUGS.Databuff[85] =0;
			CommunWithDUGS.Databuff[86] =0;
			CommunWithDUGS.Databuff[87] =0;
			CommunWithDUGS.Databuff[88] =0;
			CommunWithDUGS.Databuff[89] =0;
			CommunWithDUGS.Databuff[90] =0;
			CommunWithDUGS.Databuff[91] =0;
			CommunWithDUGS.Databuff[92] =(uint16)(SerialNumber>>48);
			CommunWithDUGS.Databuff[93] =(uint16)(SerialNumber>>32);
			CommunWithDUGS.Databuff[94] =(uint16)(SerialNumber>>16);
			CommunWithDUGS.Databuff[95] =(uint16)SerialNumber;
			//CommunWithDUGS.Databuff[96]=RecvDGUSSuccessTest;
			//CommunWithDUGS.Databuff[97]=SendDGUSSuccessTest;
			//CommunWithDUGS.Databuff[98]=RecvDTUSuccessTest;
			//CommunWithDUGS.Databuff[99]=SendDTUSuccessTest;
			break;
		case 0x822003://����߼�����
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2003;
			
			CommunWithDUGS.Databuff[0]= 0; 		
			break;
		case 0x822004://���ػ�
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2004;
			
			CommunWithDUGS.Databuff[0]= OnOffCommand; 		
			break;
		case 0x822005://��д��־
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2005;


			CommunWithDUGS.Databuff[0]=	0x00; 				

			break;
		case 0x822006://�߼�����
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2006;
			//�������ݵȲ���
			if(BackupFinishFlag||ReStoreBackupSuccess||ReStoreFactorySuccess||ClearFaultSuccess||SystemResetSuccess)
			{
				CommunWithDUGS.Databuff[0]=	0x02; 				//�ɹ�
				BackupFinishFlag =0;
				ReStoreBackupSuccess =0;
				ReStoreFactorySuccess =0;
				ClearFaultSuccess =0;
				SystemResetSuccess =0;
			}
			else if(ReStoreBackupFailure||ReStoreFactoryFailure)
			{
				CommunWithDUGS.Databuff[0]=	0x03; 				//ʧ��
				ReStoreBackupFailure =0;
				ReStoreFactoryFailure =0;
			}
			else if(IllegalOperation)
			{
				CommunWithDUGS.Databuff[0]=	0x04; 				//�Ƿ�����
			}
			else
			{
				CommunWithDUGS.Databuff[0]=	0x00;
			}
			break;
		case 0x821C00:
			CommunWithDUGS.Framelength =MainDataNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1c00;
			
			for(i=0;i<100;i++)
			{
				CommunWithDUGS.Databuff[i] =dsp_data[i];
			}
			tmp31=dsp_data[3]*xiuzhen_dianyajibian2/10000;
			CommunWithDUGS.Databuff[3]=(uint16)(tmp31);//��ya������
			tmp31=dsp_data[4]*xiuzhen_dianyajibian2/10000;
			CommunWithDUGS.Databuff[4]=(uint16)(tmp31);//��ya������
			tmp31=dsp_data[5]*xiuzhen_dianyajibian2/10000;
			CommunWithDUGS.Databuff[5]=(uint16)(tmp31);//��ya������
			
			tmp31=dsp_data[10]*xiuzhen_dianliujibian2/10000;
			CommunWithDUGS.Databuff[10]=(uint16)(tmp31);//����������
			tmp31=dsp_data[11]*xiuzhen_dianliujibian2/10000;
			CommunWithDUGS.Databuff[11]=(uint16)(tmp31);//����������
			tmp31=dsp_data[12]*xiuzhen_dianliujibian2/10000;
			CommunWithDUGS.Databuff[12]=(uint16)(tmp31);//����������
			
			if(dsp_data[42]>32768)//�ж�A�޹������Ƿ�Ϊ��
				itmp14=0-(65536-dsp_data[42]);
			else
				itmp14=dsp_data[42];				
			tmp15=itmp14*10000/xiuzhen_wugong2;
			CommunWithDUGS.Databuff[42]=(uint16)(tmp15);//�޹�����
			
			if(dsp_data[43]>32768)//�ж�b�޹������Ƿ�Ϊ��
				itmp14=0-(65536-dsp_data[43]);
			else
				itmp14=dsp_data[43];				
			tmp15=itmp14*10000/xiuzhen_wugong2;
			CommunWithDUGS.Databuff[43]=(uint16)(tmp15);//�޹�����
			
			if(dsp_data[44]>32768)//�ж�c�޹������Ƿ�Ϊ��
				itmp14=0-(65536-dsp_data[44]);
			else
				itmp14=dsp_data[44];				
			tmp15=itmp14*10000/xiuzhen_wugong2;
			CommunWithDUGS.Databuff[44]=(uint16)(tmp15);//�޹�����
			
			
			tmp31=dsp_data[45]*10000/xiuzhen_jibian2;
			CommunWithDUGS.Databuff[45]=(uint16)(tmp31);//���书��
			tmp31=dsp_data[46]*10000/xiuzhen_jibian2;
			CommunWithDUGS.Databuff[46]=(uint16)(tmp31);//���书��
			tmp31=dsp_data[47]*10000/xiuzhen_jibian2;
			CommunWithDUGS.Databuff[47]=(uint16)(tmp31);//���书��
			
			
			if(dsp_data[42]>32768)//�ж�A�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[42];
			else
				tmp16=dsp_data[42];			
			if(dsp_data[45]>32768)//�ж�A���书���Ƿ�Ϊ��
				tmp17=65536-dsp_data[45];
			else
				tmp17=dsp_data[45];				
			if(dsp_data[39]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[39];
			else
				tmp18=dsp_data[39];		
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16)*(tmp16)+(tmp17)*(tmp17)))*10000/xiuzhen_pf2;
			if(dsp_data[42]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[13]=(uint16)(tmp15);//PF	A
			
			if(dsp_data[43]>32768)//�ж�B�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[43];
			else
				tmp16=dsp_data[43];			
			if(dsp_data[46]>32768)//�ж�B���书���Ƿ�Ϊ��
				tmp17=65536-dsp_data[46];
			else
				tmp17=dsp_data[46];
			if(dsp_data[40]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[40];
			else
				tmp18=dsp_data[40];		
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16)*(tmp16)+(tmp17)*(tmp17)))*10000/xiuzhen_pf2;
			if(dsp_data[43]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[14]=(uint16)(tmp15);//PF	B
	
			if(dsp_data[44]>32768)//�ж�C�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[44];
			else
				tmp16=dsp_data[44];			
			if(dsp_data[47]>32768)//�ж�C���书���Ƿ�Ϊ��
				tmp17=65536-dsp_data[47];
			else
				tmp17=dsp_data[47];
			if(dsp_data[41]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[41];
			else
				tmp18=dsp_data[41];		
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+(tmp16)*(tmp16)+(tmp17)*(tmp17)))*10000/xiuzhen_pf2;
			if(dsp_data[44]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[15]=(uint16)(tmp15);//PF  C
			
	//	CommunWithDUGS.Databuff[14]=(uint16)(dsp_data[14]*xiuzhen_pf2/10000);
		
	//	CommunWithDUGS.Databuff[15]=(uint16)(dsp_data[15]*xiuzhen_pf2/10000);

			if(dsp_data[42]>32768)//�ж�A�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[42];
			else
				tmp16=dsp_data[42];	
			
			if(dsp_data[39]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[39];
			else
				tmp18=dsp_data[39];
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+tmp16*tmp16))*10000/xiuzhen_cos2;
			if(dsp_data[42]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[16]=(uint16)(tmp15);//Acos��
			
	
			if(dsp_data[43]>32768)//�ж�B�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[43];
			else
				tmp16=dsp_data[43];			
			if(dsp_data[40]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[40];
			else
				tmp18=dsp_data[40];
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+tmp16*tmp16))*10000/xiuzhen_cos2;
			if(dsp_data[43]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[17]=(uint16)(tmp15);//Bcos��	
	
	
			if(dsp_data[44]>32768)//�ж�C�޹������Ƿ�Ϊ��
				tmp16=65536-dsp_data[44];
			else
				tmp16=dsp_data[44];			
			if(dsp_data[41]>32768)//�ж�A�й������Ƿ�Ϊ��
				tmp18=65536-dsp_data[41];
			else
				tmp18=dsp_data[41];
			tmp15=(tmp18*1000) /(sqrt(tmp18*tmp18+tmp16*tmp16))*10000/xiuzhen_cos2;
			if(dsp_data[44]>32768) tmp15=-1*tmp15;
			CommunWithDUGS.Databuff[18]=(uint16)(tmp15);//Ccos��
			
			tmp31=dsp_data[32]*xiuzhen_shuchu2/10000;
			CommunWithDUGS.Databuff[32]=(uint16)(tmp31);//�������
			tmp31=dsp_data[33]*xiuzhen_shuchu2/10000;
			CommunWithDUGS.Databuff[33]=(uint16)(tmp31);//�������
			tmp31=dsp_data[34]*xiuzhen_shuchu2/10000;
			CommunWithDUGS.Databuff[34]=(uint16)(tmp31);//�������
			

			//�ĳɸ�����г������	sqrt��2��2+3��2+������������+50��2��
			zongxiebo_A=0;
			zongxiebo_B=0;
			zongxiebo_C=0;
			for(i=1;i<50;i++)
			{
				if(Selected_parameter[i]==1)
				{
					zongxiebo_A+=(system_currentA[i]*10000/xiuzhen_fuzhi2)*(system_currentA[i]*10000/xiuzhen_fuzhi2);
					zongxiebo_B+=(system_currentB[i]*10000/xiuzhen_fuzhi2)*(system_currentB[i]*10000/xiuzhen_fuzhi2);
					zongxiebo_C+=(system_currentC[i]*10000/xiuzhen_fuzhi2)*(system_currentC[i]*10000/xiuzhen_fuzhi2);
				}
				else
				{
					zongxiebo_A+=system_currentA[i]*system_currentA[i];
					zongxiebo_B+=system_currentB[i]*system_currentB[i];
					zongxiebo_C+=system_currentC[i]*system_currentC[i];
				}
				
			}
			
			CommunWithDUGS.Databuff[100]=(uint16)(sqrt(zongxiebo_A));//A����г������
			CommunWithDUGS.Databuff[101]=(uint16)(sqrt(zongxiebo_B));//B����г������
			CommunWithDUGS.Databuff[102]=(uint16)(sqrt(zongxiebo_C));//C����г������
			
			CommunWithDUGS.Databuff[103]=CTOutcurA;
			CommunWithDUGS.Databuff[104]=CTOutcurB;
			CommunWithDUGS.Databuff[105]=CTOutcurC;
			CommunWithDUGS.Databuff[106]=Ubalance;
			CommunWithDUGS.Databuff[107]=HighVolA;
			CommunWithDUGS.Databuff[108]=HighVolB;
			CommunWithDUGS.Databuff[109]=HighVolC;
			
			break;
		case 0x8220C8:
			CommunWithDUGS.Framelength =SelectedNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x20c8;

			for(i=0;i<19;i++)
			{
				j=main_parameter[31+i];
				if(j>0)Selected_parameter[j-1]=1;
			}
			Selected_parameter[50]=AI_select;//��������ѡ��
			
			for(i=0;i<SelectedNUM;i++)
			{
				CommunWithDUGS.Databuff[i] =Selected_parameter[i];
			}

			break;
		case 0x821100:
			CommunWithDUGS.Framelength =RunningDataNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1100;

			for(i=0;i<50;i++)
			{
				switch(HistogramPageID)
				{
					case 10:
						CommunWithDUGS.Databuff[i] =votageA[i];
						CommunWithDUGS.Databuff[i+50] =votage_phaseA[i];
						break;
					case 11:
						CommunWithDUGS.Databuff[i] =votageB[i];
						CommunWithDUGS.Databuff[i+50] =votage_phaseB[i];
						break;
					case 12:
						CommunWithDUGS.Databuff[i] =votageC[i];
						CommunWithDUGS.Databuff[i+50] =votage_phaseC[i];
						break;
					case 13:
						CommunWithDUGS.Databuff[0] =system_currentA[0];						
					  if(Selected_parameter[i]==1)	CommunWithDUGS.Databuff[i] =(uint16)(system_currentA[i]*10000/xiuzhen_fuzhi2);
						else CommunWithDUGS.Databuff[i] =system_currentA[i];
						CommunWithDUGS.Databuff[i+50] =system_phaseA[i];
						break;
					case 14:
						CommunWithDUGS.Databuff[0] =system_currentB[0];
						if(Selected_parameter[i]==1)	CommunWithDUGS.Databuff[i] =(uint16)(system_currentB[i]*10000/xiuzhen_fuzhi2);
						else CommunWithDUGS.Databuff[i] =system_currentB[i];
						CommunWithDUGS.Databuff[i+50] =system_phaseB[i];
						break;
					case 15:
						CommunWithDUGS.Databuff[0] =system_currentC[0];
						if(Selected_parameter[i]==1)	CommunWithDUGS.Databuff[i] =(uint16)(system_currentC[i]*10000/xiuzhen_fuzhi2);
						else CommunWithDUGS.Databuff[i] =system_currentC[i];
						CommunWithDUGS.Databuff[i+50] =system_phaseC[i];
						break;
					case 16:
						//�ӻ�����
						if(i<10)
						{
							CommunWithDUGS.Databuff[i] =slave_data[SlaveID][i];
							CommunWithDUGS.Databuff[i+50] =slave_Maxmin[SlaveID][i];
						}						
						break;
					case 17:
						//dsp����
						CommunWithDUGS.Databuff[i] =dsp_data[i];
						CommunWithDUGS.Databuff[i+50] =dsp_data[50+i];
						break;
					case 18:
						CommunWithDUGS.Databuff[i] =capa[i];
						CommunWithDUGS.Databuff[i+50] =capa[50+i];
						break;
					case 19:
						CommunWithDUGS.Databuff[i] =capa[100+i];
						CommunWithDUGS.Databuff[i+50] =capa[150+i];
						break;
					case 20:
						CommunWithDUGS.Databuff[i] =capa[200+i];
						CommunWithDUGS.Databuff[i+50] =capa[250+i];
						break;
					default:
						CommunWithDUGS.Databuff[i] =votageA[i];
						CommunWithDUGS.Databuff[i+50] =votage_phaseA[i];
						break;
				}
			}
			break;
		case 0x821200:
			CommunWithDUGS.Framelength =SlaveDataNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1200;
				
			for(i=0;i<10;i++)
			{
				CommunWithDUGS.Databuff[i]=slave_data[SlaveID][i]; 
			}
			break;
		case 0x821300:
			CommunWithDUGS.Framelength =SlaveMaxMinNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1300;
			
			for(i=0;i<10;i++)
			{
				CommunWithDUGS.Databuff[i]=slave_Maxmin[SlaveID][i]; 
			}
			break;

		case 0x821500:
			CommunWithDUGS.Framelength =MainParameterNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1500;
			for(i=0;i<100;i++)
			{
				CommunWithDUGS.Databuff[i]=main_parameter[i];
			}	
			CommunWithDUGS.Databuff[10] =main_parameter[10]%10;//��������
			CommunWithDUGS.Databuff[15] =main_parameter[15]%10;//����������

			break;
		case 0x821564:
			CommunWithDUGS.Framelength =ARMParameterNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1564;
			
			CommunWithDUGS.Databuff[0] = LocalAddr;				//0564
			CommunWithDUGS.Databuff[1] = UART_BPS;				//0565
			CommunWithDUGS.Databuff[2] =RemoteEnable;			//0566
			CommunWithDUGS.Databuff[3] =local_time.RTC_Year;	//0567
			CommunWithDUGS.Databuff[4] =local_time.RTC_Mon;		//0568
			CommunWithDUGS.Databuff[5] =local_time.RTC_Mday;	//0569
			CommunWithDUGS.Databuff[6] =local_time.RTC_Hour;	//056A
			CommunWithDUGS.Databuff[7] =local_time.RTC_Min;		//056B			
			CommunWithDUGS.Databuff[8] =OnOffStatus;			//056C
			
			CommunWithDUGS.Databuff[9] =CT_MAIN1;			//056D CTλ��1
			CommunWithDUGS.Databuff[10] =CT_MAIN2;			//056E CTλ��2
			// CommunWithDUGS.Databuff[11] =1;								//056F 
			// CommunWithDUGS.Databuff[12] =main_parameter[0];			//0570 
			// CommunWithDUGS.Databuff[13] =main_parameter[93];		//0571 
			
			// CommunWithDUGS.Databuff[14] =AlarmTime2[0];			//0572 
			// CommunWithDUGS.Databuff[15] =AlarmTime2[1];			//0573
			// CommunWithDUGS.Databuff[16] =AlarmTime2[2];			//0574
			// CommunWithDUGS.Databuff[17] =AlarmTime2[3];			//0575
			// CommunWithDUGS.Databuff[18] =AlarmTime2[4];			//0576
			
			// CommunWithDUGS.Databuff[19] =AlarmTime3[0];			//0577
			// CommunWithDUGS.Databuff[20] =AlarmTime3[1];			//0578
			// CommunWithDUGS.Databuff[21] =AlarmTime3[2];			//0579
			// CommunWithDUGS.Databuff[22] =AlarmTime3[3];			//057A
			// CommunWithDUGS.Databuff[23] =AlarmTime3[4];			//057B
		
			 CommunWithDUGS.Databuff[24] =ntc_type;//AlarmTime4[0];			//057C
			// CommunWithDUGS.Databuff[25] =0;			//057D
			// CommunWithDUGS.Databuff[26] =0;			//057E
			// CommunWithDUGS.Databuff[27] =0;			//057F
			// CommunWithDUGS.Databuff[28] =0;			//0580

			// HarmonicBal =main_parameter[6]/100;tmp16 =main_parameter[6]%100;
			// ReactiveBal =tmp16/10;			   tmp16 =main_parameter[6]%10;
			// ActiveBal	=tmp16;
			// if(ReactiveBal==1||HarmonicBal==1)ApparentBal=1;
			
			// CommunWithDUGS.Databuff[29] =ActiveBal;			//0581
			// CommunWithDUGS.Databuff[30] =ReactiveBal;		//0582
			// CommunWithDUGS.Databuff[31] =HarmonicBal;		//0583
			// CommunWithDUGS.Databuff[32] =ApparentBal;		//0584
			// CommunWithDUGS.Databuff[33] =SetupMode;		//0585
			// CommunWithDUGS.Databuff[34] =enhance;		//0586
			// CommunWithDUGS.Databuff[35] =0;		//0587
			// CommunWithDUGS.Databuff[36] =0;		//0588
			// CommunWithDUGS.Databuff[37] =0;		//0589
			// CommunWithDUGS.Databuff[38] =0;		//058A
			// CommunWithDUGS.Databuff[39] =0;		//058B

			// CommunWithDUGS.Databuff[40] =(uint16)(ProjectNo>>16);//058C
			// CommunWithDUGS.Databuff[41] =(uint16)ProjectNo;		//058D
			// CommunWithDUGS.Databuff[42] =ProductionNo;			//058E

			// //VolOnOffEnable =main_parameter[73]%10;
			// CommunWithDUGS.Databuff[43] =0;						//��ѹ����ʹ��//058F
			// CurOnOffEnable =main_parameter[73]/10;
			// CommunWithDUGS.Databuff[44] =CurOnOffEnable;		//��������ʹ��//0590

			// MainCTPhase =main_parameter[10]/10000;tmp16 =main_parameter[10]%10000;
			// MainCTDirectionC=tmp16/1000;tmp16 =main_parameter[10]%1000;
			// MainCTDirectionB=tmp16/100;tmp16 =main_parameter[10]%100;
			// MainCTDirectionA=tmp16/10;tmp16 =main_parameter[10]%10;
			// MainCTLocation  =tmp16;
			
			// CommunWithDUGS.Databuff[45] =MainCTLocation;		//��������λ��//0591
			// CommunWithDUGS.Databuff[46] =MainCTDirectionA;		//������������A//0592
			// CommunWithDUGS.Databuff[47] =MainCTDirectionB;		//������������B//0593
			// CommunWithDUGS.Databuff[48] =MainCTDirectionC;		//������������C//0594
			// CommunWithDUGS.Databuff[49] =MainCTPhase; 			//������������//0595
			
			// OutCTPhase =main_parameter[15]/10000;tmp16 =main_parameter[15]%10000;
			// OutCTDirectionC=tmp16/1000;tmp16 =main_parameter[15]%1000;
			// OutCTDirectionB=tmp16/100;tmp16 =main_parameter[15]%100;
			// OutCTDirectionA=tmp16/10;tmp16 =main_parameter[15]%10;
			
			// CommunWithDUGS.Databuff[50] =OutCTPhase; 			//��������������//0596
			
			// CommunWithDUGS.Databuff[51] =Position[0];			//��λ2	
			// CommunWithDUGS.Databuff[52] =Group[0];				//��2
			// CommunWithDUGS.Databuff[53] =Capacitance[0];		//��ֵ2

			// CommunWithDUGS.Databuff[54] =Position[1];			//��λ3	
			// CommunWithDUGS.Databuff[55] =Group[1];				//��3
			// CommunWithDUGS.Databuff[56] =Capacitance[1];		//��ֵ3

			// CommunWithDUGS.Databuff[57] =Position[2];			//��λ4	
			// CommunWithDUGS.Databuff[58] =Group[2];				//��4
			// CommunWithDUGS.Databuff[59] =Capacitance[2];		//��ֵ4

			// CommunWithDUGS.Databuff[60] =Position[3];			//��λ5	
			// CommunWithDUGS.Databuff[61] =Group[3];				//��5
			// CommunWithDUGS.Databuff[62] =Capacitance[3];		//��ֵ5

			// CommunWithDUGS.Databuff[63] =Position[4];			//��λ6	
			// CommunWithDUGS.Databuff[64] =Group[4];				//��6
			// CommunWithDUGS.Databuff[65] =Capacitance[4];		//��ֵ6

			// CommunWithDUGS.Databuff[66] =Position[5];			//��λ7	
			// CommunWithDUGS.Databuff[67] =Group[5];				//��7
			// CommunWithDUGS.Databuff[68] =Capacitance[5];		//��ֵ7

			// CommunWithDUGS.Databuff[69] =Position[6];			//��λ8	
			// CommunWithDUGS.Databuff[70] =Group[6];				//��8
			// CommunWithDUGS.Databuff[71] =Capacitance[6];		//��ֵ8

			// CommunWithDUGS.Databuff[72] =Position[7];			//��λ9	
			// CommunWithDUGS.Databuff[73] =Group[7];				//��9
			// CommunWithDUGS.Databuff[74] =Capacitance[7];		//��ֵ9

			// CommunWithDUGS.Databuff[75] =Position[8];			//��λ10	
			// CommunWithDUGS.Databuff[76] =Group[8];				//��10
			// CommunWithDUGS.Databuff[77] =Capacitance[8];		//��ֵ10

			// CommunWithDUGS.Databuff[78] =OutCTDirectionA;		//�������������A
			// CommunWithDUGS.Databuff[79] =OutCTDirectionB;		//�������������B
			// CommunWithDUGS.Databuff[80] =OutCTDirectionC;		//�������������C

			// CommunWithDUGS.Databuff[81] =Position[9];			//��λ11	
			// CommunWithDUGS.Databuff[82] =Group[9];				//��11
			// CommunWithDUGS.Databuff[83] =Capacitance[9];		//��ֵ11

			// CommunWithDUGS.Databuff[84] =Position[10];			//��λ12	
			// CommunWithDUGS.Databuff[85] =Group[10];				//��12
			// CommunWithDUGS.Databuff[86] =Capacitance[10];		//��ֵ12

			// CommunWithDUGS.Databuff[87] =Position[11];			//��λ13	
			// CommunWithDUGS.Databuff[88] =Group[11];				//��13
			// CommunWithDUGS.Databuff[89] =Capacitance[11];		//��ֵ13

			// CommunWithDUGS.Databuff[90] =Position[12];			//��λ14	
			// CommunWithDUGS.Databuff[91] =Group[12];				//��14
			// CommunWithDUGS.Databuff[92] =Capacitance[12];		//��ֵ14

			// CommunWithDUGS.Databuff[93] =Position[13];			//��λ15	
			// CommunWithDUGS.Databuff[94] =Group[13];				//��15
			// CommunWithDUGS.Databuff[95] =Capacitance[13];		//��ֵ15

			// CommunWithDUGS.Databuff[96] =Position[14];			//��λ16
			// CommunWithDUGS.Databuff[97] =Group[14];				//��16
			// CommunWithDUGS.Databuff[98] =Capacitance[14];		//��16
			// CommunWithDUGS.Databuff[99] =0;						//Ԥ��
			
			for(i=0; i<100; i++)	
			{
				ARM_param[i] =	CommunWithDUGS.Databuff[i];//ARM������
			}
			
			break;
		case 0x821D00:
			CommunWithDUGS.Framelength =ARMParameterNUM2;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1D00;
			j=0;
			for(i=0;i<32;i++)
			{
				CommunWithDUGS.Databuff[j++]=PassiveChannel[i];
				CommunWithDUGS.Databuff[j++]=PassiveValue[i];
			}
			CommunWithDUGS.Databuff[j++] =PassiveCom;			//��Դ�豸
			for(i=j;i<100;i++)
			{
				CommunWithDUGS.Databuff[i]=0;
			}
			break;
		case 0x821AAA://���������ȡ
			CommunWithDUGS.Framelength =11;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1AAA;	
		
			CommunWithDUGS.Databuff[0] = xiuzhen_dianliujibian;				//��������������
			CommunWithDUGS.Databuff[1] = xiuzhen_fuzhi;				//2-50�η�ֵ����
			CommunWithDUGS.Databuff[2] = xiuzhen_pf;			//pf����
			CommunWithDUGS.Databuff[3] = xiuzhen_cos;	//cos������
			CommunWithDUGS.Databuff[4] = xiuzhen_shuchu;		//���������Чֵ����
			CommunWithDUGS.Databuff[5] = xiuzhen_wugong;	//�޹���������
			CommunWithDUGS.Databuff[6] = xiuzhen_jibian;	//���书������		xiuzhen_rongliang,xiuzhen_mode,xiuzhen_bili;
			CommunWithDUGS.Databuff[7] = xiuzhen_rongliang;//�������
		  CommunWithDUGS.Databuff[8] = xiuzhen_mode;//����ģʽ
		  CommunWithDUGS.Databuff[9] = xiuzhen_bili;//���������%
			CommunWithDUGS.Databuff[10] = xiuzhen_dianyajibian;				//��ya����������
		
			break;
		
		case 0x821E00:
			CommunWithDUGS.Framelength =CorrectParamNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1E00;
			j=0;
			CommunWithDUGS.Databuff[j++]=CorrectParam[0];
			CommunWithDUGS.Databuff[j++]=CorrectParam[1];
			CommunWithDUGS.Databuff[j++]=CorrectParam[2];
			CommunWithDUGS.Databuff[j++]=CorrectParam[3];
			CommunWithDUGS.Databuff[j++]=CorrectParam[4];
			CommunWithDUGS.Databuff[j++]=CorrectParam[5];
			CommunWithDUGS.Databuff[j++]=CorrectParam[6];
			CommunWithDUGS.Databuff[j++]=CorrectParam[7];
			CommunWithDUGS.Databuff[j++]=CorrectParam[8];
			CommunWithDUGS.Databuff[j++]=CorrectParam[9];
			CommunWithDUGS.Databuff[j++]=CorrectParam[10];
			break;
		case 0x821600:
			CommunWithDUGS.Framelength =CorrectionNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1600;
			
			for(i=0;i<100;i++)
			{
				CommunWithDUGS.Databuff[i]=load_correctionA[i];
			}
			break;
		case 0x821800:
			CommunWithDUGS.Framelength =CorrectionNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1800;
			
			for(i=0;i<100;i++)
			{
				CommunWithDUGS.Databuff[i]=load_correctionB[i];
			}
			break;
		case 0x821900:
			CommunWithDUGS.Framelength =CorrectionNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1900;
			
			for(i=0;i<100;i++)
			{
				CommunWithDUGS.Databuff[i]=load_correctionC[i];
			}
			break;
		case 0x821A00:
			CommunWithDUGS.Framelength =PassiveParameterNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1A00;

			//�������뷢�͵�ǰ����
			if(CurrentPageID==0x0042)//��������
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=main_parameter[i];
				}
			}
			else if(CurrentPageID==0x0043)//�ӻ�����
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=slave_parameter[SlaveID][i];
				}
			}
			else if(CurrentPageID==0x0044)//ARM����
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=ARM_param[i];
				}
			}
			else if(CurrentPageID==0x0045)//��������1
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=load_correctionA[i];
				}
			}
			else if(CurrentPageID==0x0046)//��������2
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=load_correctionB[i];
				}
			}
			else if(CurrentPageID==0x0047)//��������3
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=load_correctionC[i];
				}
			}
			else if(CurrentPageID==0x0048)//����Ͷ�� 8007
			{
				for(i=0;i<100;i++)
				{
					CommunWithDUGS.Databuff[i]=spcial_parameter[i];
				}
			}
			

			
			
			
			break;


		case 0x821A64:
			CommunWithDUGS.Framelength =ManualPassiveNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1A64;

			if(lanya_flag==1)
			{			
					for(i=0;i<32;i++)
				{
					CommunWithDUGS.Databuff[i]= ManualPassive_aisle[i];
					lanya_flag=0;
				}																			
			}
			else
			{
				if(dsp_data[96]==0)
				{
					ManualPassiveSwitchFlagBak =0;
					OnekeyClean =0;
				}
				ManualPassiveSwitch =0;
				for(i=0;i<32;i++)
				{
					ManualPassive_aisle[i] =0;
				}
				for(i=0;i<32;i++)
				{
					CommunWithDUGS.Databuff[i]=ManualPassive_aisle[i];
				}
			}
			break;
		case 0x821A84:
			CommunWithDUGS.Framelength =DSPPassiveNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1A84;


			if((main_parameter[70]==2)||(main_parameter[70]==1))	//���ܵ��ݻ� ֱ��
			{
				IntPassiveState 	=(uint32)((dsp_data[94]<<16)|dsp_data[90]);
				IntPassiveFault 	=(uint32)((dsp_data[95]<<16)|dsp_data[91]);
				IntPassiveSwitch 	=(uint32)((dsp_data[96]<<16)|dsp_data[92]);
				for(i=0;i<32;i++)
				{
					IntPassive.State[i] =(IntPassiveState&1<<i)>>i;			//���ܵ�������״̬
					IntPassive.Fault[i] =(IntPassiveFault&1<<i)>>i;			//���ܵ��ݱ���״̬
					IntPassive.Switch[i] =(IntPassiveSwitch&1<<i)>>i;		//���ܵ���Ͷ��״̬

					if(IntPassive.State[i]==0)IntPassive_aisle[i]=0;
					else
					{
						IntPassive_aisle[i] =IntPassive.State[i];
						if(IntPassive.Switch[i]==1)
						{
							IntPassive_aisle[i]=2;
							if(IntPassive.Ready[i] ==0)
							{
								IntPassive.Ready[i] =1;
								if(IntPassive.Times[i]<255)IntPassive.Times[i]++;
							}
						}
						else IntPassive.Ready[i] =0;
						
						if(IntPassive.Fault[i]==1)IntPassive_aisle[i]=3;
					}
					CommunWithDUGS.Databuff[i]=IntPassive_aisle[i];
				}
			}
			else
			{
				for(i=0;i<16;i++)
				{
					CommunWithDUGS.Databuff[i]=0;
				}
			}	
			break;
		case 0x821700:
			CommunWithDUGS.Framelength =SlaveParameterNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1700;
			if(SaveOrReadFlag==1)
			{
				switch(SlaveID)
				{
					case 0:ReadSlaveParameterFlag=1;break;
					case 1:ReadSlaveParameter2Flag=1;break;
					case 2:ReadSlaveParameter3Flag=1;break;
					case 3:ReadSlaveParameter4Flag=1;break;
					case 4:ReadSlaveParameter5Flag=1;break;
					case 5:ReadSlaveParameter6Flag=1;break;
					case 6:ReadSlaveParameter7Flag=1;break;
					case 7:ReadSlaveParameter8Flag=1;break;
					case 8:ReadSlaveParameter9Flag=1;break;
					case 9:ReadSlaveParameter10Flag=1;break;
					case 10:ReadSlaveParameter11Flag=1;break;
					case 11:ReadSlaveParameter12Flag=1;break;
					case 12:ReadSlaveParameter13Flag=1;break;
					case 13:ReadSlaveParameter14Flag=1;break;
					case 14:ReadSlaveParameter15Flag=1;break;
					case 15:ReadSlaveParameter16Flag=1;break;
					default:break;
				}
			}
			for(i=0;i<SlaveParameterNUM;i++)
			{
				CommunWithDUGS.Databuff[i]=slave_parameter[SlaveID][i];				
			}
			break;
		case 0x821B00:
			CommunWithDUGS.Framelength =DataLogNUM;
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1B00;

			tmp16 =Waveform2ID%13;
			if(tmp16==0)tmp16=13;
			
			if(DatalogMax<13)
			{
				if(tmp16>DatalogMax) 
				{
					addr =13-(tmp16-DatalogMax);
				}
				else addr =DatalogMax-tmp16;
					
				ReadEeprom(DatalogStAddr[addr],Tmp_DataLog,EepDatalogNum);
			}
			else if(DatalogMax==13)
			{
				if(tmp16<=DatalogStart)
				{
					addr =DatalogStart-tmp16;
				}
				else if(tmp16>DatalogStart)
				{
					addr =13-(tmp16-DatalogStart);
				}

				ReadEeprom(DatalogStAddr[addr],Tmp_DataLog,EepDatalogNum);
			}
			/*for(i=0;i<38;i++)
			{
				Datalog[i]=(Tmp_DataLog[2*i]<<8)|(Tmp_DataLog[2*i+1]);
			}*/
				
			for(i=0;i<30;i++)
			{
				CommunWithDUGS.Databuff[i]=(uint16)((Tmp_DataLog[2*i]<<8)|Tmp_DataLog[2*i+1]);
			}
			for(i=0;i<16;i++)
			{
				CommunWithDUGS.Databuff[i+30]=(uint16)(Tmp_DataLog[i+60]);
			}
			break;	
		case 0x821000://��һ�й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1000;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  		CommunWithDUGS.Databuff[0]=0xA3AD;
					CommunWithDUGS.Databuff[1]=0xA3AD;
		  			for(i=2;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
						
				}
				else if(gz_dan[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,0,5);
					}
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  		CommunWithDUGS.Databuff[0]=0xA3AD;
					CommunWithDUGS.Databuff[1]=0xA3AD;
		  			for(i=2;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,0,5);
					}
				}
			}
			
			break;

		case 0x821029://�ڶ��й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1029;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(gz_dan[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,1,5);
					}
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,1,5);
					}
				}
			}
			break;
		case 0x821052://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x1052;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
					CommunWithDUGS.Framelength =21;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(gz_dan[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,2,5);
					}
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,2,5);
					}
				}
			}
			break;
		case 0x82107B://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x107B;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
					ReadFaultRecordFinishFlag =1;
				}
				else if(gz_dan[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,3,5);
					}
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,3,5);
					}
				}
			}
			break;
		case 0x8210A4://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x10A4;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
					ReadFaultRecordFinishFlag =1;
				}
				else if(gz_dan[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,4,5);
					}
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(4*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					{
						i =FaultProcess(FaultRecordID,4,5);
					}
				}
			}
			break;
		case 0x8210CD://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x10CD;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
					ReadFaultRecordFinishFlag =1;
				}
				else if(gz_dan[0]<(7*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					 i=FaultProcess(FaultRecordID,5,7);
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(7*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					i =FaultProcess(FaultRecordID,5,7);
				}
			}
			break;
		case 0x8210F6://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x10F6;
			if(CurrentPageID==0x0400)
			{
				if(gz_dan[0]==0)
				{
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
					ReadFaultRecordFinishFlag =1;
				}
				else if(gz_dan[0]<(7*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					 i=FaultProcess(FaultRecordID,6,7);
				}
			}
			else if(CurrentPageID==0x0600)
			{
				if(Event_Logging[0]==0)
				{
		  			//CommunWithDUGS.Databuff[0]=0xCEDE;
		  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else if(Event_Logging[0]<(7*(FaultRecordID-1)))
				{
					for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				}
				else
				{
					i =FaultProcess(FaultRecordID,6,7);
				}
			}
			break;
		case 0x82211F://��һ�й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x211F;
			if(SlaveFault[SlaveID][0]==0)
			{
	  			CommunWithDUGS.Databuff[0]=0xA3AD;
					CommunWithDUGS.Databuff[1]=0xA3AD;
		  			for(i=2;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(1,0,7);
			}

			break;

		case 0x822148://�ڶ��й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2148;
			if(SlaveFault[SlaveID][0]==0)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(1,1,7);
			}
			break;
		case 0x822171://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2171;
			if(SlaveFault[SlaveID][0]==0)
			{
				CommunWithDUGS.Framelength =21;
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(1,2,7);
			}

			break;
		case 0x82219A://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x219A;

			if(SlaveFault[SlaveID][0]==0)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(1,3,7);
			}
			break;
		case 0x8221C3://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x21C3;

			if(SlaveFault[SlaveID][0]==0)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(1,4,7);
			}
			break;
		case 0x8221EC://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x21EC;

			if(SlaveFault[SlaveID][0]==0)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(1,5,7);
			}
			break;
		case 0x822215://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2215;

			if(SlaveFault[SlaveID][0]==0)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(1,6,7);
			}
			break;
		case 0x82223E://��һ�й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x223E;
			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			//CommunWithDUGS.Databuff[0]=0xCEDE;
	  			for(i=2;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(2,0,7);
			}

			break;

		case 0x822267://�ڶ��й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2267;
			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(2,1,7);
			}
			break;
		case 0x822290://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2290;
			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
				CommunWithDUGS.Framelength =21;
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
			}
			else
			{
				i =FaultProcess(2,2,7);
			}

			break;
		case 0x8222B9://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x22B9;

			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(2,3,7);
			}
			break;
		case 0x8222E2://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x22E2;

			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(2,4,7);
			}
			break;
		case 0x82230B://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x230B;

			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(2,5,7);
			}
			break;
		case 0x822334://�����й���
			CommunWithDUGS.Framelength =21;
			
			CommunWithDUGS.Funcode= 0x82;
			CommunWithDUGS.StartAddr= 0x2334;

			if(SlaveFault[SlaveID][0]==0||SlaveFault[SlaveID][0]<7)
			{
	  			for(i=0;i<48;i++){CommunWithDUGS.Databuff[i] =0x20;}
				ReadFaultRecordFinishFlag =1;
			}
			else
			{
				 i=FaultProcess(2,6,7);
			}
			break;


		case 0x8320C8:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x20C8;
			CommunWithDUGS.Datalength=SelectedNUM;

			break;


			break;
		case 0x831500:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1500;
			CommunWithDUGS.Datalength= MainParameterNUM;

			break;
		case 0x831564:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1564;
			CommunWithDUGS.Datalength= ARMParameterNUM;

			break;
		case 0x831D00:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1D00;
			CommunWithDUGS.Datalength= 65;

			break;
		
		case 0x831AAA:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1AAA;
			CommunWithDUGS.Datalength= 11;

			break;
		
		case 0x831E00:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1E00;
			CommunWithDUGS.Datalength= 11;

			break;
		case 0x831567:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1567;
			CommunWithDUGS.Datalength= TimeSetNUM;
			break;
		case 0x831600:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1600;
			CommunWithDUGS.Datalength= CorrectionNUM;

			break;
		case 0x831800:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1800;
			CommunWithDUGS.Datalength= CorrectionNUM;

			break;
		case 0x831900:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1900;
			CommunWithDUGS.Datalength= CorrectionNUM;
			
			break;
		case 0x831A00:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1A00;
			CommunWithDUGS.Datalength= PassiveParameterNUM;
			
			break;
		case 0x831A64:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1A64;
			CommunWithDUGS.Datalength= ManualPassiveNUM;
			
			break;
		case 0x831700:
			CommunWithDUGS.Framelength =1;
			CommunWithDUGS.Funcode= 0x83;
			CommunWithDUGS.StartAddr= 0x1700;
			CommunWithDUGS.Datalength= SlaveParameterNUM;

			break;

		
		default:break;
	}	
}
void SendDatatoDUGS(void)
{
	uint8 i=0,j,Send[256];
	uint16 SendNUM,Crc_data=0;
	uint8  b,*pointer8;
	if(CommunWithDUGS.Funcode ==0x82)
	{
		Send[i++] = (uint8)(CommunWithDUGS.Framehead>>8);
		Send[i++] = (uint8)(CommunWithDUGS.Framehead&0xff);
		Send[i++] = (CommunWithDUGS.Framelength<<1)+5;
		Send[i++] = CommunWithDUGS.Funcode;
		Send[i++] = (uint8)(CommunWithDUGS.StartAddr>>8);
		Send[i++] = (uint8)(CommunWithDUGS.StartAddr&0xff);
		for(j=0;j<CommunWithDUGS.Framelength;j++)
		{
			Send[i++]= (uint8)(CommunWithDUGS.Databuff[j]>>8);
			Send[i++]= (uint8)(CommunWithDUGS.Databuff[j]&0xff);
		}
		pointer8=Send;Crc_data=crc_chk(pointer8+3,i-3);//У��
		Send[i++]=(uint8)(Crc_data&0xff); 
		Send[i++]=(uint8)(Crc_data>>8);
		pointer8=Send; 
		uart_write3;
	  	for(SendNUM=i;SendNUM>0;SendNUM--){b=*pointer8;UART3_SendByte(b);pointer8++;}//ѡ��UART����
	  	uart_read3;
	}
	else if(CommunWithDUGS.Funcode==0x83)
	{
		Send[i++] = (uint8)(CommunWithDUGS.Framehead>>8);
		Send[i++] = (uint8)(CommunWithDUGS.Framehead&0xff);
		Send[i++] = 0x06;
		Send[i++] = CommunWithDUGS.Funcode;
		Send[i++] = (uint8)(CommunWithDUGS.StartAddr>>8);
		Send[i++] = (uint8)(CommunWithDUGS.StartAddr&0xff);
		Send[i++] = CommunWithDUGS.Datalength;
		pointer8=Send;Crc_data=crc_chk(pointer8+3,i-3);//У��
		Send[i++]=(uint8)(Crc_data&0xff); 
		Send[i++]=(uint8)(Crc_data>>8);
		pointer8=Send; 
		puyaotest =1;
		uart_write3;
	  	for(SendNUM=i;SendNUM>0;SendNUM--){b=*pointer8;UART3_SendByte(b);pointer8++;}//ѡ��UART����
	  	uart_read3;
		puyaotest =2;
	}
	else if(CommunWithDUGS.Funcode==0x84)
	{
		Send[i++] = (uint8)(CommunWithDUGS.Framehead>>8);
		Send[i++] = (uint8)(CommunWithDUGS.Framehead&0xff);
		Send[i++] = (CommunWithDUGS.Framelength<<1)+4;
		Send[i++] = CommunWithDUGS.Funcode;
		Send[i++] = CommunWithDUGS.Aisle;
		for(j=0;j<CommunWithDUGS.Framelength;j++)
		{
			Send[i++]= (uint8)(CommunWithDUGS.Databuff[j]>>8);
			Send[i++]= (uint8)(CommunWithDUGS.Databuff[j]&0xff);
		}
		pointer8=Send;Crc_data=crc_chk(pointer8+3,i-3);//У��
		Send[i++]=(uint8)(Crc_data&0xff); 
		Send[i++]=(uint8)(Crc_data>>8);
		pointer8=Send; 
		uart_write3;
	  	for(SendNUM=i;SendNUM>0;SendNUM--){b=*pointer8;UART3_SendByte(b);pointer8++;}//ѡ��UART����
	  	uart_read3;
	}
	//uart_read0;
	SendDTUSuccessTest++;
	if(led2_flag==0){led2_ON;led2_flag=1;}
	else{led2_OFF;led2_flag=0;}
}
void RecvDataFromDUGS(void)
{
	uint8  RecvDatalengh,Eeptempbuff[1000];
	uint16 i,j,RecvDatabuff[127];
	uint32 RecvFramehead;
	uint64 tmp64Val;

	RecvFramehead = (uint32)((RecvDUGSBuf[3]<<16)|(RecvDUGSBuf[4]<<8)|RecvDUGSBuf[5]);
	//RecvStartAddr = (uint16)((RecvDUGSBuf[4]<<8)|RecvDUGSBuf[5]);
	RecvDatalengh = RecvDUGSBuf[6];
	for(i=0;i<RecvDatalengh;i++)
	{
		RecvDatabuff[i] =(uint16)((RecvDUGSBuf[7+(i<<1)]<<8)|RecvDUGSBuf[8+(i<<1)]);
	}
	switch(RecvFramehead)
	{
		case 0x832001:
				ScreenType		=RecvDatabuff[0];				//��Ļ����
				LogoType		=RecvDatabuff[1];				//Logo����
				RestoreCommand  =RecvDatabuff[2];				//�ָ���������
				LocalOnOff 	=RecvDatabuff[3];				//���ػ�����
				SaveOrReadFlag	=RecvDatabuff[4];				//��ȡ�����Ӧ��
				//ManualPassiveSwitchFlag=RecvDatabuff[5];		//�߼�����
				Password		=RecvDatabuff[6];				//����
				HistogramSetting=RecvDatabuff[7];				//��״ͼ��ʾѡ��
				WaveformAccuracy=RecvDatabuff[8];				//���ξ���
				WaveformID		=RecvDatabuff[9];				//����ͼID
				Waveform2ID		=RecvDatabuff[10];				//����ͼ2ID
				HistogramID		=RecvDatabuff[11];				//��״ͼID
				HistogramPageID =RecvDatabuff[12];				//��״ͼҳ��ID
				SlaveID			=RecvDatabuff[13];				//�ӻ�ҳ��ID
				CorrectionID	=RecvDatabuff[14];				//��������ҳ��
				PrimevalID		=RecvDatabuff[15];				//��������ID
				CurrentPageID 	=RecvDatabuff[16];				//ҳ��ID
				FaultRecordID	=RecvDatabuff[17];				//���ϼ�¼ҳ��ID
				FirstEnterWindowFlag =0;

				
				DUGSProcess();
				if(RestoreCommand!=0)
				{
					RestoreProcess();
				}
				break;
		case 0x8320C8:
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						Selected_parameter[i] = RecvDatabuff[i];						//ѡ�β���	
					}
					
					AI_select=Selected_parameter[50];
					
					if(AI_select==1)	//�ж��Ƿ�Ϊ����ѡ��
					{
						for(i=0;i<50;i++)
						{
							Selected_parameter[i] = 0;						//ѡ�β���	
						}
						Selected_parameter[2] =1;
						Selected_parameter[4] =1;
						Selected_parameter[6] =1;
						main_parameter[30]=3;
						main_parameter[31]=3;
						main_parameter[32]=5;
						main_parameter[33]=7;
						for(i=0;i<16;i++)
						{
								main_parameter[34+i] =0;
						}
					}
					else
					{
						j=0;
						for(i=0;i<50;i++)
						{
							if(Selected_parameter[i]==1)
							{
								main_parameter[31+j] =i+1;
								j++;
							}
							if(j>18)break;
						}
						main_parameter[30] =j;
						for(i=0;i<19-j;i++)
						{
								main_parameter[31+j+i] =0;
						}
					}
					
				}
				break;
		case 0x831500:
				if(RecvDatabuff[0]==0)ParameterError =1;
				if(ParameterError==0)
				{
					for(i=0;i<100;i++)
					{
						main_parameter[i] = RecvDatabuff[i];						//������	
					}
					if(CT_MAIN1>0)main_parameter[10]=CT_MAIN1;//����������װλ��
					if(CT_MAIN2>0)main_parameter[15]=CT_MAIN2;//������������װλ��

					//main_parameter[17] = RecvDatabuff[17]/10;					//���ܵ��ݲ�����
					// if(main_parameter[72]==2)//���Ϊ�ֶ�Ͷ��
					// {
					// 	ManualPassiveSwitchFlag=1;
					// }
					// else 
					// {
					// 	ManualPassiveSwitchFlag =0;//�Զ�Ͷ��
					// 	OnekeyClean =1;//һ���г�
					// }
					
					if(SaveOrReadFlag==3)
					{	
						WriteMainParameterFlag=1;
						// WriteCorrectionParameterFlag=1;
						// WriteCorrectionParameter2Flag=1;
						// WriteCorrectionParameter3Flag=1;
						// WritePassiveParameterFlag =1;
						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
							Eeptempbuff[j++] =(uint8)(main_parameter[i]);
						}
						
						I2C_write;													//д��I2C
					   	WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
						I2C_read;													//��ȡI2C
					}
				}
				break;
		case 0x831564:
				if(RecvDatabuff[0]==0)ParameterError =1;
				if(ParameterError==0)
				{					
					
					LocalAddr 	 =RecvDatabuff[0];
					UART_BPS  	 =RecvDatabuff[1];
					RemoteEnable =RecvDatabuff[2];
					UART1_Init();
					UART0_Init();
					//RTCStop();RTCInit();     //��ʼ��
					//local_time.RTC_Year = RecvDatabuff[3];
					//local_time.RTC_Mon  = RecvDatabuff[4];
					//local_time.RTC_Mday = RecvDatabuff[5];
					//local_time.RTC_Hour = RecvDatabuff[6];
	     			//local_time.RTC_Min  = RecvDatabuff[7];
	     			//RTCSetTime( local_time );//����ʱ��
	     			//RTCStart();              //����RTC

					OnOffStatus =RecvDatabuff[8];
					
					CT_MAIN1=RecvDatabuff[9];//����������װλ��
					CT_MAIN2=RecvDatabuff[10];//������������װλ��
					// AlarmTime1[2]=RecvDatabuff[11];//��ʱ1	��	
					// AlarmTime1[3]=RecvDatabuff[12];//��ʱ1 	ʱ
					// AlarmTime1[4]=RecvDatabuff[13];//��ʱ1	��	
					
					// AlarmTime2[0]=RecvDatabuff[14];//��ʱ2	ʹ��
					// AlarmTime2[1]=RecvDatabuff[15];//��ʱ2 	ʱ
					// AlarmTime2[2]=RecvDatabuff[16];//��ʱ2	��
					// AlarmTime2[3]=RecvDatabuff[17];//��ʱ2	ʱ					
					// AlarmTime2[4]=RecvDatabuff[18];//��ʱ2 	��
					
					// AlarmTime3[0]=RecvDatabuff[19];//��ʱ3	ʹ��
					// AlarmTime3[1]=RecvDatabuff[20];//��ʱ3	ʱ
					// AlarmTime3[2]=RecvDatabuff[21];//��ʱ3 	��
					// AlarmTime3[3]=RecvDatabuff[22];//��ʱ3	ʱ
					// AlarmTime3[4]=RecvDatabuff[23];//��ʱ3	��	

					ntc_type=RecvDatabuff[24];//NTCģʽ��0ΪĬ�ϣ�1Ϊ690����
//					AlarmTime4[0]=RecvDatabuff[24];//��ʱ4 ʹ��
//					AlarmTime4[1]=RecvDatabuff[25];//��ʱ4	ʱ
//					AlarmTime4[2]=RecvDatabuff[26];//��ʱ4	��
//					AlarmTime4[3]=RecvDatabuff[27];//��ʱ4	ʱ
//					AlarmTime4[4]=RecvDatabuff[28];//��ʱ4	��
					
					// ActiveBal=RecvDatabuff[29];//�й���ƽ��
					// ReactiveBal=RecvDatabuff[30];//�޹���ƽ��
					// HarmonicBal=RecvDatabuff[31];//г����ƽ��
					// ApparentBal=RecvDatabuff[32];//���ڲ�ƽ��

					// if(ApparentBal2!=ApparentBal)
					// {
					// 	if(ApparentBal==1){HarmonicBal=1;ReactiveBal=1;}				
					// 	else {HarmonicBal=0;ReactiveBal=0;}
					// 	ApparentBal2=ApparentBal;
					// }			
										
					// main_parameter[6] =ActiveBal+ReactiveBal*10+HarmonicBal*100;
					// SetupMode =RecvDatabuff[33];//��װ���÷�ʽ
					
					// enhance=RecvDatabuff[34];//��ǿģʽ
					/*slave_Reset[4] =RecvDatabuff[34];//��ʱ5	��
					slave_Reset[5] =RecvDatabuff[35];//��ʱ5	��
					slave_Reset[6] =RecvDatabuff[36];//��ʱ5	��
					slave_Reset[7] =RecvDatabuff[37];//��ʱ5	��
					slave_Reset[8] =RecvDatabuff[38];//��ʱ5	��
					slave_Reset[9] =RecvDatabuff[39];//��ʱ5	��*/

					// ProjectNo=(uint32)((RecvDatabuff[40]<<16)|RecvDatabuff[41]);//��Ŀ��
					
					// ProductionNo =RecvDatabuff[42];//������
					// tmp64Val=1000;
					// tmp64Val =(uint64) (ProjectNo)*tmp64Val;
					// SerialNumber = 2000000000000000+tmp64Val+ProductionNo;
					//word=(uint16)(SerialNumber%9999);									//��ȡ����

					// VolOnOffEnable =RecvDatabuff[43];
					// CurOnOffEnable =RecvDatabuff[44];
					// main_parameter[73] =VolOnOffEnable+CurOnOffEnable*10;	//��λ����
					// 														//0���豸�����õ�ѹ���ػ�����
					// 														//1���豸���õ�ѹ���ػ�����

					// 														//ʮλ��
					// 														//0���豸�����õ������ػ�����
					// 														//1���豸���õ������ػ�����
					// MainCTLocation 	=RecvDatabuff[45];
					// MainCTDirectionA =RecvDatabuff[46];
					// MainCTDirectionB =RecvDatabuff[47];
					// MainCTDirectionC =RecvDatabuff[48];
					// MainCTPhase 	=RecvDatabuff[49];
					// main_parameter[10] =MainCTLocation+MainCTDirectionA*10+MainCTDirectionB*100+MainCTDirectionC*1000+MainCTPhase*10000;	
					//AidCTLocation 	=RecvDatabuff[48];
					//AidCTDirection 	=RecvDatabuff[49];
					// OutCTPhase 		=RecvDatabuff[50];
					
					// OutCTDirectionA =RecvDatabuff[78];
					// OutCTDirectionB =RecvDatabuff[79];
					// OutCTDirectionC =RecvDatabuff[80];
					// main_parameter[15] =OutCTDirectionA*10+OutCTDirectionB*100+OutCTDirectionC*1000+OutCTPhase*10000;
					
					// Position[0] 	=RecvDatabuff[51];										//��λ2
					// Group[0] 		=RecvDatabuff[52];										//��2
					// Capacitance[0]	=RecvDatabuff[53];										//��ֵ2

					// Position[1] 	=RecvDatabuff[54];										//��λ3
					// Group[1] 		=RecvDatabuff[55];										//��3
					// Capacitance[1]	=RecvDatabuff[56];										//��ֵ3

					// Position[2] 	=RecvDatabuff[57];										//��λ4
					// Group[2] 		=RecvDatabuff[58];										//��4
					// Capacitance[2]	=RecvDatabuff[59];										//��ֵ4

					// Position[3] 	=RecvDatabuff[60];										//��λ5
					// Group[3] 		=RecvDatabuff[61];										//��5
					// Capacitance[3]	=RecvDatabuff[62];										//��ֵ5

					// Position[4] 	=RecvDatabuff[63];										//��λ6
					// Group[4] 		=RecvDatabuff[64];										//��6
					// Capacitance[4]	=RecvDatabuff[65];										//��ֵ6

					// Position[5] 	=RecvDatabuff[66];										//��λ7
					// Group[5] 		=RecvDatabuff[67];										//��7
					// Capacitance[5]	=RecvDatabuff[68];										//��ֵ7

					// Position[6] 	=RecvDatabuff[69];										//��λ8
					// Group[6] 		=RecvDatabuff[70];										//��8
					// Capacitance[6]	=RecvDatabuff[71];										//��ֵ8

					// Position[7] 	=RecvDatabuff[72];										//��λ9
					// Group[7] 		=RecvDatabuff[73];										//��9
					// Capacitance[7]	=RecvDatabuff[74];										//��ֵ9

					// Position[8] 	=RecvDatabuff[75];										//��λ10
					// Group[8] 		=RecvDatabuff[76];										//��10
					// Capacitance[8]	=RecvDatabuff[77];										//��ֵ10

					//OutCTDirectionA =RecvDatabuff[78];
					//OutCTDirectionB =RecvDatabuff[79];
					//OutCTDirectionC =RecvDatabuff[80];

					
					// Position[9] 	=RecvDatabuff[81];										//��λ11
					// Group[9] 		=RecvDatabuff[82];										//��11
					// Capacitance[9]	=RecvDatabuff[83];										//��ֵ11

					// Position[10] 	=RecvDatabuff[84];										//��λ12
					// Group[10] 		=RecvDatabuff[85];										//��12
					// Capacitance[10]	=RecvDatabuff[86];										//��ֵ12

					// Position[11] 	=RecvDatabuff[87];										//��λ13
					// Group[11] 		=RecvDatabuff[88];										//��13
					// Capacitance[11]	=RecvDatabuff[89];										//��ֵ13

					// Position[12] 	=RecvDatabuff[90];										//��λ14
					// Group[12] 		=RecvDatabuff[91];										//��14
					// Capacitance[12]	=RecvDatabuff[92];										//��ֵ14

					// Position[13] 	=RecvDatabuff[93];										//��λ15
					// Group[13] 		=RecvDatabuff[94];										//��15
					// Capacitance[13]	=RecvDatabuff[95];										//��ֵ15

					// Position[14] 	=RecvDatabuff[96];										//��λ16
					// Group[14] 		=RecvDatabuff[97];										//��16
					// Capacitance[14]	=RecvDatabuff[98];										//��ֵ16
					// if(SetupMode==0)
					// {
					// 	CabinProcess();			//��������
					// }
					// if(SetupMode==1)
					// {	
					// 	ChannelProcess();			//ֱ��
					// }
					if(SaveOrReadFlag==3)
					{	
						SaveARMParaProcess();
					}
				}
				break;
		case 0x831567:
				RTCStop();RTCInit();     //��ʼ��
				local_time.RTC_Year = RecvDatabuff[0];
				local_time.RTC_Mon  = RecvDatabuff[1];
				local_time.RTC_Mday = RecvDatabuff[2];
				local_time.RTC_Hour = RecvDatabuff[3];
     			local_time.RTC_Min  = RecvDatabuff[4];
     			RTCSetTime( local_time );//����ʱ��
     			RTCStart();              //����RTC
				break;
		case 0x831600:
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						load_correctionA[i] = RecvDatabuff[i];//����A
					}
					if(SaveOrReadFlag==3)
					{
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;

						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionA[i]);
						}
						

						I2C_write;
					   	WriteEeprom(EepCorrectionAddr,Eeptempbuff,200);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C
					}
				}
				break;
		case 0x831800:
				if(ParameterError==0)
				{
					if(ini_version_flag==1)
					{
						ini_version_a=ini_version_b;
						if(ini_version_b==23100)
						{
							load_correctionA[0]=10600;						
							load_correctionA[6]=10155;							load_correctionA[16]=11000;
							
							load_correctionA[52]=707;						load_correctionA[54]=1049;
							load_correctionA[56]=1725;						load_correctionA[59]=2225;
							load_correctionA[60]=2632;						load_correctionA[62]=3297;
							load_correctionA[64]=4120;						load_correctionA[66]=4476;
							load_correctionA[68]=4476;						load_correctionA[70]=4876;
							load_correctionA[72]=5676;						load_correctionA[74]=6076;
							
							load_correctionB[2]=10100;						load_correctionB[4]=9915;
							load_correctionB[6]=9959;							load_correctionB[8]=10619;
							load_correctionB[10]=9404;						load_correctionB[12]=9128;
							load_correctionB[56]=100;						load_correctionB[58]=420;
							load_correctionB[60]=35900;							load_correctionB[62]=50;
							load_correctionB[72]=1350;						load_correctionB[74]=1400;
						}
						else if(ini_version_b==23099)
						{
							load_correctionA[0]=10600;						
							load_correctionA[6]=10155;							load_correctionA[16]=11000;
							
							load_correctionA[52]=707;						load_correctionA[54]=1049;
							load_correctionA[56]=1725;						load_correctionA[59]=2225;
							load_correctionA[60]=2632;						load_correctionA[62]=3297;
							load_correctionA[64]=4120;						load_correctionA[66]=4476;
							load_correctionA[68]=4476;						load_correctionA[70]=4876;
							load_correctionA[72]=5676;						load_correctionA[74]=6076;
							
							load_correctionB[2]=10100;						load_correctionB[4]=9915;
							load_correctionB[6]=9959;							load_correctionB[8]=10619;
							load_correctionB[10]=9404;						load_correctionB[12]=9128;
							load_correctionB[56]=100;						load_correctionB[58]=420;
							load_correctionB[60]=35900;							load_correctionB[62]=50;
							load_correctionB[72]=1350;						load_correctionB[74]=1400;
						}
						else if(ini_version_b==13100)
						{
							load_correctionA[0]=10600;						
							load_correctionA[6]=10155;							load_correctionA[16]=11000;
							
							load_correctionA[52]=707;						load_correctionA[54]=1049;
							load_correctionA[56]=1725;						load_correctionA[59]=2225;
							load_correctionA[60]=2632;						load_correctionA[62]=3297;
							load_correctionA[64]=4120;						load_correctionA[66]=4476;
							load_correctionA[68]=4476;						load_correctionA[70]=4876;
							load_correctionA[72]=5676;						load_correctionA[74]=6076;
							
							load_correctionB[2]=10100;						load_correctionB[4]=9915;
							load_correctionB[6]=9959;							load_correctionB[8]=10619;
							load_correctionB[10]=9404;						load_correctionB[12]=9128;
							load_correctionB[56]=100;						load_correctionB[58]=420;
							load_correctionB[60]=35900;							load_correctionB[62]=50;
							load_correctionB[72]=1350;						load_correctionB[74]=1400;
						}
						else if(ini_version_b==13099)
						{
							load_correctionA[0]=10600;						
							load_correctionA[6]=10155;							load_correctionA[16]=11000;
							
							load_correctionA[52]=707;						load_correctionA[54]=1049;
							load_correctionA[56]=1725;						load_correctionA[59]=2225;
							load_correctionA[60]=2632;						load_correctionA[62]=3297;
							load_correctionA[64]=4120;						load_correctionA[66]=4476;
							load_correctionA[68]=4476;						load_correctionA[70]=4876;
							load_correctionA[72]=5676;						load_correctionA[74]=6076;
							
							load_correctionB[2]=10100;						load_correctionB[4]=9915;
							load_correctionB[6]=9959;							load_correctionB[8]=10619;
							load_correctionB[10]=9404;						load_correctionB[12]=9128;
							load_correctionB[56]=100;						load_correctionB[58]=420;
							load_correctionB[60]=35900;							load_correctionB[62]=50;
							load_correctionB[72]=1350;						load_correctionB[74]=1400;
						}
						else 
						{
							load_correctionA[0]=10600;						
							load_correctionA[6]=10155;							load_correctionA[16]=11000;
							
							load_correctionA[52]=707;						load_correctionA[54]=1049;
							load_correctionA[56]=1725;						load_correctionA[59]=2225;
							load_correctionA[60]=2632;						load_correctionA[62]=3297;
							load_correctionA[64]=4120;						load_correctionA[66]=4476;
							load_correctionA[68]=4476;						load_correctionA[70]=4876;
							load_correctionA[72]=5676;						load_correctionA[74]=6076;
							
							load_correctionB[2]=10100;						load_correctionB[4]=9915;
							load_correctionB[6]=9959;							load_correctionB[8]=10619;
							load_correctionB[10]=9404;						load_correctionB[12]=9128;
							load_correctionB[56]=100;						load_correctionB[58]=420;
							load_correctionB[60]=35900;							load_correctionB[62]=50;
							load_correctionB[72]=1350;						load_correctionB[74]=1400;
							ini_version_a=10000;
						}
							
							WriteMainParameterFlag=1;
							WriteCorrectionParameterFlag=1;
							WriteCorrectionParameter2Flag=1;
							WriteCorrectionParameter3Flag=1;
							WritePassiveParameterFlag =1;
							
							j=0;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
							}
							

							I2C_write;
								WriteEeprom(EepCorrectionAddr+200,Eeptempbuff,200);  				//����������д����ʽ������
							I2C_read;														//��ȡI2C													//��ȡI2C						
							
							SaveARMParaProcess();//����ini_version_a
							ini_version_flag=0;
					}
					else
					{
						for(i=0;i<RecvDatalengh;i++)
						{
							load_correctionB[i] = RecvDatabuff[i];//����A
						}
						if(SaveOrReadFlag==3)
						{
							WriteMainParameterFlag=1;
							WriteCorrectionParameterFlag=1;
							WriteCorrectionParameter2Flag=1;
							WriteCorrectionParameter3Flag=1;
							WritePassiveParameterFlag =1;
							
							j=0;
							for(i=0;i<100;i++)
							{
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]>>8);
								Eeptempbuff[j++] =(uint8)(load_correctionB[i]);
							}
							

							I2C_write;
								WriteEeprom(EepCorrectionAddr+200,Eeptempbuff,200);  				//����������д����ʽ������
							I2C_read;														//��ȡI2C													//��ȡI2C
						}
					}
					
				}
				break;
		case 0x831900:
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						load_correctionC[i] = RecvDatabuff[i];//����A
					}
					if(SaveOrReadFlag==3)
					{
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;

						j=0;
						for(i=0;i<100;i++)
						{
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]>>8);
							Eeptempbuff[j++] =(uint8)(load_correctionC[i]);
						}
						

						I2C_write;
					   	WriteEeprom(EepCorrectionAddr+400,Eeptempbuff,200);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C														//��ȡI2C
					}
				}
				break;

		case 0x831A00:
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						if((i>9&&i<43))continue;				//�������������͵�ַ
						Passive_parameter[i] = RecvDatabuff[i];//��Դ����
					}
					if(SaveOrReadFlag==3)
					{
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;

						I2C_write;														//д��I2C
					   	WriteEeprom(EepPassiveAddr,RecvDUGSBuf+7,RecvDatalengh<<1);  				//����������д����ʽ������
						I2C_read;														//��ȡI2C
					}
				}
				break;
		case 0x831A64:															//�ֶ���ԴͶ��
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						ManualPassive_aisle[i] = RecvDatabuff[i];
					}
					ManualPassiveSwitch =0;
					for(i=0;i<32;i++)
					{		
						ManualPassiveSwitch |= ManualPassive_aisle[i]<<i;
					}
				}
				break;
		case 0x831700:
				if(ParameterError==0)
				{
					for(i=0;i<RecvDatalengh;i++)
					{
						//if((i>49 && i<60)||(i>60 && i<93 ))continue;
						slave_parameter[SlaveID][i] = RecvDatabuff[i];
					}
					if(SaveOrReadFlag==3)
					{
						switch(SlaveID)
						{
							case 0:WriteSlaveParameterFlag=1;break;
							case 1:WriteSlaveParameter2Flag=1;break;
							case 2:WriteSlaveParameter3Flag=1;break;
							case 3:WriteSlaveParameter4Flag=1;break;
							case 4:WriteSlaveParameter5Flag=1;break;
							case 5:WriteSlaveParameter6Flag=1;break;
							case 6:WriteSlaveParameter7Flag=1;break;
							case 7:WriteSlaveParameter8Flag=1;break;
							case 8:WriteSlaveParameter9Flag=1;break;
							case 9:WriteSlaveParameter10Flag=1;break;
							case 10:WriteSlaveParameter11Flag=1;break;
							case 11:WriteSlaveParameter12Flag=1;break;
							case 12:WriteSlaveParameter13Flag=1;break;
							case 13:WriteSlaveParameter14Flag=1;break;
							case 14:WriteSlaveParameter15Flag=1;break;
//							case 15:WriteSlaveParameter16Flag=1;break;
							default:break;
						}
						WriteMainParameterFlag=1;
						WriteCorrectionParameterFlag=1;
						WriteCorrectionParameter2Flag=1;
						WriteCorrectionParameter3Flag=1;
						WritePassiveParameterFlag =1;
					}
					else if(SaveOrReadFlag==4)
					{
						for(j=0;j<main_parameter[99];j++)
						{
							switch(SlaveID)
							{
								case 0:WriteSlaveParameterFlag=1;break;
								case 1:WriteSlaveParameter2Flag=1;break;
								case 2:WriteSlaveParameter3Flag=1;break;
								case 3:WriteSlaveParameter4Flag=1;break;
								case 4:WriteSlaveParameter5Flag=1;break;
								case 5:WriteSlaveParameter6Flag=1;break;
								case 6:WriteSlaveParameter7Flag=1;break;
								case 7:WriteSlaveParameter8Flag=1;break;
								case 8:WriteSlaveParameter9Flag=1;break;
								case 9:WriteSlaveParameter10Flag=1;break;
								case 10:WriteSlaveParameter11Flag=1;break;
								case 11:WriteSlaveParameter12Flag=1;break;
								case 12:WriteSlaveParameter13Flag=1;break;
								case 13:WriteSlaveParameter14Flag=1;break;
								case 14:WriteSlaveParameter15Flag=1;break;
//								case 15:WriteSlaveParameter16Flag=1;break;
								default:break;
							}
							WriteMainParameterFlag=1;
							WriteCorrectionParameterFlag=1;
							WriteCorrectionParameter2Flag=1;
							WriteCorrectionParameter3Flag=1;
							WritePassiveParameterFlag =1;
							if(j==SlaveID)continue;
							for(i=0;i<100;i++)
							{
								//if((i>49 && i<60)||(i>60 && i<93 ))continue;
								slave_parameter[j][i] = slave_parameter[SlaveID][i] ;
							}
						}
					}
				}
				break;
		case 0x831D00://ֱ������
				if(ParameterError==0)
				{
					for(i=0;i<32;i++)
					{
						PassiveChannel[i] = RecvDatabuff[2*i];//��Դͨ��ģʽ
						PassiveValue[i] = RecvDatabuff[2*i+1];//��Դͨ������
					}
					PassiveCom =RecvDatabuff[64];
					j=0;
					for(i=0;i<32;i++)
					{
						Eeptempbuff[j++] =(uint8)(PassiveChannel[i]>>8);
						Eeptempbuff[j++] =(uint8)(PassiveChannel[i]);
					}
					for(i=0;i<32;i++)
					{
						Eeptempbuff[j++] =(uint8)(PassiveValue[i]>>8);
						Eeptempbuff[j++] =(uint8)(PassiveValue[i]);
					}
					Eeptempbuff[j++] =(uint8)(PassiveCom>>8);
					Eeptempbuff[j++] =(uint8)(PassiveCom);
					I2C_write;														//д��I2C
					WriteEeprom(EepARMParamddr+200,Eeptempbuff,130);  				//����������д����ʽ������
					I2C_read;
				}
				break;
		case 0x831AAA://����д��
				if(ParameterError==0)
				{
					xiuzhen_dianliujibian=RecvDatabuff[0];				//��������������
					xiuzhen_fuzhi=RecvDatabuff[1];				//2-50�η�ֵ����
					xiuzhen_pf=RecvDatabuff[2];			//pf����
					xiuzhen_cos=RecvDatabuff[3];	//cos������
					xiuzhen_shuchu=RecvDatabuff[4];		//���������Чֵ����
					xiuzhen_wugong=RecvDatabuff[5];	//�޹���������
					xiuzhen_jibian=RecvDatabuff[6];	//���书������		
					xiuzhen_rongliang=RecvDatabuff[7];
					xiuzhen_mode	=	RecvDatabuff[8];
					xiuzhen_bili=RecvDatabuff[9];
					xiuzhen_dianyajibian=RecvDatabuff[10];				//��������������
					if(SaveOrReadFlag==3)
					{
						SaveARMParaProcess();
					}
					
				}
				break;
				
		case 0x831E00://�Զ�����
				if(ParameterError==0)
				{
					for(i=0;i<11;i++)
					{
						CorrectParam[i] = RecvDatabuff[i];
					}
					for(i=0;i<11;i++)
					{
						if(CorrectParam[i]==0)continue;
						if(i==0)
						{
							slave_parameter[0][21] =(slave_parameter[0][21]*CorrectParam[0])/slave_data[0][0];		//ֱ�����ѹA
						}
						else if(i==1)
						{
							slave_parameter[0][22] =(slave_parameter[0][22]*CorrectParam[1])/slave_data[0][1];		//ֱ�����ѹB
						}
						else if(i==2)
						{
							slave_parameter[0][15] =(slave_parameter[0][15]*CorrectParam[2])/dsp_data[0];			//ϵͳ��ѹA
						}
						else if(i==3)
						{
							slave_parameter[0][16] =(slave_parameter[0][16]*CorrectParam[3])/dsp_data[1];			//ϵͳ��ѹB
						}
						else if(i==4)
						{
							slave_parameter[0][17] =(slave_parameter[0][17]*CorrectParam[4])/dsp_data[2];			//ϵͳ��ѹC
						}
						else if(i==5)
						{
							main_parameter[60] =(main_parameter[60]*CorrectParam[5])/dsp_data[6];				//ϵͳ����A
						}
						else if(i==6)
						{
							main_parameter[61] =(main_parameter[61]*CorrectParam[6])/dsp_data[7];				//ϵͳ����B
						}
						else if(i==7)
						{
							main_parameter[62] =(main_parameter[62]*CorrectParam[7])/dsp_data[8];				//ϵͳ����C
						}
						else if(i==8)
						{
							main_parameter[66] =(main_parameter[66]*CorrectParam[8])/dsp_data[32];			//�������A
						}
						else if(i==9)
						{
							main_parameter[67] =(main_parameter[67]*CorrectParam[9])/dsp_data[33];			//�������B
						}
						else if(i==10)
						{
							main_parameter[68] =(main_parameter[68]*CorrectParam[10])/dsp_data[34];			//�������C
						}
					}
					j=0;
					for(i=0;i<11;i++)
					{
						Eeptempbuff[j++] =(uint8)(CorrectParam[i]>>8);
						Eeptempbuff[j++] =(uint8)(CorrectParam[i]);
					}
					//I2C_write;														//д��I2C
					//WriteEeprom(EepARMCorrectAddr,Eeptempbuff,ARMCorrectNumMax);  				//����������д����ʽ������
					//I2C_read;

					WriteSlaveParameterFlag=1;
					WriteMainParameterFlag=1;
					WriteCorrectionParameterFlag=1;
					WriteCorrectionParameter2Flag=1;
					WriteCorrectionParameter3Flag=1;
					WritePassiveParameterFlag =1;
				}
				break;
		default:break;		
	}
	

}
void DUGSProcess(void)//��Ļ���洦��
{
	//uint8 FirstEnterPageFlag=0;
	
	if(CurrentPageID != LastPageID)	//���β���ͬһ����ID�У������һ�ν����־λ
	{		
		RunningInfoWindowFlag =0;						//������Ϣ����
		WaveformWindowFlag =0;							//���ν���
		HistogramWindowFlag =0;						//��״ͼ����
		HistoryFaultWindowFlag =0;						//���ϼ�¼����
		DebugWindowFlag =0;							//�������ݽ���
		HarmonicWindowFlag =0;							//г�����ݽ���
		SlaveFaultWindowFlag =0;
		SlaveWindowFlag =0;
		EventLogWindowFlag =0;						//�¼���¼ҳ��
		PassiveWindowFlag =0;						//��ԴͶ��ҳ��
		DataLogWindowFlag =0;						//���ݼ�¼ҳ��
		WindowbakFlag=0;
		//FirstEnterPageFlag =1;			// �״ν������
	}
	if(SlaveIDbak !=SlaveID)	//���β���ͬһ�ӻ�ID�������¶�ȡ�ӻ����ݺͲ���
	{
			SlaveWindowFlag =0;
	}
	switch(CurrentPageID)		//������ĻID������ͬ��Ļ����
	{
		case	0x0000:
			if(DUGSSystemParametersInitFlag==1)
			{
				DUGSInit();

				DUGSSystemParametersInitFlag =0;
			}
			break;


		//��������
		case	0x0010:
		
		break;

		//���ܵ���
		case	0x0020:

			break;
		//��¼��Ϣ
		case	0x0030:
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadSelectedtimesFlag=1;
				DUGSReadMainParameterFlag=1;
				DUGSReadSelectedtimesFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteSelectedtimesFlag=1;DUGSWriteMainParameterFlag=1;DUGSWritePassiveParameterFlag=1;}	
			break;

		case	0x0040://�û�����8888
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadMainParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby)
			{
				ChangeStatus(WriteParameters);
				DUGSWriteMainParameterFlag=1;
			}
			break;

		case	0x0041://�߼�����1234	
			break;

		case	0x0042://��������8001
		case	0x0043://�ӻ�����8002
		case	0x0044://ARM����8003
		case	0x0045://����1����8004
		case	0x0046://����2����8005
		case	0x0047://����3����8006
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby)
			{
				ChangeStatus(WriteParameters);
				DUGSWriteParameterFlag=1;
			}
			break;
		
		case	0x0048://Ͷ�в���8007 
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3))
			{
				ChangeStatus(WriteParameters);
				DUGSWriteParameterFlag=1;
			}
			break;
		



//////////////////////////////////////////��������///////////////////////////////////////////////			
		case  0x0308: //�ֶ�ѡ��
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadSelectedtimesFlag=1;
			}
			else if(SaveOrReadFlag==2 && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteSelectedtimesFlag=1;}
			break;
		case  0x0302://��������
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadSelectedtimesFlag=1;
				DUGSReadMainParameterFlag=1;
				DUGSReadSelectedtimesFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteSelectedtimesFlag=1;DUGSWriteMainParameterFlag=1;DUGSWritePassiveParameterFlag=1;}	
			break;
		case  0x0301://����������
		//case  0x0302://��������
		case  0x0303://ϵͳ����
		case  0x0304://���ػ�
		case  0x0305://ͨѶʱ������
		case  0x0306://��Դ����
		case  0x0307:// ��λ����
		//case  0x0308:// ��Դ����
		
		case  0x0311: //���Ҳ�������1
		case  0x0312: //���Ҳ�������2
		case  0x0313: //���Ҳ�������3
		case  0x0314: //���Ҳ�������4
		case  0x0315: //���Ҳ�������5
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadMainParameterFlag=1;
				DUGSReadSelectedtimesFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				ParameterError =0;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteMainParameterFlag=1;DUGSWritePassiveParameterFlag=1;}
			else if(SaveOrReadFlag==5){ChangeStatus(WriteParameters);DUGSTimesetFlag=1;}
			else if(SaveOrReadFlag==7 )
			{
				if(LocalOnOff==0x01)
				{
					OnOffCommand =0x01;
				}
				else if(LocalOnOff==0x00)
				{	
					OnOffCommand =0x00;
				}
				RecoverProcessFlag =1;
			}
			break;
//////////////////////////////////////////�ӻ�����///////////////////////////////////////////////	
		case  0x0321://�ӻ���������
			if(SlaveWindowFlag==0){DUGSReadSlaveDataFlag=1;SlaveWindowFlag=1;}break;
		case  0x0322://�ӻ��߼�����
			if(SlaveWindowFlag==0){DUGSReadSlaveMaxMinFlag=1;SlaveWindowFlag=1;}break;	
		case  0x0323://�ӻ�����1
		case  0x0324://�ӻ�����2
		case  0x0325://�ӻ�����3
		case  0x0326://�ӻ�����4
		case  0x0327://�ӻ�����5
		case  0x0328://�ӻ�����6
		case  0x0329://�ӻ�����7
		case  0x032A://�ӻ�����8
		case  0x032B://�ӻ�����9
		case  0x032C://�ӻ�����10
			if(SaveOrReadFlag==1||SlaveWindowFlag==0){ChangeStatus(ReadParameters);DUGSReadSlaveParameterFlag=1;SlaveWindowFlag=1;ReadCount =0;}
			else if((SaveOrReadFlag==3||SaveOrReadFlag==4) && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteSlaveParameterFlag=1;}
			break;
		case  0x032F://�ӻ����ϼ�¼
			if(FirstEnterWindowFlag ==0)
			{
				SlaveFaultWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
//////////////////////////////////////////��������///////////////////////////////////////////////	
   		case  0x0350: //�߼�����
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadSelectedtimesFlag=1;
				DUGSReadMainParameterFlag=1;
				//DUGSReadSlaveParameterFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				ParameterError =0;
				
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby)
			{	
				ChangeStatus(WriteParameters);
				DUGSWriteMainParameterFlag=1;
				//DUGSWriteSlaveParameterFlag=1;
				DUGSWriteCorrectionParameterFlag=1;
				DUGSWriteCorrectionParameter2Flag=1;
				DUGSWriteCorrectionParameter3Flag=1;
				DUGSWritePassiveParameterFlag =1;
				//SpiWrite_Enable
			}
			break;
//////////////////////////////////////////ʵʱ����///////////////////////////////////////////////	
		case  0x0220://����-ALL
		case  0x0221://����-L1
		case  0x0222://����-L2
		case  0x0223://����-L3
			if(FirstEnterWindowFlag ==0)
			{
				WaveformWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
		case  0x0230://��״ͼ-ALL
		case  0x0231://��״ͼ-L1
		case  0x0232://��״ͼ-L2
		case  0x0233://��״ͼ-L3
			if(FirstEnterWindowFlag ==0)
			{
				HistogramWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
		case  0x0400://���ϼ�¼
			if(FirstEnterWindowFlag ==0)
			{	
				//FaultRecordID=1;
				HistoryFaultWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
		case  0x0600://�¼���¼
			if(FirstEnterWindowFlag ==0)
			{
				//FaultRecordID=1;
				EventLogWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
		case  0x0700://��ԴͶ�н���
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadMainParameterFlag=1;
				//DUGSReadSlaveParameterFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				FirstEnterWindowFlag=1;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby)
			{
				ChangeStatus(WriteParameters);
				DUGSWriteMainParameterFlag=1;
				//DUGSWriteSlaveParameterFlag=1;
				DUGSWriteCorrectionParameterFlag=1;
				DUGSWriteCorrectionParameter2Flag=1;
				DUGSWriteCorrectionParameter3Flag=1;
				DUGSWritePassiveParameterFlag =1;
			}
			else if(SaveOrReadFlag==6)
			{
				OnekeyClean=1;
			}
			break;
		case  0x0100://������
		case  0x0800://��Ļ���ػ�����
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadMainParameterFlag=1;
				//DUGSReadSlaveParameterFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
				FirstEnterWindowFlag=1;
			}
			else if((SaveOrReadFlag==2||SaveOrReadFlag==3) )
			{
				if(LocalOnOff==0x01)
				{
					OnOffCommand =0x01;
				}
				else if(LocalOnOff==0x00)
				{	
					OnOffCommand =0x00;
				}
				RecoverProcessFlag =1;
			}
			break;
			
		case  0x0666://����ҳ��
			if(SaveOrReadFlag==1)
			{
				ChangeStatus(ReadParameters);
				DUGSReadMainParameterFlag=1;
				DUGSRead666Flag=1;
				
			}
			else if(SaveOrReadFlag==2||SaveOrReadFlag==3)
			{
				ChangeStatus(WriteParameters);
				DUGSWriteMainParameterFlag=1;
				DUGSWrite666Flag=1;
				//RecoverProcessFlag =1;
			}
			break;
			
		case  0x0900://���ݼ�¼
			if(FirstEnterWindowFlag ==0)
			{	
				DataLogWindowFlag =1;
				FirstEnterWindowFlag =1;
			}
			break;
		case  0x0A00://һ������
			/*if(SaveOrReadFlag==1)
			{	
				ChangeStatus(ReadParameters);
				//DUGSReadARMCorrectFlag=1;
			}
			else */
			if(SaveOrReadFlag==3 && APFStatus<=SystemStandby)
			{
				DUGSWriteARMCorrectFlag=1;
			}
			break;
		case  0x0A01://��������
		if(SaveOrReadFlag==1)
		{
			ChangeStatus(ReadParameters);DUGSReadMainParameterFlag=1;DUGSReadPassiveParameterFlag=1;DUGSReadSlaveParameterFlag=1;
		}
		else if((SaveOrReadFlag==2||SaveOrReadFlag==3) && APFStatus<=SystemStandby){ChangeStatus(WriteParameters);DUGSWriteMainParameterFlag=1;DUGSWritePassiveParameterFlag=1;DUGSWriteSlaveParameterFlag=1;SlaveID=0;}
   		default:
			break;        
	}
	LastPageID =CurrentPageID;
	SlaveIDbak =SlaveID;
}
void DUGSInit(void)
{


	//ChangePage(0x0B);

	
	RestoreCommand  =0;						//�ָ���������

	if(OnOffStatus==0)
	{
		OnOffCommand= 0;					//���ػ�����
	}
	else									//��������
	{
		OnOffCommand= 1;					//���ػ�����
	}
	SaveOrReadFlag	=0;						//��ȡ�����Ӧ��
	//ManualPassiveSwitchFlag=0;				//�߼�����
	Password		=0;						//����
	HistogramSetting=7;						//����ͼ��ʾ
	WaveformAccuracy=0;						//���ξ���
	WaveformID		=0;						//����ͼID
	Waveform2ID		=1;						//����ͼ2ID
	HistogramID		=0;						//��״ͼID
	SlaveID			=0;						//�ӻ�ҳ��ID
	CorrectionID	=0;						//��������ҳ��
	PrimevalID		=0;						//��������ID
	CurrentPageID 	=0x0100;				//ҳ��ID
	FaultRecordID	=1;						//���ϼ�¼ҳ��ID
	APFStatus		=0;						//ϵͳ�Լ���
	SwitchON		=0;						//ͨѶδ����

	
	DUGSInitFlag	=1;							//��ʼ����ɣ���������

}
void ChangePage(uint16 PageID)
{
	static uint16 DelayTime=0;
	uint8 i=0,Send[150],b,*pointer8;
	uint16 SendNUM,Crc_data;
	
	Send[i++] = 0x5A;				  //֡ͷ��
	Send[i++] = 0xA5;				  //֡ͷ��
	Send[i++] = 0x09;				  //���ݳ���
	Send[i++] = 0x82;				  //ָ��
	Send[i++] = 0x00;				  //�Ĵ�����ַ��
	Send[i++] = 0x84;				  //�Ĵ�����ַ��
	Send[i++] = 0x5A;				  //D3
	Send[i++] = 0x01;				  //D2
	Send[i++] = (uint8)(PageID >> 8); //D1
	Send[i++] = (uint8)(PageID);	  //D0
	pointer8=Send;Crc_data=crc_chk(pointer8+3,i-3);//У��
	Send[i++]=(uint8)(Crc_data&0xff);
	Send[i++]=(uint8)(Crc_data>>8);
	pointer8 = Send;
	//	uart_write1;
	for (SendNUM = i; SendNUM > 0; SendNUM--)
	{
		b = *pointer8;
		UART3_SendByte(b);
		pointer8++;
	} //ѡ��UART����
		//	uart_read1;
	DelayTime = GetSystemTime(t10ms);
	while (1)
	{
		if (GetCurrentTime(t10ms, DelayTime) > 4)
			break; //�ӳ�50ms��
	}
}
void ChangeStatus(uint16 Status)
{
	static uint16 DelayTime=0;
	uint8 i=0,Send[150],b,*pointer8;
	uint16 SendNUM,Crc_data;
	
	Send[i++] = 0x5A;
	Send[i++] = 0xA5;
	Send[i++] = 0x07;
	Send[i++] = 0x82;
	Send[i++] = 0x20;
	Send[i++] = 0x13;
	Send[i++] = (uint8)(Status >> 8);
	Send[i++] = (uint8)(Status);
	pointer8=Send;Crc_data=crc_chk(pointer8+3,i-3);//У��
	Send[i++]=(uint8)(Crc_data&0xff);
	Send[i++]=(uint8)(Crc_data>>8);
	pointer8 = Send;
	//uart_write3;
	for (SendNUM = i; SendNUM > 0; SendNUM--)
	{
		b = *pointer8;
		UART3_SendByte(b);
		pointer8++;
	} //ѡ��UART����
	//uart_read3;
	DelayTime = GetSystemTime(t10ms);
	while (1)
	{
		if (GetCurrentTime(t10ms, DelayTime) > 4)
			break; //�ӳ�50ms��
	}
}

void RestoreProcess(void)
{
	uint8 tmp8[1664],temp[100]/*,Eeptempbuff[200]*/;
	uint16	i,j;
	uint32 	tmp32;
	uint64	tmp64H,tmp64L,tmp64Val;

	
	switch(RestoreCommand)
	{
		case 1://���ݲ���
			ChangeStatus(WriteParameters);
			i=0;
			for(j=0;j<100;j++)
			{	
				Init_parameter[i++] =(uint8)(main_parameter[j]>>8);
				Init_parameter[i++] =(uint8)(main_parameter[j]);										//����������
			}
			i=0;
			for(j=0;j<100;j++)
			{	
				Init_parameter[i+200] =(uint8)(Passive_parameter[j]>>8);
				i++;
				Init_parameter[i+200] =(uint8)(Passive_parameter[j]);					//������Դ����
				i++;
			}
			i=0;
			for(j=0; j<100; j++)	
			{
				Init_parameter[2*i+400] =(uint8)(load_correctionA[j]>>8); 
				Init_parameter[2*i+401] =(uint8)(load_correctionA[j]); 
				Init_parameter[2*i+600] =(uint8)(load_correctionB[j]>>8); 
				Init_parameter[2*i+601] =(uint8)(load_correctionB[j]); 
				Init_parameter[2*i+800] =(uint8)(load_correctionC[j]>>8); 
				Init_parameter[2*i+801] =(uint8)(load_correctionC[j]); 									//������������
				i++;
			}
			Init_parameter[1000] =(uint8)(LocalAddr>>8);
			Init_parameter[1001] =(uint8)(LocalAddr);
			Init_parameter[1002] =(uint8)(UART_BPS>>8);
			Init_parameter[1003] =(uint8)(UART_BPS);
			Init_parameter[1004] =(uint8)(RemoteEnable>>8);
			Init_parameter[1005] =(uint8)(RemoteEnable);

			Init_parameter[1006] =(uint8)(OnOffStatus>>8);
			Init_parameter[1007] =(uint8)(OnOffStatus);
			Init_parameter[1008] =(uint8)(AlarmTime1[0]>>8);
			Init_parameter[1009] =(uint8)(AlarmTime1[0]);
			Init_parameter[1010] =(uint8)(AlarmTime1[1]>>8);
			Init_parameter[1011] =(uint8)(AlarmTime1[1]);
			Init_parameter[1012] =(uint8)(AlarmTime1[2]>>8);
			Init_parameter[1013] =(uint8)(AlarmTime1[2]);
			Init_parameter[1014] =(uint8)(AlarmTime1[3]>>8);
			Init_parameter[1015] =(uint8)(AlarmTime1[3]);
			Init_parameter[1016] =(uint8)(AlarmTime1[4]>>8);
			Init_parameter[1017] =(uint8)(AlarmTime1[4]);
			
			Init_parameter[1018] =(uint8)(AlarmTime2[0]>>8);
			Init_parameter[1019] =(uint8)(AlarmTime2[0]);
			Init_parameter[1020] =(uint8)(AlarmTime2[1]>>8);
			Init_parameter[1021] =(uint8)(AlarmTime2[1]);
			Init_parameter[1022] =(uint8)(AlarmTime2[2]>>8);
			Init_parameter[1023] =(uint8)(AlarmTime2[2]);
			Init_parameter[1024] =(uint8)(AlarmTime2[3]>>8);
			Init_parameter[1025] =(uint8)(AlarmTime2[3]);
			Init_parameter[1026] =(uint8)(AlarmTime2[4]>>8);
			Init_parameter[1027] =(uint8)(AlarmTime2[4]);
			
			Init_parameter[1028] =(uint8)(AlarmTime3[0]>>8);
			Init_parameter[1029] =(uint8)(AlarmTime3[0]);
			Init_parameter[1030] =(uint8)(AlarmTime3[1]>>8);
			Init_parameter[1031] =(uint8)(AlarmTime3[1]);
			Init_parameter[1032] =(uint8)(AlarmTime3[2]>>8);
			Init_parameter[1033] =(uint8)(AlarmTime3[2]);
			Init_parameter[1034] =(uint8)(AlarmTime3[3]>>8);
			Init_parameter[1035] =(uint8)(AlarmTime3[3]);
			Init_parameter[1036] =(uint8)(AlarmTime3[4]>>8);
			Init_parameter[1037] =(uint8)(AlarmTime3[4]);
		
			Init_parameter[1038] =(uint8)ntc_type;//(AlarmTime4[0]>>8);
			Init_parameter[1039] =0;//(uint8)(AlarmTime4[0]);
			Init_parameter[1040] =0;//(uint8)(AlarmTime4[1]>>8);
			Init_parameter[1041] =0;//(uint8)(AlarmTime4[1]);
			Init_parameter[1042] =0;//(uint8)(AlarmTime4[2]>>8);
			Init_parameter[1043] =0;//(uint8)(AlarmTime4[2]);
			Init_parameter[1044] =0;//(uint8)(AlarmTime4[3]>>8);
			Init_parameter[1045] =0;//(uint8)(AlarmTime4[3]);
			Init_parameter[1046] =0;//(uint8)(AlarmTime4[4]>>8);
			Init_parameter[1047] =0;//(uint8)(AlarmTime4[4]);
			
			Init_parameter[1048] =(uint8)(ResetCount>>8);
			Init_parameter[1049] =(uint8)(ResetCount);
					
			Init_parameter[1050] =(uint8)(ActiveBal>>8);
			Init_parameter[1051] =(uint8)(ActiveBal);
			Init_parameter[1052] =(uint8)(ReactiveBal>>8);
			Init_parameter[1053] =(uint8)(ReactiveBal);
			Init_parameter[1054] =(uint8)(HarmonicBal>>8);
			Init_parameter[1055] =(uint8)(HarmonicBal);
			Init_parameter[1056] =(uint8)(ApparentBal>>8);
			Init_parameter[1057] =(uint8)(ApparentBal);
			Init_parameter[1058] =(uint8)(SetupMode>>8);
			Init_parameter[1059] =(uint8)(SetupMode);
			Init_parameter[1060] = 0;
			Init_parameter[1061] = 0;
			Init_parameter[1062] = 0;
			Init_parameter[1063] = 0;
			Init_parameter[1064] = 0;
			Init_parameter[1065] = 0;
			Init_parameter[1066] = 0;
			Init_parameter[1067] = 0;
			Init_parameter[1068] = 0;
			Init_parameter[1069] = 0;
			Init_parameter[1070] =(uint8)(ProjectNo>>24);
			Init_parameter[1071] =(uint8)(ProjectNo>>16);
			Init_parameter[1072] =(uint8)(ProjectNo>>8);
			Init_parameter[1073] =(uint8)(ProjectNo);
			Init_parameter[1074] =(uint8)(ProductionNo>>8);
			Init_parameter[1075] =(uint8)(ProductionNo);

			Init_parameter[1076] =(uint8)(VolOnOffEnable>>8);
			Init_parameter[1077] =(uint8)(VolOnOffEnable);
			Init_parameter[1078] =(uint8)(CurOnOffEnable>>8);
			Init_parameter[1079] =(uint8)(CurOnOffEnable);
			
			Init_parameter[1080] =(uint8)(MainCTLocation>>8);
			Init_parameter[1081] =(uint8)(MainCTLocation);
			Init_parameter[1082] =(uint8)(MainCTDirectionA>>8);
			Init_parameter[1083] =(uint8)(MainCTDirectionA);
			Init_parameter[1084] =(uint8)(MainCTDirectionB>>8);
			Init_parameter[1085] =(uint8)(MainCTDirectionB);

			Init_parameter[1086] =(uint8)(MainCTDirectionC>>8);
			Init_parameter[1087] =(uint8)(MainCTDirectionC);
			Init_parameter[1088] =(uint8)(MainCTPhase>>8);
			Init_parameter[1089] =(uint8)(MainCTPhase);
			Init_parameter[1090] =(uint8)(OutCTPhase>>8);
			Init_parameter[1091] =(uint8)(OutCTPhase);

			Init_parameter[1092] =(uint8)(Position[0]>>8);
			Init_parameter[1093] =(uint8)(Position[0]);
			Init_parameter[1094] =(uint8)(Group[0]>>8);
			Init_parameter[1095] =(uint8)(Group[0]);
			Init_parameter[1096] =(uint8)(Capacitance[0]>>8);
			Init_parameter[1097] =(uint8)(Capacitance[0]);

			Init_parameter[1098] =(uint8)(Position[1]>>8);
			Init_parameter[1099] =(uint8)(Position[1]);
			Init_parameter[1100] =(uint8)(Group[1]>>8);
			Init_parameter[1101] =(uint8)(Group[1]);
			Init_parameter[1102] =(uint8)(Capacitance[1]>>8);
			Init_parameter[1103] =(uint8)(Capacitance[1]);

			Init_parameter[1104] =(uint8)(Position[2]>>8);
			Init_parameter[1105] =(uint8)(Position[2]);
			Init_parameter[1106] =(uint8)(Group[2]>>8);
			Init_parameter[1107] =(uint8)(Group[2]);
			Init_parameter[1108] =(uint8)(Capacitance[2]>>8);
			Init_parameter[1109] =(uint8)(Capacitance[2]);

			Init_parameter[1110] =(uint8)(Position[3]>>8);
			Init_parameter[1111] =(uint8)(Position[3]);
			Init_parameter[1112] =(uint8)(Group[3]>>8);
			Init_parameter[1113] =(uint8)(Group[3]);
			Init_parameter[1114] =(uint8)(Capacitance[3]>>8);
			Init_parameter[1115] =(uint8)(Capacitance[3]);

			Init_parameter[1116] =(uint8)(Position[4]>>8);
			Init_parameter[1117] =(uint8)(Position[4]);
			Init_parameter[1118] =(uint8)(Group[4]>>8);
			Init_parameter[1119] =(uint8)(Group[4]);
			Init_parameter[1120] =(uint8)(Capacitance[4]>>8);
			Init_parameter[1121] =(uint8)(Capacitance[4]);

			Init_parameter[1122] =(uint8)(Position[5]>>8);
			Init_parameter[1123] =(uint8)(Position[5]);
			Init_parameter[1124] =(uint8)(Group[5]>>8);
			Init_parameter[1125] =(uint8)(Group[5]);
			Init_parameter[1126] =(uint8)(Capacitance[5]>>8);
			Init_parameter[1127] =(uint8)(Capacitance[5]);

			Init_parameter[1128] =(uint8)(Position[6]>>8);
			Init_parameter[1129] =(uint8)(Position[6]);
			Init_parameter[1130] =(uint8)(Group[6]>>8);
			Init_parameter[1131] =(uint8)(Group[6]);
			Init_parameter[1132] =(uint8)(Capacitance[6]>>8);
			Init_parameter[1133] =(uint8)(Capacitance[6]);

			Init_parameter[1134] =(uint8)(Position[7]>>8);
			Init_parameter[1135] =(uint8)(Position[7]);
			Init_parameter[1136] =(uint8)(Group[7]>>8);
			Init_parameter[1137] =(uint8)(Group[7]);
			Init_parameter[1138] =(uint8)(Capacitance[7]>>8);
			Init_parameter[1139] =(uint8)(Capacitance[7]);

			Init_parameter[1140] =(uint8)(Position[8]>>8);
			Init_parameter[1141] =(uint8)(Position[8]);
			Init_parameter[1142] =(uint8)(Group[8]>>8);
			Init_parameter[1143] =(uint8)(Group[8]);
			Init_parameter[1144] =(uint8)(Capacitance[8]>>8);
			Init_parameter[1145] =(uint8)(Capacitance[8]);
			
			Init_parameter[1146] =(uint8)(OutCTDirectionA>>8);
			Init_parameter[1147] =(uint8)(OutCTDirectionA);
			Init_parameter[1148] =(uint8)(OutCTDirectionB>>8);
			Init_parameter[1149] =(uint8)(OutCTDirectionB);
			Init_parameter[1150] =(uint8)(OutCTDirectionC>>8);
			Init_parameter[1151] =(uint8)(OutCTDirectionC);

			Init_parameter[1152] =(uint8)(Position[9]>>8);
			Init_parameter[1153] =(uint8)(Position[9]);
			Init_parameter[1154] =(uint8)(Group[9]>>8);
			Init_parameter[1155] =(uint8)(Group[9]);
			Init_parameter[1156] =(uint8)(Capacitance[9]>>8);
			Init_parameter[1157] =(uint8)(Capacitance[9]);

			Init_parameter[1158] =(uint8)(Position[10]>>8);
			Init_parameter[1159] =(uint8)(Position[10]);
			Init_parameter[1160] =(uint8)(Group[10]>>8);
			Init_parameter[1161] =(uint8)(Group[10]);
			Init_parameter[1162] =(uint8)(Capacitance[10]>>8);
			Init_parameter[1163] =(uint8)(Capacitance[10]);

			Init_parameter[1164] =(uint8)(Position[11]>>8);
			Init_parameter[1165] =(uint8)(Position[11]);
			Init_parameter[1166] =(uint8)(Group[11]>>8);
			Init_parameter[1167] =(uint8)(Group[11]);
			Init_parameter[1168] =(uint8)(Capacitance[11]>>8);
			Init_parameter[1169] =(uint8)(Capacitance[11]);

			Init_parameter[1170] =(uint8)(Position[12]>>8);
			Init_parameter[1171] =(uint8)(Position[12]);
			Init_parameter[1172] =(uint8)(Group[12]>>8);
			Init_parameter[1173] =(uint8)(Group[12]);
			Init_parameter[1174] =(uint8)(Capacitance[12]>>8);
			Init_parameter[1175] =(uint8)(Capacitance[12]);

			Init_parameter[1176] =(uint8)(Position[13]>>8);
			Init_parameter[1177] =(uint8)(Position[13]);
			Init_parameter[1178] =(uint8)(Group[13]>>8);
			Init_parameter[1179] =(uint8)(Group[13]);
			Init_parameter[1180] =(uint8)(Capacitance[13]>>8);
			Init_parameter[1181] =(uint8)(Capacitance[13]);

			Init_parameter[1182] =(uint8)(Position[14]>>8);
			Init_parameter[1183] =(uint8)(Position[14]);
			Init_parameter[1184] =(uint8)(Group[14]>>8);
			Init_parameter[1185] =(uint8)(Group[14]);
			Init_parameter[1186] =(uint8)(Capacitance[14]>>8);
			Init_parameter[1187] =(uint8)(Capacitance[14]);

			i=0;
			for(j=0; j<32; j++)	
			{
				Init_parameter[2*i+1200] =(uint8)(PassiveChannel[j]>>8); 
				Init_parameter[2*i+1201] =(uint8)(PassiveChannel[j]);
				Init_parameter[2*i+1264] =(uint8)(PassiveValue[j]>>8); 
				Init_parameter[2*i+1265] =(uint8)(PassiveValue[j]); 
				i++;
			}
			Init_parameter[1568]=(uint8)(TotalEnergyHB>>24);	
			Init_parameter[1569]=(uint8)(TotalEnergyHB>>16);							//�ۼƽ����� HH
			Init_parameter[1570]=(uint8)(TotalEnergyHB>>8);
			Init_parameter[1571]=(uint8)(TotalEnergyHB);								//�ۼƽ����� H
			Init_parameter[1572]=(uint8)(TotalEnergyLB>>24);
			Init_parameter[1573]=(uint8)(TotalEnergyLB>>16);							//�ۼƽ����� L
			Init_parameter[1574]=(uint8)(TotalEnergyLB>>8);
			Init_parameter[1575]=(uint8)(TotalEnergyLB);								//�ۼƽ����� LL
			Init_parameter[1576]=(uint8)(TotalRunningTime>>24);
			Init_parameter[1577]=(uint8)(TotalRunningTime>>16);						//�ۼ�����ʱ��H
			Init_parameter[1578]=(uint8)(TotalRunningTime>>8);
			Init_parameter[1579]=(uint8)(TotalRunningTime);							//�ۼ�����ʱ��L
			Init_parameter[1580]=(uint8)(DailyEnergy>>24);
			Init_parameter[1581]=(uint8)(DailyEnergy>>16);							//���ս����� H
			Init_parameter[1582]=(uint8)(DailyEnergy>>8);
			Init_parameter[1583]=(uint8)(DailyEnergy);								//���ս����� L
			Init_parameter[1584]=(uint8)(DailyRunningTime>>8);
			Init_parameter[1585]=(uint8)(DailyRunningTime);							//��������ʱ��
			Init_parameter[1586]=(uint8)(EepromYear>>8);
			Init_parameter[1587]=(uint8)(EepromYear);								//��¼��
			Init_parameter[1588]=(uint8)(EepromMon>>8);
			Init_parameter[1589]=(uint8)(EepromMon);								//��¼��
			Init_parameter[1590]=(uint8)(EepromDay>>8);
			Init_parameter[1591]=(uint8)(EepromDay);								//��¼��
			Init_parameter[1592]= 0;
			Init_parameter[1593]= 0;											//ǰһ���ۼƽ����� HH
			Init_parameter[1594]= 0;
			Init_parameter[1595]= 0;											//ǰһ���ۼƽ����� HL
			//Init_parameter[1220] =0xA5;
			//Init_parameter[1221] =0xA5;
			//Init_parameter[1222] =0xA5;
			//Init_parameter[1223] =0xA5;
			
			Init_parameter[1596] =0xA5;
			Init_parameter[1597] =0xA5;
			Init_parameter[1598] =0xA5;
			Init_parameter[1599] =0xA5;
			I2C_write;
    		WriteEeprom(EepParameterAddrBak,Init_parameter,ParameterNumMaxBak);						//���ݲ�����Eeprom
   			I2C_read;
			BackupFinishFlag =1;
			
			break;
		case 2://ȡ�ر��ݲ���
			ChangeStatus(ReadParameters);
			ReadEeprom(EepParameterAddrBak,Init_parameter,ParameterNumMaxBak);						//ȡ�ر��ݲ���
			if(Init_parameter[ParameterNumMaxBak-1]==0xA5
			 &&Init_parameter[ParameterNumMaxBak-2]==0xA5
			 &&Init_parameter[ParameterNumMaxBak-3]==0xA5
			 &&Init_parameter[ParameterNumMaxBak-4]==0xA5
			 &&Init_parameter[ParameterNumMax-1]==0xA5
			 &&Init_parameter[ParameterNumMax-2]==0xA5
			 &&Init_parameter[ParameterNumMax-3]==0xA5
			 &&Init_parameter[ParameterNumMax-4]==0xA5
			 )	//�ж��Ƿ�д��
			{
				I2C_write;
				WriteEeprom(EepParameterAddr,Init_parameter,ParameterNumMax);   					//����������д����ʽ������
				I2C_read;

				/*I2C_write;
				WriteEeprom(EepPassiveAddr,Init_parameter+ParameterNumMax,PassiveNumMax);   					//����������д����ʽ������
				I2C_read;*/
				
				for(i=0;i<100;i++)
				{	
					main_parameter[i]=(Init_parameter[2*i]<<8)|Init_parameter[2*i+1];										//��ʼ��������
				}
				/*for(i=0;i<10;i++)
				{
					slave_Enable[i]= main_parameter[i+89];																	//�ӻ�ʹ��
				}*/
				for(i=0;i<50;i++)
				{
					Selected_parameter[i] =0;																				//ѡ�β���
				}
				for(i=0;i<19;i++)
				{
					j=main_parameter[31+i];
					if(j>0)Selected_parameter[j-1]=1;
				}
				for(i=0;i<100;i++)
				{
					Passive_parameter[i] =	(Init_parameter[2*i+200]<<8)|Init_parameter[2*i+201];							//��ʼ����Դ������
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
				
				AlarmTime1[0]=(Init_parameter[1008]<<8|Init_parameter[1009]);//��ʱ1	ʹ��
				AlarmTime1[1]=(Init_parameter[1010]<<8|Init_parameter[1011]);//��ʱ1	ʱ
				AlarmTime1[2]=(Init_parameter[1012]<<8|Init_parameter[1013]);//��ʱ1	��
				AlarmTime1[3]=(Init_parameter[1014]<<8|Init_parameter[1015]);//��ʱ1	ʹ��
				AlarmTime1[4]=(Init_parameter[1016]<<8|Init_parameter[1017]);//��ʱ1	ʱ
				
				AlarmTime2[0]=(Init_parameter[1018]<<8|Init_parameter[1019]);//��ʱ2	ʹ��
				AlarmTime2[1]=(Init_parameter[1020]<<8|Init_parameter[1021]);//��ʱ2	ʱ
				AlarmTime2[2]=(Init_parameter[1022]<<8|Init_parameter[1023]);//��ʱ2	��
				AlarmTime2[3]=(Init_parameter[1024]<<8|Init_parameter[1025]);//��ʱ2	ʱ					
				AlarmTime2[4]=(Init_parameter[1026]<<8|Init_parameter[1027]);//��ʱ2	��
				
				AlarmTime3[0]=(Init_parameter[1028]<<8|Init_parameter[1029]);//��ʱ3	ʱ
				AlarmTime3[1]=(Init_parameter[1030]<<8|Init_parameter[1031]);//��ʱ3	��	
				AlarmTime3[2]=(Init_parameter[1032]<<8|Init_parameter[1033]);//��ʱ3 ʹ��
				AlarmTime3[3]=(Init_parameter[1034]<<8|Init_parameter[1035]);//��ʱ3	ʱ
				AlarmTime3[4]=(Init_parameter[1036]<<8|Init_parameter[1037]);//��ʱ3	��	

				ntc_type=(Init_parameter[1038]<<8|Init_parameter[1039]);//��ʱ4 ʹ��
//				AlarmTime4[1]=(Init_parameter[1040]<<8|Init_parameter[1041]);//��ʱ4	ʱ
//				AlarmTime4[2]=(Init_parameter[1042]<<8|Init_parameter[1043]);//��ʱ4	��
//				AlarmTime4[3]=(Init_parameter[1044]<<8|Init_parameter[1045]);//��ʱ4	ʱ
//				AlarmTime4[4]=(Init_parameter[1046]<<8|Init_parameter[1047]);//��ʱ4	��

				ActiveBal =(Init_parameter[1048]<<8|Init_parameter[1049]);//�й���ƽ��
				ReactiveBal=(Init_parameter[1050]<<8|Init_parameter[1051]);//�޹���ƽ��
				HarmonicBal=(Init_parameter[1052]<<8|Init_parameter[1053]);//г����ƽ��
				ApparentBal=(Init_parameter[1054]<<8|Init_parameter[1055]);//���ڲ�ƽ��
				ApparentBal2=ApparentBal;
				SetupMode=(Init_parameter[1056]<<8|Init_parameter[1057]);
				//slave_Reset[4]=(Init_parameter[1058]<<8|Init_parameter[1059]);
				//slave_Reset[5]=(Init_parameter[1060]<<8|Init_parameter[1061]);
				//slave_Reset[6]=(Init_parameter[1062]<<8|Init_parameter[1063]);
				//slave_Reset[7]=(Init_parameter[1064]<<8|Init_parameter[1065]);
				//slave_Reset[8]=(Init_parameter[1066]<<8|Init_parameter[1067]);
				//slave_Reset[9]=(Init_parameter[1068]<<8|Init_parameter[1069]);


				ProjectNo=	((Init_parameter[1070]<<24)\
							 |(Init_parameter[1071]<<16)\
							 |(Init_parameter[1072]<<8)\
							 |(Init_parameter[1073]));
				ProductionNo=(Init_parameter[1074]<<8|Init_parameter[1075]);

				//ProjectNo =2016072801;
				//ProductionNo =1;
				tmp64Val=1000;
				tmp64Val =(uint64) (ProjectNo)*tmp64Val;
				SerialNumber = 2000000000000000+tmp64Val+ProductionNo;
				
				Position[0]=(Init_parameter[1092]<<8|Init_parameter[1093]);
				Group[0]=(Init_parameter[1094]<<8|Init_parameter[1095]);
				Capacitance[0]=(Init_parameter[1096]<<8|Init_parameter[1097]);

				Position[1]=(Init_parameter[1098]<<8|Init_parameter[1099]);
				Group[1]=(Init_parameter[1100]<<8|Init_parameter[1101]);
				Capacitance[1]=(Init_parameter[1102]<<8|Init_parameter[1103]);

				Position[2]=(Init_parameter[1104]<<8|Init_parameter[1105]);
				Group[2]=(Init_parameter[1106]<<8|Init_parameter[1107]);
				Capacitance[2]=(Init_parameter[1108]<<8|Init_parameter[1109]);

				Position[3]=(Init_parameter[1110]<<8|Init_parameter[1111]);
				Group[3]=(Init_parameter[1112]<<8|Init_parameter[1113]);
				Capacitance[3]=(Init_parameter[1114]<<8|Init_parameter[1115]);

				Position[4]=(Init_parameter[1116]<<8|Init_parameter[1117]);
				Group[4]=(Init_parameter[1118]<<8|Init_parameter[1119]);
				Capacitance[4]=(Init_parameter[1120]<<8|Init_parameter[1121]);

				Position[5]=(Init_parameter[1122]<<8|Init_parameter[1123]);
				Group[5]=(Init_parameter[1124]<<8|Init_parameter[1125]);
				Capacitance[5]=(Init_parameter[1126]<<8|Init_parameter[1127]);

				Position[6]=(Init_parameter[1128]<<8|Init_parameter[1129]);
				Group[6]=(Init_parameter[1130]<<8|Init_parameter[1131]);
				Capacitance[6]=(Init_parameter[1132]<<8|Init_parameter[1133]);

				Position[7]=(Init_parameter[1134]<<8|Init_parameter[1135]);
				Group[7]=(Init_parameter[1136]<<8|Init_parameter[1137]);
				Capacitance[7]=(Init_parameter[1138]<<8|Init_parameter[1139]);

				Position[8]=(Init_parameter[1140]<<8|Init_parameter[1141]);
				Group[8]=(Init_parameter[1142]<<8|Init_parameter[1143]);
				Capacitance[8]=(Init_parameter[1144]<<8|Init_parameter[1145]);
				
				OutCTDirectionA=(Init_parameter[1146]<<8|Init_parameter[1147]);
				OutCTDirectionB=(Init_parameter[1148]<<8|Init_parameter[1149]);
				OutCTDirectionC=(Init_parameter[1150]<<8|Init_parameter[1151]);
				
				Position[9]=(Init_parameter[1152]<<8|Init_parameter[1153]);
				Group[9]=(Init_parameter[1154]<<8|Init_parameter[1155]);
				Capacitance[9]=(Init_parameter[1156]<<8|Init_parameter[1157]);

				Position[10]=(Init_parameter[1158]<<8|Init_parameter[1159]);
				Group[10]=(Init_parameter[1160]<<8|Init_parameter[1161]);
				Capacitance[10]=(Init_parameter[1162]<<8|Init_parameter[1163]);

				Position[11]=(Init_parameter[1164]<<8|Init_parameter[1165]);
				Group[11]=(Init_parameter[1166]<<8|Init_parameter[1167]);
				Capacitance[11]=(Init_parameter[1168]<<8|Init_parameter[1169]);

				Position[12]=(Init_parameter[1170]<<8|Init_parameter[1171]);
				Group[12]=(Init_parameter[1172]<<8|Init_parameter[1173]);
				Capacitance[12]=(Init_parameter[1174]<<8|Init_parameter[1175]);

				Position[13]=(Init_parameter[1176]<<8|Init_parameter[1177]);
				Group[13]=(Init_parameter[1178]<<8|Init_parameter[1179]);
				Capacitance[13]=(Init_parameter[1180]<<8|Init_parameter[1181]);

				Position[14]=(Init_parameter[1182]<<8|Init_parameter[1183]);
				Group[14]=(Init_parameter[1184]<<8|Init_parameter[1185]);
				Capacitance[14]=(Init_parameter[1186]<<8|Init_parameter[1187]);
				for(i=0;i<32;i++)
				{
					PassiveChannel[i]=(Init_parameter[1200+2*i]<<8|Init_parameter[1201+2*i]);
				}
				for(i=0;i<32;i++)
				{
					PassiveValue[i]=(Init_parameter[1264+2*i]<<8|Init_parameter[1265+2*i]);
				}
				if(SetupMode==0)
				{
					CabinProcess();
				}
				if(SetupMode==1)
				{
					ChannelProcess();
				}
				//ProjectNo =2016072801;
				//ProductionNo =1;
				
				
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
				ReStoreBackupSuccess =1;
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag=1;
				WriteCorrectionParameter2Flag=1;
				WriteCorrectionParameter3Flag=1;
				WritePassiveParameterFlag=1;
				DUGSReadMainParameterFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
			}
			else ReStoreBackupFailure =1;

			break;
		case 3://�ָ�Ĭ��
			ChangeStatus(ReadParameters);
			ReadEeprom(EepParameterInitAddr,Eep_parameter,ParameterInitNumMax);						//��ȡEeprom�ڲ��� 	

			tmp32 = ((Eep_parameter[ParameterInitNumMax-4]<<24)
					|(Eep_parameter[ParameterInitNumMax-3]<<16)
					|(Eep_parameter[ParameterInitNumMax-2]<<8)
					|(Eep_parameter[ParameterInitNumMax-1]));
			if(tmp32 ==EepromVersion)																//�жϳ��������汾�Ƿ����
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

				
				for(i=0;i<100;i++)
				{	
					main_parameter[i]=(Init_parameter[2*i]<<8)|Init_parameter[2*i+1];										//��ʼ��������
				}
				/*for(i=0;i<10;i++)
				{
					slave_Enable[i]= main_parameter[i+89];																	//�ӻ�ʹ��
				}*/
				for(i=0;i<50;i++)
				{
					Selected_parameter[i] =0;																				//ѡ�β���
				}
				for(i=0;i<19;i++)
				{
					j=main_parameter[31+i];
					if(j>0)Selected_parameter[j-1]=1;
				}
				for(i=0; i<100; i++)	
				{
					Passive_parameter[i] =	(Init_parameter[2*i+200]<<8)|Init_parameter[2*i+201];							//��ʼ����Դ������
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
				
				AlarmTime1[0]=(Init_parameter[1008]<<8|Init_parameter[1009]);//��ʱ1	ʹ��
				AlarmTime1[1]=(Init_parameter[1010]<<8|Init_parameter[1011]);//��ʱ1	ʱ
				AlarmTime1[2]=(Init_parameter[1012]<<8|Init_parameter[1013]);//��ʱ1	��
				AlarmTime1[3]=(Init_parameter[1014]<<8|Init_parameter[1015]);//��ʱ1	ʹ��
				AlarmTime1[4]=(Init_parameter[1016]<<8|Init_parameter[1017]);//��ʱ1	ʱ
				
				AlarmTime2[0]=(Init_parameter[1018]<<8|Init_parameter[1019]);//��ʱ2	ʹ��
				AlarmTime2[1]=(Init_parameter[1020]<<8|Init_parameter[1021]);//��ʱ2	ʱ
				AlarmTime2[2]=(Init_parameter[1022]<<8|Init_parameter[1023]);//��ʱ2	��
				AlarmTime2[3]=(Init_parameter[1024]<<8|Init_parameter[1025]);//��ʱ2	ʱ					
				AlarmTime2[4]=(Init_parameter[1026]<<8|Init_parameter[1027]);//��ʱ2	��
				
				AlarmTime3[0]=(Init_parameter[1028]<<8|Init_parameter[1029]);//��ʱ3	ʱ
				AlarmTime3[1]=(Init_parameter[1030]<<8|Init_parameter[1031]);//��ʱ3	��	
				AlarmTime3[2]=(Init_parameter[1032]<<8|Init_parameter[1033]);//��ʱ3 ʹ��
				AlarmTime3[3]=(Init_parameter[1034]<<8|Init_parameter[1035]);//��ʱ3	ʱ
				AlarmTime3[4]=(Init_parameter[1036]<<8|Init_parameter[1037]);//��ʱ3	��	

				ntc_type=Init_parameter[1038];//
//				AlarmTime4[0]=(Init_parameter[1038]<<8|Init_parameter[1039]);//��ʱ4 ʹ��
//				AlarmTime4[1]=(Init_parameter[1040]<<8|Init_parameter[1041]);//��ʱ4	ʱ
//				AlarmTime4[2]=(Init_parameter[1042]<<8|Init_parameter[1043]);//��ʱ4	��
//				AlarmTime4[3]=(Init_parameter[1044]<<8|Init_parameter[1045]);//��ʱ4	ʱ
//				AlarmTime4[4]=(Init_parameter[1046]<<8|Init_parameter[1047]);//��ʱ4	��

				ActiveBal =(Init_parameter[1048]<<8|Init_parameter[1049]);//�й���ƽ��
				ReactiveBal=(Init_parameter[1050]<<8|Init_parameter[1051]);//�޹���ƽ��
				HarmonicBal=(Init_parameter[1052]<<8|Init_parameter[1053]);//г����ƽ��
				ApparentBal=(Init_parameter[1054]<<8|Init_parameter[1055]);//���ڲ�ƽ��
				SetupMode=1;
				//slave_Reset[4]=(Init_parameter[1058]<<8|Init_parameter[1059]);
				//slave_Reset[5]=(Init_parameter[1060]<<8|Init_parameter[1061]);
				//slave_Reset[6]=(Init_parameter[1062]<<8|Init_parameter[1063]);
				//slave_Reset[7]=(Init_parameter[1064]<<8|Init_parameter[1065]);
				//slave_Reset[8]=(Init_parameter[1066]<<8|Init_parameter[1067]);
				//slave_Reset[9]=(Init_parameter[1068]<<8|Init_parameter[1069]);


				ProjectNo=	((Init_parameter[1070]<<24)\
							 |(Init_parameter[1071]<<16)\
							 |(Init_parameter[1072]<<8)\
							 |(Init_parameter[1073]));
				ProductionNo=(Init_parameter[1074]<<8|Init_parameter[1075]);

				//ProjectNo =2016072801;
				//ProductionNo =1;
				tmp64Val=1000;
				tmp64Val =(uint64) (ProjectNo)*tmp64Val;
				SerialNumber = 2000000000000000+tmp64Val+ProductionNo;
				
				Position[0]=(Init_parameter[1092]<<8|Init_parameter[1093]);
				Group[0]=(Init_parameter[1094]<<8|Init_parameter[1095]);
				Capacitance[0]=(Init_parameter[1096]<<8|Init_parameter[1097]);

				Position[1]=(Init_parameter[1098]<<8|Init_parameter[1099]);
				Group[1]=(Init_parameter[1100]<<8|Init_parameter[1101]);
				Capacitance[1]=(Init_parameter[1102]<<8|Init_parameter[1103]);

				Position[2]=(Init_parameter[1104]<<8|Init_parameter[1105]);
				Group[2]=(Init_parameter[1106]<<8|Init_parameter[1107]);
				Capacitance[2]=(Init_parameter[1108]<<8|Init_parameter[1109]);

				Position[3]=(Init_parameter[1110]<<8|Init_parameter[1111]);
				Group[3]=(Init_parameter[1112]<<8|Init_parameter[1113]);
				Capacitance[3]=(Init_parameter[1114]<<8|Init_parameter[1115]);

				Position[4]=(Init_parameter[1116]<<8|Init_parameter[1117]);
				Group[4]=(Init_parameter[1118]<<8|Init_parameter[1119]);
				Capacitance[4]=(Init_parameter[1120]<<8|Init_parameter[1121]);

				Position[5]=(Init_parameter[1122]<<8|Init_parameter[1123]);
				Group[5]=(Init_parameter[1124]<<8|Init_parameter[1125]);
				Capacitance[5]=(Init_parameter[1126]<<8|Init_parameter[1127]);

				Position[6]=(Init_parameter[1128]<<8|Init_parameter[1129]);
				Group[6]=(Init_parameter[1130]<<8|Init_parameter[1131]);
				Capacitance[6]=(Init_parameter[1132]<<8|Init_parameter[1133]);

				Position[7]=(Init_parameter[1134]<<8|Init_parameter[1135]);
				Group[7]=(Init_parameter[1136]<<8|Init_parameter[1137]);
				Capacitance[7]=(Init_parameter[1138]<<8|Init_parameter[1139]);

				Position[8]=(Init_parameter[1140]<<8|Init_parameter[1141]);
				Group[8]=(Init_parameter[1142]<<8|Init_parameter[1143]);
				Capacitance[8]=(Init_parameter[1144]<<8|Init_parameter[1145]);
				OutCTDirectionA=(Init_parameter[1146]<<8|Init_parameter[1147]);
				OutCTDirectionB=(Init_parameter[1148]<<8|Init_parameter[1149]);
				OutCTDirectionC=(Init_parameter[1150]<<8|Init_parameter[1151]);
				
				Position[9]=(Init_parameter[1152]<<8|Init_parameter[1153]);
				Group[9]=(Init_parameter[1154]<<8|Init_parameter[1155]);
				Capacitance[9]=(Init_parameter[1156]<<8|Init_parameter[1157]);

				Position[10]=(Init_parameter[1158]<<8|Init_parameter[1159]);
				Group[10]=(Init_parameter[1160]<<8|Init_parameter[1161]);
				Capacitance[10]=(Init_parameter[1162]<<8|Init_parameter[1163]);

				Position[11]=(Init_parameter[1164]<<8|Init_parameter[1165]);
				Group[11]=(Init_parameter[1166]<<8|Init_parameter[1167]);
				Capacitance[11]=(Init_parameter[1168]<<8|Init_parameter[1169]);

				Position[12]=(Init_parameter[1170]<<8|Init_parameter[1171]);
				Group[12]=(Init_parameter[1172]<<8|Init_parameter[1173]);
				Capacitance[12]=(Init_parameter[1174]<<8|Init_parameter[1175]);

				Position[13]=(Init_parameter[1176]<<8|Init_parameter[1177]);
				Group[13]=(Init_parameter[1178]<<8|Init_parameter[1179]);
				Capacitance[13]=(Init_parameter[1180]<<8|Init_parameter[1181]);

				Position[14]=(Init_parameter[1182]<<8|Init_parameter[1183]);
				Group[14]=(Init_parameter[1184]<<8|Init_parameter[1185]);
				Capacitance[14]=(Init_parameter[1186]<<8|Init_parameter[1187]);
				for(i=0;i<32;i++)
				{
					PassiveChannel[i]=(Init_parameter[1200+2*i]<<8|Init_parameter[1201+2*i]);
				}
				for(i=0;i<32;i++)
				{
					PassiveValue[i]=(Init_parameter[1264+2*i]<<8|Init_parameter[1265+2*i]);
				}
				if(SetupMode==0)
				{
					CabinProcess();
				}
				if(SetupMode==1)
				{
					ChannelProcess();
				}
				//ProjectNo =2016072801;
				//ProductionNo =1;
				
				
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
				
				ReStoreFactorySuccess =1;
				WriteMainParameterFlag=1;
				WriteCorrectionParameterFlag=1;
				WriteCorrectionParameter2Flag=1;
				WriteCorrectionParameter3Flag=1;
				WritePassiveParameterFlag=1;
				DUGSReadMainParameterFlag=1;
				DUGSReadCorrectionParameterFlag=1;
				DUGSReadCorrectionParameter2Flag=1;
				DUGSReadCorrectionParameter3Flag=1;
				DUGSReadPassiveParameterFlag=1;
			}
			else ReStoreFactoryFailure=1;
			break;
		case 4://�������
			ChangeStatus(ClearFaults);
			for(i=0;i<FaultDataNumMax;i++){gz_dan[i]=0;}											//���ϼ�¼����
			I2C_write;//aWP
  			WriteEeprom(EepFaultDataAddr,gz_dan,FaultDataNumMax);
  			I2C_read;//aWP
  			for(i=0;i<EventLogNumMax;i++){Event_Logging[i]=0;}											//���ϼ�¼����
			I2C_write;//aWP
  			WriteEeprom(EepEventLogAddr,Event_Logging,EventLogNumMax);
  			I2C_read;//aWP
			for(i=0;i<16;i++)
			{
				for(j=0;j<100;j++)
				{
					SlaveFault[i][j]=0;																//�ӻ���������
				}
			}
			for(i=0;i<SlaveFaultNumMax;i++)
			{
				tmp8[i] =0;
			}
  			I2C_write;//aWP
  			WriteEeprom(EepSlaveFaultAddr,tmp8,SlaveFaultNumMax);
  			I2C_read;//aWP
  			ClearFaultSuccess =1;
			break;
		case 5://ǿ������
			/*if(SystemStatus==SystemPfault)															//ϵͳ����
			{
				for(i=0;i<main_parameter[99];i++)
				{
					slave_Enable[i] =22;
					main_parameter[89+i] =slave_Enable[i];
				}
				j=0;
				for(i=0;i<100;i++)
				{
					Eeptempbuff[j++] =(uint8)(main_parameter[i]>>8);
					Eeptempbuff[j++] =(uint8)(main_parameter[i]);
				}
				I2C_write;													//д��I2C
				WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
				I2C_read;	
				WriteMainParameterFlag =1;
				
				ResetCount=50000;
				slave_Reset[0] =50000;
				slave_Reset[1] =50000;
				slave_Reset[2] =50000;
				slave_Reset[3] =50000;
				slave_Reset[4] =50000;
				slave_Reset[5] =50000;
				slave_Reset[6] =50000;
				slave_Reset[7] =50000;
				slave_Reset[8] =50000;
				slave_Reset[9] =50000;
				
				
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
				WriteEeprom(EepParameterAddr+1048,temp,8);   			//����������д����ʽ������
				I2C_read;

				I2C_write;													//д��I2C
				WriteEeprom(EepParameterAddr+1056,temp+8,14);   			//����������д����ʽ������
				I2C_read;

				//Reset();
				DUGSReadMainParameterFlag=1;
				SystemResetSuccess =1;
				IllegalOperation =0;
				 
				SpiComm_Disable;										//  ϵͳǿ������
			}
			else 
			{
				ChangeStatus(IllegalOperating);
				IllegalOperation =1;
			}*/
			SpiComm_Disable;										//  ϵͳǿ������
			SystemResetSuccess =1;
			break;
		case 6://ϵͳ��ʼ��
			/*RTCStop();RTCInit();     //��ʼ��
			local_time.RTC_Year = 2015;
			local_time.RTC_Mon  = 1;
			local_time.RTC_Mday = 1;
			local_time.RTC_Hour = 0;
     		local_time.RTC_Min  = 0;
     		RTCSetTime( local_time );//����ʱ��
     		RTCStart();*/              //����RTC
//     		TotalEnergyHB =0; 
//			TotalEnergyLB =0;
//			TotalRunningTime =0;
//			
//			DailyEnergy =0;
//			DailyRunningTime =0;
//			
//			EepromYear =2015;
//			EepromMon =1;
//			EepromDay =1;

//			TotalEnergyBakHB =0;
//			TotalEnergyBakLB =0;
//			TotalRunningTimeBak =0;
//			temp[0] =(uint8)(TotalEnergyHB>>24);
//			temp[1] =(uint8)(TotalEnergyHB>>16);
//			temp[2] =(uint8)(TotalEnergyHB>>8);
//			temp[3] =(uint8)(TotalEnergyHB);		
//			temp[4] =(uint8)(TotalEnergyLB>>24);
//			temp[5] =(uint8)(TotalEnergyLB>>16);
//			temp[6] =(uint8)(TotalEnergyLB>>8);
//			temp[7] =(uint8)(TotalEnergyLB);
//			temp[8] =(uint8)(TotalRunningTime>>24);
//			temp[9] =(uint8)(TotalRunningTime>>16);
//			temp[10] =(uint8)(TotalRunningTime>>8);
//			temp[11] =(uint8)(TotalRunningTime);
//			temp[12] =(uint8)(DailyEnergy>>24);
//			temp[13] =(uint8)(DailyEnergy>>16);
//			temp[14] =(uint8)(DailyEnergy>>8);
//			temp[15] =(uint8)(DailyEnergy);
//			temp[16] =(uint8)(DailyRunningTime>>8);
//			temp[17] =(uint8)(DailyRunningTime);

//			I2C_write;
//			WriteEeprom(EepParameterAddr+1184,temp,18);
//			I2C_read;

//			temp[0] =(uint8)(DailyEnergy>>24);
//			temp[1] =(uint8)(DailyEnergy>>16);
//			temp[2] =(uint8)(DailyEnergy>>8);
//			temp[3] =(uint8)(DailyEnergy);
//			temp[4] =(uint8)(DailyRunningTime>>8);
//			temp[5] =(uint8)(DailyRunningTime);
//			temp[6] =(uint8)(EepromYear>>8);
//			temp[7] =(uint8)(EepromYear);
//			temp[8] =(uint8)(EepromMon>>8);
//			temp[9] =(uint8)(EepromMon);
//			temp[10] =(uint8)(EepromDay>>8);
//			temp[11] =(uint8)(EepromDay);
//			temp[12] =(uint8)(TotalEnergyBakHB>>24);
//			temp[13] =(uint8)(TotalEnergyBakHB>>16);
//			temp[14] =(uint8)(TotalEnergyBakHB>>8);
//			temp[15] =(uint8)(TotalEnergyBakHB);
//			temp[16] =(uint8)(TotalEnergyBakLB>>24);
//			temp[17] =(uint8)(TotalEnergyBakLB>>16);
//			temp[18] =(uint8)(TotalEnergyBakLB>>8);
//			temp[19] =(uint8)(TotalEnergyBakLB);
//			//temp[20] =(uint8)(TotalRunningTimeBak>>24);
//			//temp[21] =(uint8)(TotalRunningTimeBak>>16);
//			//temp[22] =(uint8)(TotalRunningTimeBak>>8);
//			//temp[23] =(uint8)(TotalRunningTimeBak);
//			I2C_write;
//			WriteEeprom(EepParameterAddr+1196,temp,20);   					//����������д����ʽ������
//			I2C_read;

					TotalEnergyHB =0; 
					TotalEnergyLB =0;
					TotalRunningTime =0;
					
					DailyEnergy =0;
					DailyRunningTime =0;
					
					EepromYear =2015;
					EepromMon =1;
					EepromDay =1;

					TotalEnergyBakHB =0;
					TotalEnergyBakLB =0;
					TotalRunningTimeBak =0;
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

		
					I2C_write;
					WriteEeprom(EepParameterAddr+1568,temp,24);
					I2C_read;


			SystemInitSuccess =1;
			break;
		default:
			break;
	}
	RestoreProcessFlag=1;

}
uint8 FaultProcess(uint8 FaultPageNo,uint8 row,uint8 type)//����������ҳ�棬��ǰ������ÿҳ������
{
	uint8 i,FaultMax,FaultCurrentNo,tmp[42],tmpValH,tmpValL;
	uint16 Integer,remainder,FaultAddr;
	volatile uint8	FaultReadFaultFlag=0;

	if(CurrentPageID ==0x0400)	//��������
	{
		FaultMax =gz_dan[0];
	}//��������
	else if(CurrentPageID ==0x0600)	//��������
	{
		FaultMax =Event_Logging[0];
	}//�¼�����
	else FaultMax =SlaveFault[SlaveID][0];
	Integer =FaultMax/type;									//����
	remainder =FaultMax%type;								//����

	if(row >type)row = type;								// ��������С�� ����������(4��/7��)
	if(FaultPageNo>Integer+1)FaultPageNo =Integer+1;		// ��ǰ����ҳ��������С�ڵ�����ҳ��
	if(type ==0)FaultReadFaultFlag=1;						// ���������Ϊ0������
	if(FaultPageNo ==0)FaultReadFaultFlag=1;				// �����ǰҳ��Ϊ0������
	
	if(Integer+1>FaultPageNo)			//������ϴ���ҳ���ڵ�ǰҳ
	{	
		FaultCurrentNo =FaultMax-(FaultPageNo-1)*type;//��ǰҳ���к�=��������-����ҳ��*ÿҳ����
		FaultAddr =(FaultCurrentNo-row)*7;		//���ϵ�ַ=�����ϵ�ǰҳ����-��ǰ������*7
	}
	else
	{
		if(remainder>0)
		{
			FaultCurrentNo =remainder;
			if(FaultCurrentNo > row)
			{
				FaultAddr =(FaultCurrentNo-row)*7;
			}
			else FaultReadFaultFlag =1;
		}
		else FaultReadFaultFlag =1; 
	}
	
	if(CurrentPageID ==0x0400)	//��������
	{
		FaultCode.No    =gz_dan[FaultAddr];		
		FaultCode.Year  =gz_dan[FaultAddr-1];
		FaultCode.Month =gz_dan[FaultAddr-2];
		FaultCode.Day   =gz_dan[FaultAddr-3];
		FaultCode.Hour  =gz_dan[FaultAddr-4];
		FaultCode.Min   =gz_dan[FaultAddr-5];
		FaultCode.Code  =gz_dan[FaultAddr-6];	
	}
	else if(CurrentPageID ==0x0600)	//�¼���¼
	{
		FaultCode.No    =Event_Logging[FaultAddr];		
		FaultCode.Year  =Event_Logging[FaultAddr-1];
		FaultCode.Month =Event_Logging[FaultAddr-2];
		FaultCode.Day   =Event_Logging[FaultAddr-3];
		FaultCode.Hour  =Event_Logging[FaultAddr-4];
		FaultCode.Min   =Event_Logging[FaultAddr-5];
		FaultCode.Code  =Event_Logging[FaultAddr-6];	
	}
	else
	{
		//(SlaveID)			//�ӻ�����
		{
			FaultCode.No    =SlaveFault[SlaveID][FaultAddr];		
			FaultCode.Year  =SlaveFault[SlaveID][FaultAddr-1];
			FaultCode.Month =SlaveFault[SlaveID][FaultAddr-2];
			FaultCode.Day   =SlaveFault[SlaveID][FaultAddr-3];
			FaultCode.Hour  =SlaveFault[SlaveID][FaultAddr-4];
			FaultCode.Min   =SlaveFault[SlaveID][FaultAddr-5];
			FaultCode.Code  =SlaveFault[SlaveID][FaultAddr-6];
		}
	}
	


	if(FaultReadFaultFlag ==1)
	{
		for(i=0;i<42;i++)
		{
			tmp[i]=' ';
		}
	}
	else
	{
		tmp[0]=FaultCode.No/100+0x30;			   
		tmp[1]=(FaultCode.No%100)/10+0x30;	
		tmp[2]=FaultCode.No%10+0x30; 		   
		tmp[3]='.';	
		tmp[4]=' ';
		tmp[5]=' ';
		tmp[6]='2';
		tmp[7]='0';	
		tmp[8]=FaultCode.Year/10+0x30;
		tmp[9]=FaultCode.Year%10+0x30;			   
		tmp[10]='-'; 			
		tmp[11]=FaultCode.Month/10+0x30;
		tmp[12]=FaultCode.Month%10+0x30;						   
		tmp[13]='-'; 					   
		tmp[14]=FaultCode.Day/10+0x30;
		tmp[15]=FaultCode.Day%10+0x30;			   
		tmp[16]='-'; 					   
		tmp[17]=FaultCode.Hour/10+0x30;
		tmp[18]=FaultCode.Hour%10+0x30;				 
		tmp[19]='-'; 					   
		tmp[20]=FaultCode.Min/10+0x30;
		tmp[21]=FaultCode.Min%10+0x30;
		tmp[22]=' ';
		tmp[23]=' ';
		tmp[24]=' ';
		tmp[25]=' ';
		tmp[26]=' ';
		tmp[27]=' ';//0xB4;
		tmp[28]=' ';//0xFA;
		tmp[29]=' ';//0xC2;
		tmp[30]=' ';//0xEB;
		tmp[31]=' ';//0x3a;
		if(FaultCode.Code>99)
		{
			tmpValH =FaultCode.Code/100;
			tmpValL =FaultCode.Code%100;
			
			tmp[32]=tmpValH+0x30;
			tmp[33]=tmpValL/10+0x30;
			tmp[34]=tmpValL%10+0x30;
			tmp[35]=' ';
			tmp[36]=' ';
			tmp[37]=' ';
			tmp[38]=' ';
			tmp[39]=' ';
			tmp[40]=' ';
			tmp[41]=' ';
		}
		else 
		{
			if(FaultCode.Code>9)
			{
				tmp[32]=FaultCode.Code/10+0x30;
				tmp[33]=FaultCode.Code%10+0x30;
				tmp[34]=' ';
				tmp[35]=' ';
				tmp[36]=' ';
				tmp[37]=' ';
				tmp[38]=' ';
				tmp[39]=' ';
				tmp[40]=' ';
				tmp[41]=' ';
			}
			else 
			{
				tmp[32]=FaultCode.Code+0x30;
				tmp[33]=' ';
				tmp[34]=' ';
				tmp[35]=' ';
				tmp[36]=' ';
				tmp[37]=' ';
				tmp[38]=' ';
				tmp[39]=' ';
				tmp[40]=' ';
				tmp[41]=' ';
			}
		}
	}
	for(i=0;i<21;i++)
	{
		CommunWithDUGS.Databuff[i]=(tmp[2*i]<<8)|tmp[2*i+1];
	}
	return(FaultReadFaultFlag);
}
void StatusProcess(void)
{
	uint8 i,j=0/*k,Eeptempbuff[200]*/;
	//static uint8 Slave_DisableFlag[10] ={0,0,0,0,0,0,0,0,0,0};
	
	APFStatus =dsp_data[78];//��DSP��״̬
	MainErrorCode = dsp_data[79];//��DSP�Ĺ��ϴ���
	//MainErrorCode =13;
	for(i=0;i<10;i++)
	{
		SlaveErrorCode[i] =dsp_data[80+i];//�ӻ�1-10���ϴ���
	}
	
	//SlaveErrorCode[0] =255;
	//SlaveErrorCode[2] =255;
/////////////////////////////////////////����״̬����////////////////////////////////////////////// 
	if(MainErrorCode==0)
	{
		switch(APFStatus)
		{
			case 0: 
				SystemStatus = SystemChecking;
				DispMode =0;		//ϵͳ�Լ�
				break;
			case 1:
				SystemStatus = SystemSetting;
				DispMode =1;		//����������
				break;
			case 2:
				SystemStatus = SystemStandby;
				DispMode =2;		//����
				break;
			case 3: 
				SystemStatus = SystemReady;
				DispMode =3;		//����
//				for(i=0;i<10;i++)
//				{SlaveStatus[i]=2;}
				break;
			case 4: 
				SystemStatus = SystemDerating;
				DispMode =3;		//����
				break;
			case 5: 
				SystemStatus = SystemRunning;
				DispMode =3;		//����
				break;
			case 6:
				SystemStatus = SystemReseting;
				DispMode =4;		//�ȴ���������
				break;
			default:break;
		}
	}
	else
	{
		if(MainErrorCode>10 && MainErrorCode<100)
		{
			SystemStatus = SystemProtect;
			DispMode =MainErrorCode-2;
			alarm_time = RTCGetTime();		//��ʱ���
		}
		else if(MainErrorCode>99 && MainErrorCode<399)
		{
			if(MainErrorCode==100)
			{
				DispMode =42;
			}
			else if(MainErrorCode==200)
			{
				DispMode =43;
			}
			else if(MainErrorCode==300)
			{
				MainErrorCode =MainErrorCode-250;
				DispMode =44;
				SystemStatus = SystemProtect;
				alarm_time = RTCGetTime();		//��ʱ���
			}
			else
			{	
				if(MainErrorCode>300)
				{
					MainErrorCode =MainErrorCode-250;
				}
				DispMode =(MainErrorCode%100)-2;
				SystemStatus = SystemProtect;
				alarm_time = RTCGetTime();		//��ʱ���
			}
			
		}
		else if(MainErrorCode>9999 && MainErrorCode<11000)
		{
			SystemStatus = SystemParameterError;
			DispMode =45;
		}
		else if(MainErrorCode>10999 && MainErrorCode<=12000)
		{
			SystemStatus = SystemParameterError;
			DispMode =45;
			//current_time = RTCGetTime();		//��ʱ���
		}
		else
		{
			switch(APFStatus)
			{
				case 0: 
					SystemStatus = SystemChecking;
					//DispMode =0;		//ϵͳ�Լ�
					break;
				case 1:
					SystemStatus = SystemSetting;
					//DispMode =1;		//����������
					break;
				case 2:
					SystemStatus = SystemStandby;
					//DispMode =2;		//����
					break;
				case 3: 
					SystemStatus = SystemReady;
					//DispMode =3;		//����
//					for(i=0;i<10;i++)
//					{SlaveStatus[i]=2;}
					break;
				case 4: 
					SystemStatus = SystemDerating;
					//DispMode =3;		//����
					break;
				case 5: 
					SystemStatus = SystemRunning;
					//DispMode =3;		//����
					break;
				case 6:
					SystemStatus = SystemReseting;
					//DispMode =4;		//�ȴ���������
					break;
				default:break;
			}
			DispMode =MainErrorCode-2;
		}
	}
/////////////////////////////////////////�ӻ�״̬����//////////////////////////////////////////////	
	for(i=0;i<main_parameter[99];i++)
	{
		/*if(slave_Reset[0] ==0||ResetCount==0)
		{
			DispMode =49;
			SystemStatus =SystemPfault;
		}
		else
		{
			if(slave_Reset[i] ==0 && Slave_DisableFlag[i] ==0)
			{
				Slave_DisableFlag[i] =1;
				slave_Enable[i] =0;
				main_parameter[89+i] =slave_Enable[i];
				//main_parameter[99] =main_parameter[99]-1;
				k=0;
				for(i=0;i<100;i++)
				{
					Eeptempbuff[k++] =(uint8)(main_parameter[i]>>8);
					Eeptempbuff[k++] =(uint8)(main_parameter[i]);
				}
				I2C_write;													//д��I2C
				WriteEeprom(EepParameterAddr,Eeptempbuff,200);   			//����������д����ʽ������
				I2C_read;	
				WriteMainParameterFlag =1;
				DUGSReadMainParameterFlag =1;
				
			}
		}*/
		
		/*if(slave_Enable[j]!=22)
		{
			SlaveStatus[j++] =0;
			continue;
		}*/
		if(SlaveErrorCode[i] ==0)
		{
			//SlaveErrorCode[i] =dsp_data[80+i];
			SlaveFaultFlag[i] =SystemStatus;
			if(SystemStatus == SystemChecking)//�Լ�״̬
			{
				SlaveStatus[i] =0;
			}
			else if(SystemStatus ==SystemSetting||SystemStatus ==SystemStandby||SystemStatus ==SystemReseting)
			{
				SlaveStatus[i] =1;
			}
			else if(SystemStatus ==SystemReady||SystemStatus ==SystemDerating||SystemStatus ==SystemRunning)
			{
				SlaveStatus[i] =2;
			}
			else if(SystemStatus ==SystemProtect)
			{
				SlaveStatus[i] =3;
			}
			else if(SystemStatus ==SystemPfault)
			{
				SlaveStatus[i] =4;
			}
			else SlaveStatus[i] =1;
		}
		else
		{
			if(SlaveErrorCode[i]>4 && SlaveErrorCode[i]<100)
			{
				SlaveStatus[i] =3;
				alarm_time = RTCGetTime();		//��ʱ���
				SlaveFaultFlag[i] =SystemProtect;
			}
			else if(SlaveErrorCode[i]>99 && SlaveErrorCode[i] <200)
			{
				SlaveStatus[i] =4;
				SlaveFaultFlag[i] =SystemParameterError;
			}
			else if(SlaveErrorCode[i]>199 && SlaveErrorCode[i]<256)
			{
				SlaveStatus[i] =4;
				alarm_time = RTCGetTime();		//��ʱ���
				SlaveFaultFlag[i] =SystemPfault;
			}
			else
			{
				puyaotestfault++;
				if(SystemStatus == SystemChecking)
				{
					SlaveStatus[i] =0;
				}
				else if(SystemStatus ==SystemSetting||SystemStatus ==SystemStandby||SystemStatus ==SystemReseting)
				{
					SlaveStatus[i] =1;
				}
				else if(SystemStatus ==SystemReady||SystemStatus ==SystemDerating||SystemStatus ==SystemRunning)
				{
					SlaveStatus[i] =2;
				}
				else if(SystemStatus ==SystemProtect)
				{
					SlaveStatus[i] =3;
				}
				else if(SystemStatus ==SystemPfault)
				{
					SlaveStatus[i] =4;
				}
				else SlaveStatus[i] =1;
			}
		}
		j++;
	}
	//if(j<9)//ʣ��ӻ� ��ʾδ����
	//{
		for(i=j;i<15;i++)
		{
			SlaveStatus[i] =0;
		}
	//}
}
void SaveARMParaProcess(void)
{
	uint8 Eeptempbuff[200];
	
	Eeptempbuff[0] =(uint8)(LocalAddr>>8);
	Eeptempbuff[1] =(uint8)(LocalAddr);
	Eeptempbuff[2] =(uint8)(UART_BPS>>8);
	Eeptempbuff[3] =(uint8)(UART_BPS);
	Eeptempbuff[4] =(uint8)(RemoteEnable>>8);
	Eeptempbuff[5] =(uint8)(RemoteEnable);

	Eeptempbuff[6] =(uint8)(OnOffStatus>>8);
	Eeptempbuff[7] =(uint8)(OnOffStatus);	
	//�޸�ԭ�� ѡ�β����ᱻ��Ļ�ϼĴ���31-49���
	Eeptempbuff[8] =(uint8)(AlarmTime1[0]>>8);
	Eeptempbuff[9] =(uint8)(AlarmTime1[0]);
	Eeptempbuff[10] =(uint8)(AlarmTime1[1]>>8);
	Eeptempbuff[11] =(uint8)(AlarmTime1[1]);
	Eeptempbuff[12] =(uint8)(AlarmTime1[2]>>8);
	Eeptempbuff[13] =(uint8)(AlarmTime1[2]);
	Eeptempbuff[14] =(uint8)(AlarmTime1[3]>>8);
	Eeptempbuff[15] =(uint8)(AlarmTime1[3]);
	Eeptempbuff[16] =(uint8)(AlarmTime1[4]>>8);
	Eeptempbuff[17] =(uint8)(AlarmTime1[4]);
				
	Eeptempbuff[18] =(uint8)(AlarmTime2[0]>>8);
	Eeptempbuff[19] =(uint8)(AlarmTime2[0]);
	Eeptempbuff[20] =(uint8)(AlarmTime2[1]>>8);
	Eeptempbuff[21] =(uint8)(AlarmTime2[1]);
	Eeptempbuff[22] =(uint8)(AlarmTime2[2]>>8);
	Eeptempbuff[23] =(uint8)(AlarmTime2[2]);
	Eeptempbuff[24] =(uint8)(AlarmTime2[3]>>8);
	Eeptempbuff[25] =(uint8)(AlarmTime2[3]);
	Eeptempbuff[26] =(uint8)(AlarmTime2[4]>>8);
	Eeptempbuff[27] =(uint8)(AlarmTime2[4]);
				
	Eeptempbuff[28] =(uint8)(AlarmTime3[0]>>8);
	Eeptempbuff[29] =(uint8)(AlarmTime3[0]);
	Eeptempbuff[30] =(uint8)(AlarmTime3[1]>>8);
	Eeptempbuff[31] =(uint8)(AlarmTime3[1]);
	Eeptempbuff[32] =(uint8)(AlarmTime3[2]>>8);
	Eeptempbuff[33] =(uint8)(AlarmTime3[2]);
	Eeptempbuff[34] =(uint8)(AlarmTime3[3]>>8);
	Eeptempbuff[35] =(uint8)(AlarmTime3[3]);
	Eeptempbuff[36] =(uint8)(AlarmTime3[4]>>8);
	Eeptempbuff[37] =(uint8)(AlarmTime3[4]);
		
		//��ʱ4�ر�ʹ��
//	Eeptempbuff[38] =(uint8)(AlarmTime4[0]>>8);
//	Eeptempbuff[39] =(uint8)(AlarmTime4[0]);
//	Eeptempbuff[40] =(uint8)(AlarmTime4[1]>>8);
//	Eeptempbuff[41] =(uint8)(AlarmTime4[1]);
//	Eeptempbuff[42] =(uint8)(AlarmTime4[2]>>8);
//	Eeptempbuff[43] =(uint8)(AlarmTime4[2]);
//	Eeptempbuff[44] =(uint8)(AlarmTime4[3]>>8);
//	Eeptempbuff[45] =(uint8)(AlarmTime4[3]);
//	Eeptempbuff[46] =(uint8)(AlarmTime4[4]>>8);
//	Eeptempbuff[47] =(uint8)(AlarmTime4[4]);
	Eeptempbuff[38]=ntc_type;//NTC��������

				
	Eeptempbuff[48] =(uint8)(ActiveBal>>8);
	Eeptempbuff[49] =(uint8)(ActiveBal);
	
	Eeptempbuff[50] =(uint8)(ReactiveBal>>8);
	Eeptempbuff[51] =(uint8)(ReactiveBal);
	Eeptempbuff[52] =(uint8)(HarmonicBal>>8);
	Eeptempbuff[53] =(uint8)(HarmonicBal);
	Eeptempbuff[54] =(uint8)(ApparentBal>>8);
	Eeptempbuff[55] =(uint8)(ApparentBal);
	Eeptempbuff[56] =(uint8)(SetupMode>>8);
	Eeptempbuff[57] =(uint8)(SetupMode);

	Eeptempbuff[58] =(uint8)(xiuzhen_dianliujibian>>8);
	Eeptempbuff[59] =(uint8)(xiuzhen_dianliujibian);
	Eeptempbuff[60] =(uint8)(xiuzhen_fuzhi>>8);
	Eeptempbuff[61] =(uint8)(xiuzhen_fuzhi);
	Eeptempbuff[62] =(uint8)(xiuzhen_pf>>8);
	Eeptempbuff[63] =(uint8)(xiuzhen_pf);
	Eeptempbuff[64] =(uint8)(xiuzhen_cos>>8);
	Eeptempbuff[65] =(uint8)(xiuzhen_cos);
	Eeptempbuff[66] =(uint8)(xiuzhen_shuchu>>8);
	Eeptempbuff[67] =(uint8)(xiuzhen_shuchu);
	Eeptempbuff[68] =(uint8)(xiuzhen_wugong>>8);
	Eeptempbuff[69] =(uint8)(xiuzhen_wugong);
	Eeptempbuff[70] =(uint8)(ProjectNo>>24);
	Eeptempbuff[71] =(uint8)(ProjectNo>>16);
	Eeptempbuff[72] =(uint8)(ProjectNo>>8);
	Eeptempbuff[73] =(uint8)(ProjectNo);
	Eeptempbuff[74] =(uint8)(ProductionNo>>8);
	Eeptempbuff[75] =(uint8)(ProductionNo);

	Eeptempbuff[76] =(uint8)(VolOnOffEnable>>8);
	Eeptempbuff[77] =(uint8)(VolOnOffEnable);
	Eeptempbuff[78] =(uint8)(CurOnOffEnable>>8);
	Eeptempbuff[79] =(uint8)(CurOnOffEnable);
			
	Eeptempbuff[80] =(uint8)(MainCTLocation>>8);
	Eeptempbuff[81] =(uint8)(MainCTLocation);
	Eeptempbuff[82] =(uint8)(MainCTDirectionA>>8);
	Eeptempbuff[83] =(uint8)(MainCTDirectionA);
	Eeptempbuff[84] =(uint8)(MainCTDirectionB>>8);
	Eeptempbuff[85] =(uint8)(MainCTDirectionB);
	Eeptempbuff[86] =(uint8)(MainCTDirectionC>>8);
	Eeptempbuff[87] =(uint8)(MainCTDirectionC);
	Eeptempbuff[88] =(uint8)(MainCTPhase>>8);
	Eeptempbuff[89] =(uint8)(MainCTPhase);
	Eeptempbuff[90] =(uint8)(OutCTPhase>>8);
	Eeptempbuff[91] =(uint8)(OutCTPhase);

	Eeptempbuff[92] =(uint8)(Position[0]>>8);
	Eeptempbuff[93] =(uint8)(Position[0]);
	Eeptempbuff[94] =(uint8)(Group[0]>>8);
	Eeptempbuff[95] =(uint8)(Group[0]);
	Eeptempbuff[96] =(uint8)(Capacitance[0]>>8);
	Eeptempbuff[97] =(uint8)(Capacitance[0]);
	Eeptempbuff[98] =(uint8)(Position[1]>>8);
	Eeptempbuff[99] =(uint8)(Position[1]);
	Eeptempbuff[100] =(uint8)(Group[1]>>8);
	Eeptempbuff[101] =(uint8)(Group[1]);
	Eeptempbuff[102] =(uint8)(Capacitance[1]>>8);
	Eeptempbuff[103] =(uint8)(Capacitance[1]);
	Eeptempbuff[104] =(uint8)(Position[2]>>8);
	Eeptempbuff[105] =(uint8)(Position[2]);
	Eeptempbuff[106] =(uint8)(Group[2]>>8);
	Eeptempbuff[107] =(uint8)(Group[2]);
	Eeptempbuff[108] =(uint8)(Capacitance[2]>>8);
	Eeptempbuff[109] =(uint8)(Capacitance[2]);
	Eeptempbuff[110] =(uint8)(Position[3]>>8);
	Eeptempbuff[111] =(uint8)(Position[3]);
	Eeptempbuff[112] =(uint8)(Group[3]>>8);
	Eeptempbuff[113] =(uint8)(Group[3]);
	Eeptempbuff[114] =(uint8)(Capacitance[3]>>8);
	Eeptempbuff[115] =(uint8)(Capacitance[3]);
	Eeptempbuff[116] =(uint8)(Position[4]>>8);
	Eeptempbuff[117] =(uint8)(Position[4]);
	Eeptempbuff[118] =(uint8)(Group[4]>>8);
	Eeptempbuff[119] =(uint8)(Group[4]);
	Eeptempbuff[120] =(uint8)(Capacitance[4]>>8);
	Eeptempbuff[121] =(uint8)(Capacitance[4]);
	Eeptempbuff[122] =(uint8)(Position[5]>>8);
	Eeptempbuff[123] =(uint8)(Position[5]);
	Eeptempbuff[124] =(uint8)(Group[5]>>8);
	Eeptempbuff[125] =(uint8)(Group[5]);
	Eeptempbuff[126] =(uint8)(Capacitance[5]>>8);
	Eeptempbuff[127] =(uint8)(Capacitance[5]);
	Eeptempbuff[128] =(uint8)(Position[6]>>8);
	Eeptempbuff[129] =(uint8)(Position[6]);
	Eeptempbuff[130] =(uint8)(Group[6]>>8);
	Eeptempbuff[131] =(uint8)(Group[6]);
	Eeptempbuff[132] =(uint8)(Capacitance[6]>>8);
	Eeptempbuff[133] =(uint8)(Capacitance[6]);
	Eeptempbuff[134] =(uint8)(Position[7]>>8);
	Eeptempbuff[135] =(uint8)(Position[7]);
	Eeptempbuff[136] =(uint8)(Group[7]>>8);
	Eeptempbuff[137] =(uint8)(Group[7]);
	Eeptempbuff[138] =(uint8)(Capacitance[7]>>8);
	Eeptempbuff[139] =(uint8)(Capacitance[7]);
	Eeptempbuff[140] =(uint8)(Position[8]>>8);
	Eeptempbuff[141] =(uint8)(Position[8]);
	Eeptempbuff[142] =(uint8)(Group[8]>>8);
	Eeptempbuff[143] =(uint8)(Group[8]);
	Eeptempbuff[144] =(uint8)(Capacitance[8]>>8);
	Eeptempbuff[145] =(uint8)(Capacitance[8]);
	Eeptempbuff[146] =(uint8)(OutCTDirectionA>>8);
	Eeptempbuff[147] =(uint8)(OutCTDirectionA);
	Eeptempbuff[148] =(uint8)(OutCTDirectionB>>8);
	Eeptempbuff[149] =(uint8)(OutCTDirectionB);
	Eeptempbuff[150] =(uint8)(OutCTDirectionC>>8);
	Eeptempbuff[151] =(uint8)(OutCTDirectionC);
	Eeptempbuff[152] =(uint8)(Position[9]>>8);
	Eeptempbuff[153] =(uint8)(Position[9]);
	Eeptempbuff[154] =(uint8)(Group[9]>>8);
	Eeptempbuff[155] =(uint8)(Group[9]);
	Eeptempbuff[156] =(uint8)(Capacitance[9]>>8);
	Eeptempbuff[157] =(uint8)(Capacitance[9]);
	Eeptempbuff[158] =(uint8)(Position[10]>>8);
	Eeptempbuff[159] =(uint8)(Position[10]);
	Eeptempbuff[160] =(uint8)(Group[10]>>8);
	Eeptempbuff[161] =(uint8)(Group[10]);
	Eeptempbuff[162] =(uint8)(Capacitance[10]>>8);
	Eeptempbuff[163] =(uint8)(Capacitance[10]);
	Eeptempbuff[164] =(uint8)(Position[11]>>8);
	Eeptempbuff[165] =(uint8)(Position[11]);
	Eeptempbuff[166] =(uint8)(Group[11]>>8);
	Eeptempbuff[167] =(uint8)(Group[11]);
	Eeptempbuff[168] =(uint8)(Capacitance[11]>>8);
	Eeptempbuff[169] =(uint8)(Capacitance[11]);
	Eeptempbuff[170] =(uint8)(Position[12]>>8);
	Eeptempbuff[171] =(uint8)(Position[12]);
	Eeptempbuff[172] =(uint8)(Group[12]>>8);
	Eeptempbuff[173] =(uint8)(Group[12]);
	Eeptempbuff[174] =(uint8)(Capacitance[12]>>8);
	Eeptempbuff[175] =(uint8)(Capacitance[12]);
	Eeptempbuff[176] =(uint8)(Position[13]>>8);
	Eeptempbuff[177] =(uint8)(Position[13]);
	Eeptempbuff[178] =(uint8)(Group[13]>>8);
	Eeptempbuff[179] =(uint8)(Group[13]);
	Eeptempbuff[180] =(uint8)(Capacitance[13]>>8);
	Eeptempbuff[181] =(uint8)(Capacitance[13]);
	Eeptempbuff[182] =(uint8)(Position[14]>>8);
	Eeptempbuff[183] =(uint8)(Position[14]);
	Eeptempbuff[184] =(uint8)(Group[14]>>8);
	Eeptempbuff[185] =(uint8)(Group[14]);
	Eeptempbuff[186] =(uint8)(Capacitance[14]>>8);
	Eeptempbuff[187] =(uint8)(Capacitance[14]);
	
	Eeptempbuff[188] = (uint8)(xiuzhen_jibian>>8);
	Eeptempbuff[189] = (uint8)(xiuzhen_jibian);
	Eeptempbuff[190] = AI_select;									//����ѡ��
	Eeptempbuff[191] = enhance;								//��ǿģʽ
	Eeptempbuff[192] = (uint8)(xiuzhen_rongliang>>8);										//
	Eeptempbuff[193] = (uint8)(xiuzhen_rongliang);	
	Eeptempbuff[194] = xiuzhen_mode;										//
	Eeptempbuff[195] = xiuzhen_bili;
	Eeptempbuff[196] = (uint8)(ini_version_a>>8);									//
	Eeptempbuff[197] = (uint8)(ini_version_a);
	Eeptempbuff[198] = (uint8)(xiuzhen_dianyajibian>>8);										//
	Eeptempbuff[199] = (uint8)(xiuzhen_dianyajibian);
	I2C_write;													//д��I2C
	WriteEeprom(EepARMParamddr,Eeptempbuff,200);   			//����������д����ʽ������
	I2C_read;
}
void CabinProcess(void)
{
	uint8 i;
	
	/***********************************��λ��Դ����**********************************************/
	main_parameter[99] =1;

	for(i=0;i<15;i++)				//��i=16��Ϊ15��
	{
		if(Position[i]==1)
		main_parameter[99]++;
	}
	if(main_parameter[99]>16)main_parameter[99]=16;
	/***********************************��λ2��Դ����**********************************************/
	for(i=0;i<16;i++)
	{
		Passive_parameter[10+i] = 0; 	//��10��11��12--25ȫ����0x0a0a
	}
	if(Position[0]==2)
	{
		if(Group[0]==0)
		{
			Passive_parameter[10] = 0;			
			Passive_parameter[11] = 0;
			Passive_parameter[12] = 0;
		}
		else if(Group[0]==1)
		{
			Passive_parameter[10] = (Capacitance[0])/Group[0];
			Passive_parameter[11] = 0;
			Passive_parameter[12] = 0;
		}
		else if(Group[0]==2)
		{
			Passive_parameter[10] = (Capacitance[0])/Group[0];
			Passive_parameter[11] = (Capacitance[0])/Group[0];
			Passive_parameter[12] = 0;
		}
		else 
		{
			Passive_parameter[10] = (Capacitance[0])/Group[0];
			Passive_parameter[11] = (Capacitance[0])/Group[0];
			Passive_parameter[12] = (Capacitance[0])/Group[0];
		}
	}
	else if(Position[0]==3)
	{
		Passive_parameter[10] = 10000+(Capacitance[0])/3;
		Passive_parameter[11] = 20000+(Capacitance[0])/3;
		Passive_parameter[12] = 30000+(Capacitance[0])/3;
	}
	else if(Position[0]==4)
	{
		Passive_parameter[10] = 40000+Capacitance[0];
		Passive_parameter[11] = 40000+Capacitance[0];
		Passive_parameter[12] = 40000+Capacitance[0];
	}
	else if(Position[0]==5)
	{
		Passive_parameter[10] = 50000+Capacitance[0];
	}
	else
	{
		Passive_parameter[10] = 0;
		Passive_parameter[11] = 0;
		Passive_parameter[12] = 0;
	}
	/***********************************��λ3��Դ����**********************************************/
	if(Position[1]==2)
	{
		if(Group[1]==0)
		{
			Passive_parameter[13] = 0;
			Passive_parameter[14] = 0;
			Passive_parameter[15] = 0;
		}
		else if(Group[1]==1)
		{
			Passive_parameter[13] = (Capacitance[1])/Group[1];
			Passive_parameter[14] = 0;
			Passive_parameter[15] = 0;
		}
		else if(Group[1]==2)
		{
			Passive_parameter[13] = (Capacitance[1])/Group[1];
			Passive_parameter[14] = (Capacitance[1])/Group[1];
			Passive_parameter[15] = 0;
		}
		else
		{
			Passive_parameter[13] = (Capacitance[1])/Group[1];
			Passive_parameter[14] = (Capacitance[1])/Group[1];
			Passive_parameter[15] = (Capacitance[1])/Group[1];
		}
	}
	else if(Position[1]==3)
	{
		Passive_parameter[13] = 10000+(Capacitance[1])/3;
		Passive_parameter[14] = 20000+(Capacitance[1])/3;
		Passive_parameter[15] = 30000+(Capacitance[1])/3;
	}
	else if(Position[1]==4)
	{
		Passive_parameter[13] = 40000+Capacitance[1];
		Passive_parameter[14] = 40000+Capacitance[1];
		Passive_parameter[15] = 40000+Capacitance[1];
	}
	else if(Position[1]==5)
	{
		Passive_parameter[11] = 50000+Capacitance[1];
	}
	else
	{
		Passive_parameter[13] = 0;
		Passive_parameter[14] = 0;
		Passive_parameter[15] = 0;
	}
	/***********************************��λ4��Դ����**********************************************/
	if(Position[2]==2)
	{
		if(Group[2]==0)
		{
			Passive_parameter[16] = 0;
			Passive_parameter[17] = 0;
			Passive_parameter[18] = 0;
		}
		else if(Group[2]==1)
		{
			Passive_parameter[16] = (Capacitance[2])/Group[2];
			Passive_parameter[17] = 0;
			Passive_parameter[18] = 0;
		}
		else if(Group[2]==2)
		{
			Passive_parameter[16] = (Capacitance[2])/Group[2];
			Passive_parameter[17] = (Capacitance[2])/Group[2];
			Passive_parameter[18] = 0;
		}
		else 
		{
			Passive_parameter[16] = (Capacitance[2])/Group[2];
			Passive_parameter[17] = (Capacitance[2])/Group[2];
			Passive_parameter[18] = (Capacitance[2])/Group[2];
		}
	}
	else if(Position[2]==3)
	{
		Passive_parameter[16] = 10000+(Capacitance[2])/3;
		Passive_parameter[17] = 20000+(Capacitance[2])/3;
		Passive_parameter[18] = 30000+(Capacitance[2])/3;
	}
	else if(Position[2]==4)
	{
		Passive_parameter[16] = 40000+Capacitance[2];
		Passive_parameter[17] = 40000+Capacitance[2];
		Passive_parameter[18] = 40000+Capacitance[2];
	}
	else if(Position[2]==5)
	{
		Passive_parameter[12] = 50000+Capacitance[2];
	}
	else
	{
		Passive_parameter[16] = 0;
		Passive_parameter[17] = 0;
		Passive_parameter[18] = 0;
	}
	/***********************************��λ5��Դ����**********************************************/
	if(Position[3]==2)
	{
		if(Group[3]==0)
		{
			Passive_parameter[19] = 0;
			Passive_parameter[20] = 0;
			Passive_parameter[21] = 0;
		}
		else if(Group[3]==1)
		{
			Passive_parameter[19] = (Capacitance[3])/Group[3];
			Passive_parameter[20] = 0;
			Passive_parameter[21] = 0;
		}
		else if(Group[3]==2)
		{
			Passive_parameter[19] = (Capacitance[3])/Group[3];
			Passive_parameter[20] = (Capacitance[3])/Group[3];
			Passive_parameter[21] = 0;
		}
		else 
		{
			Passive_parameter[19] = (Capacitance[3])/Group[3];
			Passive_parameter[20] = (Capacitance[3])/Group[3];
			Passive_parameter[21] = (Capacitance[3])/Group[3];
		}
	}
	else if(Position[3]==3)
	{
		Passive_parameter[19] = 10000+(Capacitance[3])/3;
		Passive_parameter[20] = 20000+(Capacitance[3])/3;
		Passive_parameter[21] = 30000+(Capacitance[3])/3;
	}
	else if(Position[3]==4)
	{
		Passive_parameter[19] = 40000+Capacitance[3];
		Passive_parameter[20] = 40000+Capacitance[3];
		Passive_parameter[21] = 40000+Capacitance[3];
	}
	else if(Position[3]==5)
	{
		Passive_parameter[13] = 50000+Capacitance[3];
	}
	else
	{
		Passive_parameter[19] = 0;
		Passive_parameter[20] = 0;
		Passive_parameter[21] = 0;
	}
	/***********************************��λ6��Դ����**********************************************/
	if(Position[4]==2)
	{
		if(Group[4]==0)
		{
			Passive_parameter[22] = 0;
			Passive_parameter[23] = 0;
			Passive_parameter[24] = 0;
		}
		else if(Group[4]==1)
		{
			Passive_parameter[22] = (Capacitance[4])/Group[4];
			Passive_parameter[23] = 0;
			Passive_parameter[24] = 0;
		}
		else if(Group[4]==2)
		{
			Passive_parameter[22] =(Capacitance[4])/Group[4];
			Passive_parameter[23] =(Capacitance[4])/Group[4];
			Passive_parameter[24] = 0;
		}
		else 
		{
			Passive_parameter[22] =(Capacitance[4])/Group[4];
			Passive_parameter[23] =(Capacitance[4])/Group[4];
			Passive_parameter[24] =(Capacitance[4])/Group[4];
		}
	}
	else if(Position[4]==3)
	{
		Passive_parameter[22] = 10000+(Capacitance[4])/3;
		Passive_parameter[23] = 20000+(Capacitance[4])/3;
		Passive_parameter[24] = 30000+(Capacitance[4])/3;
	}
	else if(Position[4]==4)
	{
		Passive_parameter[22] = 40000+Capacitance[4];
		Passive_parameter[23] = 40000+Capacitance[4];
		Passive_parameter[24] = 40000+Capacitance[4];
	}
	else if(Position[4]==5)
	{
		Passive_parameter[14] = 50000+Capacitance[4];
	}
	else
	{
		Passive_parameter[22] = 0;
		Passive_parameter[23] = 0;
		Passive_parameter[24] = 0;
	}
	/***********************************��λ7��Դ����**********************************************/
	if(Position[5]==2)
	{
		if(Group[5]==0)
		{
			Passive_parameter[25] = 0;
			Passive_parameter[26] = 0;
			Passive_parameter[27] = 0;
		}
		else if(Group[5]==1)
		{
			Passive_parameter[25] = (Capacitance[5])/Group[5];
			Passive_parameter[26] = 0;
			Passive_parameter[27] = 0;
		}
		else if(Group[5]==2)
		{
			Passive_parameter[25] = (Capacitance[5])/Group[5];
			Passive_parameter[26] = (Capacitance[5])/Group[5];
			Passive_parameter[27] = 0;
		}
		else 
		{
			Passive_parameter[25] = (Capacitance[5])/Group[5];
			Passive_parameter[26] = (Capacitance[5])/Group[5];
			Passive_parameter[27] = (Capacitance[5])/Group[5];
		}
	}
	else if(Position[5]==3)
	{
		Passive_parameter[25] = 10000+(Capacitance[5])/3;
		Passive_parameter[26] = 20000+(Capacitance[5])/3;
		Passive_parameter[27] = 30000+(Capacitance[5])/3;
	}
	else if(Position[5]==4)
	{
		Passive_parameter[25] = 40000+Capacitance[5];
		Passive_parameter[26] = 40000+Capacitance[5];
		Passive_parameter[27] = 40000+Capacitance[5];
	}
	else if(Position[5]==5)
	{
		Passive_parameter[15] = 50000+Capacitance[5];
	}
	else
	{
		Passive_parameter[25] = 0;
		Passive_parameter[26] = 0;
		Passive_parameter[27] = 0;
	}
	/***********************************��λ8��Դ����**********************************************/
	if(Position[6]==2)
	{
		if(Group[6]==0)
		{
			Passive_parameter[28] = 0;
			Passive_parameter[29] = 0;
			Passive_parameter[30] = 0;
		}
		else if(Group[6]==1)
		{
			Passive_parameter[28] = (Capacitance[6])/Group[6];
			Passive_parameter[29] = 0;
			Passive_parameter[30] = 0;
		}
		else if(Group[6]==2)
		{
			Passive_parameter[28] = (Capacitance[6])/Group[6];
			Passive_parameter[29] = (Capacitance[6])/Group[6];
			Passive_parameter[30] = 0;
		}
		else 
		{
			Passive_parameter[28] = (Capacitance[6])/Group[6];
			Passive_parameter[29] = (Capacitance[6])/Group[6];
			Passive_parameter[30] = (Capacitance[6])/Group[6];
		}
	}
	else if(Position[6]==3)
	{
		Passive_parameter[28] = 10000+(Capacitance[6])/3;
		Passive_parameter[29] = 20000+(Capacitance[6])/3;
		Passive_parameter[30] = 30000+(Capacitance[6])/3;
	}
	else if(Position[6]==4)
	{
		Passive_parameter[28] = 40000+Capacitance[6];
		Passive_parameter[29] = 40000+Capacitance[6];
		Passive_parameter[30] = 40000+Capacitance[6];
	}
	else if(Position[6]==5)
	{
		Passive_parameter[16] = 50000+Capacitance[6];
	}
	else
	{
		Passive_parameter[28] = 0;
		Passive_parameter[29] = 0;
		Passive_parameter[30] = 0;
	}
	/***********************************��λ9��Դ����**********************************************/
	if(Position[7]==2)
	{
		if(Group[7]==0)
		{
			Passive_parameter[31] = 0;
			Passive_parameter[32] = 0;
			Passive_parameter[33] = 0;
		}
		else if(Group[7]==1)
		{
			Passive_parameter[31] = (Capacitance[7])/Group[7];
			Passive_parameter[32] = 0;
			Passive_parameter[33] = 0;
		}
		else if(Group[7]==2)
		{
			Passive_parameter[31] = (Capacitance[7])/Group[7];
			Passive_parameter[32] = (Capacitance[7])/Group[7];
			Passive_parameter[33] = 0;
		}
		else 
		{
			Passive_parameter[31] = (Capacitance[7])/Group[7];
			Passive_parameter[32] = (Capacitance[7])/Group[7];
			Passive_parameter[33] = (Capacitance[7])/Group[7];
		}
	}
	else if(Position[7]==3)
	{
		Passive_parameter[31] = 10000+(Capacitance[7])/3;
		Passive_parameter[32] = 20000+(Capacitance[7])/3;
		Passive_parameter[33] = 30000+(Capacitance[7])/3;
	}
	else if(Position[7]==4)
	{
		Passive_parameter[31] = 40000+Capacitance[7];
		Passive_parameter[32] = 40000+Capacitance[7];
		Passive_parameter[33] = 40000+Capacitance[7];
	}
	else if(Position[7]==5)
	{
		Passive_parameter[17] = 50000+Capacitance[7];
	}
	else
	{
		Passive_parameter[31] = 0;
		Passive_parameter[32] = 0;
		Passive_parameter[33] = 0;
	}
	/***********************************��λ10��Դ����**********************************************/
	if(Position[8]==2)
	{
		if(Group[8]==0)
		{
			Passive_parameter[34] = 0;
			Passive_parameter[35] = 0;
			Passive_parameter[36] = 0;
		}
		else if(Group[8]==1)
		{
			Passive_parameter[34] = (Capacitance[8])/Group[8];
			Passive_parameter[35] = 0;
			Passive_parameter[36] = 0;
		}
		else if(Group[8]==2)
		{
			Passive_parameter[34] = (Capacitance[8])/Group[8];
			Passive_parameter[35] = (Capacitance[8])/Group[8];
			Passive_parameter[36] = 0;
		}
		else 
		{
			Passive_parameter[34] = (Capacitance[8])/Group[8];
			Passive_parameter[35] = (Capacitance[8])/Group[8];
			Passive_parameter[36] = (Capacitance[8])/Group[8];
		}
	}
	else if(Position[8]==3)
	{
		Passive_parameter[34] = 10000+(Capacitance[8])/3;
		Passive_parameter[35] = 20000+(Capacitance[8])/3;
		Passive_parameter[36] = 30000+(Capacitance[8])/3;
	}
	else if(Position[8]==4)
	{
		Passive_parameter[34] = 40000+Capacitance[8];
		Passive_parameter[35] = 40000+Capacitance[8];
		Passive_parameter[36] = 40000+Capacitance[8];
	}
	else if(Position[8]==5)
	{
		Passive_parameter[18] = 50000+Capacitance[8];
	}
	else
	{
		Passive_parameter[34] = 0;
		Passive_parameter[35] = 0;
		Passive_parameter[36] = 0;
	}
	/***********************************��λ11��Դ����**********************************************/
	if(Position[9]==2)
	{
		if(Group[9]==0)
		{
			Passive_parameter[37] = 0;
			Passive_parameter[38] = 0;
			Passive_parameter[39] = 0;
		}
		else if(Group[9]==1)
		{
			Passive_parameter[37] = (Capacitance[9])/Group[9];
			Passive_parameter[38] = 0;
			Passive_parameter[39] = 0;
		}
		else if(Group[9]==2)
		{
			Passive_parameter[37] = (Capacitance[9])/Group[9];
			Passive_parameter[38] = (Capacitance[9])/Group[9];
			Passive_parameter[39] = 0;
		}
		else 
		{
			Passive_parameter[37] = (Capacitance[9])/Group[9];
			Passive_parameter[38] = (Capacitance[9])/Group[9];
			Passive_parameter[39] = (Capacitance[9])/Group[9];
		}
	}
	else if(Position[9]==3)
	{
		Passive_parameter[37] = 10000+(Capacitance[9])/3;
		Passive_parameter[38] = 20000+(Capacitance[9])/3;
		Passive_parameter[39] = 30000+(Capacitance[9])/3;
	}
	else if(Position[9]==4)
	{
		Passive_parameter[37] = 40000+Capacitance[9];
		Passive_parameter[38] = 40000+Capacitance[9];
		Passive_parameter[39] = 40000+Capacitance[9];
	}
	else if(Position[9]==5)
	{
		Passive_parameter[19] = 50000+Capacitance[9];
	}
	else
	{
		Passive_parameter[37] = 0;
		Passive_parameter[38] = 0;
		Passive_parameter[39] = 0;
	}	
	/***********************************��λ12��Դ����**********************************************/
	/*if(Position[10]==2)
	{
		if(Group[10]==0)
		{
			Passive_parameter[40] = 0;
			Passive_parameter[41] = 0;
			Passive_parameter[42] = 0;
		}
		else if(Group[10]==1)
		{
			Passive_parameter[40] = Capacitance[10]*10;
			Passive_parameter[41] = 0;
			Passive_parameter[42] = 0;
		}
		else if(Group[10]==2)
		{
			Passive_parameter[40] = Capacitance[10]*10;
			Passive_parameter[41] = Capacitance[10]*10;
			Passive_parameter[42] = 0;
		}
		else 
		{
			Passive_parameter[40] = Capacitance[10]*10;
			Passive_parameter[41] = Capacitance[10]*10;
			Passive_parameter[42] = Capacitance[10]*10;
		}
	}
	else if(Position[10]==3)
	{
		Passive_parameter[40] = 10000+Capacitance[10]*10;
		Passive_parameter[41] = 20000+Capacitance[10]*10;
		Passive_parameter[42] = 30000+Capacitance[10]*10;
	}
	else if(Position[10]==4)
	{
		Passive_parameter[40] = 40000+Capacitance[10]*10;
		Passive_parameter[41] = 40000+Capacitance[10]*10;
		Passive_parameter[42] = 40000+Capacitance[10]*10;
	}
	else if(Position[10]==5)
	{
		Passive_parameter[20] = 50000+Capacitance[10]*10;
	}
	else
	{
		Passive_parameter[40] = 0;
		Passive_parameter[41] = 0;
		Passive_parameter[42] = 0;
	}*/
	/***********************************��λ13��Դ����**********************************************/
	/*if(Position[11]==2)
	{
		if(Group[11]==0)
		{
			Passive_parameter[43] = 0;
			Passive_parameter[44] = 0;
			Passive_parameter[45] = 0;
		}
		else if(Group[11]==1)
		{
			Passive_parameter[43] = Capacitance[11]*10;
			Passive_parameter[44] = 0;
			Passive_parameter[45] = 0;
		}
		else if(Group[11]==2)
		{
			Passive_parameter[43] = Capacitance[11]*10;
			Passive_parameter[44] = Capacitance[11]*10;
			Passive_parameter[45] = 0;
		}
		else 
		{
			Passive_parameter[43] = Capacitance[11]*10;
			Passive_parameter[44] = Capacitance[11]*10;
			Passive_parameter[45] = Capacitance[11]*10;
		}
	}
	else if(Position[11]==3)
	{
		Passive_parameter[43] = 10000+Capacitance[11]*10;
		Passive_parameter[44] = 20000+Capacitance[11]*10;
		Passive_parameter[45] = 30000+Capacitance[11]*10;
	}
	else if(Position[11]==4)
	{
		Passive_parameter[43] = 40000+Capacitance[11]*10;
		Passive_parameter[44] = 40000+Capacitance[11]*10;
		Passive_parameter[45] = 40000+Capacitance[11]*10;
	}
	else if(Position[11]==5)
	{
		Passive_parameter[21] = 50000+Capacitance[11]*10;
	}
	else
	{
		Passive_parameter[43] = 0;
		Passive_parameter[44] = 0;
		Passive_parameter[45] = 0;
	}*/	
	/***********************************��λ14��Դ����**********************************************/
	/*if(Position[12]==2)
	{
		if(Group[12]==0)
		{
			Passive_parameter[46] = 0;
			Passive_parameter[47] = 0;
			Passive_parameter[48] = 0;
		}
		else if(Group[12]==1)
		{
			Passive_parameter[46] = Capacitance[12]*10;
			Passive_parameter[47] = 0;
			Passive_parameter[48] = 0;
		}
		else if(Group[12]==2)
		{
			Passive_parameter[46] = Capacitance[12]*10;
			Passive_parameter[47] = Capacitance[12]*10;
			Passive_parameter[48] = 0;
		}
		else 
		{
			Passive_parameter[46] = Capacitance[12]*10;
			Passive_parameter[47] = Capacitance[12]*10;
			Passive_parameter[48] = Capacitance[12]*10;
		}
	}
	else if(Position[12]==3)
	{
		Passive_parameter[46] = 10000+Capacitance[12]*10;
		Passive_parameter[47] = 20000+Capacitance[12]*10;
		Passive_parameter[48] = 30000+Capacitance[12]*10;
	}
	else if(Position[12]==4)
	{
		Passive_parameter[46] = 40000+Capacitance[12]*10;
		Passive_parameter[47] = 40000+Capacitance[12]*10;
		Passive_parameter[48] = 40000+Capacitance[12]*10;
	}
	else if(Position[12]==5)
	{
		Passive_parameter[22] = 50000+Capacitance[12]*10;
	}
	else
	{
		Passive_parameter[46] = 0;
		Passive_parameter[47] = 0;
		Passive_parameter[48] = 0;
	}*/
	/***********************************��λ14��Դ����**********************************************/
	/*if(Position[13]==2)
	{
		if(Group[13]==0)
		{
			Passive_parameter[49] = 0;
			Passive_parameter[50] = 0;
			Passive_parameter[51] = 0;
		}
		else if(Group[13]==1)
		{
			Passive_parameter[49] = Capacitance[13]*10;
			Passive_parameter[50] = 0;
			Passive_parameter[51] = 0;
		}
		else if(Group[13]==2)
		{
			Passive_parameter[49] = Capacitance[13]*10;
			Passive_parameter[50] = Capacitance[13]*10;
			Passive_parameter[51] = 0;
		}
		else 
		{
			Passive_parameter[49] = Capacitance[13]*10;
			Passive_parameter[50] = Capacitance[13]*10;
			Passive_parameter[51] = Capacitance[13]*10;
		}
	}
	else if(Position[13]==3)
	{
		Passive_parameter[49] = 10000+Capacitance[13]*10;
		Passive_parameter[50] = 20000+Capacitance[13]*10;
		Passive_parameter[51] = 30000+Capacitance[13]*10;
	}
	else if(Position[13]==4)
	{
		Passive_parameter[49] = 40000+Capacitance[13]*10;
		Passive_parameter[50] = 40000+Capacitance[13]*10;
		Passive_parameter[51] = 40000+Capacitance[13]*10;
	}
	else if(Position[13]==5)
	{
		Passive_parameter[23] = 50000+Capacitance[13]*10;
	}
	else
	{
		Passive_parameter[49] = 0;
		Passive_parameter[50] = 0;
		Passive_parameter[51] = 0;
	}*/	




}
void ChannelProcess(void)
{	
	uint8 i;

	if(PassiveCom<1) PassiveCom=1;
	if(PassiveCom>16) PassiveCom=16;
	main_parameter[99] =PassiveCom;
	for(i=0;i<32;i++)
	{
		if(PassiveChannel[i]==0)
		{
			Passive_parameter[10+i]=0;
		}
		else if(PassiveChannel[i]==1)
		{
			Passive_parameter[10+i]=PassiveValue[i];		//����
		}
		else if(PassiveChannel[i]==2||PassiveChannel[i]==3||PassiveChannel[i]==4)
		{
			Passive_parameter[10+i]=(PassiveChannel[i]-1)*10000+PassiveValue[i];	//�ֲ�
		}
		else if(PassiveChannel[i]==5||PassiveChannel[i]==6||PassiveChannel[i]==7)
		{
			Passive_parameter[10+i]=40000+PassiveValue[i];	//����
		}
	}
}

//ARM�������Զ�ˢ��
void ARM_proess(void)
{
	uint64 tmp64Val;
	LocalAddr 	 =ARM_param[0];
	UART_BPS  	 =ARM_param[1];
	RemoteEnable =ARM_param[2];
	UART3_Init();
	UART0_Init();


	OnOffStatus =ARM_param[8];
	
	AlarmTime1[0]=ARM_param[9];//��ʱ1 	ʹ��
	AlarmTime1[1]=ARM_param[10];//��ʱ1 	ʱ
	AlarmTime1[2]=ARM_param[11];//��ʱ1	��	
	AlarmTime1[3]=ARM_param[12];//��ʱ1 	ʱ
	AlarmTime1[4]=ARM_param[13];//��ʱ1	��	
	
	AlarmTime2[0]=ARM_param[14];//��ʱ2	ʹ��
	AlarmTime2[1]=ARM_param[15];//��ʱ2 	ʱ
	AlarmTime2[2]=ARM_param[16];//��ʱ2	��
	AlarmTime2[3]=ARM_param[17];//��ʱ2	ʱ					
	AlarmTime2[4]=ARM_param[18];//��ʱ2 	��
	
	AlarmTime3[0]=ARM_param[19];//��ʱ3	ʹ��
	AlarmTime3[1]=ARM_param[20];//��ʱ3	ʱ
	AlarmTime3[2]=ARM_param[21];//��ʱ3 	��
	AlarmTime3[3]=ARM_param[22];//��ʱ3	ʱ
	AlarmTime3[4]=ARM_param[23];//��ʱ3	��	

	ntc_type=ARM_param[24];//��ʱ4 ʹ��
//	AlarmTime4[1]=ARM_param[25];//��ʱ4	ʱ
//	AlarmTime4[2]=ARM_param[26];//��ʱ4	��
//	AlarmTime4[3]=ARM_param[27];//��ʱ4	ʱ
//	AlarmTime4[4]=ARM_param[28];//��ʱ4	��
	
	ActiveBal=ARM_param[29];//�й���ƽ��
	ReactiveBal=ARM_param[30];//�޹���ƽ��
	HarmonicBal=ARM_param[31];//г����ƽ��
	ApparentBal=ARM_param[32];//���ڲ�ƽ��

	if(ApparentBal2!=ApparentBal)
	{
		if(ApparentBal==1){HarmonicBal=1;ReactiveBal=1;}				
		else {HarmonicBal=0;ReactiveBal=0;}
		ApparentBal2=ApparentBal;
	}			
						
	main_parameter[6] =ActiveBal+ReactiveBal*10+HarmonicBal*100;
	SetupMode =ARM_param[33];//��װ���÷�ʽ
	
	enhance=ARM_param[34];//��ǿģʽ

	ProjectNo=(uint32)((ARM_param[40]<<16)|ARM_param[41]);//��Ŀ��
	
	ProductionNo =ARM_param[42];//������
	tmp64Val=1000;
	tmp64Val =(uint64) (ProjectNo)*tmp64Val;
	SerialNumber = 2000000000000000+tmp64Val+ProductionNo;
	word=(uint16)(SerialNumber%9999);									//��ȡ����

	VolOnOffEnable =ARM_param[43];
	CurOnOffEnable =ARM_param[44];
	main_parameter[73] =VolOnOffEnable+CurOnOffEnable*10;	//��λ����
															//0���豸�����õ�ѹ���ػ�����
															//1���豸���õ�ѹ���ػ�����

															//ʮλ��
															//0���豸�����õ������ػ�����
															//1���豸���õ������ػ�����
	MainCTLocation 	=ARM_param[45];
	MainCTDirectionA =ARM_param[46];
	MainCTDirectionB =ARM_param[47];
	MainCTDirectionC =ARM_param[48];
	MainCTPhase 	=ARM_param[49];
	main_parameter[10] =MainCTLocation+MainCTDirectionA*10+MainCTDirectionB*100+MainCTDirectionC*1000+MainCTPhase*10000;	
	//AidCTLocation 	=ARM_param[48];
	//AidCTDirection 	=ARM_param[49];
	OutCTPhase 		=ARM_param[50];
	
	OutCTDirectionA =ARM_param[78];
	OutCTDirectionB =ARM_param[79];
	OutCTDirectionC =ARM_param[80];
	main_parameter[15] =OutCTDirectionA*10+OutCTDirectionB*100+OutCTDirectionC*1000+OutCTPhase*10000;
	
	Position[0] 	=ARM_param[51];										//��λ2
	Group[0] 		=ARM_param[52];										//��2
	Capacitance[0]	=ARM_param[53];										//��ֵ2

	Position[1] 	=ARM_param[54];										//��λ3
	Group[1] 		=ARM_param[55];										//��3
	Capacitance[1]	=ARM_param[56];										//��ֵ3

	Position[2] 	=ARM_param[57];										//��λ4
	Group[2] 		=ARM_param[58];										//��4
	Capacitance[2]	=ARM_param[59];										//��ֵ4

	Position[3] 	=ARM_param[60];										//��λ5
	Group[3] 		=ARM_param[61];										//��5
	Capacitance[3]	=ARM_param[62];										//��ֵ5

	Position[4] 	=ARM_param[63];										//��λ6
	Group[4] 		=ARM_param[64];										//��6
	Capacitance[4]	=ARM_param[65];										//��ֵ6

	Position[5] 	=ARM_param[66];										//��λ7
	Group[5] 		=ARM_param[67];										//��7
	Capacitance[5]	=ARM_param[68];										//��ֵ7

	Position[6] 	=ARM_param[69];										//��λ8
	Group[6] 		=ARM_param[70];										//��8
	Capacitance[6]	=ARM_param[71];										//��ֵ8

	Position[7] 	=ARM_param[72];										//��λ9
	Group[7] 		=ARM_param[73];										//��9
	Capacitance[7]	=ARM_param[74];										//��ֵ9

	Position[8] 	=ARM_param[75];										//��λ10
	Group[8] 		=ARM_param[76];										//��10
	Capacitance[8]	=ARM_param[77];										//��ֵ10
	
	Position[9] 	=ARM_param[81];										//��λ11
	Group[9] 		=ARM_param[82];										//��11
	Capacitance[9]	=ARM_param[83];										//��ֵ11

	Position[10] 	=ARM_param[84];										//��λ12
	Group[10] 		=ARM_param[85];										//��12
	Capacitance[10]	=ARM_param[86];										//��ֵ12

	Position[11] 	=ARM_param[87];										//��λ13
	Group[11] 		=ARM_param[88];										//��13
	Capacitance[11]	=ARM_param[89];										//��ֵ13

	Position[12] 	=ARM_param[90];										//��λ14
	Group[12] 		=ARM_param[91];										//��14
	Capacitance[12]	=ARM_param[92];										//��ֵ14

	Position[13] 	=ARM_param[93];										//��λ15
	Group[13] 		=ARM_param[94];										//��15
	Capacitance[13]	=ARM_param[95];										//��ֵ15

	Position[14] 	=ARM_param[96];										//��λ16
	Group[14] 		=ARM_param[97];										//��16
	Capacitance[14]	=ARM_param[98];										//��ֵ16
	if(SetupMode==0)
	{
		CabinProcess();			//��������
	}
	if(SetupMode==1)
	{	
		ChannelProcess();			//ֱ��
	}
	
	SaveARMParaProcess();
	
}
/******************************************************************************
                           End Of File
******************************************************************************/
