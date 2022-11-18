/*============================================================
LPC1768 学习板 IAP功能演示
作者: Mingtree	ycxms88@163.com   http://www.mcu123.com
==============================================================
*/


#include "iap.h"
#include "rtc.h"

#define IAP_LOCATION 0x7FFFFFF1 // IAP 程序入口 （1）
typedef void (*IAP) (unsigned int [ ] , unsigned int [ ]); // 定义函数类型指针




/*定义CCLK 值大小，单位为KHz*/
#define IAP_FCCLK 96000
#define IAP_ENTER_ADR 0x1FFF1FF1 // IAP 入口地址定义
uint32 paramin[8]; // IAP 入口参数缓冲区
uint32 paramout[8]; // IAP 出口参数缓冲区
 uint8  buf_Read[512];


#define IAP_SELECTOR  50
#define IAP_RAMTOFLASH	51
#define IAP_ERASESECTOR	52
#define IAP_BLANKCHECKSECTOR 53
#define READPARTID 54
#define READBOOTCODEVERSION 55
#define IAP_COMPARE 56
			                                                  
extern __align(4) uint8 Parameter[ParameterNumMax];
extern __align(4) uint8 ParameterBak[ParameterInitNumMax];
/******************************************************************************************
** 函数名称： SelSector
** 函数功能： IAP 操作缓冲区选择，代码为50。
** 入口参数： sec1 起始扇区
** sec2 终止扇区
** 出口参数： IAP 操作状态码
** IAP 返回值（paramout 缓冲区）
******************************************************************************************/
uint32 SelSector(uint8 sec1,uint8 sec2)
{
paramin[0] = IAP_SELECTOR; // 设置命令字
paramin[1] = sec1; // 设置参数
paramin[2] = sec2;
(*(void(*)())IAP_ENTER_ADR)(paramin,paramout); // 调用IAP 服务程序
return(paramout[0]); // 返回状态码
}

/******************************************************************************************
** 函数名称： EraseSector
** 函数功能： 擦除扇区，命令代码52。
** 入口参数： sec1 起始扇区
** sec2 终止扇区
** 出口参数： IAP 操作状态码
** IAP 返回值（paramout 缓冲区）
******************************************************************************************/
uint32 EraseSector(uint32 sec1,uint32 sec2)
{
paramin[0] = IAP_ERASESECTOR; // 设置命令字
paramin[1] = sec1; // 设置参数
paramin[2] = sec2;
paramin[3] = IAP_FCCLK;
(*(void(*)())IAP_ENTER_ADR)(paramin,paramout); // 调用IAP 服务程序
return(paramout[0]); // 返回状态码
}

/******************************************************************************************
** 函数名称： RamToFlash
** 函数功能： 复制RAM 的数据到FLASH，命令代码51。
** 入口参数： dst 目标地址，即FLASH 起始地址，以256 字节为分界
** src 源地址，即RAM 地址，地址必须字对其
** no 复制字节个数，为256/512/1024/4096
** 出口参数： IAP 操作状态码
** IAP 返回值（paramout 缓冲区）
******************************************************************************************/
uint32 RamToFlash(uint32 dst, uint32 src, uint32 no)
{
paramin[0] = IAP_RAMTOFLASH; //设置命令字
paramin[1] = dst; //设置参数
paramin[2] = src;
paramin[3] = no;
paramin[4] = IAP_FCCLK;
(*(void(*)())IAP_ENTER_ADR)(paramin,paramout); //调用IAP 服务程序
return(paramout[0]); //返回状态码
}

/******************************************************************************************
** 函数名称： Compare
** 函数功能： 校验数据，命令代码56。
** 入口参数： dst 目标地址，即RAM/FLASH 起始地址，地址必须字对齐
** src 源地址，即RAM/RAM 地址，地址必须字对齐
** no 比较字节个数，必须能被4 整除
** 出口参数： IAP 操作状态码
** IAP 返回值（paramout 缓冲区）
******************************************************************************************/
uint32 Compare(uint32 dst, uint32 src, uint32 no)
{
paramin[0] = IAP_COMPARE; // 设置命令字
paramin[1] = dst; // 设置参数
paramin[2] = src;
paramin[3] = no;
(*(void(*)())IAP_ENTER_ADR)(paramin,paramout); // 调用IAP 服务程序
return(paramout[0]); // 返回状态码
}

#define DestAddr 0x00078000 // 扇区29的起始地址
#define DestAddr1 0x00070000 // 扇区28的起始地址  TT
#define DestAddr2 0x00068000 //扇区27的起始地址

/*void read_sec(void)
{
 uint32 i;
 volatile int8 *wr_ptr; 
 wr_ptr = (volatile int8 *)DestAddr;
 for ( i= 0; i < ParameterInitNumMax; i++ ){main_parameterbak[i]=*wr_ptr++ ;}
}*/

