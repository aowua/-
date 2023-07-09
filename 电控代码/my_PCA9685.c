#include "my_PCA9685.h"
#include "my_iic.h"
#include "delay.h"
#include <math.h>


//初始化PCA9685需要的通信协议，设置占空比与角度的转换。
//off = 145+angle*2.4
void PCA9685_Init(float hz,u8 angle)
{
	u32 off = 0;
	IIC_Init();
	PCA9685_Write(PCA_Model,0x00);
	PCA9685_setFreq(hz);
	off = (u32)(145+angle*2.4);
//	PCA9685_setPWM(0,0,off);
//	PCA9685_setPWM(1,0,off);
//	PCA9685_setPWM(2,0,off);
//	PCA9685_setPWM(3,0,off);
//	PCA9685_setPWM(4,0,off);
//	PCA9685_setPWM(5,0,off);
//	PCA9685_setPWM(6,0,off);
//	PCA9685_setPWM(7,0,off);
//	PCA9685_setPWM(8,0,off);
//	PCA9685_setPWM(9,0,off);
//	PCA9685_setPWM(10,0,off);
//	PCA9685_setPWM(11,0,off);
//	PCA9685_setPWM(12,0,off);
//	PCA9685_setPWM(13,0,off);
//	PCA9685_setPWM(14,0,off);
//	PCA9685_setPWM(15,0,off);

	delay_ms(100);
	
}

void PCA9685_Write(u8 addr,u8 data) //IIC写数据
{
	IIC_Start(); //起始位
	
	IIC_Send_Byte(PCA_Addr); //设备地址
	IIC_NAck(); //应答为0
	
	IIC_Send_Byte(addr); //寄存器地址
	IIC_NAck(); //应答为0
	
	IIC_Send_Byte(data); //写入数据
	IIC_NAck(); //应答为0
	
	IIC_Stop(); //停止位
}

u8 PCA9685_Read(u8 addr) //IIC读数据
{
	u8 data;
	
	IIC_Start(); //起始位
	
	IIC_Send_Byte(PCA_Addr); //设备地址
	IIC_NAck(); //应答为0
	
	IIC_Send_Byte(addr); //寄存器地址
	IIC_NAck(); //应答为0
	
	IIC_Stop(); //停止位，相当于应答为0
	
	delay_us(10);

	
	IIC_Start(); //起始位，再次起始是因为半双工

	IIC_Send_Byte(PCA_Addr|0x01); //再次发送设备地址
	IIC_NAck(); //应答为0，这里不一致
	
	data = IIC_Read_Byte(0);
	
	IIC_Stop(); //停止位
	
	return data;
	
}

void PCA9685_setPWM(u8 num,u32 on,u32 off) //设置PCA某个模块的PWM，一次通信，多次写
{
	IIC_Start();
	
	IIC_Send_Byte(PCA_Addr);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(LED0_ON_L+4*num);
	IIC_Wait_Ack();
	
	IIC_Send_Byte(on&0xFF); //计数到on跳为高电平，计数到off跳为低电平
	IIC_Wait_Ack();
	
	IIC_Send_Byte(on>>8); //16位on赋值给两个寄存器
	IIC_Wait_Ack();
	
	IIC_Send_Byte(off&0xFF); //一般on置为0，off等价占空比
	IIC_Wait_Ack();
	
	IIC_Send_Byte(off>>8); 
	IIC_Wait_Ack();
	
	IIC_Stop();
	
}

void PCA9685_setFreq(float freq)
{
	u8 prescale,oldmode,newmode;
	
	double prescaleval;
	
	//freq *= 0.92;
	prescaleval = 25000000;
	prescaleval /= 4096;
	prescaleval /= freq;
	prescaleval -= 1;
	prescale = floor(prescaleval+0.5f);
	oldmode = PCA9685_Read(PCA_Model); //一次通信，读数据
	newmode = (oldmode&0x7F)|0x10; //0x7F:0111,1111; 0x10:0001,0000
	PCA9685_Write(PCA_Model,newmode); //需要设置mode才能改预分频
	PCA9685_Write(PCA_Pre,prescale);
	PCA9685_Write(PCA_Model,oldmode);
	delay_ms(5);
	PCA9685_Write(PCA_Model,oldmode|0xa1);
		
}

void setAngle(u8 num,u8 angle)
{
	u32 off = 0;
	off = (u32)(158+angle*2.2); // (0.5+angle/180*2*(2.5-0.5))/20 = off/4096 好像有问题
	PCA9685_setPWM(num,0,off);
}



