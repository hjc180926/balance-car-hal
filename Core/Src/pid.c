#include "pid.h"
#include "encoder.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu6050.h"
#include "motor.h"
#include <stdlib.h>
//传感器数据
int Encoder_Left, Encoder_Right;
float pitch,roll,yaw;
short gyrox,gyroy,gyroz;
short aacx,aacy,aacz;

//闭环控制中间变量
int Vertical_out,Velocity_out,Turn_out,Target_Speed,Target_turn,MOTO1,MOTO2;
//无循迹=-0.25，有循迹=2.4
float Med_Angle=-3.5;//平衡时角度值偏移量（机械中值）

//参数
/*
直立环计算公式：Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y
直立环的值直接给电机的，电机为-7200~7200，所以直立环范围也是-7200~7200
算直立kp范围：小车角度大概为0~60度，取60，30，10。7200/度数=120，240,720，因为不会时刻满转，所以kp范围约0~1000
算直立kd范围：实测摇晃小车时的角速度范围约0~3000,。7200/2000=3.6，所以kd范围大概是0~10.

速度环计算公式：Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
			   Err=(encoder_L+encoder_R)-Target
速度环，输入速度偏差量，输出一个角度值。
速度偏差Err是用编码器测速计算的，实测满转两编码测速都是90多
算速度kp范围：输出角度值约0~60,（0~60）/90+90=0~1/3.所以kp范围约0~1
根据工程经验ki=kp*1/200

极性：
先测试直立环 KP，给一个值，如果小车向前倾，轮子向前，小车向后，轮子向后，极性正确。
然后。KP赋值完后，继续直立环的 KD ，依旧是小车向前倾，轮子向前转。极性正确。

测试速度环的极性时，先将直立环的 KPKD 调成一个较小的值，
然后给速度环的 KP 与 KI 赋个大的值。
如果小车轮子速度逐渐变大，最后到最大值，那么就是极性正确。
如果小车现象的轮子转的很慢，而且怎么摇晃它，速度都不会怎么改变，那就是极性错误。


*/
float Vertical_Kp=210,Vertical_Kd=2.28; //直立环 数量级（Kp：0~1000、Kd：0~10）
float Velocity_Kp=-0.3,Velocity_Ki=-0.0015; //速度环 数量级（Kp：0~1）极性为负
float Turn_Kp,Turn_Kd;		   //转向环，kp越大，转向越快，kd保证走直线
/*
有超声波，有循迹，中值2
180 2.38
-0.52 -0.0027
10 0.6

无超声波，无循迹，中值-3.5
180 2.38
-0.52 -0.0027
10 0.6




*/



uint8_t stop;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim4;
//遥控
extern float distance;
extern uint8_t Fore,Back,Left,Right;

#define SPEED_Y 16 //俯仰(前后)最大设定速度
#define SPEED_Z 150//偏航(左右)最大设定速度 

//直立环PD控制器
//输入：期望角度、真实角度、角速度
//角速度值传那个，看具体硬件安装
int Vertical(float Med ,float Angle, float gyro_Y)
{
	int temp;
	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
	return temp;
}


//速度环PI控制器
//输入：期望速度、左编码器速度、右编码器速度
int Velocity(int Target,int encoder_L,int encoder_R)
{
	static int Err_LowOut_last,Encoder_S=0;
	static float a=0.7;
	int Err,Err_LowOut,temp;//偏差值

	Err=(encoder_L+encoder_R)-Target;
	//滤波
	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
	Err_LowOut_last=Err_LowOut;
	
	//积分
	Encoder_S += Err_LowOut;

	//积分限幅（-20000~20000）（防溢出，减小pi的影响，防止过大）
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
    if(stop==1)Encoder_S=0,stop=0;
	//5、速度环计算
	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;

	return temp;
}

//转向环PD控制器
//输入：角速度、角度值
int Turn(float gyro_Z,int Target_turn)
{
	int temp;
	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
	return temp;
}


void Control(void)//每10ms一次，用速度递增好过直接赋值大速度
{
	int PWM_out;
	//1、读取编码器和陀螺仪的数据
	Encoder_Left=Read_Speed(&htim2);
	Encoder_Right=-Read_Speed(&htim4);
	mpu_dmp_get_data(&pitch,&roll,&yaw);
	MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);
	MPU_Get_Accelerometer(&aacx,&aacy,&aacz);
	//遥控
	if((Fore==0)&&(Back==0))Target_Speed=0;//未接受到前进后退指令-->速度清零，稳在原地
	if(Fore==1)
	{
//		if(distance<50)  //超声波避障
//			Target_Speed++;
//		else
			Target_Speed--;//      看小车极性++或--
	}
	if(Back==1){Target_Speed++;}//
	Target_Speed=Target_Speed>SPEED_Y?SPEED_Y:(Target_Speed<-SPEED_Y?(-SPEED_Y):Target_Speed);//限幅
	
	/*左右*/
	if((Left==0)&&(Right==0))Target_turn=0;
	if(Left==1)Target_turn-=30;	//左转
	if(Right==1)Target_turn+=30;	//右转
	Target_turn=Target_turn>SPEED_Z?SPEED_Z:(Target_turn<-SPEED_Z?(-SPEED_Z):Target_turn);//限幅( (20*100) * 100   )
	
	/*转向约束*/
	if((Left==0)&&(Right==0))Turn_Kd=0.6;//若无左右转向指令，则开启转向约束
	else if((Left==1)||(Right==1))Turn_Kd=0;//若左右转向指令接收到，则去掉转向约束

	
	//2、将数据传入PID控制器，计算输出结果，即左右电机转速值
	Velocity_out=Velocity(Target_Speed,Encoder_Left,Encoder_Right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,roll,gyrox);
	Turn_out=Turn(gyroz,Target_turn);
	PWM_out=Vertical_out;
	MOTO1=PWM_out-Turn_out;
	MOTO2=PWM_out+Turn_out;
	Limit(&MOTO1,&MOTO2);
//调试使用
//  Load(0,0);
	Load(MOTO1,MOTO2);
}










