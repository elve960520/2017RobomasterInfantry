#include "motor.h"


#define P_DATA 1
#define I_DATA 0
#define D_DATA 0

extern int mode;

extern u8 canbuf[8];
extern int global_rel_speed[4];
extern u8 canbuf_yaw[4];									//can总线 云台角度 控制发送缓存区
extern int global_read_range;						//将云台电机反馈值传到电机控制函数

extern int set_Yaw_speed;								//云台运行速度设置
extern int set_pitch_speed;								//云台运行速度设置

extern int Yao_R_UD,Yao_R_LR;    //右侧摇杆处理数据
extern int Yao_L_UD,Yao_L_LR;    //左侧摇杆处理数据

extern int mouse_up_down_speed;
extern int mouse_lift_right_speed;
extern int mouse_lift_key;
extern int mouse_right_key;

typedef struct pid {
	float SetSpeed;            //定义设定值     
	float ActualSpeed;        //定义实际值     
	float err;                //定义偏差值      
	float err_next;            //定义上一个偏差值     
	float err_last;            //定义最上前的偏差值      
	float Kp, Ki, Kd;            //定义比例、积分、微分系数 
}pid;
typedef struct yawPid {
	float SetSpeed;            //定义设定值     
	float ActualSpeed;        //定义实际值     
	float err;                //定义偏差值      
	float err_next;            //定义上一个偏差值     
	float err_last;            //定义最上前的偏差值   
	float err_sum;            //总偏移
	float Kp, Ki, Kd;            //定义比例、积分、微分系数 
} yawPid;

static pid PID1;
static pid *sptr1 = &PID1;
static pid PID2;
static pid *sptr2 = &PID2;
static pid PID3;
static pid *sptr3 = &PID3;
static pid PID4;
static pid *sptr4 = &PID4;
static yawPid PID5;
static yawPid *sptr5 = &PID5;

void PID_Init(void)
{
	sptr1->SetSpeed = 0.0;
	sptr1->ActualSpeed = 0.0;
	sptr1->err = 0.0;
	sptr1->err_last = 0.0;
	sptr1->err_next = 0.0;
	sptr1->Kp = 8;
	sptr1->Ki = 0.08;
	sptr1->Kd = 0.05;
	
	sptr2->SetSpeed = 0.0;
	sptr2->ActualSpeed = 0.0;
	sptr2->err = 0.0;
	sptr2->err_last = 0.0;
	sptr2->err_next = 0.0;
	sptr2->Kp = 8;
	sptr2->Ki = 0.08;
	sptr2->Kd = 0.03;
	
	sptr3->SetSpeed = 0.0;
	sptr3->ActualSpeed = 0.0;
	sptr3->err = 0.0;
	sptr3->err_last = 0.0;
	sptr3->err_next = 0.0;
	sptr3->Kp = 8;
	sptr3->Ki = 0.04;
	sptr3->Kd = 0.03;
	
	sptr4->SetSpeed = 0.0;
	sptr4->ActualSpeed = 0.0;
	sptr4->err = 0.0;
	sptr4->err_last = 0.0;
	sptr4->err_next = 0.0;
	sptr4->Kp = 8;
	sptr4->Ki = 0.02;
	sptr4->Kd = 0.05;
	
	sptr5->SetSpeed = 0.0;
	sptr5->ActualSpeed = 0.0;
	sptr5->err = 0.0;
	sptr5->err_last = 0.0;
	sptr5->err_next = 0.0;
	sptr5->err_sum = 0.0;
	sptr5->Kp = 0.02;
	sptr5->Ki = 0.0;
	sptr5->Kd = 0.01;
}
float PID1_realize(int SetSpeed,int ActualSpeed) {
	static float incrementSpeed;
	sptr1->SetSpeed = SetSpeed;
	sptr1->err = sptr1->SetSpeed - ActualSpeed;
	incrementSpeed += sptr1->Kp*(sptr1->err - sptr1->err_next) + sptr1->Ki*sptr1->err + sptr1->Kd*(sptr1->err - 2*sptr1->err_next + sptr1->err_last);
	sptr1->err_last = sptr1->err_next;
	sptr1->err_next = sptr1->err;
	return (incrementSpeed);
}
float PID2_realize(int SetSpeed,int ActualSpeed) {
	static float incrementSpeed;
	sptr2->SetSpeed = SetSpeed;
	sptr2->err = sptr2->SetSpeed - ActualSpeed;
	incrementSpeed += sptr2->Kp*(sptr2->err - sptr2->err_next) + sptr2->Ki*sptr2->err + sptr2->Kd*(sptr2->err - 2*sptr2->err_next + sptr2->err_last);
	sptr2->err_last = sptr2->err_next;
	sptr2->err_next = sptr2->err;
	return (incrementSpeed);
}
float PID3_realize(int SetSpeed,int ActualSpeed) {
	static float incrementSpeed;
	sptr3->SetSpeed = SetSpeed;
	sptr3->err = sptr3->SetSpeed - ActualSpeed;
	incrementSpeed += sptr3->Kp*(sptr3->err - sptr3->err_next) + sptr3->Ki*sptr3->err + sptr3->Kd*(sptr3->err - 2*sptr3->err_next + sptr3->err_last);
	sptr3->err_last = sptr3->err_next;
	sptr3->err_next = sptr3->err;
	return (incrementSpeed);
}
float PID4_realize(int SetSpeed,int ActualSpeed) {
	static float incrementSpeed;
	sptr4->SetSpeed = SetSpeed;
	sptr4->err = sptr4->SetSpeed - ActualSpeed;
	incrementSpeed += sptr4->Kp*(sptr4->err - sptr4->err_next) + sptr4->Ki*sptr4->err + sptr4->Kd*(sptr4->err - 2*sptr4->err_next + sptr4->err_last);
	sptr4->err_last = sptr4->err_next;
	sptr4->err_next = sptr4->err;
	return (incrementSpeed);
}
float PID5_realize(int SetRange,int ActualRange) {
	static float incrementSpeed;
	float dErr;
	sptr5->SetSpeed = SetRange;
	sptr5->err = sptr5->SetSpeed - ActualRange;
//	incrementSpeed += sptr5->Kp*(sptr5->err - sptr5->err_next) + sptr5->Ki*sptr5->err + sptr5->Kd*(sptr5->err - 2*sptr5->err_next + sptr5->err_last);
	sptr5->err_sum+=sptr5->err;
	dErr = sptr5->err-sptr5->err_next;
	incrementSpeed = sptr5->Kp*sptr5->err+sptr5->Ki*sptr5->err_sum+sptr5->Kd*dErr;
	sptr5->err_last = sptr5->err_next;
	sptr5->err_next = sptr5->err;
	if(incrementSpeed>=1)
	{
		incrementSpeed+=300;
	}
	else if(incrementSpeed<=-1)
	{
		incrementSpeed-=560;
	}
	else if(incrementSpeed>=2000)
	{
		incrementSpeed=2000;
	}
	else if(incrementSpeed<=-2000)
	{
		incrementSpeed=-2000;
	}
	return (incrementSpeed);
}



//void motorInit()
//{
//		set_speed[0] = set_speed[1] = set_speed[2] = set_speed[3] = 0;
//	canbuf[0]=set_speed[0]/256; canbuf[1]=set_speed[0]%256; 
//	canbuf[2]=set_speed[1]/256; canbuf[3]=set_speed[1]%256; 
//	canbuf[4]=set_speed[2]/256; canbuf[5]=set_speed[2]%256; 
//	canbuf[6]=set_speed[3]/256; canbuf[7]=set_speed[3]%256;	
//	CAN1_Send_Msg(canbuf,8);//发送8个字节	
//	set_Yaw_speed = 0;
//	canbuf_yaw[0]=set_Yaw_speed/256; canbuf_yaw[1]=set_Yaw_speed%256;
//	CAN1_Send_Msg_yaw(canbuf_yaw,2);//发送8个字节
//}

void operation()
{
	int v1,v2,v3,v4;
	int set_v1,set_v2,set_v3,set_v4;
	int n = 10;
	int zhuan_speed;
	int haha_set_range,hehe_set_range;
//	int Yao_R_LR;
	int x,y;
//	float iError1,iError2,iError3,iError4;
//	float iIncpid1,iIncpid2,iIncpid3,iIncpid4;
//	float set_D1,set_D2, set_D3, set_D4;
	
//	Yao_R_LR = arr_b[1][7]*1024+arr_b[2][0]*512+arr_b[2][1]*256+arr_b[2][2]*128+arr_b[2][3]*64+arr_b[2][4]*32+arr_b[2][5]*16+arr_b[2][6]*8+arr_b[2][7]*4+arr_b[3][0]*2+arr_b[3][1];
//	Yao_R_LR -= YAO_MIDDLE;
	if(mode == 0)
	{zhuan_speed = -Yao_L_LR;
	 haha_set_range = Yao_L_UD;//设置哈哈的云台位置（期望值）
	hehe_set_range = Yao_L_UD;
//	printf("%d      ,%d\r\n",haha_set_range,global_read_range);
		}
	else
	{zhuan_speed = -mouse_lift_right_speed*15;
	 haha_set_range = mouse_up_down_speed*15;
	hehe_set_range = mouse_up_down_speed*15;}
//	zhuan_speed = -Yao_L_LR;
	
	x =Yao_R_LR;
	y =Yao_R_UD;
	
	if(x==0 && y==0)
	{	
		v1 = v2 = v3 = v4 = 0;
	}
	else if((x>-1 && y>-1))//第一象限,  右前
	{	
		if(y >= x)	//偏前
		{
			v1=v3=y;
			v2=v4=y-x;
		}
		else  			//偏右
		{
			v1=v3=x;			
			v2=v4=y-x;
		}
	}
	else if(x<1 && y>-1)//第二象限， 左前
	{
		if(x<-1172)
			x=-1172;
		if(y >= (-x))//偏前
		{
			v2=v4=y;
			v1=v3=y+x;
		}
		else   //偏左
		{
			v2=v4=(-x);
			v1=v3=y+x;
		}
	}
	else if(x<1 && y<1)//第三象限，左后
	{
		if(y<-1172)
			y=-1172;
		if(x<-1172)
			x=-1172;
		if(y >= x)//偏左
		{
			v1=v3=x;
			v2=v4=y-x;
		}
		else//偏后
		{
			v1=v3=y-x;
			v2=v4=y;
		}	
	}
	else if(x>-1 && y<-1)//第四象限，右后
	{
		if(y<-1172)
			y=-1172;
		if(x > (-y))//偏右
		{
			v1=v3=x+y;
			v2=v4=-x;
		}
		else//偏后
		{
			v1=v3=x+y;
			v2=v4=y;
		}	
	}
	else
	{
		v1 = v2 = v3 = v4 = 0;
	}
	
	//处理发送控制程序
	v1 = v1+zhuan_speed; v2 = v2-zhuan_speed; v3 = v3-zhuan_speed; v4 = v4+zhuan_speed; 
	v1 = -v1;v4 = -v4;
	

//	printf("v1 = %4d ----- v2 = %4d ----- V3 = %4d ----- v4 = %4d \r\n",v1,v2,v3,v4);	
//	v1 = -200;
	set_v1 = PID1_realize(v1,global_rel_speed[0]);
	set_v2 = PID2_realize(v2,global_rel_speed[1]);
	set_v3 = PID3_realize(v3,global_rel_speed[2]);
	set_v4 = PID4_realize(v4,global_rel_speed[3]);
	set_pitch_speed = (int)PID5_realize(7800,global_read_range);
//	set_pitch_speed = 290;
//	if(v1 > 0) v1+=400; 	else if(v1 < 0)  v1-=700;  		else v1 = 0;	
//	if(v2 > 0) v2+=500; 	else if(v2 < 0)  v2-=700;  		else v2 = 0;
//	if(v3 > 0) v3+=700; 	else if(v3 < 0)  v3-=800;  		else v3 = 0;
//	if(v4 > 0) v4+=400; 	else if(v4 < 0)  v4-=800;  		else v4 = 0;
	
//	if(v1 > 0) set_v1+=400; 	else if(v1 < 0)  set_v1-=700;  		else set_v1 = 0;	
//	if(v2 > 0) set_v2+=500; 	else if(v2 < 0)  set_v2-=700;  		else set_v2 = 0;
//	if(v3 > 0) set_v3+=700; 	else if(v3 < 0)  set_v3-=800;  		else set_v3 = 0;
//	if(v4 > 0) set_v4+=400; 	else if(v4 < 0)  set_v4-=800;  		else set_v4 = 0;


//	
	canbuf[0]=(u8)(set_v1*n/2560); 
	canbuf[1]=(u8)(set_v1*n%2560); 
	canbuf[2]=(u8)(set_v2*n/2560); 
	canbuf[3]=(u8)(set_v2*n%2560); 
	canbuf[4]=(u8)(set_v3*n/2560); 
	canbuf[5]=(u8)(set_v3*n%2560); 
	canbuf[6]=(u8)(set_v4*n/2560); 
	canbuf[7]=(u8)(set_v4*n%2560); 
	CAN1_Send_Msg(canbuf,8);//发送8个字节
	canbuf_yaw[0]=set_Yaw_speed/256; //哼哼
	canbuf_yaw[1]=set_Yaw_speed%256;
	canbuf_yaw[2]=set_pitch_speed/256; //哈哈
	canbuf_yaw[3]=set_pitch_speed%256;
	CAN1_Send_Msg_yaw(canbuf_yaw,4);//发送8个字节

		
}


//void operation_yaw(int Read_yaw, int Set_yaw,  u8 arr_b[18][8], u8 chx[18], PIDtypedef *PIDx)
//{
//	int pid_end;
//	u8 canbuf_yaw[2];
//	PIDx->setpoint = Set_yaw;
//	pid_end=incPIDcalc(PIDx,Read_yaw);
//	printf("PID--------------------------------------------------------------------PID %d\r\n",pid_end);
//	printf("PID---------------------------------------------------------------Read_yaw %d\r\n",Read_yaw);
//	if(pid_end > 0) pid_end+=450;
//	else if(pid_end < 0) pid_end-=600;  //抬力
//	else pid_end = 0;
//	if(pid_end > 1000) pid_end = 1000;
//	else if(pid_end < -1000) pid_end = -1000;
//	else pid_end = pid_end;
//	canbuf_yaw[0]=pid_end/256;
//	canbuf_yaw[1]=pid_end%256;
//	CAN1_Send_Msg_yaw(canbuf_yaw,2);//发送8个字节	
//}





