#include "main.h"

extern int mid_v1,mid_set_v1;

//变量定义区
int set_speed[4] = {0, 0, 0, 0};  //4轮速度设置
u8 canbuf[8];										//can总线 电机电流 控制发送缓存区
u8 canbuf_R[5][8];								//电机反馈数据缓冲区
int real_speed[4];								//电机反馈 速度值计算存储区
int motor_angle[4];								//电机反馈 机械角度值计算存储区
int global_rel_speed[4];						//将电机反馈值传到电机控制函数
int global_read_range;						//将云台电机反馈值传到电机控制函数

int set_Yaw_speed;								//云台运行速度设置
int set_pitch_speed;								//云台运行速度设置
u8 canbuf_yaw[4];									//can总线 云台角度 控制发送缓存区
int Read_angle;										//云台反馈 实际机械角度
int Read_Current;
int Set_Current;
int Set_angle =0;									//云台设置 目标机械角度

extern u8 chx[18]; 											//顺序存储的18字节遥控器基础原码
extern u8 chx_b[18][8];									//2进制原码存储
extern int Yao_R_UD ,Yao_R_LR;    //右侧摇杆数据
extern int Yao_L_UD ,Yao_L_LR;		//左

extern u8 key_OFF;
extern u8 key_CL;
extern u8 key_HL;
extern u8 key_GPS;
extern u8 key_ATTI_middle;
extern u8 key_ATTI_down;

extern int mouse_up_down_speed;
extern int mouse_lift_right_speed;
extern int mouse_lift_key;
extern int mouse_right_key;

extern int keyboard[16];
extern char keyboard_key[16];

extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u8  USART2_RX_BUF[USART_REC_LEN]; //


int i,j;
u8 key;
u8 Send_Count;

int main(void)
{ 
	//初始化区
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为100000
	uart2_init(100000);	//初始化串口波特率为100000
	LED_Init();					//初始化LED 
	KEY_Init(); 				//按键初始化  
 	TIM2_PWM_Init(20000-1,84-1);	//84M/84=1Mhz的计数频率
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,0);//CAN初始化环回模式,波特率1Mbps    
	
	PID_Init();
	RemoteInit();
 	
	set_speed[0] = set_speed[1] = set_speed[2] = set_speed[3] = 0;
	canbuf[0]=set_speed[0]/256; canbuf[1]=set_speed[0]%256; 
	canbuf[2]=set_speed[1]/256; canbuf[3]=set_speed[1]%256; 
	canbuf[4]=set_speed[2]/256; canbuf[5]=set_speed[2]%256; 
	canbuf[6]=set_speed[3]/256; canbuf[7]=set_speed[3]%256;	
	CAN1_Send_Msg(canbuf,8);//发送8个字节	
	set_Yaw_speed = 0;
	canbuf_yaw[0]=set_Yaw_speed/256; canbuf_yaw[1]=set_Yaw_speed%256;
	CAN1_Send_Msg_yaw(canbuf_yaw,2);//发送8个字节
	
	//电调解锁
	for(i = 0; i < 1000; i++)
	{
		TIM_SetCompare1(TIM2,1000);	//修改比较值，修改占空比
		TIM_SetCompare3(TIM2,1000);	//修改比较值，修改占空比
		TIM_SetCompare4(TIM2,1000);	//修改比较值，修改占空比
		delay_ms(1);
//		CAN1_Send_Msg(canbuf,8);//发送8个字节
//		CAN1_Send_Msg_yaw(canbuf_yaw,2);//发送8个字节
	}
	
	while(1)
	{	
//		key=KEY_Scan(0);
		//从接收缓存器中倒序读取18个数据
//		i = 35,j = 0;
////		while(USART_RX_BUF[i-1]!=217) i--;//定位帧头数据，以左侧摇杆向下锁定为标准  		 
////		if((USART_RX_BUF[i])>80) //遥控器数据接收正常
//		while(USART2_RX_BUF[i]==0) i--;//定位帧头数据，以左侧摇杆向下锁定为标准
//		if((USART2_RX_BUF[i])!=0)
//		{		
//			while(1)
//			{
//				chx[j] = USART2_RX_BUF[i];
//				i--;
//				j++;
//				if(j == 18) break;
//			}
//			
//			//将所有数据码转换成二进制方便查看
//			for(i = 0; i < 18; i++)
//			{
//					chx_b[i][7] = chx[i]&0x01?1:0;
//					chx_b[i][6] = chx[i]&0x02?1:0;
//					chx_b[i][5] = chx[i]&0x04?1:0;
//					chx_b[i][4] = chx[i]&0x08?1:0;
//					chx_b[i][3] = chx[i]&0x10?1:0;
//					chx_b[i][2] = chx[i]&0x20?1:0;
//					chx_b[i][1] = chx[i]&0x40?1:0;
//					chx_b[i][0] = chx[i]&0x80?1:0;
//			}
//			
//			//计算十进制右侧摇杆两轴数据
//			Yao_R_UD = chx_b[3][2]*1024+chx_b[3][3]*512+chx_b[3][4]*256+chx_b[3][5]*128+chx_b[3][6]*64+chx_b[3][7]*32+chx_b[4][0]*16+chx_b[4][1]*8+chx_b[4][2]*4+chx_b[4][3]*2+chx_b[4][4];
//			Yao_R_LR = chx_b[4][5]*1024+chx_b[4][6]*512+chx_b[4][7]*256+chx_b[5][0]*128+chx_b[5][1]*64+chx_b[5][2]*32+chx_b[5][3]*16+chx_b[5][4]*8+chx_b[5][5]*4+chx_b[5][6]*2+chx_b[5][7];				
//			Yao_L_UD = chx_b[0][4]*1024+chx_b[0][5]*512+chx_b[0][6]*256+chx_b[0][7]*128+chx_b[1][0]*64+chx_b[1][1]*32+chx_b[1][2]*16+chx_b[1][3]*8+chx_b[1][4]*4+chx_b[1][5]*2+chx_b[1][6];
//			Yao_L_LR = chx_b[1][7]*1024+chx_b[2][0]*512+chx_b[2][1]*256+chx_b[2][2]*128+chx_b[2][3]*64+chx_b[2][4]*32+chx_b[2][5]*16+chx_b[2][6]*8+chx_b[2][7]*4+chx_b[3][0]*2+chx_b[3][1];
//			
//			operation(Yao_R_LR, Yao_R_UD, chx_b, chx); //调用平移运动控制函数
//			LED1  = ! LED1;
//		}
//		else
//		{		
//			LED0 = !LED0;
//			delay_ms(100);
//		}
		GetRemoteData();
		operation(); //调用平移运动控制函数
//		if(key_OFF) printf("key_OFF\r\n");
//		if(key_CL) printf("key_CL\r\n");
//		if(key_HL) printf("key_HL\r\n");
//		if(key_GPS) printf("key_GPS\r\n");
//		if(key_ATTI_middle) printf("key_ATTI_middle\r\n");
//		if(key_ATTI_down) printf("key_ATTI_down\r\n");
//		if(mouse_lift_key) printf("mouse_lift_key\r\n");
//		if(mouse_right_key) printf("mouse_right_key\r\n");
//		delay_ms(100);
//		
//		printf("mouse_up_down_speed = %d\r\n",mouse_up_down_speed);
//		printf("mouse_lift_right_speed = %d\r\n",mouse_lift_right_speed);
		
		for(i = 0; i < 16; i++)
		{
			if(keyboard[i]) printf("%c\r\n",keyboard_key[i]);
		}
//		printf("\r\n\r\n\r\n\r\n");

//		printf("Yao_R_UD = %d\r\n",Yao_R_UD);
//		printf("Yao_R_LR = %d\r\n",Yao_R_LR);
//		printf("Yao_L_UD = %d\r\n",Yao_L_UD);
//		printf("Yao_L_LR = %d\r\n",Yao_L_LR);
		//接收电机反馈
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		//云台机械角度反馈值计算
		
//		set_Yaw_speed = -1500;//哼哼
//		set_pitch_speed = 1200;//哈哈
//		canbuf_yaw[0]=set_Yaw_speed/256; 
//		canbuf_yaw[1]=set_Yaw_speed%256;
//		canbuf_yaw[2]=set_pitch_speed/256; 
//		canbuf_yaw[3]=set_pitch_speed%256;
//		CAN1_Send_Msg_yaw(canbuf_yaw,4);//发送8个字节
		
		
		Read_angle = canbuf_R[4][0]<<8 | canbuf_R[4][1];
		Read_Current = canbuf_R[4][2]<<8 | canbuf_R[4][3];
		Set_Current = canbuf_R[4][4]<<8 | canbuf_R[4][5];
		if(Read_angle) printf("Read_angle:%d  %d  %d\r\n",Read_angle,Read_Current,Set_Current);
		global_read_range = Read_angle;
		for(i = 0; i < 4; i++)
		{
			motor_angle[i] = canbuf_R[i][0]<<8 | canbuf_R[i][1];
			real_speed[i] = canbuf_R[i][2]<<8 | canbuf_R[i][3];
			if(real_speed[i] > 32767)
			{
				real_speed[i]=real_speed[i]-65536;
			}		
//				printf("---------------.............................-----Motor%1d real_speed: %5d",1,real_speed[0]);
				global_rel_speed[i]=real_speed[i]/7;
				//printf("   Motor_angle  %5d",motor_angle[i]);
//				printf("\r\n");
		}//printf("\r\n");printf("\r\n");
		
//		DataScope_Get_Channel_Data(mid_v1/1000.0,2);
//		DataScope_Get_Channel_Data(mid_set_v1/1000.0,3);
		
//		DataScope_Get_Channel_Data(real_speed[0]/7/1000.0,1);
//		DataScope_Get_Channel_Data(real_speed[1]/1000.0,2);
//		DataScope_Get_Channel_Data(real_speed[2]/1000.0,3);
//		DataScope_Get_Channel_Data(real_speed[3]/1000.0,4);
		Send_Count = DataScope_Data_Generate(10); //生成10个通道的 格式化帧数据，返回帧数据长度
		for( i = 0 ; i < Send_Count; i++)  //循环发送,直到发送完毕   
		{
			//printf("%c",DataScope_OutPut_Buffer[i]); 
		}
		delay_ms(10);		

		//射击,摩擦轮启动	
		if(chx_b[0][2] == 0 && chx_b[0][3] == 1) 
		{
			TIM_SetCompare1(TIM2,1250);	//修改比较值，修改占空比
		}
		else if(chx_b[0][2] == 1 && chx_b[0][3] == 1)
		{
			TIM_SetCompare1(TIM2,1150);	//修改比较值，修改占空比
		}
		else 
		{		
			TIM_SetCompare1(TIM2,1000);	//修改比较值，修改占空比
		}

		//左侧控制摩擦轮转速
		if(chx_b[0][0] == 0 && chx_b[0][1] == 1) 
		{
			TIM_SetCompare3(TIM2,1600);	//修改比较值，修改占空比
			TIM_SetCompare4(TIM2,1600);	//修改比较值，修改占空比
		}
		else if(chx_b[0][0] == 1 && chx_b[0][1] == 1)
		{
			TIM_SetCompare3(TIM2,1400);	//修改比较值，修改占空比
			TIM_SetCompare4(TIM2,1400);	//修改比较值，修改占空比
		}
		else 
		{		
			TIM_SetCompare3(TIM2,1000);	//修改比较值，修改占空比
			TIM_SetCompare4(TIM2,1000);	//修改比较值，修改占空比
		}

		CAN1_Send_Msg(canbuf,8);//发送8个字节
//		CAN1_Send_Msg_yaw(canbuf_yaw,2);//发送2个字节
	   
	} 
}
