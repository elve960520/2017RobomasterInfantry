#include "main.h"

extern int mid_v1,mid_set_v1;

//����������
int set_speed[4] = {0, 0, 0, 0};  //4���ٶ�����
u8 canbuf[8];										//can���� ������� ���Ʒ��ͻ�����
u8 canbuf_R[5][8];								//����������ݻ�����
int real_speed[4];								//������� �ٶ�ֵ����洢��
int motor_angle[4];								//������� ��е�Ƕ�ֵ����洢��
int global_rel_speed[4];						//���������ֵ����������ƺ���
int global_read_range;						//����̨�������ֵ����������ƺ���

int set_Yaw_speed;								//��̨�����ٶ�����
int set_pitch_speed;								//��̨�����ٶ�����
u8 canbuf_yaw[4];									//can���� ��̨�Ƕ� ���Ʒ��ͻ�����
int Read_angle;										//��̨���� ʵ�ʻ�е�Ƕ�
int Read_Current;
int Set_Current;
int Set_angle =0;									//��̨���� Ŀ���е�Ƕ�

extern u8 chx[18]; 											//˳��洢��18�ֽ�ң��������ԭ��
extern u8 chx_b[18][8];									//2����ԭ��洢
extern int Yao_R_UD ,Yao_R_LR;    //�Ҳ�ҡ������
extern int Yao_L_UD ,Yao_L_LR;		//��

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

extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u8  USART2_RX_BUF[USART_REC_LEN]; //


int i,j;
u8 key;
u8 Send_Count;

int main(void)
{ 
	//��ʼ����
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);    //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ100000
	uart2_init(100000);	//��ʼ�����ڲ�����Ϊ100000
	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//������ʼ��  
 	TIM2_PWM_Init(20000-1,84-1);	//84M/84=1Mhz�ļ���Ƶ��
	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_4tq,CAN_BS1_9tq,3,0);//CAN��ʼ������ģʽ,������1Mbps    
	
	PID_Init();
	RemoteInit();
 	
	set_speed[0] = set_speed[1] = set_speed[2] = set_speed[3] = 0;
	canbuf[0]=set_speed[0]/256; canbuf[1]=set_speed[0]%256; 
	canbuf[2]=set_speed[1]/256; canbuf[3]=set_speed[1]%256; 
	canbuf[4]=set_speed[2]/256; canbuf[5]=set_speed[2]%256; 
	canbuf[6]=set_speed[3]/256; canbuf[7]=set_speed[3]%256;	
	CAN1_Send_Msg(canbuf,8);//����8���ֽ�	
	set_Yaw_speed = 0;
	canbuf_yaw[0]=set_Yaw_speed/256; canbuf_yaw[1]=set_Yaw_speed%256;
	CAN1_Send_Msg_yaw(canbuf_yaw,2);//����8���ֽ�
	
	//�������
	for(i = 0; i < 1000; i++)
	{
		TIM_SetCompare1(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare3(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		TIM_SetCompare4(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		delay_ms(1);
//		CAN1_Send_Msg(canbuf,8);//����8���ֽ�
//		CAN1_Send_Msg_yaw(canbuf_yaw,2);//����8���ֽ�
	}
	
	while(1)
	{	
//		key=KEY_Scan(0);
		//�ӽ��ջ������е����ȡ18������
//		i = 35,j = 0;
////		while(USART_RX_BUF[i-1]!=217) i--;//��λ֡ͷ���ݣ������ҡ����������Ϊ��׼  		 
////		if((USART_RX_BUF[i])>80) //ң�������ݽ�������
//		while(USART2_RX_BUF[i]==0) i--;//��λ֡ͷ���ݣ������ҡ����������Ϊ��׼
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
//			//������������ת���ɶ����Ʒ���鿴
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
//			//����ʮ�����Ҳ�ҡ����������
//			Yao_R_UD = chx_b[3][2]*1024+chx_b[3][3]*512+chx_b[3][4]*256+chx_b[3][5]*128+chx_b[3][6]*64+chx_b[3][7]*32+chx_b[4][0]*16+chx_b[4][1]*8+chx_b[4][2]*4+chx_b[4][3]*2+chx_b[4][4];
//			Yao_R_LR = chx_b[4][5]*1024+chx_b[4][6]*512+chx_b[4][7]*256+chx_b[5][0]*128+chx_b[5][1]*64+chx_b[5][2]*32+chx_b[5][3]*16+chx_b[5][4]*8+chx_b[5][5]*4+chx_b[5][6]*2+chx_b[5][7];				
//			Yao_L_UD = chx_b[0][4]*1024+chx_b[0][5]*512+chx_b[0][6]*256+chx_b[0][7]*128+chx_b[1][0]*64+chx_b[1][1]*32+chx_b[1][2]*16+chx_b[1][3]*8+chx_b[1][4]*4+chx_b[1][5]*2+chx_b[1][6];
//			Yao_L_LR = chx_b[1][7]*1024+chx_b[2][0]*512+chx_b[2][1]*256+chx_b[2][2]*128+chx_b[2][3]*64+chx_b[2][4]*32+chx_b[2][5]*16+chx_b[2][6]*8+chx_b[2][7]*4+chx_b[3][0]*2+chx_b[3][1];
//			
//			operation(Yao_R_LR, Yao_R_UD, chx_b, chx); //����ƽ���˶����ƺ���
//			LED1  = ! LED1;
//		}
//		else
//		{		
//			LED0 = !LED0;
//			delay_ms(100);
//		}
		GetRemoteData();
		operation(); //����ƽ���˶����ƺ���
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
		//���յ������
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		CAN1_Receive_Msg(canbuf_R);
		//��̨��е�Ƕȷ���ֵ����
		
//		set_Yaw_speed = -1500;//�ߺ�
//		set_pitch_speed = 1200;//����
//		canbuf_yaw[0]=set_Yaw_speed/256; 
//		canbuf_yaw[1]=set_Yaw_speed%256;
//		canbuf_yaw[2]=set_pitch_speed/256; 
//		canbuf_yaw[3]=set_pitch_speed%256;
//		CAN1_Send_Msg_yaw(canbuf_yaw,4);//����8���ֽ�
		
		
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
		Send_Count = DataScope_Data_Generate(10); //����10��ͨ���� ��ʽ��֡���ݣ�����֡���ݳ���
		for( i = 0 ; i < Send_Count; i++)  //ѭ������,ֱ���������   
		{
			//printf("%c",DataScope_OutPut_Buffer[i]); 
		}
		delay_ms(10);		

		//���,Ħ��������	
		if(chx_b[0][2] == 0 && chx_b[0][3] == 1) 
		{
			TIM_SetCompare1(TIM2,1250);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else if(chx_b[0][2] == 1 && chx_b[0][3] == 1)
		{
			TIM_SetCompare1(TIM2,1150);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else 
		{		
			TIM_SetCompare1(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}

		//������Ħ����ת��
		if(chx_b[0][0] == 0 && chx_b[0][1] == 1) 
		{
			TIM_SetCompare3(TIM2,1600);	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare4(TIM2,1600);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else if(chx_b[0][0] == 1 && chx_b[0][1] == 1)
		{
			TIM_SetCompare3(TIM2,1400);	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare4(TIM2,1400);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}
		else 
		{		
			TIM_SetCompare3(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
			TIM_SetCompare4(TIM2,1000);	//�޸ıȽ�ֵ���޸�ռ�ձ�
		}

		CAN1_Send_Msg(canbuf,8);//����8���ֽ�
//		CAN1_Send_Msg_yaw(canbuf_yaw,2);//����2���ֽ�
	   
	} 
}
