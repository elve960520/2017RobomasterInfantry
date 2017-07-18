#include "remote.h"

extern u8 USART2_RX_BUF[200];

int mode = 0; //0������ң��    1��ʹ��������

u8 chx[18];					//˳��洢��18�ֽ�ң��������ԭ��
u8 chx_b[18][8];			//2����ԭ��洢

u16 Yao_R_UD_original,Yao_R_LR_original;    //�Ҳ�ҡ��ԭʼ���� 
u16 Yao_L_UD_original,Yao_L_LR_original;    //���ҡ��ԭʼ���� 

int Yao_R_UD,Yao_R_LR;    //�Ҳ�ҡ�˴�������
int Yao_L_UD,Yao_L_LR;    //���ҡ�˴�������

u8 key_OFF;
u8 key_CL;
u8 key_HL;
u8 key_GPS;
u8 key_ATTI_middle;
u8 key_ATTI_down;

int mouse_up_down_speed;
int mouse_lift_right_speed;
int mouse_lift_key;
int mouse_right_key;

int keyboard[16];
char keyboard_key[16] = {'Q','W','E','R','A','S','D','F','G','Z','X','C','V','B','H','T'};




void RemoteInit(void)
{
	int i;
	
	for(i = 0; i < 36; i++)
		USART2_RX_BUF[i] = 0;
	for(i = 0; i < 8; i++)
		chx[i] = 0;
	Yao_R_UD = 0;
	Yao_R_LR = 0;
	Yao_L_UD = 0;
	Yao_L_LR = 0;

	Yao_R_UD_original = 0;
	Yao_R_LR_original = 0;
	Yao_L_UD_original = 0;
	Yao_L_LR_original = 0;
}



void GetRemoteData(void)
{
	int i, j;
	if(mode) //���������������
	{
		//ң������⣬���ҡ�����������׽�����������
		while(1)
		{
			i = 35,j = 0;
			while(USART2_RX_BUF[i-1]!=217)			//��λ֡ͷ���ݣ������ҡ����������Ϊ��׼	
			{			
				i--;
				if(i == 0)  break;
			}
				
			if(i && USART2_RX_BUF[i]>80) 			//ң�������ݽ�������
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP�������ͣ� ��ͬBEEP=0;
				break;
			}
			else
			{
				GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP�������ͣ� ��ͬBEEP=0;
				delay_ms(100);                    //��ʱ300ms
				GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
				delay_ms(100); 										//��ʱ300ms
			}
		}
		
			for(j = 0; j < 18; j++,i--)
			{
				chx[j] = USART2_RX_BUF[i];
			}
			
			//������������ת���ɶ����Ʒ���鿴
			for(i = 0; i < 18; i++)
			{
				chx_b[i][7] = chx[i]&0x01?1:0;
				chx_b[i][6] = chx[i]&0x02?1:0;
				chx_b[i][5] = chx[i]&0x04?1:0;
				chx_b[i][4] = chx[i]&0x08?1:0;
				chx_b[i][3] = chx[i]&0x10?1:0;
				chx_b[i][2] = chx[i]&0x20?1:0;
				chx_b[i][1] = chx[i]&0x40?1:0;
				chx_b[i][0] = chx[i]&0x80?1:0;
			}
			
			if(chx_b[0][0]==1 && chx_b[0][1]==1) { key_OFF = 0; key_CL = 1; key_HL = 0; }
			if(chx_b[0][0]==0 && chx_b[0][1]==1) { key_OFF = 1; key_CL = 0; key_HL = 0; }
			if(chx_b[0][0]==1 && chx_b[0][1]==0) { key_OFF = 0; key_CL = 0; key_HL = 1; }
			
			if(chx_b[0][2]==1 && chx_b[0][3]==1) { key_GPS = 0; key_ATTI_middle = 1; key_ATTI_down = 0; }
			if(chx_b[0][2]==0 && chx_b[0][3]==1) { key_GPS = 1; key_ATTI_middle = 0; key_ATTI_down = 0; }
			if(chx_b[0][2]==1 && chx_b[0][3]==0) { key_GPS = 0; key_ATTI_middle = 0; key_ATTI_down = 1; }

			//����ʮ�����Ҳ�ҡ����������
			Yao_R_UD_original = ((int)(chx[3]<<5) | (int)(chx[4]>>3))& 0x07FF;
			Yao_R_LR_original = ((int)(chx[4]<<8) | (int)(chx[5]))& 0x7ff;				
			Yao_L_UD_original = ((int)(chx[0]<<7) | (int)(chx[1]>>1)) & 0x7ff;
			Yao_L_LR_original = ((int)(chx[1]<<10) | (int)(chx[2]<<2) | (int)(chx[3]>>6)) & 0x7ff;	
			
			//���������
			Yao_R_UD = Yao_R_UD_original - MIDDLE;
			Yao_R_LR = Yao_R_LR_original - MIDDLE;
			Yao_L_UD = Yao_L_UD_original - MIDDLE;
			Yao_L_LR = Yao_L_LR_original - MIDDLE;
			
			if(chx[10]) mouse_right_key = 1; else mouse_right_key = 0;
			if(chx[11]) mouse_lift_key = 1; else mouse_lift_key = 0;
			
			mouse_up_down_speed = chx[14]<<8 | chx[15];
			mouse_lift_right_speed = chx[16]<<8 | chx[17];
			if(mouse_lift_right_speed > 30000) mouse_lift_right_speed -= 65536;
			if(mouse_up_down_speed > 30000) mouse_up_down_speed -= 65536;
			
			//�����ж�
			if(chx_b[9][1]) keyboard[0] = 1; else keyboard[0] = 0;			
			if(chx_b[9][7]) keyboard[1] = 1; else keyboard[1] = 0;
			if(chx_b[9][0]) keyboard[2] = 1; else keyboard[2] = 0;
			if(chx_b[8][7]) keyboard[3] = 1; else keyboard[3] = 0;
			if(chx_b[9][5]) keyboard[4] = 1; else keyboard[4] = 0;
			if(chx_b[9][6]) keyboard[5] = 1; else keyboard[5] = 0;
			if(chx_b[9][4]) keyboard[6] = 1; else keyboard[6] = 0;
			if(chx_b[8][6]) keyboard[7] = 1; else keyboard[7] = 0;
			if(chx_b[8][5]) keyboard[8] = 1; else keyboard[8] = 0;
			if(chx_b[8][4]) keyboard[9] = 1; else keyboard[9] = 0;
			if(chx_b[8][3]) keyboard[10] = 1; else keyboard[10] = 0;
			if(chx_b[8][2]) keyboard[11] = 1; else keyboard[11] = 0;
			if(chx_b[8][1]) keyboard[12] = 1; else keyboard[12] = 0;
			if(chx_b[8][0]) keyboard[13] = 1; else keyboard[13] = 0;
			if(chx_b[9][3]) keyboard[14] = 1; else keyboard[14] = 0;
			if(chx_b[9][2]) keyboard[15] = 1; else keyboard[15] = 0;
//		else
//		{
//				GPIO_ResetBits(GPIOF,GPIO_Pin_8); //BEEP�������ͣ� ��ͬBEEP=0;
//				delay_ms(100);                    //��ʱ300ms
//				GPIO_SetBits(GPIOF,GPIO_Pin_8);   //BEEP�������ߣ� ��ͬBEEP=1;
//				delay_ms(100); 			
//		}
	}
	else
	{
		//�ӽ��ջ������е����ȡ18������
		i = 35;
		while(USART2_RX_BUF[i]==0) 
		{
			i--;
			if(i == 0) break;
		}
		
		if(i)
		{
			for(j = 0; j < 6; j++,i--)
			{
				chx[j] = USART2_RX_BUF[i];
			}
			
			//������������ת���ɶ����Ʒ���鿴
			for(i = 0; i < 6; i++)
			{
				chx_b[i][7] = chx[i]&0x01?1:0;
				chx_b[i][6] = chx[i]&0x02?1:0;
				chx_b[i][5] = chx[i]&0x04?1:0;
				chx_b[i][4] = chx[i]&0x08?1:0;
				chx_b[i][3] = chx[i]&0x10?1:0;
				chx_b[i][2] = chx[i]&0x20?1:0;
				chx_b[i][1] = chx[i]&0x40?1:0;
				chx_b[i][0] = chx[i]&0x80?1:0;
			}
			if(chx_b[0][0]==1 && chx_b[0][1]==1) { key_OFF = 0; key_CL = 1; key_HL = 0; }
			if(chx_b[0][0]==0 && chx_b[0][1]==1) { key_OFF = 1; key_CL = 0; key_HL = 0; }
			if(chx_b[0][0]==1 && chx_b[0][1]==0) { key_OFF = 0; key_CL = 0; key_HL = 1; }
			
			if(chx_b[0][2]==1 && chx_b[0][3]==1) { key_GPS = 0; key_ATTI_middle = 1; key_ATTI_down = 0; }
			if(chx_b[0][2]==0 && chx_b[0][3]==1) { key_GPS = 1; key_ATTI_middle = 0; key_ATTI_down = 0; }
			if(chx_b[0][2]==1 && chx_b[0][3]==0) { key_GPS = 0; key_ATTI_middle = 0; key_ATTI_down = 1; }

			//����ʮ�����Ҳ�ҡ����������
			Yao_R_UD_original = ((int)(chx[3]<<5) | (int)(chx[4]>>3))& 0x07FF;
			Yao_R_LR_original = ((int)(chx[4]<<8) | (int)(chx[5]))& 0x7ff;				
			Yao_L_UD_original = ((int)(chx[0]<<7) | (int)(chx[1]>>1)) & 0x7ff;
			Yao_L_LR_original = ((int)(chx[1]<<10) | (int)(chx[2]<<2) | (int)(chx[3]>>6)) & 0x7ff;	
			
			//���������
			Yao_R_UD = Yao_R_UD_original - MIDDLE;
			Yao_R_LR = Yao_R_LR_original - MIDDLE;
			Yao_L_UD = Yao_L_UD_original - MIDDLE;
			Yao_L_LR = Yao_L_LR_original - MIDDLE;
		}
	}
}
