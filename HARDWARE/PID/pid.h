#include "sys.h"
#include "can.h"
#include "math.h"
#include "usart.h"

typedef struct 
{
	int setpoint;			//�趨Ŀ��
	
	int current_error;		//��ǰ���
	int pre_error;			//
	int last_error;			//
	
	int sum_error;			//����ۼ�
	
	float P;				//��������
	float I ;				//���ֳ���
	float D;				//΢�ֳ���
}PIDtypedef;

int incPIDcalc(PIDtypedef *PIDx,int nextpoint);


