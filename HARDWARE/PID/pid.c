#include "pid.h"

//λ��ʽ PID
int incPIDcalc(PIDtypedef *PIDx,int nextpoint)
{
	int error,iincpid;
	int p, i, d;
	
	error=nextpoint-PIDx->setpoint;            //����ƫ������
	
	PIDx->pre_error = PIDx->last_error;
	
	PIDx->last_error=PIDx->current_error;
	
	PIDx->current_error=error;
	
	PIDx->sum_error+=PIDx->current_error;
	
	p=PIDx->P*PIDx->current_error;          //����PIDֵ
	i=PIDx->I*PIDx->sum_error;
	d=PIDx->D*(PIDx->current_error-PIDx->last_error);
	
	iincpid=p+i+d;             //
	
	return  (iincpid);
}
