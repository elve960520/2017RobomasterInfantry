#include "sys.h"
#include "can.h"
#include "math.h"
#include "usart.h"

typedef struct 
{
	int setpoint;			//设定目标
	
	int current_error;		//当前误差
	int pre_error;			//
	int last_error;			//
	
	int sum_error;			//误差累计
	
	float P;				//比例常数
	float I ;				//积分常数
	float D;				//微分常数
}PIDtypedef;

int incPIDcalc(PIDtypedef *PIDx,int nextpoint);


