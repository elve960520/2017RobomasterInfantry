#include "sys.h"
#include "can.h"
#include "math.h"
#include "usart.h"
#include "delay.h"
//#include "pid.h"

#define sin45 0.7071
#define sin135 0.7071

#define YAO_MAX 3220							//摇杆位置常量
#define YAO_MIN 620
#define YAO_MIDDLE 1024

//角度转弧度
#define j2h(x) (3.1415926*(x)/180.0)     

void operation(void);
//void operation_yaw(int Read_yaw, int Set_yaw,  u8 arr_b[18][8], u8 chx[18], PIDtypedef *PIDx);
void PID_Init(void);







