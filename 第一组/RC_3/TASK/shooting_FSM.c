#include "includeh.h"
extern SHOOTING_STATE SHOOT_STATE ;




void shooting_FSM_task(void *pvParameters)
{
	
switch(SHOOT_STATE)
{
	case SHOOTING_1://Ò»ºÅ
		break;
	case SHOOTING_2:
		break;
	case SHOOTING_3:
		break;
	case SHOOTING_4:
		break;
	case SHOOTING_5:
		break;
	case SHOOTING_6:
		break;
	case STOP_SHOOTING:
		break;
	
  default:
		break;
}
vTaskDelay(5);
}	
