#ifndef _LASER_H_
#define _LASER_H_

#include "sys.h"

void usart2_init(uint32_t bound);

void usart2_sendData(uint8_t *data);
uint16_t usart2_getRxData(uint8_t *buf, uint16_t len);


#endif
