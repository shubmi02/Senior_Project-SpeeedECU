/*
 * inputBuffer.h
 *
 *  Created on: Nov 5, 2024
 *      Author: brand
 */

#ifndef INC_INPUTBUFFER_H_
#define INC_INPUTBUFFER_H_

#include <stdint.h>
#define BUFFER_SIZE 10

extern int writeIndx;
extern int readIndx;
extern uint32_t buffer[BUFFER_SIZE];

int bufferPut(uint32_t item);
int bufferGet(uint32_t * value);
int isBufferFull();
int isBufferEmpty();



#endif /* INC_INPUTBUFFER_H_ */
