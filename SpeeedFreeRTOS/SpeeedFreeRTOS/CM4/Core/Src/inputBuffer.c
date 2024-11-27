/*
 * inputBuffer.c
 *
 *  Created on: Nov 5, 2024
 *      Author: brand
 */


#include "inputBuffer.h"

int writeIndx = 0;
int readIndx  = 0;
uint32_t buffer[BUFFER_SIZE];


int bufferPut(uint32_t item)
{
	if ((writeIndx + 1) % BUFFER_SIZE == readIndx)
	{
		// buffer is full, avoid overflow
		return 0;
	}
	buffer[writeIndx] = item;
	writeIndx = (writeIndx + 1) % BUFFER_SIZE;
	return 1;
}
int bufferGet(uint32_t * value)
{
	if (readIndx == writeIndx)
	{
		// buffer is empty
		return 0;
	}

	*value = buffer[readIndx];
	readIndx = (readIndx + 1) % BUFFER_SIZE;
	return 1;
}
int isBufferFull()
{
	return (writeIndx + 1) % BUFFER_SIZE == readIndx;
}
int isBufferEmpty()
{
	return readIndx == writeIndx;
}
