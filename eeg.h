#ifndef EEG_H_
#define EEG_H_

#include "Parser_Filter.h"
#include <time.h>
#include <queue>
#include <math.h>
#include <time.h>
#include <iostream>

// Class EEG
class EEG {
	public:
		EEG();
		int serial_port;
		struct Queue* queue; 
		struct Queue* queue1;
		ThinkGearStreamParser parser;	
};

float getElapsedTime(struct timespec *prev); 
void initEEG(EEG *EEG);
unsigned char eegRead(EEG *EEG);

#endif