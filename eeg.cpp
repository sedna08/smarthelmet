#include <stdio.h>
#include "eeg.h"
#include "Parser_Filter.h"
#include <string.h>
#include <errno.h>
#include <csignal>
#include <unistd.h>
#include <time.h>
#include <stdlib.h>
#include <math.h>
#include <pigpio.h>
#include <iostream>

using namespace std;


EEG::EEG() {
    queue = createQueue(8);
    queue1 = createQueue(4);

}

void initEEG(EEG *EEG) {
    
    char port[] = "/dev/ttyAMA0";
	
	THINKGEAR_initParser( &EEG->parser, PARSER_TYPE_PACKETS, handleDataValueFunc, NULL);
    if ((EEG->serial_port = serOpen (port, 57600,0)) < 0)	/* open serial port */
	{
		fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
		printf("\nUnable to open serial device");
		//return 1 ;
	}
}

unsigned char eegRead(EEG *EEG) {
    unsigned char streamByte = serReadByte (EEG->serial_port);
	THINKGEAR_parseByte( &EEG->parser, streamByte, EEG->queue, EEG->queue1);
    return streamByte;
}
