#include "Parser_Filter.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <pigpio.h>
#include <stdlib.h> 
#include <limits.h> 


/* Decoder states (Packet decoding) */
#define PARSER_STATE_NULL           0x00  /* NULL state */
#define PARSER_STATE_SYNC           0x01  /* Waiting for SYNC byte */
#define PARSER_STATE_SYNC_CHECK     0x02  /* Waiting for second SYNC byte */
#define PARSER_STATE_PAYLOAD_LENGTH 0x03  /* Waiting for payload[] length */
#define PARSER_STATE_PAYLOAD        0x04  /* Waiting for next payload[] byte */
#define PARSER_STATE_CHKSUM         0x05  /* Waiting for chksum byte */

/* Decoder states (2-byte raw decoding) */
#define PARSER_STATE_WAIT_HIGH      0x06  /* Waiting for high byte */
#define PARSER_STATE_WAIT_LOW       0x07  /* High r'cvd.  Expecting low part */

/* Other constants */
#define PARSER_SYNC_BYTE            0xAA  /* Syncronization byte */
#define PARSER_EXCODE_BYTE          0x55  /* EXtended CODE level byte */

/* Declare private function prototypes */
int parsePacketPayload( ThinkGearStreamParser *parser, struct Queue* queue, struct Queue* queue1 );
int parseDataRow( ThinkGearStreamParser *parser, unsigned char *rowPtr );


//int ctr=0;
double total_Ctr=0;
int ctr2=0;
int uCtr=0;
double Filtered_Data[512];
//extern int startIMUFlag;
FILE *Data_Text;
double dataArray[15];
extern int prevVal, currVal;
extern int elapsedS, elapsedMS;
extern int startS, startMS;
extern int currentS, currentMS;
extern double timeMS;
extern int startIMUFlag;
extern int startCVFlag;


/*
 * See header file for interface documentation.
 */

struct Queue* createQueue(unsigned capacity) 
{ 
    struct Queue* queue = (struct Queue*) malloc(sizeof(struct Queue)); 
    queue->capacity = capacity; 
    queue->front = queue->size = 0;  
    queue->rear = capacity - 1;  
    queue->array = (double*) malloc(queue->capacity * sizeof(double)); 
    return queue; 
} 

int isFull(struct Queue* queue) 
{  return (queue->size == queue->capacity);  } 

int isEmpty(struct Queue* queue) 
{  return (queue->size == 0); } 

void enqueue(struct Queue* queue, double item) 
{ 
    if (isFull(queue)){
        //int itemR = queue->array[queue->front]; 
        queue->front = (queue->front + 1)%queue->capacity; 
        queue->size = queue->size - 1;
    }
    queue->rear = (queue->rear + 1)%queue->capacity; 
    queue->array[queue->rear] = item; 
    queue->size = queue->size + 1; 
} 

int dequeue(struct Queue* queue) 
{ 
    if (isEmpty(queue)) 
        return INT_MIN; 
    double item = queue->array[queue->front]; 
    queue->front = (queue->front + 1)%queue->capacity; 
    queue->size = queue->size - 1; 
    return item; 
} 



 
int
THINKGEAR_initParser( ThinkGearStreamParser *parser,
                      unsigned char parserType,
                      void (*handleDataValueFunc)(
                          unsigned char extendedCodeLevel,
                          unsigned char code, unsigned char numBytes,
                          const unsigned char *value, void *customData, struct Queue* queue, struct Queue* queue1),
                      void *customData) {

    if( !parser ) return( -1 );

    /* Initialize the parser's state based on the parser type */
    switch( parserType ) {
        case( PARSER_TYPE_PACKETS ):
            parser->state = PARSER_STATE_SYNC;
            break;
        case( PARSER_TYPE_2BYTERAW ):
            parser->state = PARSER_STATE_WAIT_HIGH;
            break;
        default: return( -2 );
    }

    /* Save parser type */
    parser->type = parserType;

    /* Save user-defined handler function and data pointer */
    parser->handleDataValue = handleDataValueFunc;
    parser->customData = customData;

    return( 0 );
}

/*
 * See header file for interface documentation.
 */
int THINKGEAR_parseByte( ThinkGearStreamParser *parser, unsigned char byte, struct Queue* queue, struct Queue* queue1) {

    int returnValue = 0;

    if( !parser ) return( -1 );

    /* Pick handling according to current state... */
    switch( parser->state ) {

        /* Waiting for SyncByte */
        case( PARSER_STATE_SYNC ):
		// printf("\ncase: PARSER_STATE_SYNC");
            if( byte == PARSER_SYNC_BYTE ) {
                parser->state = PARSER_STATE_SYNC_CHECK;
            }
            break;

        /* Waiting for second SyncByte */
        case( PARSER_STATE_SYNC_CHECK ):
		// printf("\ncase: PARSER_STATE_SYNC_CHECK");
            if( byte == PARSER_SYNC_BYTE ) {
                parser->state = PARSER_STATE_PAYLOAD_LENGTH;
            } else {
                parser->state = PARSER_STATE_SYNC;
            }
            break;

        /* Waiting for Data[] length */
        case( PARSER_STATE_PAYLOAD_LENGTH ):
		// printf("\ncase: PARSER_STATE_PAYLOAD_LENGTH");
            parser->payloadLength = byte;
            if( parser->payloadLength > 170 ) {
                parser->state = PARSER_STATE_SYNC;
                returnValue = -3;
            } else if( parser->payloadLength == 170 ) {
                returnValue = -4;
            } else {
                parser->payloadBytesReceived = 0;
                parser->payloadSum = 0;
                parser->state = PARSER_STATE_PAYLOAD;
            }
            break;

        /* Waiting for Payload[] bytes */
        case( PARSER_STATE_PAYLOAD ):
		// printf("\ncase: PARSER_STATE_PAYLOAD");
            parser->payload[parser->payloadBytesReceived++] = byte;
            parser->payloadSum = (unsigned char)(parser->payloadSum + byte);
            if( parser->payloadBytesReceived >= parser->payloadLength ) {
                parser->state = PARSER_STATE_CHKSUM;
            }
            break;

        /* Waiting for CKSUM byte */
        case( PARSER_STATE_CHKSUM ):
		// printf("\ncase: PARSER_STATE_CHKSUM");
            parser->chksum = byte;
            parser->state = PARSER_STATE_SYNC;
            if( parser->chksum != ((~parser->payloadSum)&0xFF) ) {
                returnValue = -2;
            } else {
                returnValue = 1;
                parsePacketPayload( parser , queue, queue1);
            }
            break;

        /* Waiting for high byte of 2-byte raw value */
        case( PARSER_STATE_WAIT_HIGH ):
		// printf("\ncase: PARSER_STATE_WAIT_HIGH");

            /* Check if current byte is a high byte */
            if( (byte & 0xC0) == 0x80 ) {
                /* High byte recognized, will be saved as parser->lastByte */
                parser->state = PARSER_STATE_WAIT_LOW;
            }
            break;

        /* Waiting for low byte of 2-byte raw value */
        case( PARSER_STATE_WAIT_LOW ):
		// printf("\ncase: PARSER_STATE_WAIT_LOW");		

            /* Check if current byte is a valid low byte */
            if( (byte & 0xC0) == 0x40 ) {

                /* Stuff the high and low part of the raw value into an array */
                parser->payload[0] = parser->lastByte;
                parser->payload[1] = byte;

                /* Notify the handler function of received raw value */
                if( parser->handleDataValue ) {
                    parser->handleDataValue( 0, PARSER_CODE_RAW_SIGNAL, 2,
                                             parser->payload,
                                             parser->customData, queue, queue1 );
                }

                returnValue = 1;
            }

            /* Return to start state waiting for high */
            parser->state = PARSER_STATE_WAIT_HIGH;

            break;

        /* unrecognized state */
        default:
		// printf("\ndefault case");
            parser->state = PARSER_STATE_SYNC;
            returnValue = -5;
            break;
    }

    /* Save current byte */
    parser->lastByte = byte;

    return( returnValue );
}

/**
 * Parses each row of data from the @c packet's Data[] block,
 * updating the fields of @c data as appropriate.
 */
int parsePacketPayload( ThinkGearStreamParser *parser, struct Queue* queue, struct Queue* queue1) {

    unsigned char i = 0;
    unsigned char extendedCodeLevel = 0;
    unsigned char code = 0;
    unsigned char numBytes = 0;

    /* Parse all bytes from the payload[] */
    while( i < parser->payloadLength ) {

        /* Parse possible EXtended CODE bytes */
        while( parser->payload[i] == PARSER_EXCODE_BYTE ) {
            extendedCodeLevel++;
            i++;
        }

        /* Parse CODE */
        code = parser->payload[i++];

        /* Parse value length */
        if( code >= 0x80 ) numBytes = parser->payload[i++];
        else               numBytes = 1;

        /* Call the callback function to handle the DataRow value */
        if( parser->handleDataValue ) {
            parser->handleDataValue( extendedCodeLevel, code, numBytes, parser->payload+i, parser->customData, queue, queue1);
            
            //printf("Extended Code Level: %d",extendedCodeLevel);
            
        }
        i = (unsigned char)(i + numBytes);
    }

    return( 0 );
}

void handleDataValueFunc( unsigned char extendedCodeLevel,
                            unsigned char code,
                            unsigned char numBytes,
                            const unsigned char *value,
                            void *customData, struct Queue* queue, struct Queue* queue1 ) {
    short raw;
    double sum;

    if( extendedCodeLevel == 0 ) {  
        int delta=0;
        int theta=0;
        int lowalpha=0;
        int highalpha=0;
        int lowbeta=0;
        int highbeta=0;
        int lowgamma=0;
        int midgamma=0;
        switch( code ) {
            case( 0x02 ):
                ctr2++;
		        // printf("\n code = 0x02");
                gpioWrite(17,1);
                //printf( "\n[%d] Poor Signal (0-255): %d", ctr2,value[0] & 0xFF);
                dataArray[11] = (double) (value[0] & 0xFF);
                //printf("\nUpdating Poor Signal Quality time: %lf\n", (double) ((double) elapsedS + timeMS));
                if(ctr2<4){ //first 3 seconds check for initialization
                    if(((int)(value[0] & 0xFF))>0){
                        printf("\nERROR");
                        // turn on LED indicator
                        gpioWrite(4,1);
                        //turn on buzzer here using a variable
                    }
                    else{
                        // turn off LED indicator
                        gpioWrite(4,0);
                        //turn off buzzer
                    }
                }
                else{
                    if(((int)(value[0] & 0xFF))!=0){
                        // turn on LED indicator
                        gpioWrite(4,1);
                    }
                    else {
                        // turn off LED indicator
                        gpioWrite(4,0);
                    }
                }
                startIMUFlag=1;
                //startCVFlag = 1;
                break;
            case ( 0x80 ):
               	// printf("\n code = 0x08"); 
                raw = 0;
                raw = (((short)( (value[0]&0xFF) << 8)) | (short) (value[1]&0xFF));
                if(uCtr<1){
                    //Data_Text = fopen("Filtered_Data.dat","a");
                    uCtr++;
                }
                
                enqueue(queue,(double)raw);
                sum=0;
                for(int i=0;i<(queue->size);i++){
                    sum+=(queue->array[i]);   
                }
                sum/=8;
                
                enqueue(queue1,(double)sum);
                sum=0;
                for(int i=0;i<(queue1->size);i++){
                    sum+=(queue1->array[i]);   
                }
                sum/=4;
                
                //dataArray[0] = total_Ctr/512.0;
                dataArray[1] = (double) raw;
                dataArray[2] = sum;
                //printf("Updating EEG value time: %lf\n", (double) ((double) elapsedS + timeMS));
                total_Ctr++;
                startIMUFlag = 1;
                //startCVFlag = 1;
                break;
            case ( 0x83 ):
                //printf( "EXCODE level: %d CODE: 0x%02X vLength: %d\n", extendedCodeLevel, code, numBytes );
                //printf( "Data value(s) Delta, Theta, Low-alpha, High-alpha, Low-beta, High-beta, Low-gamma, Mid-gamma:" );
                //printf("\n");
                int i;
                for( i=0; i<numBytes; i++ ) {
                    if(i<=2) {
                        delta = delta << 8;
                        delta = (delta | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=5) {
                        theta = theta << 8;
                        theta = (theta | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=8) {
                        lowalpha = lowalpha << 8;
                        lowalpha = (lowalpha | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=11) {
                        highalpha = highalpha << 8;
                        highalpha = (highalpha | (value[i] & 0xFF)) & 0xFFFFFF;
                    } 
                    else if(i<=14) {
                        lowbeta = lowbeta << 8;
                        lowbeta = (lowbeta | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=17) {
                        highbeta = highbeta << 8;
                        highbeta = (highbeta | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=20) {
                        lowgamma = lowgamma << 8;
                        lowgamma = (lowgamma | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    else if(i<=23) {
                        midgamma = midgamma << 8;
                        midgamma = (midgamma | (value[i] & 0xFF)) & 0xFFFFFF;
                    }
                    //printf( " %02X", value[i] & 0xFF );  
                }
                // storing normalized EEG Bands values
                dataArray[3] = (double) delta / delta;
                dataArray[4] = (double) theta / delta;
                dataArray[5] = (double) lowalpha / delta;
                dataArray[6] = (double) highalpha / delta;
                dataArray[7] = (double) lowbeta / delta;
                dataArray[8] = (double) highbeta / delta;
                dataArray[9] = (double) lowgamma / delta;
                dataArray[10] = (double) midgamma / delta;
                //printf("Updating EEG bands time: %lf\n", (double) ((double) elapsedS + timeMS));
                //printf( "\n" );
                startIMUFlag = 1;
                //startCVFlag = 1;
                break; 
            default:
                break;
        }
    }
}

