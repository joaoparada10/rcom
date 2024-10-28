// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "utils.h"
#include <unistd.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source
int alarmEnabled = TRUE;
int alarmCount = 0;
volatile sig_atomic_t alarmTriggered = FALSE;
enum state globalState = start;
LinkLayer globalConnectionParameters;
static int frameCounter, numberRetransmitions, numberTimeouts = 0;



////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    globalConnectionParameters = connectionParameters;

    

    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        perror("Error opening serial port. \n");
        return -1;
    }



    if (connectionParameters.role == LlRx){

        if (frameStateMachine(TX_ADDRESS,SET_FRAME) == ack){
            unsigned char flag, address, control, bcc;
            unsigned char buf [5] = {0};
            flag = FLAG;
            address = TX_ADDRESS;
            control = UA_FRAME;
            bcc = address ^ control;

            buf[0] = flag;
            buf[1] = address;
            buf[2] = control;
            buf[3] = bcc;
            buf[4] = flag;
            int bytes = writeBytes((char *)buf, 5);
            printf("%d bytes written\n", bytes);
            return 1; 
            }

    }

    if (connectionParameters.role == LlTx){
        unsigned char flag, address, control, bcc;
        unsigned char buf [5] = {0};
        flag = FLAG;
        address = TX_ADDRESS;
        control = SET_FRAME;
        bcc = address ^ control;

        buf[0] = flag;
        buf[1] = address;
        buf[2] = control;
        buf[3] = bcc;
        buf[4] = flag;

        (void)signal(SIGALRM, alarmHandler);
        alarmEnabled = FALSE;
        alarmCount = 0;

        while (alarmCount < connectionParameters.nRetransmissions)
        {
            
            if (alarmEnabled == FALSE)
            {
                int bytes = writeBytes((char *)buf, 5);
                printf("%d bytes written\n", bytes);
                alarm(globalConnectionParameters.timeout);
                alarmEnabled = TRUE;
                if (frameStateMachine(TX_ADDRESS, UA_FRAME) == ack){
                    alarm(0);
                    return 1;
                }
                if (alarmTriggered) {
                    alarmTriggered = FALSE; // Reset flag
                    alarmEnabled = FALSE;
                    printf("Retransmitting after timeout.\n");
                }
            }

        }
        printf("Didn't get receiver response after %d retransmissions. Exiting.\n", alarmCount);
    }
    return -1;

}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    unsigned char frame[MAX_IFRAME_SIZE];
    int frameIndex = 0;
    frame[frameIndex++] = FLAG;     // FLAG
    frame[frameIndex++] = TX_ADDRESS;   // A
    unsigned char C = (frameCounter % 2 == 0) ? IFRAME_0 : IFRAME_1;
    frame[frameIndex++] = C;    // C
    frame[frameIndex++] = TX_ADDRESS ^ C;   // BCC1
    unsigned char bcc2 = 0;
    unsigned char stuffedFrame[MAX_PAYLOAD_SIZE * 2];
    int stuffedSize = stuffBytes(buf, bufSize, stuffedFrame, &bcc2);
    memcpy(&frame[frameIndex], stuffedFrame, stuffedSize);  // Data
    frameIndex += stuffedSize;
    frame[frameIndex++] = bcc2;     // BCC2
    frame[frameIndex++] = FLAG;    // FLAG


    (void)signal(SIGALRM, alarmHandler);
    alarmEnabled = FALSE;
    alarmCount = 0;
    int bytesWritten = 0;
    while (alarmCount < globalConnectionParameters.nRetransmissions)
        {
            
            if (alarmEnabled == FALSE)
            {
                bytesWritten = writeBytes((char *)frame, frameIndex);
                printf("bcc1 = %x \n", frame[3]);
                printf("%d bytes written\n", bytesWritten);
                alarm(globalConnectionParameters.timeout);
                alarmEnabled = TRUE;
                unsigned char RRFrame = (frameCounter % 2 == 0) ? RR1_FRAME : RR0_FRAME;
                enum flag response = frameStateMachine(TX_ADDRESS, RRFrame);
                if (response == ack){
                    alarm(0);
                    globalState = start;
                    frameCounter++;
                    return stuffedSize;
                }
                else if (response == rej){
                    printf("Received REJ, retransmitting.\n");
                    alarmCount = 0;
                    alarm(0);  // Cancel any existing alarm
                    alarmEnabled = FALSE;
                    globalState = start;
                    numberRetransmitions++;
                    continue;
                }                    
                if (alarmTriggered) {
                    alarmTriggered = FALSE; // Reset flag
                    alarmEnabled = FALSE;
                    printf("Retransmitting after timeout.\n");
                }
            }

    }
    printf("Didn't get receiver response after %d retransmissions. Exiting.\n", alarmCount);
    return -1;
}


////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char bcc1 = 0;
    unsigned char IFrame = (frameCounter % 2 == 0) ? IFRAME_0 : IFRAME_1;
    if (frameStateMachine(TX_ADDRESS,IFrame) == processing_data){
        while (1){    // in practice its while(true)
            if(readByte((char *)&bcc1) == 1)
            {
                printf ("sup");
                if (bcc1 != (TX_ADDRESS ^ IFrame)){
                printf("Invalid BCC1, header error. bcc1 = %x \n", bcc1);
                sendREJ();
                globalState = start;
                continue;
                }
                unsigned char dataFrame[MAX_PAYLOAD_SIZE + 1];
                unsigned char bcc2 = 0;
                int dataFrameSize = readStuffedFrame(dataFrame, &bcc2);
                if (bcc2 != dataFrame[dataFrameSize]){
                    perror ("Invalid BCC2, data error.");
                    sendREJ();
                    globalState = start;
                    continue;
                }
                else{
                    printf("Information Frame %d acknowledged. Sending RR signal.", frameCounter);
                    memcpy(packet, dataFrame, --dataFrameSize);     // need to remove BCC2 from buffer
                    sendRR();
                    globalState = start;
                    return dataFrameSize;
                }
            }
        }
    }
    return -1;
}



////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    int clstat;

    if(globalConnectionParameters.role == LlRx){
        if(frameStateMachine(TX_ADDRESS, DISC_FRAME) == ack){
            unsigned char buf[5];
            buf[0] = FLAG;
            buf[1] = RX_ADDRESS;
            buf[2] = DISC_FRAME;
            buf[3] = buf[1] ^ buf[2];
            buf[4] = FLAG;

            writeBytes((char *)buf, 5);
            globalState = start;
            if(frameStateMachine(TX_ADDRESS, UA_FRAME) == ack){
                clstat = closeSerialPort();
                return clstat;
            }
        }
    }
    else if(globalConnectionParameters.role == LlTx){
        unsigned char buf[5];
        buf[0] = FLAG;
        buf[1] = TX_ADDRESS;
        buf[2] = DISC_FRAME;
        buf[3] = buf[1] ^ buf[2]; 
        buf[4] = FLAG;

        (void)signal(SIGALRM, alarmHandler);
        alarmEnabled = FALSE;
        alarmCount = 0;

        while(alarmCount < globalConnectionParameters.nRetransmissions){
            if(alarmEnabled == FALSE){
                writeBytes((char *)buf,5);
                alarm(globalConnectionParameters.timeout);
                alarmEnabled = TRUE;
                if(frameStateMachine(TX_ADDRESS, UA_FRAME) == ack){
                    alarm(0);
                    buf[0] = FLAG;
                    buf[1] = TX_ADDRESS;
                    buf[2] = UA_FRAME;
                    buf[3] = buf[1] ^ buf[2];
                    buf[4] = FLAG;

                    writeBytes((char *)buf,5);
                    printf("Statistics: \n Frames transmitted = %d \n Number of retransmissions = %d \n Number of timeouts = %d \n", frameCounter,numberRetransmitions,numberTimeouts);
                    clstat = closeSerialPort();
                    return clstat;
                }
                if (alarmTriggered) {
                    alarmTriggered = FALSE;
                    alarmEnabled = FALSE;
                    printf("Retransmitting after timeout.\n");
                }
            }
        }

    }
    return -1;
}

int frameStateMachine(unsigned char address, unsigned char control){
    char byte;
    enum flag response;
    globalState = start;
    while (globalState != stop && globalState != receiving_data && alarmEnabled == TRUE)
    {   
        if (alarmTriggered) {
            printf("Timeout in frameStateMachine\n");
            numberTimeouts++;
            return -1;
        }
        if(readByte(&byte) == 1){
            switch (globalState){
                case start:
                    if (byte == FLAG){

                        printf("Flag = 0x%02X\n", byte);
                        globalState = flag_rcv;
                        printf("globalState = FLAG \n");
                    }
                    break;

                case flag_rcv:
                    
                    if (byte == TX_ADDRESS && address == TX_ADDRESS){
                        printf("A = 0x%02X\n", byte);
                        globalState = a_rcv;
                        printf("globalState = A \n");
                    }
                    else if (byte == RX_ADDRESS && address == RX_ADDRESS){
                        printf("A = 0x%02X\n", byte);
                        globalState = a_rcv;
                        printf("globalState = A \n");
                    }

                    else if (byte == FLAG){
                        printf("Flag = 0x%02X\n", byte);
                        globalState = flag_rcv;
                        printf("globalState = FLAG 2 \n");
                    }
                    else {
                        globalState = start;
                        printf("globalState = START \n");
                    }
                    break;

                case a_rcv:
                    
                    if ((byte == SET_FRAME && control == SET_FRAME) || (byte == UA_FRAME && control == UA_FRAME) || (byte == RR0_FRAME && control == RR0_FRAME) ||
                    (byte == RR1_FRAME && control == RR1_FRAME) || (byte == DISC_FRAME && control == DISC_FRAME)){
                        printf("C = 0x%02X\n", byte);
                        control = byte;
                        globalState = c_rcv;
                        response = ack;
                        printf("globalState = C \n");
                    }
                    else if ((byte == IFRAME_0 && control == IFRAME_0) || (byte == IFRAME_1 && control == IFRAME_1)){
                        globalState = receiving_data;
                        response = processing_data;
                        printf("Receiving Information Frame! C = 0x%02X\n", byte);
                        
                    }
                    else if ((byte == REJ0_FRAME && control == RR1_FRAME) || (byte == REJ1_FRAME && control == RR0_FRAME)){
                        printf("C = 0x%02X\n", byte);
                        control = byte;
                        globalState = c_rcv;
                        response = rejected;
                        printf("globalState = C \n");
                    }
                    else if (byte == FLAG){
                        printf("Flag = 0x%02X\n", byte);
                        
                        globalState = flag_rcv;
                        printf("globalState = FLAG \n");
                    }
                    else {
                        globalState = start;
                        printf("globalState = START \n");
                    }
                    break;
                case c_rcv:
                    
                    if (byte == (address ^ control)){
                        printf("BCC = 0x%02X\n", byte);
                        globalState = bcc1_ok;
                        printf("globalState = BCC \n");
                    }
                    else if (byte == FLAG){
                        printf("Flag = 0x%02X\n", byte);
                        globalState = flag_rcv;
                        printf("globalState = FLAG \n");
                    }
                    else {
                        globalState = start;
                        printf("globalState = START \n");
                    }
                    break;
                case bcc1_ok:
                    if (byte == FLAG){
                        printf("FLAG = 0x%02X\n", byte);
                        globalState = stop;
                        printf("globalState = STOP \n");
                    }
                    else {
                        globalState = start;
                        printf("globalState = START \n");
                    }
                    break;
                default:
                    break;
            }
        }
    }

    return response;
}


void alarmHandler(int signal)
{
    alarmTriggered = TRUE;
    alarmCount++;

    printf("Alarm #%d triggered\n", alarmCount);
}

int stuffBytes(const unsigned char *input, int inputSize, unsigned char *output, unsigned char *bcc2) {

    int outputIndex = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == FLAG) {
            *bcc2 ^= FLAG;
            output[outputIndex++] = ESC;
            output[outputIndex++] = FLAG_ESC;
        } else if (input[i] == ESC) {
            *bcc2 ^= ESC;
            output[outputIndex++] = ESC;
            output[outputIndex++] = ESC_ESC;
        } else {
            *bcc2 ^= input[i];
            output[outputIndex++] = input[i];
        }
    }
    return outputIndex;
}

int readStuffedFrame(unsigned char *dataFrame, unsigned char *bcc2){
    int dataFrameIndex = 0;
    unsigned char byte;
    while (dataFrameIndex <= MAX_PAYLOAD_SIZE){
        
        if(readByte((char *)&byte) == 1)
        {   
            if (byte == FLAG){
                printf("Found flag at the end of payload. Payload size = %d", dataFrameIndex);
                break;
            }
            if (byte == ESC){
                readByte((char *)&byte);
                if (byte == FLAG_ESC){
                    *bcc2 ^=FLAG;
                    dataFrame[dataFrameIndex++] = FLAG;
                }
                else if (byte == ESC_ESC){
                    *bcc2 ^= ESC;
                    dataFrame[dataFrameIndex++] = ESC;
                }
                else {
                    perror("Invalid byte stuffing sequence");
                    return -1;
                }
            }
            else {
                *bcc2 ^= byte;
                dataFrame[dataFrameIndex++] = byte;
            }
        }
    }
    if (dataFrameIndex > MAX_PAYLOAD_SIZE + 1){
        perror("Invalid payload. Size surpasses MAX_PAYLOAD_SIZE.");
        return -1;
    }
    return dataFrameIndex;
}

void sendRR(){
    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = TX_ADDRESS;

    if(frameCounter % 2 == 0)
        buf[2] = RR1_FRAME;
    else
        buf[2] = RR0_FRAME;

    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;

    frameCounter++;
    writeBytes((char *)buf, 5);
    printf("Sent RR %d\n", buf[2]);
}

void sendREJ(){
    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = TX_ADDRESS;

    if(frameCounter % 2 == 0)
        buf[2] = REJ0_FRAME;
    else
        buf[2] = REJ0_FRAME;

    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    

    writeBytes((char *)buf, 5);
    printf("Sent REJ %d\n", buf[2]);
}