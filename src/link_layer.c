// Link layer protocol implementation

#include "link_layer.h"
#include "serial_port.h"
#include "utils.h"
#include <unistd.h>
#include <signal.h>

// MISC
#define _POSIX_SOURCE 1 // POSIX compliant source

volatile int STOP = FALSE;
int alarmEnabled = FALSE;
int alarmCount = 0;
enum state state = start;
LinkLayer globalConnectionParameters;
static int frameCounter = 0;


int information_frame_check(unsigned char address, unsigned char control)
int supervision_frame_check(unsigned char address, unsigned char control){
    unsigned char temp [1] = {0};
    while (state != stop || state == rejected)
    {
        readByte(temp);
        switch (state){
            case start:
                if (temp[0] == FLAG){

                    printf("Flag = 0x%02X\n", temp[0]);
                    state = flag_rcv;
                    printf("state = FLAG \n");
                }
                
                break;

            case flag_rcv:
                
                if (temp[0] == TX_ADDRESS && address == TX_ADDRESS){
                    printf("A = 0x%02X\n", temp[0]);
                    state = a_rcv;
                    printf("state = A \n");
                }
                else if (temp[0] == RX_ADDRESS && address == RX_ADDRESS){
                    printf("A = 0x%02X\n", temp[0]);
                    state = a_rcv;
                    printf("state = A \n");
                }

                else if (temp[0] == FLAG){
                    printf("Flag = 0x%02X\n", temp[0]);
                    state = flag_rcv;
                    printf("state = FLAG \n");
                }
                else {
                    state = start;
                    printf("state = START \n");
                }
                break;

            case a_rcv:
                
                if ((temp[0] == SET_FRAME && control == SET_FRAME) || (temp[0] == UA_FRAME && control == UA_FRAME) || (temp[0] == RR0_FRAME && control == RR0_FRAME) ||
                (temp[0] == RR1_FRAME && control == RR1_FRAME) || (temp[0] == DISC_FRAME && control == DISC_FRAME)){
                    printf("C = 0x%02X\n", temp[0]);
                    control = temp[0];
                    state = c_rcv;
                    printf("state = C \n");
                }
                else if ((temp[0] == REJ0_FRAME && control == RR1_FRAME) || (temp[0] == REJ1_FRAME && control == RR0_FRAME)){
                    printf("C = 0x%02X\n", temp[0]);
                    control = temp[0];
                    state = rejected;
                    printf("state = C \n");
                    break;
                }
                else if (temp[0] == FLAG){
                    printf("Flag = 0x%02X\n", temp[0]);
                    
                    state = flag_rcv;
                    printf("state = FLAG \n");
                }
                else {
                    state = start;
                    printf("state = START \n");
                }
                break;
            case c_rcv:
                
                if (temp[0] == address ^ control){
                    printf("BCC = 0x%02X\n", temp[0]);
                    state = bcc1_ok;
                    printf("state = BCC \n");
                }
                else if (temp[0] == FLAG){
                    printf("Flag = 0x%02X\n", temp[0]);
                    state = flag_rcv;
                    printf("state = FLAG \n");
                }
                else {
                    state = start;
                    printf("state = START \n");
                }
                break;
            case bcc1_ok:
                if (temp[0] == FLAG){
                    printf("FLAG = 0x%02X\n", temp[0]);
                    state = stop;
                    printf("state = STOP \n");
                }
                else {
                    state = start;
                    printf("state = START \n");
                }
                break;

        }
    }

    if (state == rejected){
        state = start;
        return REJECTED;
    }
    else if (state == stop){
        state = start;
        return ACK;
    }
    else {
        state = start;
        return ERROR;
        }


}
////////////////////////////////////////////////
// LLOPEN
////////////////////////////////////////////////
int llopen(LinkLayer connectionParameters)
{
    globalConnectionParameters = connectionParameters;

    unsigned char flag, address, control, bcc;

    if (openSerialPort(connectionParameters.serialPort,
                       connectionParameters.baudRate) < 0)
    {
        return -1;
    }



    if (connectionParameters.role == LlRx){

        if (supervision_frame_check(TX_ADDRESS,SET_FRAME))
            return 0;   // ALL GOOD! START DATA TRANSFER

    }

    if (connectionParameters.role == LlTx){

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
        int bytesRead = 0;

        while (alarmCount < connectionParameters.nRetransmissions)
        {
            
            if (alarmEnabled == FALSE)
            {
                int bytes = writeBytes(buf, 5);
                printf("%d bytes written\n", bytes);


                if (supervision_frame_check(TX_ADDRESS, UA_FRAME))
                    return 0;
                    // CONNECTED! START DATA TRANSFER
                
                
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

        }


        return -1;

    }


}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize) {
    unsigned char frame[MAX_SIZE];
    int frameIndex = 0;
    frame[frameIndex++] = FLAG;
    frame[frameIndex++] = TX_ADDRESS;
    unsigned char C = (frameCounter % 2 == 0) ? FRAME_0 : FRAME_1;
    frame[frameIndex++] = C;
    frame[frameIndex++] = TX_ADDRESS ^ C;
    memcpy(&frame[frameIndex], buf, bufSize);
    frameIndex += bufSize;
    unsigned char BCC2 = calculateBCC2(buf, bufSize);

    frame[frameIndex++] = BCC2;
    frame[frameIndex++] = FLAG;
    unsigned char stuffedFrame[MAX_SIZE * 2];
    int stuffedSize = stuffBytes(frame, frameIndex, stuffedFrame);

    alarmCount = 0;
    int bytesWritten = 0;
    while (alarmCount < connectionParameters.nRetransmissions)
        {
            
            if (alarmEnabled == FALSE)
            {
                bytesWritten = writeBytes(stuffedFrame, stuffedSize);
                printf("%d bytes written\n", bytesWritten);
                
                unsigned char RRFrame = (frameCounter % 2 == 0) ? RR1_FRAME : RR0_FRAME;

                int response = supervision_frame_check(TX_ADDRESS, RRFrame);

                if (response == ACK){
                    alarm(0);
                    break;
                }
                else if (response == REJECTED){
                    alarmCount++;
                    continue;
                }                    
                
                
                alarm(connectionParameters.timeout);
                alarmEnabled = TRUE;
            }

        }

    

    frameCounter++;

    return bytesWritten >= 0 ? stuffedSize : -1;
}

int readStuffedFrame(int frameIndex, unsigned char *frame){
    unsigned char byte;
    int receivingFrame = 0;

    while (1) {
        int result = readByte((char *)&byte);
        if (result == -1) {
            perror("Error reading byte");
            return -1;
        } else if (result == 0) {
            continue;
        }
        if (byte == FLAG) {
            if (receivingFrame) {
                frame[frameIndex++] = byte;
                break;
            } else {
                frame[frameIndex++] = byte;
                receivingFrame = 1;
            }
        } else if (receivingFrame) {
            frame[frameIndex++] = byte;
        }
        if (frameIndex >= MAX_SIZE * 2) {
            perror("Frame too large");
            return -1;
        }
    }

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet) {
    unsigned char frame[MAX_SIZE * 2];
    int frameIndex = 0;
    while (readStuffedFrame(frameIndex, frame) != 0){
        sendREJ();
        continue;
    }
    unsigned char destuffedFrame[MAX_SIZE];
    int destuffedSize = destuffBytes(frame, frameIndex, destuffedFrame);
    if (destuffedSize <= 0) {
        perror("Error destuffing frame");
        return -1;
    }
    if (destuffedFrame[0] != FLAG || destuffedFrame[destuffedSize - 1] != FLAG) {
        perror("Frame does not start or end with FLAG");
        return -1;
    }

    unsigned char address = destuffedFrame[1];
    unsigned char control = destuffedFrame[2];
    unsigned char bcc1 = destuffedFrame[3];

    if (bcc1 != (address ^ control)) {
        perror("BCC1 validation failed");
        return -1;
    }

    int dataLength = destuffedSize - 6;
    unsigned char *data = &destuffedFrame[4];
    unsigned char bcc2 = destuffedFrame[destuffedSize - 2];

    unsigned char calculatedBCC2 = calculateBCC2(data, dataLength);
    if (bcc2 != calculatedBCC2) {
        perror("BCC2 validation failed");
        return -1;
    }

    memcpy(packet, data, dataLength);

    unsigned char RRFrame = (control & 0x80) ? RR0_FRAME : RR1_FRAME;
    if (sendSupervisionFrame(TX_ADDRESS, RRFrame) == -1) {
        perror("Error sending RR frame");
        return -1;
    }

    return dataLength;
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
    

    writeBytes(buf, 5);
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
    

    writeBytes(buf, 5);
    printf("Sent REJ %d\n", buf[2]);
}

////////////////////////////////////////////////
// LLCLOSE
////////////////////////////////////////////////
int llclose(int showStatistics)
{
    int clstat;

    if(globalConnectionParameters.role == LlRx){
        if(supervision_frame_check(TX_ADDRESS, DISC_FRAME)){
            unsigned char buf[5];
            buf[0] = FLAG;
            buf[1] = RX_ADDRESS;
            buf[2] = DISC_FRAME;
            buf[3] = buf[1] ^ buf[2];
            buf[4] = FLAG;

            writeBytes(buf, 5);

            if(supervision_frame_check(TX_ADDRESS, UA_FRAME)){
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

        while(alarmCount < 3){
            if(alarmEnabled == FALSE){
                writeBytes(buf,5);

                if(supervision_frame_check(RX_ADDRESS, DISC_FRAME)){
                    buf[0] = FLAG;
                    buf[1] = TX_ADDRESS;
                    buf[2] = UA_FRAME;
                    buf[3] = buf[1] ^ buf[2];
                    buf[4] = FLAG;

                    writeBytes(buf,5);
                    
                    clstat = closeSerialPort();
                    return clstat;
                }

                alarm(3);
                alarmEnabled = TRUE;
            }
        }
    }
    clstat = closeSerialPort();
    return clstat;
}


void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}

unsigned char calculateBCC2(const unsigned char *data, int dataSize) {
    unsigned char bcc2 = 0;
    for (int i = 0; i < dataSize; i++) {
        bcc2 ^= data[i];
    }
    return bcc2;
}

int stuffBytes(const unsigned char *input, int inputSize, unsigned char *output) {

    int outputIndex = 0;
    for (int i = 0; i < inputSize; i++) {
        if (input[i] == FLAG) {
            output[outputIndex++] = ESC;
            output[outputIndex++] = FLAG_ESC;
        } else if (input[i] == ESC) {
            output[outputIndex++] = ESC;
            output[outputIndex++] = ESC_ESC;
        } else {
            output[outputIndex++] = input[i];
        }
    }
    return outputIndex;
}

int destuffBytes(const unsigned char *stuffedFrame, int stuffedSize, unsigned char *destuffedFrame) {
    int destuffedIndex = 0;

    for (int i = 0; i < stuffedSize; i++) {
        if (stuffedFrame[i] == ESC) { // Escape byte detected
            if (i + 1 < stuffedSize) {
                if (stuffedFrame[i + 1] == FLAG_ESC) {
                    destuffedFrame[destuffedIndex++] = FLAG;
                } else if (stuffedFrame[i + 1] == ESC_ESC) {
                    destuffedFrame[destuffedIndex++] = ESC;
                }
                i++;
            } else {
                perror("Invalid byte stuffing sequence");
                return -1;
            }
        } else {
            destuffedFrame[destuffedIndex++] = stuffedFrame[i];
        }
    }

    return destuffedIndex;
}