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
        perror("Error opening serial port.");
        return -1;
    }

    if (connectionParameters.role == LlRx)
    {

        if (frameStateMachine(TX_ADDRESS, SET_FRAME) == ack)
        {
            sendSupervisionFrame(UA_FRAME);
            return 1;
        }
    }

    if (connectionParameters.role == LlTx)
    {
        

        (void)signal(SIGALRM, alarmHandler);
        alarmEnabled = FALSE;
        alarmCount = 0;

        while (alarmCount < connectionParameters.nRetransmissions)
        {

            if (alarmEnabled == FALSE)
            {
                sendSupervisionFrame(SET_FRAME);
                printf("Sent SET Frame.\n");
                alarm(globalConnectionParameters.timeout);
                alarmEnabled = TRUE;
                enum flag response = frameStateMachine(TX_ADDRESS, UA_FRAME);
                if (response == ack)
                {
                    printf("Received UA Frame. llopen success.\n");
                    alarm(0);
                    return 1;
                }
                else if (response == disconnect || response == rej)
                {
                    printf("Error: Receiver in the middle of file transfer. Terminating...\n");
                    return -1;
                }
                
                else if (alarmTriggered && response == timeout)
                {
                    alarmTriggered = FALSE; // Reset flag
                    alarmEnabled = FALSE;
                    printf("Retransmitting after timeout.\n");
                }
                else
                {
                    printf("Received invalid answer. Retransmitting.\n");
                    numberRetransmitions++;
                    continue;
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
int llwrite(const unsigned char *buf, int bufSize)
{
    unsigned char frame[MAX_IFRAME_SIZE];
    int frameIndex = 0;
    unsigned char bcc2 = 0;
    unsigned char stuffedFrame[MAX_PAYLOAD_SIZE * 2];
    frame[frameIndex++] = FLAG;                            // FLAG
    frame[frameIndex++] = TX_ADDRESS;                      // A
    unsigned char C = (frameCounter % 2 == 0) ? IFRAME_0 : IFRAME_1;
    frame[frameIndex++] = C;                               // C
    frame[frameIndex++] = TX_ADDRESS ^ C;                  // BCC1

    int stuffedSize = stuffBytes(buf, bufSize, stuffedFrame, &bcc2);
    printf("Stuffed Payload size = %d \n", stuffedSize);
    memcpy(&frame[frameIndex], stuffedFrame, stuffedSize); // Data
    frameIndex += stuffedSize;
    if (bcc2 == FLAG) 
    { 
        frame[frameIndex++] = ESC;
        frame[frameIndex++] = FLAG_ESC;
    }
    else if (bcc2 == ESC)
    {                                                     // BCC2
        frame[frameIndex++] = ESC;
        frame[frameIndex++] = ESC_ESC;
    }
    else
    {
        frame[frameIndex++] = bcc2;
    }
    frame[frameIndex++] = FLAG;                            // FLAG

    (void)signal(SIGALRM, alarmHandler);
    alarmEnabled = FALSE;
    alarmCount = 0;
    int bytesWritten = 0;
    while (alarmCount < globalConnectionParameters.nRetransmissions)
    {

        if (alarmEnabled == FALSE)
        {
            unsigned char RRFrame = (frameCounter % 2 == 0) ? RR1_FRAME : RR0_FRAME;
            bytesWritten = writeBytes((char *)frame, frameIndex);
            printf("%d bytes written\n", bytesWritten);
            alarm(globalConnectionParameters.timeout);
            alarmEnabled = TRUE;

            enum flag response = frameStateMachine(TX_ADDRESS, RRFrame);
            if (response == ack)
            {
                alarm(0);
                alarmEnabled = FALSE;
                printf("Received ACK. FRAME NUMBER = %d \n", frameCounter % 2);
                frameCounter++;
                return stuffedSize;
            }
            else if (response == rej)
            {
                printf("Received REJ, retransmitting. BCC2 = 0x%02X. FRAME NUMBER = %d\n", bcc2, frameCounter % 2);
                alarmCount = 0;
                alarm(0);
                alarmEnabled = FALSE;
                numberRetransmitions++;
                continue;
            }
            else if (response == disconnect)
            {
                printf("Received premature disconnect, error occured. Terminating...\n");
                return -1;
            }
            if (alarmTriggered)
            {
                alarmTriggered = FALSE;
                alarmEnabled = FALSE;
                printf("Retransmitting after timeout. FRAME NUMBER = %d\n", frameCounter % 2);
            }
        }
    }
    printf("Didn't get receiver response after %d retransmissions. Exiting.\n", alarmCount);
    return -1;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    unsigned char bcc1 = 0;
    unsigned char IFrame = (frameCounter % 2 == 0) ? IFRAME_0 : IFRAME_1;
    unsigned char dataFrame[MAX_PAYLOAD_SIZE];
    unsigned char bcc2 = 0;
    int status = frameStateMachine(TX_ADDRESS, IFrame);

    while (1)
    {
        if (status == ack)
        {
            printf("Received SET Duplicate. Retransmitting UA FRAME. \n");
            sendSupervisionFrame(UA_FRAME);
            status = frameStateMachine(TX_ADDRESS, IFrame);
            continue;
        }
        else if (status == disconnect)
        {
            printf("Received SET Frame in the middle of a transfer. Error occurred, terminating... \n");
            sendSupervisionFrame(DISC_FRAME);
            return -1;
        }
        else if (readByte((char *)&bcc1) == 1)
        {
            if (bcc1 != (TX_ADDRESS ^ IFRAME_0) && bcc1 != (TX_ADDRESS ^ IFRAME_1))
            {
                printf("Invalid BCC1, header error. bcc1 = %x \n", bcc1);
                sendREJ();
                status = frameStateMachine(TX_ADDRESS, IFrame);
                continue;
            }

            int dataFrameSize = readStuffedFrame(dataFrame, &bcc2);

            if (status == discard)
            {
                printf("Received Duplicate Information Frame %d. Retransmitting RR FRAME.\n", (frameCounter - 1) % 2);
                sendRR(TRUE);
                status = frameStateMachine(TX_ADDRESS, IFrame);
                continue;
            }
            else if (dataFrameSize > 0)
            {
                printf("Information Frame %d acknowledged. Sending RR FRAME.\n", frameCounter % 2);
                memcpy(packet, dataFrame, dataFrameSize);
                sendRR(FALSE);
                return dataFrameSize;
            }
            else if (dataFrameSize == -1)
            {
                sendREJ();
                status = frameStateMachine(TX_ADDRESS, IFrame);
                continue;
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

    if (globalConnectionParameters.role == LlRx)
    {
        int status = frameStateMachine(TX_ADDRESS, DISC_FRAME);
        if (status == ack)
        {
            sendSupervisionFrame(DISC_FRAME);
            printf("llclose receiver wrote disc frame.\n");
            if (frameStateMachine(TX_ADDRESS, UA_FRAME) == ack)
            {
                printf("Statistics: \n Frames received = %d \n", frameCounter);
                clstat = closeSerialPort();
                return clstat;
            }
            else
            {
                printf("Error: Invalid UA Frame. Terminating...\n");
                return -1;
            }
        }
        else
        {
            printf("Error: Invalid DISC Frame. Terminating...\n");
            return -1;
        }
    }
    else if (globalConnectionParameters.role == LlTx)
    {
        (void)signal(SIGALRM, alarmHandler);
        alarmEnabled = FALSE;
        alarmCount = 0;

        while (alarmCount < globalConnectionParameters.nRetransmissions)
        {
            if (alarmEnabled == FALSE)
            {
                sendSupervisionFrame(DISC_FRAME);
                printf("llclose wrote disc frame.\n");
                alarm(globalConnectionParameters.timeout);
                alarmEnabled = TRUE;
                if (frameStateMachine(TX_ADDRESS, DISC_FRAME) == ack)
                {
                    alarm(0);
                    sendSupervisionFrame(UA_FRAME);
                    printf("Statistics: \n Frames transmitted = %d \n Number of retransmissions = %d \n Number of timeouts = %d \n", frameCounter, numberRetransmitions, numberTimeouts);
                    clstat = closeSerialPort();
                    return clstat;
                }
                if (alarmTriggered)
                {
                    alarmTriggered = FALSE;
                    alarmEnabled = FALSE;
                    printf("Retransmitting after timeout.\n");
                }
            }
        }
    }
    return -1;
}

int frameStateMachine(unsigned char address, unsigned char control)
{
    unsigned char byte;
    enum flag response = starting;
    globalState = start;
    while (globalState != stop && globalState != receiving_data && alarmEnabled == TRUE)
    {
        if (alarmTriggered)
        {
            printf("Timeout in frameStateMachine\n");
            numberTimeouts++;
            return timeout;
        }
        if (readByte((char *)&byte) == 1)
        {
           
            switch (globalState)
            {
            case start:
                if (byte == FLAG)
                {
                    globalState = flag_rcv;
                    //printf("globalState = START -> FLAG \n");
                }
                break;

            case flag_rcv:

                if (byte == TX_ADDRESS && address == TX_ADDRESS)
                {
                    globalState = a_rcv;
                    //printf("globalState = FLAG -> ADDRESS \n");
                }
                else if (byte == RX_ADDRESS && address == RX_ADDRESS)
                {
                    globalState = a_rcv;
                    //printf("globalState = FLAG -> ADDRESS \n");
                }

                else if (byte == FLAG)
                {
                    globalState = flag_rcv;
                    //printf("globalState = FLAG -> FLAG \n");
                }
                else
                {
                    globalState = start;
                    //printf("globalState = FLAG ->START \n");
                }
                break;

            case a_rcv:

                if ((byte == SET_FRAME && control == SET_FRAME) || (byte == UA_FRAME && control == UA_FRAME) || (byte == RR0_FRAME && control == RR0_FRAME) ||
                    (byte == RR1_FRAME && control == RR1_FRAME) || (byte == DISC_FRAME && control == DISC_FRAME) || (byte == SET_FRAME && (control == IFRAME_0 || control == IFRAME_1) && frameCounter == 0))
                {
                    control = byte;
                    globalState = c_rcv;
                    response = ack;
                    //printf("globalState = ADDRESS -> CONTROL (ACK) \n");
                }
                else if ((byte == IFRAME_0 && control == IFRAME_0) || (byte == IFRAME_1 && control == IFRAME_1))
                {
                    globalState = receiving_data;
                    response = processing_data;
                    //printf("globalState = ADDRESS -> RECEIVING_DATA \n");
                }
                else if ((byte == IFRAME_1 && control == IFRAME_0) || (byte == IFRAME_0 && control == IFRAME_1))
                {
                    globalState = receiving_data;
                    response = discard;
                    //printf("globalState = ADDRESS -> RECEIVING_DATA (DISCARD) \n");
                }
                else if (byte == REJ0_FRAME || byte == REJ1_FRAME)
                {
                    control = byte;
                    globalState = c_rcv;
                    response = rej;
                    //printf("globalState = ADDRESS -> CONTROL (REJ) \n");
                }
                else if (byte == DISC_FRAME || (byte == SET_FRAME && (control == IFRAME_0 || control == IFRAME_1) && frameCounter > 0))
                {
                    control = byte;
                    globalState = c_rcv;
                    response = disconnect;
                    //printf("globalState = ADDRESS -> CONTROL (DISCONNECT) \n");
                }
                else if (byte == FLAG)
                {
                    globalState = flag_rcv;
                    //printf("globalState = ADDRESS -> FLAG \n");
                }
                else
                {
                    globalState = start;
                    //printf("globalState = ADDRESS -> START \n");
                }
                break;
            case c_rcv:

                if (byte == (address ^ control))
                {
                    globalState = bcc1_ok;
                    //printf("globalState = CONTROL -> BCC1_OK \n");
                }
                else if (byte == FLAG)
                {
                    globalState = flag_rcv;
                    //printf("globalState = CONTROL -> FLAG \n");
                }
                else
                {
                    globalState = start;
                    //printf("globalState = CONTROL -> START \n");
                }
                break;
            case bcc1_ok:
                if (byte == FLAG)
                {
                    globalState = stop;
                    //printf("globalState = BCC1_OK -> STOP \n");
                }
                else
                {
                    globalState = start;
                    //printf("globalState = BCC1_OK -> START \n");
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

int stuffBytes(const unsigned char *input, int inputSize, unsigned char *output, unsigned char *bcc2)
{

    int outputIndex = 0;
    for (int i = 0; i < inputSize; i++)
    {

        if (input[i] == FLAG)
        {
            *bcc2 ^= FLAG;
            output[outputIndex++] = ESC;
            output[outputIndex++] = FLAG_ESC;
        }
        else if (input[i] == ESC)
        {
            *bcc2 ^= ESC;
            output[outputIndex++] = ESC;
            output[outputIndex++] = ESC_ESC;
        }
        else
        {
            *bcc2 ^= input[i];
            output[outputIndex++] = input[i];
        }
    }
    return outputIndex;
}

int readStuffedFrame(unsigned char *dataFrame, unsigned char *bcc2)
{
    int dataFrameIndex = 0;
    unsigned char byte;
    unsigned char prevbcc2 = 0;
    unsigned char prevbyte = 0;
    *bcc2 = 0;
    while (dataFrameIndex <= MAX_PAYLOAD_SIZE)
    {

        if (readByte((char *)&byte) == 1)
        {
            if (byte == FLAG)
            {
                if (prevbcc2 != prevbyte)
                {

                    printf("Invalid BCC2, data error. Found BCC2 = 0x%02X. Calculated BCC2 = 0x%02X.\n", prevbyte, prevbcc2);
                    return -1;
                }

                *bcc2 = prevbcc2;
                // printf("Found flag at the end of payload. Payload size = %d. \n" , dataFrameIndex);
                return dataFrameIndex;
            }
            if (byte == ESC)
            {
                while (readByte((char *)&byte) != 1)
                {
                    continue;
                }
                if (byte == FLAG_ESC)
                {
                    prevbcc2 = *bcc2;
                    *bcc2 ^= FLAG;
                    prevbyte = FLAG;
                    if (dataFrameIndex < MAX_PAYLOAD_SIZE)
                        dataFrame[dataFrameIndex++] = FLAG;
                }
                else if (byte == ESC_ESC)
                {
                    prevbcc2 = *bcc2;
                    *bcc2 ^= ESC;
                    prevbyte = ESC;
                    if (dataFrameIndex < MAX_PAYLOAD_SIZE)
                        dataFrame[dataFrameIndex++] = ESC;
                }
                else if ((byte == FLAG) && (dataFrameIndex == MAX_PAYLOAD_SIZE) && (*bcc2 == ESC)) // specific case where BCC2 is == ESC
                {
                    printf("Found flag at the end of payload. Payload size = %d. SPECIFIC CASE! \n", dataFrameIndex + 1);
                    return dataFrameIndex;
                }
                else
                {

                    printf("Invalid byte stuffing sequence. Got 0x%02X after ESC. Index = %d. PREV BCC2 = 0x%02X.  BCC2 = 0x%02X.\n", byte, dataFrameIndex, prevbcc2, *bcc2);
                    return -1;
                }
            }
            else
            {
                prevbcc2 = *bcc2;
                *bcc2 ^= byte;
                prevbyte = byte;
                if (dataFrameIndex < MAX_PAYLOAD_SIZE)
                    dataFrame[dataFrameIndex++] = byte;
            }
        }
    }
    return dataFrameIndex;
}

int sendRR(int discard)
{
    if (!discard)
        frameCounter++;
    if (frameCounter % 2 == 0)
        return sendSupervisionFrame(RR0_FRAME);
    else
        return sendSupervisionFrame(RR1_FRAME);
}

int sendREJ()
{
    if (frameCounter % 2 == 0)
        return sendSupervisionFrame(REJ1_FRAME);
    else
        return sendSupervisionFrame(REJ0_FRAME);
}

int sendSupervisionFrame(unsigned char control)
{
    unsigned char flag, address, bcc;
    unsigned char buf[5] = {0};
    flag = FLAG;
    address = TX_ADDRESS;
    bcc = address ^ control;

    buf[0] = flag;
    buf[1] = address;
    buf[2] = control;
    buf[3] = bcc;
    buf[4] = flag;
    int bytes = writeBytes((char *)buf, 5);
    return bytes;
}