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


int supervision_frame_check(unsigned char address, unsigned char control){
    unsigned char temp [1] = {0};
    while (state != stop)
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
                
                if (temp[0] == SET_FRAME && control == SET_FRAME){
                    printf("C = 0x%02X\n", temp[0]);
                    control = temp[0];
                    state = c_rcv;
                    printf("state = C \n");
                }
                else if (temp[0] == UA_FRAME && control == UA_FRAME){
                    printf("C = 0x%02X\n", temp[0]);
                    control = temp[0];
                    state = c_rcv;
                    printf("state = C \n");
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
                    state = bcc_ok;
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
            case bcc_ok:
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

    return 1;


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
        int bytes_read = 0;

        while (alarmCount < 4)
        {
            
            if (alarmEnabled == FALSE)
            {
                int bytes = writeBytes(buf, 5);
                printf("%d bytes written\n", bytes);


                if (supervision_frame_check(TX_ADDRESS, UA_FRAME))
                    return 0;
                    // ALL GOOD! START DATA TRANSFER
                
                
                alarm(3); // Set alarm to be triggered in 3s
                alarmEnabled = TRUE;
            }

        }


        return -1;

    }


}

////////////////////////////////////////////////
// LLWRITE
////////////////////////////////////////////////
int llwrite(const unsigned char *buf, int bufSize)
{
    // TODO

    return 0;
}

////////////////////////////////////////////////
// LLREAD
////////////////////////////////////////////////
int llread(unsigned char *packet)
{
    enum state currentState = start;
    unsigned char byte, A, C, BCC1, BCC2;
    unsigned char buf[MAX_SIZE];
    int dataSize = 0;

    while(currentState != stop){
        int res = readByte(buf);
        if(res == -1){
            perror("Erro ao ler byte em llread!");
        }

        switch(currentState){
            case start:
                if(byte == FLAG){
                    currentState = flag_rcv;
                }
                break;
            case flag_rcv:
                if(byte == TX_ADDRESS || byte == RX_ADDRESS){
                    A = byte;
                    currentState = a_rcv;
                } else if(byte != FLAG){
                    currentState = start;
                }
                break;

            case a_rcv:
                if(byte == SET_FRAME || byte == UA_FRAME || byte == DISC_FRAME || byte == RR0_FRAME ||
                byte == RR1_FRAME || byte == REJ0_FRAME || byte == REJ1_FRAME || (byte & 0x01) == 0){
                    C = byte;
                    currentState = c_rcv;
                } else if(byte == FLAG){
                    currentState = flag_rcv;
                } else {
                    currentState = start;
                }
                break;
            
            case c_rcv:
                BCC1 = A ^ C;
                if(byte == BCC1){
                    currentState = bcc_ok;
                } else if(byte == FLAG){
                    currentState = flag_rcv;
                } else {
                    currentState = start;
                }
                break;

            case bcc_ok:
                if((C & 0x01) == 0){
                    buf[dataSize++] = byte;
                    currentState = stop;
                }
                else if(byte == FLAG){
                    currentState = stop;
                } else {
                    buf[dataSize++] = byte;
                }
                break;
            case stop:
                if(C == UA_FRAME || C == SET_FRAME || C == DISC_FRAME){
                    return 0;
                } else if((C & 0x01) == 0){
                    BCC2 = 0;
                    for(int i = 0; i < dataSize; i++){
                        BCC2 ^= buf[i];
                    }

                    if(BCC2 == byte){
                        sendRR();
                        mcmcpy(packet, buf, dataSize);
                        
                        return dataSize;
                    } else {
                        sendREJ();

                        return -1;
                    }
                }
                break;
            default:
                currentState = start;
                break;
        }
    }

    return 0;
}

void sendRR(int sequence){
    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = RX_ADDRESS;

    if(sequence == 0)
        buf[2] = RR0_FRAME;
    else
        buf[2] = RR1_FRAME;

    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    

    writeBytes(buf, 5);
    printf("Sent RR%d\n", sequence);
}

void sendREJ(int sequence){
    unsigned char buf[5] = {0};
    buf[0] = FLAG;
    buf[1] = RX_ADDRESS;

    if(sequence == 0)
        buf[2] = REJ0_FRAME;
    else
        buf[2] = REJ0_FRAME;

    buf[3] = buf[1] ^ buf[2];
    buf[4] = FLAG;
    

    writeBytes(buf, 5);
    printf("Sent REJ%d\n", sequence);
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

        while(alarmCount < 4){
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