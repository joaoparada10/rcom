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
    unsigned char temp[1] = {0};
    unsigned char bcc2 = 0;
    unsigned char index = 0;
    state = start;

    while(state != stop)
    {
        readByte(temp);
        switch(state)
        {
            case start:
                if(temp[0] == FLAG)
                {
                    
                }
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
    // TODO

    int clstat = closeSerialPort();
    return clstat;
}




void alarmHandler(int signal)
{
    alarmEnabled = FALSE;
    alarmCount++;

    printf("Alarm #%d\n", alarmCount);
}