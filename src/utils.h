#ifndef _UTILS_H_
#define _UTILS_H_

#define FLAG 0x7E
#define TX_ADDRESS 0x03
#define RX_ADDRESS 0x01
#define SET_FRAME 0x03
#define UA_FRAME 0x07
#define RR0_FRAME 0xAA
#define RR1_FRAME 0xAB
#define REJ0_FRAME 0x54
#define REJ1_FRAME 0x55
#define DISC_FRAME 0x0B
#define MAX_SIZE (MAX_PAYLOAD_SIZE + 10)

enum state{
    start = 0,
    flag_rcv = 1,
    a_rcv = 2,
    c_rcv = 3,
    bcc_ok = 4,
    stop = 5,
};


#endif 