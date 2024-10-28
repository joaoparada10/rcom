#ifndef _UTILS_H_
#define _UTILS_H_

// Application layer macros
#define START 1
#define DATA 2
#define END 3
#define FILE_SIZE 0
#define FILE_NAME 1

// SIZE of maximum acceptable payload.
// Maximum number of bytes that application layer should send to link layer
#define MAX_PAYLOAD_SIZE 100
#define MAX_DATA_PER_PACKET (MAX_PAYLOAD_SIZE - 4)

// Link layer macros
#define FLAG 0x7E
#define TX_ADDRESS 0x03
#define RX_ADDRESS 0x01
#define SET_FRAME 0x03
#define UA_FRAME 0x07
#define RR0_FRAME 0xAA
#define RR1_FRAME 0xAB
#define IFRAME_0 0x00
#define IFRAME_1 0x80
#define REJ0_FRAME 0x54
#define REJ1_FRAME 0x55
#define DISC_FRAME 0x0B
#define ESC 0x7D
#define FLAG_ESC 0x5E
#define ESC_ESC 0x5D
#define MAX_IFRAME_SIZE (MAX_PAYLOAD_SIZE * 2 + 7)

enum flag{
  starting = 0,
  ack = 1,
  rej = 2,
  processing_data = 3,
};

enum state{
    start = 0,
    flag_rcv = 1,
    a_rcv = 2,
    c_rcv = 3,
    bcc1_ok = 4,
    bcc2_ok = 5,
    stop = 6,
    rejected = 7,
    receiving_data = 8,
    disc = 10,
};


#endif 