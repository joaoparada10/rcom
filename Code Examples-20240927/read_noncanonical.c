// Read from serial port in non-canonical mode
//
// Modified by: Eduardo Nuno Almeida [enalmeida@fe.up.pt]

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

// Baudrate settings are defined in <asm/termbits.h>, which is
// included by <termios.h>
#define BAUDRATE B38400
#define _POSIX_SOURCE 1 // POSIX compliant source

#define FALSE 0
#define TRUE 1
#define temp_SIZE 256

enum receiver_state{
    start = 0,
    flag_rcv = 1,
    a_rcv = 2,
    c_rcv = 3,
    bcc_ok = 4,
    stop = 5,
};

enum receiver_state state = start;

volatile int STOP = FALSE;

int main(int argc, char *argv[])
{
    // Program usage: Uses either COM1 or COM2
    const char *serialPortName = argv[1];

    if (argc < 2)
    {
        printf("Incorrect program usage\n"
               "Usage: %s <SerialPort>\n"
               "Example: %s /dev/ttyS1\n",
               argv[0],
               argv[0]);
        exit(1);
    }

    // Open serial port device for reading and writing and not as controlling tty
    // because we don't want to get killed if linenoise sends CTRL-C.
    int fd = open(serialPortName, O_RDWR | O_NOCTTY);
    if (fd < 0)
    {
        perror(serialPortName);
        exit(-1);
    }

    struct termios oldtio;
    struct termios newtio;

    // Save current port settings
    if (tcgetattr(fd, &oldtio) == -1)
    {
        perror("tcgetattr");
        exit(-1);
    }

    // Clear struct for new port settings
    memset(&newtio, 0, sizeof(newtio));

    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    // Set input mode (non-canonical, no echo,...)
    newtio.c_lflag = 0;
    newtio.c_cc[VTIME] = 0; // Inter-character timer unused
    newtio.c_cc[VMIN] = 5;  // Blocking read until 5 chars received

    // VTIME e VMIN should be changed in order to protect with a
    // timeout the reception of the following character(s)

    // Now clean the line and activate the settings for the port
    // tcflush() discards data written to the object referred to
    // by fd but not transmitted, or data received but not read,
    // depending on the value of queue_selector:
    //   TCIFLUSH - flushes data received but not read.
    tcflush(fd, TCIOFLUSH);

    // Set new port settings
    if (tcsetattr(fd, TCSANOW, &newtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    // Loop for input
    unsigned char temp[1] = {0}; // +1: Save space for the final '\0' char
    unsigned char buf[5] = {0};

    unsigned char address;
    unsigned char control;
    int byte, bytes;

    
    while (state != stop)
    {
        byte = read(fd, temp, 1);
        switch (state){
            case start:
                if (temp[0] == 0x7E){

                    printf("Flag = 0x%02X\n", temp[0]);
                    state = flag_rcv;
                    printf("state = FLAG \n");
                    buf[0] = temp[0];
                }
                
                break;

            case flag_rcv:
                
                if (temp[0] == 0x03){
                    printf("A = 0x%02X\n", temp[0]);
                    address = temp[0];
                    state = a_rcv;
                    printf("state = A \n");
                    buf[1] = temp[0];
                }

                else if (temp[0] == 0x7E){
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
                
                if (temp[0] == 0x03){
                    printf("C = 0x%02X\n", temp[0]);
                    control = temp[0];
                    state = c_rcv;
                    printf("state = C \n");
                    buf[2] = temp[0];
                }
                else if (temp[0] == 0x7E){
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
                    buf[3] = temp[0];
                }
                else if (temp[0] == 0x7E){
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
                
                if (temp[0] == 0x7E){
                    printf("FLAG = 0x%02X\n", temp[0]);
                    state = stop;
                    printf("state = STOP \n");
                    buf[4] = temp[0];
                }
                else {
                    state = start;
                    printf("state = START \n");
                }
                break;

        }

    }

    if(buf[0] == 0x7E && buf[1] == 0x03 && buf[2] == 0x03 && (buf [3] == address ^ control) && buf[4] == 0x7E)
        {
            bytes = write(fd, buf, 5);
            printf("%d bytes written\n", bytes);
            STOP = TRUE;
        }

    // The while() cycle should be changed in order to respect the specifications
    // of the protocol indicated in the Lab guide

    // Restore the old port settings
    if (tcsetattr(fd, TCSANOW, &oldtio) == -1)
    {
        perror("tcsetattr");
        exit(-1);
    }

    close(fd);

    return 0;
}
