// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole link_role;

    if (strcmp(role, "tx") == 0)
        link_role = LlTx;
    else link_role = LlRx;


    LinkLayer link_data;
    strcpy(link_data.serialPort, serialPort);
    link_data.role = link_role;
    link_data.baudRate = baudRate;
    link_data.nRetransmissions = nTries;
    link_data.timeout = timeout;

    
    int data_link_id = llopen(link_data);
    
}
