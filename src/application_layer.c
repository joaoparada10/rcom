// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "utils.h"

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole linkRole;
    if (strcmp(role, "tx") == 0)
        linkRole = LlTx;
    else linkRole = LlRx;


    LinkLayer linkData;
    strcpy(linkData.serialPort, serialPort);
    linkData.role = linkRole;
    linkData.baudRate = baudRate;
    linkData.nRetransmissions = nTries;
    linkData.timeout = timeout;
    

    
    int dataLinkId = llopen(linkData);
    
    if (linkRole == LlTx){

        FILE *file = fopen(filename, "rb");
        if (file == NULL) {
            perror("Error opening file.");
            return -1;
        }
        // Start Control Packet
        int startControlPacketSize;
        long fileSize = getFileSize(file);
        unsigned char *startControlPacket = createControlPacket(START,filename,fileSize,&startControlPacketSize);
        if (llwrite(startControlPacket, startControlPacketSize) == -1){
            perror("Error writting start control packet.");
            return -1;
        }

        // Data Packets
        unsigned char buffer[MAX_PAYLOAD_SIZE];
        int bytesRead;
        int sequence = 0;
        while ((bytesRead = fread(buffer, 1, MAX_PAYLOAD_SIZE, file)) > 0) {
            int packetSize;
            unsigned char *dataPacket = createDataPacket(sequence, buffer, bytesRead, &packetSize);
            
            if (dataPacket == NULL) {
                perror("Error creating data packet");
                break;
            }

            if (llwrite(dataPacket, packetSize) == -1) {
                perror("Error writing data packet");
                free(dataPacket); 
                break;
            }
            free(dataPacket);
            sequence++;
        }

        // End Control Packet
        int endControlPacketSize;
        unsigned char *endControlPacket = createControlPacket(END, (unsigned char *)filename, fileSize, &endControlPacketSize);
        if (llwrite(endControlPacket, endControlPacketSize) == -1) {
            perror("Error writing end control packet.");
            free(endControlPacket);
            fclose(file);
            return -1;
        }

        free(endControlPacket);
        fclose(file);
        printf("File sent successfully\n");
        return 0;
    }

    else if (linkRole == LlRx){
        // Receiver logic
        unsigned char receivedPacket[MAX_PAYLOAD_SIZE + 5];
        int packetSize;

        FILE *outputFile = fopen(filename, "wb");
        if (outputFile == NULL) {
            perror("Error opening output file.");
            return -1;
        }

        while (1) {
            if (llread(receivedPacket) < 0) {
                perror("Error reading packet");
                break;
            }

            unsigned char controlField = receivedPacket[0];
            if (controlField == START) {
                printf("Received start control packet.\n");
            } else if (controlField == DATA) {
                int sequenceNumber = receivedPacket[1];
                int L2 = receivedPacket[2];
                int L1 = receivedPacket[3];
                int dataLength = (L2 << 8) | L1;
                unsigned char *data = &receivedPacket[4];
                int writtenBytes = fwrite(data, 1, dataLength, outputFile);
                if (writtenBytes < dataLength) {
                    perror("Error writing data to output file");
                }
                printf("Received data packet with sequence number %d\n", sequenceNumber);
            } else if (controlField == END) {
                printf("Received end control packet. File transfer complete.\n");
                break;
            } else {
                perror("Received unknown packet type.");
            }
        }

        fclose(outputFile);

    }
    
}

long getFileSize(FILE *file){
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    return fileSize;
}

unsigned char *createDataPacket(int sequenceNumber, unsigned char *data, int dataSize, int *packetSize) {
    *packetSize = 1 + 1 + 2 + dataSize;  // Control field + Sequence number + L2, L1 + Data
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) {
        perror("Failed to allocate memory for data packet");
        return -1;
    }

    int index = 0;
    packet[index++] = DATA;
    packet[index++] = sequenceNumber % 100;
    packet[index++] = dataSize / 256;  // L2: High byte of data length
    packet[index++] = dataSize % 256;  // L1: Low byte of data length
    memcpy(&packet[index], data, dataSize);
    
    return packet;
}

unsigned char *createControlPacket(unsigned char controlField, unsigned char *fileName, long fileSize, int *packetSize) {
    int fileNameLength = strlen((char *)fileName);
    int fileSizeLength = sizeof(fileSize);
    *packetSize = 1 + 2 + fileSizeLength + 2 + fileNameLength;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL) {
        perror("Failed to allocate memory for control packet");
        return -1;
    }

    int index = 0;
    packet[index++] = controlField;
    packet[index++] = FILE_SIZE;
    packet[index++] = fileSizeLength;
    memcpy(&packet[index], &fileSize, fileSizeLength);
    index += fileSizeLength;
    packet[index++] = FILE_NAME;
    packet[index++] = fileNameLength;
    memcpy(&packet[index], fileName, fileNameLength);
    index += fileNameLength;

    return packet;
}
