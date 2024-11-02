// Application layer protocol implementation

#include "application_layer.h"
#include "link_layer.h"
#include "utils.h"
#include <string.h>
#include <stddef.h>
#include <stdlib.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayerRole linkRole;
    if (strcmp(role, "tx") == 0)
        linkRole = LlTx;
    else
        linkRole = LlRx;

    LinkLayer linkData;
    strcpy(linkData.serialPort, serialPort);
    linkData.role = linkRole;
    linkData.baudRate = baudRate;
    linkData.nRetransmissions = nTries;
    linkData.timeout = timeout;

    int open = llopen(linkData);
    if (open == -1)
    {
        printf("APP: Error on llopen. \n");
        return;
    }
    else if (open == 1)
        printf("APP: llopen success. \n");

    if (linkRole == LlTx)
    {

        FILE *file = fopen(filename, "rb");
        if (file == NULL)
        {
            printf("APP: Error opening file.\n");
            return;
        }
        // Start Control Packet
        int startControlPacketSize;
        long fileSize = getFileSize(file);
        unsigned char *startControlPacket = createControlPacket(START, filename, fileSize, &startControlPacketSize);
        if (llwrite(startControlPacket, startControlPacketSize) == -1)
        {
            printf("APP: Error writting start control packet.\n");
            return;
        }

        // Data Packets
        unsigned char buffer[MAX_DATA_PER_PACKET];
        int bytesRead;
        int sequence = 0;
        size_t totalBytesSent = 0;
        while ((bytesRead = fread(buffer, 1, MAX_DATA_PER_PACKET, file)) > 0)
        {
            int packetSize;
            unsigned char *dataPacket = createDataPacket(sequence, buffer, bytesRead, &packetSize);

            if (dataPacket == NULL)
            {
                printf("APP: Error creating data packet.\n");
                return;
            }
            int charsWritten = llwrite(dataPacket, packetSize);

            if (charsWritten == -1)
            {
                printf("APP: Error writing data packet. File transfer unsuccessful.\n");
                free(dataPacket);
                return;
            }
            else
            {
                totalBytesSent += bytesRead;
                float progress = (float)totalBytesSent / fileSize;
                printProgressBar(progress);
                free(dataPacket);
                sequence++;
            }
        }

        // End Control Packet
        int endControlPacketSize;
        unsigned char *endControlPacket = createControlPacket(END, filename, fileSize, &endControlPacketSize);
        if (llwrite(endControlPacket, endControlPacketSize) == -1)
        {
            printf("APP: Error writing end control packet.\n");
            free(endControlPacket);
            fclose(file);
            return;
        }

        printProgressBar(1.0);
        printf("\n");

        free(endControlPacket);
        fclose(file);
        printf("APP: File sent successfully\n");
        if (llclose(TRUE) >= 0)
        {
            printf("APP: Connection closed successfully.\n");
            return;
        }
        else
            printf("APP: Error closing connection. \n");
        return;
    }

    else if (linkRole == LlRx)
    {
        // Receiver logic
        unsigned char receivedPacket[MAX_PAYLOAD_SIZE];
        FILE *outputFile = fopen(filename, "wb");
        if (outputFile == NULL)
        {
            printf("APP: Error opening output file.\n");
            return;
        }

        while (1)
        {
            if (llread(receivedPacket) < 0)
            {
                printf("APP: Error reading packet. File transfer unsuccessful.\n");
                return;
            }

            unsigned char controlField = receivedPacket[0];
            if (controlField == START)
            {
                printf("APP: Received start control packet.\n");
            }
            else if (controlField == DATA)
            {
                int sequenceNumber = receivedPacket[1];
                int L2 = receivedPacket[2];
                int L1 = receivedPacket[3];
                int dataLength = (L2 << 8) | L1;
                unsigned char *data = &receivedPacket[4];
                int writtenBytes = fwrite(data, 1, dataLength, outputFile);
                if (writtenBytes < dataLength)
                {
                    printf("APP: Error writing data to output file.\n");
                }
                printf("APP: Received data packet with sequence number %d. Data packet size = %d.\n", sequenceNumber, writtenBytes);
            }
            else if (controlField == END)
            {
                printf("APP: Received end control packet. File transfer complete.\n");
                fclose(outputFile);
                if (llclose(TRUE) >= 0)
                {
                    printf("APP: Connection closed successfully.\n");
                    return;
                }
                else
                    printf("APP: Error closing connection. \n");
                return;
            }
            else
            {
                printf("APP: Received unknown packet type.\n");
            }
        }
    }
}

long getFileSize(FILE *file)
{
    fseek(file, 0, SEEK_END);
    long fileSize = ftell(file);
    fseek(file, 0, SEEK_SET);
    return fileSize;
}

unsigned char *createDataPacket(int sequenceNumber, unsigned char *data, int dataSize, int *packetSize)
{
    *packetSize = 1 + 1 + 2 + dataSize; // Control field + Sequence number + L2, L1 + Data
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL)
    {
        printf("APP: Failed to allocate memory for data packet.\n");
        return NULL;
    }

    int index = 0;
    packet[index++] = DATA;
    packet[index++] = sequenceNumber % 100;
    packet[index++] = dataSize / 256; // L2: High byte of data length
    packet[index++] = dataSize % 256; // L1: Low byte of data length
    memcpy(&packet[index], data, dataSize);

    return packet;
}

unsigned char *createControlPacket(unsigned char controlField, const char *fileName, long fileSize, int *packetSize)
{
    int fileNameLength = strlen(fileName);
    int fileSizeLength = sizeof(fileSize);
    *packetSize = 1 + 2 + fileSizeLength + 2 + fileNameLength;
    unsigned char *packet = (unsigned char *)malloc(*packetSize);
    if (packet == NULL)
    {
        printf("APP: Failed to allocate memory for control packet.\n");
        return NULL;
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

void printProgressBar(float progress) {
    int barWidth = 50;
    printf("\r[");
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) printf("=");
        else if (i == pos) printf(">");
        else printf(" ");
    }
    printf("] %d%%", (int)(progress * 100));
    fflush(stdout);
}
