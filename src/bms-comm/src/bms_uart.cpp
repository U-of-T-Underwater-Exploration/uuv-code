#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

#include "bms_parser.hpp"

int main() {
    int serial_port;
    struct termios tty;

    // Open serial port
    serial_port = open("/dev/ttyS0", O_RDWR);

    if (serial_port < 0) {
        perror("Error opening serial port");
        return 1;
    }

    // Get current settings
    tcgetattr(serial_port, &tty);

    // Set baud rate 115200
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    // 8N1 configuration
    tty.c_cflag &= ~PARENB; // No parity
    tty.c_cflag &= ~CSTOPB; // 1 stop bit
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;     // 8 data bits

    tty.c_cflag |= CREAD | CLOCAL;

    // Apply settings
    tcsetattr(serial_port, TCSANOW, &tty);

    // command
    while (1)
    {
        unsigned char cmd[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 
                                0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
                                0x68, 0x00, 0x00, 0x01, 0x29};

        write(serial_port, cmd, sizeof(cmd));

        std::vector<uint8_t> buf(256);
        int n = read(serial_port, buf.data(), buf.size());
        // printf("Received %d bytes:\n", n);

        // for(int i=0;i<n;i++)
        //     printf("%02X ", buf[i]);

        // printf("\n");
        BMSData bms = parseBMSFrame(buf);
        printf("%ld\n", bms.cellVoltages[0]);
    }
    close(serial_port);

    return 0;
}