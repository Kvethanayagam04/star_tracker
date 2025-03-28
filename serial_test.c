#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#define SERIAL_PORT "COM7"  // Change this to your serial port (e.g., COMx on Windows)

int main() {
    int serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        perror("Error opening serial port");
        return 1;
    }

    struct termios options;
    tcgetattr(serial_fd, &options);
    cfsetispeed(&options, B9600); // Set baud rate to 9600
    cfsetospeed(&options, B9600);
    options.c_cflag |= (CLOCAL | CREAD); // Enable receiver, ignore modem status lines
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8; // 8 data bits
    tcsetattr(serial_fd, TCSANOW, &options);

    char buffer[256];
    while (1) {
        int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            printf("Received: %s", buffer);
            fflush(stdout);
        }
        usleep(100000); // Sleep for 100ms to avoid high CPU usage
    }

    close(serial_fd);
    return 0;
}
