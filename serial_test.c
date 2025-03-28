#include <windows.h>
#include <stdio.h>

#define SERIAL_PORT "COM7" // Change this to your actual COM port

int main()
{
    HANDLE hSerial;
    DCB dcbSerialParams = {0};
    COMMTIMEOUTS timeouts = {0};
    char buffer[256];
    DWORD bytesRead;

    // Open the serial port
    hSerial = CreateFile(SERIAL_PORT, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
    if (hSerial == INVALID_HANDLE_VALUE)
    {
        printf("Error opening serial port\n");
        return 1;
    }

    // Set up serial parameters
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    if (!GetCommState(hSerial, &dcbSerialParams))
    {
        printf("Error getting serial port state\n");
        CloseHandle(hSerial);
        return 1;
    }

    dcbSerialParams.BaudRate = CBR_115200; // Set baud rate to 9600
    dcbSerialParams.ByteSize = 8;          // 8 data bits
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;

    if (!SetCommState(hSerial, &dcbSerialParams))
    {
        printf("Error setting serial port state\n");
        CloseHandle(hSerial);
        return 1;
    }

    // Set timeouts for reading
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;

    // Set the timeouts
    if (!SetCommTimeouts(hSerial, &timeouts))
    {
        printf("Error setting timeouts\n");
        CloseHandle(hSerial);
        return 1;
    }

    // Send "hello, world" string
    const char *message = "hello, world\n";
    DWORD bytesWritten;
    if (!WriteFile(hSerial, message, strlen(message), &bytesWritten, NULL))
    {
        printf("Error writing to serial port\n");
        CloseHandle(hSerial);
        return 1;
    }

    printf("Message sent: %s\n", message);

    // Listen for a response
    Sleep(1000); // Wait briefly to allow the board to respond
    if (ReadFile(hSerial, buffer, sizeof(buffer) - 1, &bytesRead, NULL) && bytesRead > 0)
    {
        buffer[bytesRead] = '\0'; // Null-terminate the received string
        printf("Received: %s\n", buffer);
    }
    else
    {
        printf("No response received or error reading from serial port\n");
    }

    // Close the serial port
    CloseHandle(hSerial);
    return 0;
}