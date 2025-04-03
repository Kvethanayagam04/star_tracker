#include <windows.h>
#include <stdio.h>

#define SERIAL_PORT "COM7" // Change this to your actual COM port

float *get_coordinates()
{
    char *coordinates = malloc(128 * sizeof(char)); // Buffer to store output
    int exit_flag = 0;                              // Flag to exit loop
    float *result = malloc(2 * sizeof(float));

    while (1)
    {
        if (exit_flag)
        {
            break;
        }

        FILE *fp;          // File pointer for pipe
        char star[128];    // Stores star name
        char command[512]; // Stores python command

        printf("What star are you looking for: \n");
        fgets(star, sizeof(star), stdin);

        // Remove newline character from input
        star[strcspn(star, "\n")] = '\0';

        // Run Python command and open a pipe to read its output
        sprintf(command,
                "python -c \"from astroquery.simbad import Simbad; result = Simbad.query_object('%s'); print(result['ra'][0], result['dec'][0]) if result is not None else print('ERROR')\"",
                star);
        fp = popen(command, "r");

        // Read output from Python script
        while (fgets(coordinates, 128, fp) != NULL)
        {
            if (strstr(coordinates, "ERROR") != NULL)
            { // Look for "ERROR" in the buffer
                printf("Star name %s was invalid, please try again\n", star);
                break; // Exit loop on error
            }
            else
            {
                int ret = sscanf(coordinates, "%f %f", &result[0], &result[1]);
                if (ret == 2 && result[1] >= 0)
                {
                    exit_flag = 1;
                }
                else if (ret == 2){
                    // means dec < 0
                    printf("Star name %s is out of visibility range, please try again\n", star);
                    break; // Exit loop on error
                }
                else
                {
                    printf("Star name %s was invalid, please try again\n", star);
                    break; // Exit loop on error
                }
            }
        }

        // Close the pipe
        pclose(fp);
    }

    free(coordinates);
    return result;
}

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

    while (1)
    {
        // Get user input for star
        float *coordinates = get_coordinates();
        char message[50];
        snprintf(message, sizeof(message), "%.2f,%.2f\n", coordinates[0], coordinates[1]);
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

        // Prompt for the next star after receiving a response
        printf("Ready for the next star.\n");
    }

    // Close the serial port
    CloseHandle(hSerial);
    return 0;
}
