#include <stdio.h>
#include <stdlib.h>
#include <string.h>

float* get_coordinates() {
    char *coordinates = malloc(128 * sizeof(char));  // Buffer to store output
    int exit_flag = 0; // Flag to exit loop
    float *result = malloc(2 * sizeof(float));

    while (1){
        if (exit_flag) {
            break;
        }

        FILE *fp;          // File pointer for pipe
        char star[128];     // Stores star name
        char command[512];  // Stores python command

        printf("What star are you looking for: \n");
        scanf("%s", star);

        // Run Python command and open a pipe to read its output
        sprintf(command, 
            "python3 -c \""
            "from astroquery.simbad import Simbad\n"
            "try:\n"
            "    result = Simbad.query_object('%s')\n"
            "    if result is None:\n"
            "        print('ERROR')\n"
            "    else:\n"
            "        print(result['ra'][0], result['dec'][0])\n"
            "except Exception:\n"
            "    print('ERROR')\"", 
            star);
        fp = popen(command, "r");

        // Read output from Python script
        while (fgets(coordinates, 128, fp) != NULL) {
            if (strstr(coordinates, "ERROR") != NULL) {  // Look for "ERROR" in the buffer
                printf("Star name %s was invalid, please try again\n", star);
                break;  // Exit loop on error
            } else {
                int ret = sscanf(coordinates, "%f %f", &result[0], &result[1]);
                if (ret == 2) {
                    exit_flag = 1;
                }
                else{
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

int main() {
    float* coordinates = get_coordinates();
    printf("ra: %f, dec: %f\n", coordinates[0], coordinates[1]);
}