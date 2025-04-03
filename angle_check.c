#include <stdio.h>

void safe_range_angles(float arr[]) {
    // Assumes ra between 0 and 360
    // Assumes dec between 0 and 90
    float ra = arr[0];
    float dec = arr[1];

    if (ra > 90 && ra < 180){
        arr[0] = 90 - ra;
        arr[1] = 180 - dec;
    }
    else if (ra >= 180 && ra <= 270){
        arr[0] = ra - 180;
        arr[1] = 180 - dec;
    }
    else if (ra > 270 && ra <= 360){
        arr[0] = ra - 360;
    }
}

int main() {
    float array1[] = {91.0, 56.3};
    float array2[] = {-95.0, 16.2};
    float array3[] = {5.0, 32.5};
    float array4[] = {180, 38.4};
    float array5[] = {197, 20.3};
    float array6[] = {296.3, 29.5};
    safe_range_angles(array1);
    safe_range_angles(array2);
    safe_range_angles(array3);
    safe_range_angles(array4);
    safe_range_angles(array5);
    safe_range_angles(array6);

    printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array1[i]);
    }
    printf("\n");

    printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array2[i]);
    }
    printf("\n");

    printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array3[i]);
    }
    printf("\n");

    printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array4[i]);
    }
    printf("\n");printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array5[i]);
    }
    printf("\n");

    printf("After modification: ");
    for (int i = 0; i < 2; i++) {
        printf("%f ", array6[i]);
    }
    printf("\n");
}