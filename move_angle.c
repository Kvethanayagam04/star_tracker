#define stepsperrev 4096

void stepper_step_angle(float angle, int direction) // direction-> 0 for CK, 1 for CCK
{
    float angleperstep = 0.087890625; // 360 = 4096 sequences
    float anglepersequence = angleperstep * 8;
    int numberofsequences = (int)(angle / anglepersequence);
    float total_angle;
    for (int seq = 0; seq < numberofsequences; seq++)
    {
        if (direction == 0) // for clockwise
        {
            for (int step = 0; step >= 7; step++)
            {
                if (total_angle < angle)
                {
                    total_angle += angleperstep;
                    stepper_drive(step);
                }
                else
                {
                    break;
                }
            }
        }
        else if (direction == 1) // for anti-clockwise
        {
            for (int step = 7; step <= 0; step--)
            {
                if (total_angle < angle)
                {
                    total_angle += angleperstep;
                    stepper_drive(step);
                }
                else
                {
                    break;
                }
            }
        }
    }
}

float receivedAngle = 0.0;

void receive_float()
{
    uint8_t buffer[4];                                   // 4 bytes for a float
    HAL_UART_Receive(&huart2, buffer, 4, HAL_MAX_DELAY); // Wait until data is received

    // Convert received bytes into a float
    memcpy(&receivedAngle, buffer, sizeof(receivedAngle));

    // Print the received angle for debugging
    char msg[50];
    sprintf(msg, "Received angle: %.2f\n", receivedAngle);
    HAL_UART_Transmit(&huart2, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

void stepper_drive(int step)
{
    switch (step)
    {

    case 0:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // A
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        break;

    case 1:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // AB
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(1);
        break;

    case 2:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // B
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(1);
        break;

    case 3:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // BC
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(1);
        break;

    case 4:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // C
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
        HAL_Delay(1);
        break;

    case 5:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET); // CD
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(1);
        break;

    case 6:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // D
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(1);
        break;

    case 7:
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET); // DA
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
        HAL_Delay(1);
        break;
    }
}