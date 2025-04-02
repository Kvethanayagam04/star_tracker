/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef hlpuart1;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* Initialization functions */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_UCPD1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_SPI1_Init(void);

/*SPI FUNCTIONS */
static uint8_t SPI_Read_Register(uint8_t reg);
static void SPI_Write_Register(uint8_t reg, uint8_t value);
static void SPI_Read_AngularRate(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z);

/* Motor movement functions */
int stepper_step_angle(float angle, int next_step, int motor);
static void stepper2_drive(int step);
static void stepper1_drive(int step);

/* Time/Date calculation Functions */
static double getJulianDate();
static double getGST();
static double getLST(double longitude);
static double getHourAngle(double RA, double longitude);

/* Interrupt Callback Function */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);

/* USER CODE BEGIN PFP */
/* USER CODE END Includes */
static float filtered_x = 0.0f;
static float filtered_y = 0.0f;
static float filtered_z = 0.0f;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */

/*SPI definitions for GYRO */
#define BUFFER_SIZE 64 // Define a suitable buffer size
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D
#define SENSITIVITY_245DPS 8.75 // Scale factor (mg/dps)

#define SPI_TIMEOUT 100
#define ALPHA 0.1

#define CS_LOW() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET)

/*Buffer for python interrupt data */
char rxBuffer[BUFFER_SIZE]; // Buffer for storing received command
uint8_t rxChar;             // Variable for receiving single characters
uint8_t counter = 0;

/* global variables to keep track of motor angles */
int next_step1 = 0;
int next_step2 = 0;
float angle1 = 0.0f, angle2 = 0.0f;
volatile uint8_t motor_move_requested1 = 0, motor_move_requested2 = 0;
float current_angle1 = 0.0f, current_angle2 = 0.0f; // initialize motor angles to origin
double cur_long = -79.4099712;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == LPUART1)
    { // Check if the interrupt is from LPUART1
        if (rxChar == '\r' || rxChar == '\n')
        {                             // If Enter is pressed
            rxBuffer[counter] = '\0'; // Null-terminate the string

            // Parse the received string
            if (sscanf(rxBuffer, "%f,%f", &angle1, &angle2) == 2)
            {                              // Ensure two floats were read
                motor_move_requested1 = 1; // Set flag
                motor_move_requested2 = 1;
                char response[BUFFER_SIZE];
                char response2[50];
                snprintf(response, BUFFER_SIZE, "\r\nParsed Values: %.2f, %.2f\r\n", angle1, angle2);
                double local_side_time = getLST(cur_long);
                angle1 = getHourAngle(angle1 / 15, cur_long) * 15;
                // Send parsed values back over UART
                HAL_UART_Transmit(&hlpuart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
                sprintf(response2, "LST: %.6f hours\r\n", local_side_time);
                HAL_UART_Transmit(&hlpuart1, (uint8_t *)response2, strlen(response2), HAL_MAX_DELAY);
            }
            else
            {
                // Error in parsing
                char error_msg[] = "\r\nParsing Error! Ensure format: float,float\r\n";
                HAL_UART_Transmit(&hlpuart1, (uint8_t *)error_msg, sizeof(error_msg) - 1, HAL_MAX_DELAY);
            }

            counter = 0; // Reset buffer for the next input
        }
        else
        {
            if (counter < BUFFER_SIZE - 1)
            { // Prevent buffer overflow
                rxBuffer[counter++] = rxChar;
            }
        }
        HAL_UART_Receive_IT(&hlpuart1, &rxChar, 1); // Restart UART reception
    }
}

int main(void)
{

    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_ICACHE_Init();
    MX_LPUART1_UART_Init();
    MX_RTC_Init();
    MX_UCPD1_Init();
    MX_USB_PCD_Init();
    MX_SPI1_Init();
    /* USER CODE BEGIN 2 */

    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    /* USER CODE END WHILE */
    char msg[] = "Waiting for input... (Press Enter to send)\r\n";
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);

    // Enable UART interrupt reception (VERY IMPORTANT)
    HAL_UART_Receive_IT(&hlpuart1, &rxChar, 1);

    /* USER CODE BEGIN 3 */
    while (1)
    {

        HAL_Delay(1000);
        if (motor_move_requested1 && angle1 != current_angle1)
        {
            next_step1 = stepper_step_angle(angle1, next_step1, 1);
            motor_move_requested1 = 0; // Reset flag
        }
        if (motor_move_requested2 && angle2 != current_angle2)
        {
            next_step2 = stepper_step_angle(angle2, next_step2, 2);
            motor_move_requested2 = 0; // Reset flag
        }
        // Add other logic here
    }
}

double getJulianDate()
{
    RTC_DateTypeDef sDate;
    RTC_TimeTypeDef sTime;
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    int Y = sDate.Year + 2000; // Adjust for 21st century
    int M = sDate.Month;
    int D = sDate.Date;
    double H;

    // Handle 12-hour format conversion to 24-hour format
    if (hrtc.Init.HourFormat == RTC_HOURFORMAT_12)
    {
        if (sTime.TimeFormat == RTC_HOURFORMAT12_PM)
        {
            // PM: 12 PM remains 12, others add 12
            H = (sTime.Hours == 12) ? 12 : sTime.Hours + 12;
        }
        else
        {
            // AM: 12 AM becomes 0, others remain the same
            H = (sTime.Hours == 12) ? 0 : sTime.Hours;
        }
    }
    else
    {
        // 24-hour format
        H = sTime.Hours;
    }

    // Convert minutes and seconds to fractional hours
    H += sTime.Minutes / 60.0 + sTime.Seconds / 3600.0;

    // Adjust year and month for January/February
    if (M <= 2)
    {
        Y -= 1;
        M += 12;
    }

    // Calculate Gregorian calendar correction terms
    int A = Y / 100;
    int B = 2 - A + (A / 4);

    // Compute Julian Date
    double JD = floor(365.25 * (Y + 4716)) + floor(30.6001 * (M + 1)) + D + H / 24.0 + B - 1524.5;
    return JD;
}

// Function to compute GST from Julian Date
double getGST()
{
    double JD = getJulianDate();
    double D = JD - 2451545.0;
    double T = D / 36525;
    double GST = 18.697374558 + 24.06570982441908 * D;
    GST = fmod(GST, 24.0);
    if (GST < 0)
    {
        GST += 24.0; // Ensure GST is in 0-24 range
    }
    return GST;
}

// Function to compute LST from GST and longitude (in degrees)
double getLST(double longitude)
{
    double GST = getGST();
    double LST = GST + longitude / 15.0;
    LST = fmod(LST, 24.0);
    if (LST < 0)
    {
        LST += 24.0; // Ensure LST is in 0-24 range
    }
    return LST;
}

// Function to compute Hour Angle (HA) from RA
double getHourAngle(double RA, double longitude)
{
    double LST = getLST(longitude);
    double HA = LST - RA;
    //    HA = fmod(HA + 24.0, 24.0); // Ensure HA is in 0-24 range
    return HA;
}
/* USER CODE END 3 */
// SPI Read Register
uint8_t SPI_Read_Register(uint8_t reg)
{
    uint8_t txData = reg | 0x80; // Set MSB to indicate Read
    uint8_t rxData = 0;
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, &txData, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, &rxData, 1, SPI_TIMEOUT);
    CS_HIGH();
    return rxData;
}

void SPI_Write_Register(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    CS_LOW();
    HAL_SPI_Transmit(&hspi1, data, 2, SPI_TIMEOUT);
    CS_HIGH();
}
// Read X, Y, Z values (16-bit signed integers)
void SPI_Read_AngularRate(int16_t *gyro_x, int16_t *gyro_y, int16_t *gyro_z)
{
    uint8_t rawData[6];

    CS_LOW();
    uint8_t txData = OUT_X_L | 0xC0; // Read multiple (MSB=1, auto-increment=1)
    HAL_SPI_Transmit(&hspi1, &txData, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(&hspi1, rawData, 6, SPI_TIMEOUT);
    CS_HIGH();

    // Convert bytes to signed 16-bit values
    *gyro_x = (int16_t)((rawData[1] << 8) | rawData[0]);
    *gyro_y = (int16_t)((rawData[3] << 8) | rawData[2]);
    *gyro_z = (int16_t)((rawData[5] << 8) | rawData[4]);
}
int stepper_step_angle(float angle, int next_step, int motor) // direction-> 0 for CW, 1 for CCW
{
    float angleperstep = (motor == 1) ? 0.087890625 / 4.66666667 : 0.087890625;        // 360 = 4096 sequences
    float angle_sign = (motor == 1) ? current_angle1 - angle : current_angle2 - angle; // Assuming current_angle1 is the initial angle
    // if angle_sign > 0 -> need to turn CW (0)
    // if angle_sign < 0 -> need to turn CCW (1)

    int direction = (angle_sign > 0) ? 0 : 1;
    float angle_to_turn = fabsf(angle_sign);

    while (angle_to_turn > 0)
    {
        if (direction == 0) // for clockwise
        {
            for (int step = next_step; step <= 7; step++)
            {
                if (angle_to_turn > 0)
                {
                    angle_to_turn -= angleperstep;
                    if (motor == 1)
                    {
                        stepper1_drive(step);
                        current_angle1 -= angleperstep;
                        if (step == 7)
                        {
                            next_step = 0;
                        }
                        else
                        {
                            next_step = step + 1;
                        }
                    }
                    else if (motor == 2)
                    {
                        stepper2_drive(step);
                        current_angle2 -= angleperstep;
                        if (step == 7)
                        {
                            next_step = 0;
                        }
                        else
                        {
                            next_step = step + 1;
                        }
                    }
                }
                else
                {
                    break;
                }
            }
        }
        else if (direction == 1) // for anti-clockwise
        {
            for (int step = next_step; step >= 0; step--)
            {
                if (angle_to_turn > 0)
                {
                    angle_to_turn -= angleperstep;
                    if (motor == 1)
                    {
                        stepper1_drive(step);
                        current_angle1 += angleperstep;
                        if (step == 0)
                        {
                            next_step = 7;
                        }
                        else
                        {
                            next_step = step - 1;
                        }
                    }
                    if (motor == 2)
                    {
                        stepper2_drive(step);
                        current_angle2 += angleperstep;
                        if (step == 0)
                        {
                            next_step = 7;
                        }
                        else
                        {
                            next_step = step - 1;
                        }
                    }
                }
                else
                {
                    break;
                }
            }
        }
    }
    return next_step;
}

void stepper2_drive(int step)
{
    // Define the GPIO pin states for each step
    const int gpio_states[8][4] = {
        {1, 0, 0, 0}, // Step 0: A
        {1, 1, 0, 0}, // Step 1: AB
        {0, 1, 0, 0}, // Step 2: B
        {0, 1, 1, 0}, // Step 3: BC
        {0, 0, 1, 0}, // Step 4: C
        {0, 0, 1, 1}, // Step 5: CD
        {0, 0, 0, 1}, // Step 6: D
        {1, 0, 0, 1}  // Step 7: DA
    };

    // Set GPIO pins based on the current step
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, gpio_states[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, gpio_states[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, gpio_states[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, gpio_states[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_Delay(1); // Delay for step timing
}

void stepper1_drive(int step)
{
    // Define the GPIO pin states for each step
    const int gpio_states[8][4] = {
        {1, 0, 0, 0}, // Step 0: A
        {1, 1, 0, 0}, // Step 1: AB
        {0, 1, 0, 0}, // Step 2: B
        {0, 1, 1, 0}, // Step 3: BC
        {0, 0, 1, 0}, // Step 4: C
        {0, 0, 1, 1}, // Step 5: CD
        {0, 0, 0, 1}, // Step 6: D
        {1, 0, 0, 1}  // Step 7: DA
    };

    // Set GPIO pins based on the current step
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, gpio_states[step][0] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, gpio_states[step][1] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, gpio_states[step][2] ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, gpio_states[step][3] ? GPIO_PIN_SET : GPIO_PIN_RESET);

    HAL_Delay(1); // Delay for step timing
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Configure the main internal regulator output voltage
     */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE0) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure LSE Drive Capability
     */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

    /** Initializes the RCC Oscillators according to the specified parameters
     * in the RCC_OscInitTypeDef structure.
     */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSE | RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 55;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    /** Initializes the CPU, AHB and APB buses clocks
     */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
    {
        Error_Handler();
    }
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void)
{

    /* USER CODE BEGIN ADC1_Init 0 */

    /* USER CODE END ADC1_Init 0 */

    ADC_MultiModeTypeDef multimode = {0};
    ADC_ChannelConfTypeDef sConfig = {0};

    /* USER CODE BEGIN ADC1_Init 1 */

    /* USER CODE END ADC1_Init 1 */

    /** Common config
     */
    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.OversamplingMode = DISABLE;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure the ADC multi-mode
     */
    multimode.Mode = ADC_MODE_INDEPENDENT;
    if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
    {
        Error_Handler();
    }

    /** Configure Regular Channel
     */
    sConfig.Channel = ADC_CHANNEL_3;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
    sConfig.SingleDiff = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ADC1_Init 2 */

    /* USER CODE END ADC1_Init 2 */
}

/**
 * @brief ICACHE Initialization Function
 * @param None
 * @retval None
 */
static void MX_ICACHE_Init(void)
{

    /* USER CODE BEGIN ICACHE_Init 0 */

    /* USER CODE END ICACHE_Init 0 */

    /* USER CODE BEGIN ICACHE_Init 1 */

    /* USER CODE END ICACHE_Init 1 */

    /** Enable instruction cache in 1-way (direct mapped cache)
     */
    if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_ICACHE_Enable() != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN ICACHE_Init 2 */

    /* USER CODE END ICACHE_Init 2 */
}

/**
 * @brief LPUART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_LPUART1_UART_Init(void)
{

    /* USER CODE BEGIN LPUART1_Init 0 */

    /* USER CODE END LPUART1_Init 0 */

    /* USER CODE BEGIN LPUART1_Init 1 */

    /* USER CODE END LPUART1_Init 1 */
    hlpuart1.Instance = LPUART1;
    hlpuart1.Init.BaudRate = 115200;
    hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
    hlpuart1.Init.StopBits = UART_STOPBITS_1;
    hlpuart1.Init.Parity = UART_PARITY_NONE;
    hlpuart1.Init.Mode = UART_MODE_TX_RX;
    hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
    if (HAL_UART_Init(&hlpuart1) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN LPUART1_Init 2 */

    /* USER CODE END LPUART1_Init 2 */
}

/**
 * @brief RTC Initialization Function
 * @param None
 * @retval None
 */
static void MX_RTC_Init(void)
{

    /* USER CODE BEGIN RTC_Init 0 */

    /* USER CODE END RTC_Init 0 */

    RTC_PrivilegeStateTypeDef privilegeState = {0};
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};

    /* USER CODE BEGIN RTC_Init 1 */

    /* USER CODE END RTC_Init 1 */

    /** Initialize RTC Only
     */
    hrtc.Instance = RTC;
    hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
    hrtc.Init.AsynchPrediv = 127;
    hrtc.Init.SynchPrediv = 255;
    hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
    hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
    hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
    hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
    hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
    if (HAL_RTC_Init(&hrtc) != HAL_OK)
    {
        Error_Handler();
    }
    privilegeState.rtcPrivilegeFull = RTC_PRIVILEGE_FULL_NO;
    privilegeState.backupRegisterPrivZone = RTC_PRIVILEGE_BKUP_ZONE_NONE;
    privilegeState.backupRegisterStartZone2 = RTC_BKP_DR0;
    privilegeState.backupRegisterStartZone3 = RTC_BKP_DR0;
    if (HAL_RTCEx_PrivilegeModeSet(&hrtc, &privilegeState) != HAL_OK)
    {
        Error_Handler();
    }

    /* USER CODE BEGIN Check_RTC_BKUP */

    /* USER CODE END Check_RTC_BKUP */

    /** Initialize RTC and set the Time and Date
     */
    sTime.Hours = 0x2 + 0x4;
    sTime.Minutes = 0x52;
    sTime.Seconds = 0x0;
    sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
    sTime.StoreOperation = RTC_STOREOPERATION_RESET;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
    sDate.Month = RTC_MONTH_APRIL;
    sDate.Date = 0x2;
    sDate.Year = 0x25;

    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN RTC_Init 2 */

    /* USER CODE END RTC_Init 2 */
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void)
{

    /* USER CODE BEGIN SPI1_Init 0 */

    /* USER CODE END SPI1_Init 0 */

    /* USER CODE BEGIN SPI1_Init 1 */

    /* USER CODE END SPI1_Init 1 */
    /* SPI1 parameter configuration*/
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 7;
    hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
    hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN SPI1_Init 2 */

    /* USER CODE END SPI1_Init 2 */
}

/**
 * @brief UCPD1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UCPD1_Init(void)
{

    /* USER CODE BEGIN UCPD1_Init 0 */

    /* USER CODE END UCPD1_Init 0 */

    LL_GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* Peripheral clock enable */
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_UCPD1);

    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOB);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA);
    /**UCPD1 GPIO Configuration
    PB15   ------> UCPD1_CC2
    PA15 (JTDI)   ------> UCPD1_CC1
    */
    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    LL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USER CODE BEGIN UCPD1_Init 1 */

    /* USER CODE END UCPD1_Init 1 */
    /* USER CODE BEGIN UCPD1_Init 2 */

    /* USER CODE END UCPD1_Init 2 */
}

/**
 * @brief USB Initialization Function
 * @param None
 * @retval None
 */
static void MX_USB_PCD_Init(void)
{

    /* USER CODE BEGIN USB_Init 0 */

    /* USER CODE END USB_Init 0 */

    /* USER CODE BEGIN USB_Init 1 */

    /* USER CODE END USB_Init 1 */
    hpcd_USB_FS.Instance = USB;
    hpcd_USB_FS.Init.dev_endpoints = 8;
    hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
    hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
    hpcd_USB_FS.Init.Sof_enable = DISABLE;
    hpcd_USB_FS.Init.low_power_enable = DISABLE;
    hpcd_USB_FS.Init.lpm_enable = DISABLE;
    hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
    if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USB_Init 2 */

    /* USER CODE END USB_Init 2 */
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /* USER CODE BEGIN MX_GPIO_Init_1 */
    /* USER CODE END MX_GPIO_Init_1 */

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    HAL_PWREx_EnableVddIO2();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0 | GPIO_PIN_3 | LED_GREEN_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | LED_RED_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | UCPD_DBN_Pin | LED_BLUE_Pin, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pins : PC0 PC3 LED_GREEN_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_3 | LED_GREEN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pin : PC1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA2 PA3 PA4 LED_RED_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | LED_RED_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB0 UCPD_DBN_Pin LED_BLUE_Pin */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | UCPD_DBN_Pin | LED_BLUE_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : UCPD_FLT_Pin */
    GPIO_InitStruct.Pin = UCPD_FLT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(UCPD_FLT_GPIO_Port, &GPIO_InitStruct);

    /*Configure GPIO pins : PD4 PD5 PD6 PD7 */
    GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* USER CODE BEGIN MX_GPIO_Init_2 */
    /* USER CODE END MX_GPIO_Init_2 */
}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

/**
void testGetJulianDate()
{
    char response[50];  // Buffer to store output string
    double julianDate = getJulianDate(); // Call the function

    // Convert double to string
    sprintf(response, "Julian Date: %.6f\r\n", julianDate);

    // Send result over UART
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
}

void testGetGST()
{
    char response[50];  // Buffer for UART transmission
    double gst = getGST(); // Call function

    sprintf(response, "GST: %.6f hours\r\n", gst);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
}

void testGetLST(double longitude)
{
    char response[50];
    double lst = getLST(longitude);

    sprintf(response, "LST: %.6f hours\r\n", lst);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
}

void testGetHourAngle(double RA, double longitude)
{
    char response[50];
    double ha = getHourAngle(RA, longitude);

    sprintf(response, "Hour Angle: %.6f hours\r\n", ha);
    HAL_UART_Transmit(&hlpuart1, (uint8_t *)response, strlen(response), HAL_MAX_DELAY);
}

**/

#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line)
{
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
