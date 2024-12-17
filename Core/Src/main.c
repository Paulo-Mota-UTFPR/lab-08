/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"
#include "cmsis_os.h" // FreeRTOS includes
#include "fatfs.h" // FATFS support

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
osThreadId_t tempTaskHandle, controlTaskHandle, usbTaskHandle;
QueueHandle_t tempQueue;
int gUSBRxBuffer[255];

/* Function Prototypes -------------------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
void StartTempTask(void *argument);
void StartControlTask(void *argument);
void StartUsbTask(void *argument);

/* Main ----------------------------------------------------------------------*/
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_ADC1_Init();
    MX_USB_DEVICE_Init();
    MX_FATFS_Init();

    // Create the FreeRTOS queue
    tempQueue = xQueueCreate(10, sizeof(double));

    // Create FreeRTOS tasks
    tempTaskHandle = xTaskCreate(StartTempTask, "TempTask", 128, NULL, 2, NULL);
    controlTaskHandle = xTaskCreate(StartControlTask, "ControlTask", 128, NULL, 2, NULL);
    usbTaskHandle = xTaskCreate(StartUsbTask, "UsbTask", 128, NULL, 1, NULL);

    // Start the scheduler
    osKernelStart();

    // Should never reach here
    while (1) {}
}

/* Tasks ---------------------------------------------------------------------*/

// Task 1: Temperature reading and calculation
define NTC_REF_RESISTANCE 10000.0
define NTC_B_CONSTANT 3950.0
define NTC_REF_TEMP 298.15
define ADC_MAX_VALUE 4096.0

define SUPPLY_VOLTAGE 3.3
void StartTempTask(void *argument) {
    double tempCelsius;
    uint32_t adcValue;

    while (1) {
        HAL_ADC_Start(&hadc1);
        HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
        adcValue = HAL_ADC_GetValue(&hadc1);
        
        double voltage = (adcValue / ADC_MAX_VALUE) * SUPPLY_VOLTAGE;
        double resistance = NTC_REF_RESISTANCE * (SUPPLY_VOLTAGE / voltage - 1);
        double temperatureK = 1 / (1 / NTC_REF_TEMP + log(resistance / NTC_REF_RESISTANCE) / NTC_B_CONSTANT);
        tempCelsius = temperatureK - 273.15;

        xQueueSend(tempQueue, &tempCelsius, portMAX_DELAY);
        osDelay(1000);
    }
}

// Task 2: Control ON/OFF based on temperature
void StartControlTask(void *argument) {
    double tempCelsius;
    uint8_t lampState = 0;

    while (1) {
        if (xQueueReceive(tempQueue, &tempCelsius, portMAX_DELAY) == pdPASS) {
            if (tempCelsius > 32.0 && lampState == 0) {
                HAL_GPIO_WritePin(GPIOA, mosfetN_Pin, GPIO_PIN_SET);
                lampState = 1;
            } else if (tempCelsius < 28.0 && lampState == 1) {
                HAL_GPIO_WritePin(GPIOA, mosfetN_Pin, GPIO_PIN_RESET);
                lampState = 0;
            }
        }
        osDelay(100);
    }
}

// Task 3: USB communication and data logging
void StartUsbTask(void *argument) {
    double tempCelsius;
    char usbBuffer[255];
    FIL file;
    UINT bytesWritten;

    while (1) {
        if (gUSBRxBuffer[0] != 0) {
            switch (gUSBRxBuffer[0]) {
                case 'T': // Send temperature
                case 't':
                    if (xQueuePeek(tempQueue, &tempCelsius, 0) == pdPASS) {
                        sprintf(usbBuffer, "Temperature: %.2f\r\n", tempCelsius);
                        CDC_Transmit_FS((uint8_t *)usbBuffer, strlen(usbBuffer));
                    }
                    break;
                case 'S': // Save temperature to file
                case 's':
                    if (f_open(&file, "temperatures.txt", FA_WRITE | FA_OPEN_APPEND) == FR_OK) {
                        if (xQueuePeek(tempQueue, &tempCelsius, 0) == pdPASS) {
                            sprintf(usbBuffer, "%.2f\r\n", tempCelsius);
                            f_write(&file, usbBuffer, strlen(usbBuffer), &bytesWritten);
                        }
                        f_close(&file);
                    } else {
                        sprintf(usbBuffer, "Error saving data\r\n");
                        CDC_Transmit_FS((uint8_t *)usbBuffer, strlen(usbBuffer));
                    }
                    break;
                default:
                    sprintf(usbBuffer, "Invalid command\r\n");
                    CDC_Transmit_FS((uint8_t *)usbBuffer, strlen(usbBuffer));
                    break;
            }
        }
        memset(gUSBRxBuffer, 0, sizeof(gUSBRxBuffer));
        osDelay(500);
    }
}

/* Peripheral Initialization Functions ---------------------------------------*/
static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.ScanConvMode = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    HAL_ADC_Init(&hadc1);

    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = mosfetN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
