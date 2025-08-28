/*
 * Capacitive Touch Sensor - CH32V203C6T6
 * RC Time Constant based touch detection using spring
 * Serial monitor output with INTEGER ONLY arithmetic
 * NOTE: Touch DECREASES the reading value (adds capacitance)
 *
 * Hardware Connections:
 * PA3 -> Spring (one end) + 10k¦¸ resistor
 * Spring acts as capacitive sensor - when touched, capacitance increases, reading decreases
 */

#include "debug.h"
#include "ch32v20x.h"

#define TOUCH_PIN       GPIO_Pin_3    // PA3
#define TOUCH_PORT      GPIOA
#define TOUCH_RCC       RCC_APB2Periph_GPIOA

// Touch sensing parameters - INTEGER ONLY
#define SAMPLE_COUNT    10           // Number of samples to average
#define CALIBRATION_COUNT 50         // Samples for baseline calibration
#define TOUCH_THRESHOLD_PERCENT 20  // Touch threshold (20% BELOW baseline) - INTEGER ONLY

// Global variables
volatile uint32_t baseline_value = 0;
volatile uint32_t touch_threshold = 0;
volatile uint8_t touch_detected = 0;
volatile uint32_t current_reading = 0;

// Function prototypes
void GPIO_Config(void);
void Touch_Calibrate(void);
uint32_t Read_Touch_Value(void);
uint32_t Measure_RC_Time(void);
uint8_t Is_Touch_Detected(void);
void Touch_Process(void);

/*********************************************************************
 * @fn      main
 * @brief   Main program.
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("CH32V203C6T6 - Capacitive Touch Sensor Test\r\n");
    printf("Spring connected to PA3 with 10k resistor\r\n");
    printf("INTEGER ONLY arithmetic - No floating point\r\n");
    printf("NOTE: Touch DECREASES reading (threshold is below baseline)\r\n\r\n");

    // Initialize GPIO
    GPIO_Config();

    printf("Initializing touch sensor...\r\n");
    Delay_Ms(1000);

    // Calibrate baseline (no touch)
    printf("Calibrating baseline... Please don't touch!\r\n");
    Touch_Calibrate();

    printf("Calibration complete!\r\n");
    printf("Baseline: %d\r\n", baseline_value);
    printf("Threshold: %d (baseline - %d%% because touch DECREASES reading)\r\n", touch_threshold, TOUCH_THRESHOLD_PERCENT);
    printf("\r\nReady for touch detection!\r\n");
    printf("Touch the spring to test...\r\n\r\n");

    while(1)
    {
        Touch_Process();
        Delay_Ms(100);  // 10Hz sampling rate for better serial readability
    }
}

/*********************************************************************
 * @fn      GPIO_Config
 * @brief   Configure GPIO for touch sensing
 * @return  none
 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(TOUCH_RCC, ENABLE);

    // Configure PA3 as input (will be switched between input/output)
    GPIO_InitStructure.GPIO_Pin = TOUCH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      Touch_Calibrate
 * @brief   Calibrate baseline touch value (no touch condition)
 * @return  none
 */
void Touch_Calibrate(void)
{
    uint32_t total = 0;
    uint32_t reading;

    // Take multiple samples for stable baseline
    for(int i = 0; i < CALIBRATION_COUNT; i++)
    {
        reading = Read_Touch_Value();
        total += reading;

        printf("Calibration sample %d: %d\r\n", i+1, reading);
        Delay_Ms(20);
    }

    // Calculate baseline average
    baseline_value = total / CALIBRATION_COUNT;

    // Set touch threshold using INTEGER ONLY math
    // Since touch DECREASES reading, threshold is BELOW baseline
    // Example: 20% below baseline = baseline - (baseline * 20 / 100)
    touch_threshold = baseline_value - (baseline_value * TOUCH_THRESHOLD_PERCENT / 100);
}

/*********************************************************************
 * @fn      Read_Touch_Value
 * @brief   Read averaged touch sensor value
 * @return  Averaged touch sensor reading
 */
uint32_t Read_Touch_Value(void)
{
    uint32_t total = 0;

    // Take multiple samples and average
    for(int i = 0; i < SAMPLE_COUNT; i++)
    {
        total += Measure_RC_Time();
        Delay_Us(100);  // Small delay between samples
    }

    return total / SAMPLE_COUNT;
}

/*********************************************************************
 * @fn      Measure_RC_Time
 * @brief   Measure RC time constant for capacitive sensing
 * @return  Time measurement (in arbitrary units)
 */
uint32_t Measure_RC_Time(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    uint32_t timeout_counter = 0;
    const uint32_t MAX_COUNT = 10000;  // Prevent infinite loop

    // Step 1: Set pin as output LOW to discharge capacitor
    GPIO_InitStructure.GPIO_Pin = TOUCH_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);

    GPIO_ResetBits(TOUCH_PORT, TOUCH_PIN);  // Discharge
    Delay_Us(10);  // Ensure full discharge

    // Step 2: Set pin as input (high impedance)
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(TOUCH_PORT, &GPIO_InitStructure);

    // Step 3: Measure time for pin to go HIGH (charging through resistor)
    timeout_counter = 0;
    while(GPIO_ReadInputDataBit(TOUCH_PORT, TOUCH_PIN) == RESET)
    {
        timeout_counter++;
        if(timeout_counter >= MAX_COUNT)
        {
            break;  // Prevent infinite loop
        }
        __NOP();  // Small delay
    }

    return timeout_counter;
}

/*********************************************************************
 * @fn      Is_Touch_Detected
 * @brief   Check if touch is detected based on threshold
 * @return  1 if touch detected, 0 if not
 */
uint8_t Is_Touch_Detected(void)
{
    // Touch detected when reading is BELOW threshold (reading decreases on touch)
    return (current_reading < touch_threshold) ? 1 : 0;
}

/*********************************************************************
 * @fn      Touch_Process
 * @brief   Main touch processing function
 * @return  none
 */
void Touch_Process(void)
{
    static uint8_t last_touch_state = 0;

    // Read current touch value
    current_reading = Read_Touch_Value();

    // Detect touch state
    touch_detected = Is_Touch_Detected();

    // Print status with clear formatting
    printf("Reading: %5d | Baseline: %5d | Threshold: %5d | ",
           current_reading, baseline_value, touch_threshold);

    if(touch_detected && !last_touch_state)
    {
        printf("TOUCH DETECTED!\r\n");
    }
    else if(!touch_detected && last_touch_state)
    {
        printf("Touch Released\r\n");
    }
    else if(touch_detected)
    {
        printf("Touching...\r\n");
    }
    else
    {
        printf("No Touch\r\n");
    }

    last_touch_state = touch_detected;

    // Check for drift using INTEGER ONLY math
    // Check if reading is less than 50% of baseline (baseline / 2)
    // or greater than 200% of baseline (baseline * 2)
    if(current_reading < (baseline_value / 2) ||
       current_reading > (baseline_value * 2))
    {
        printf("WARNING: Reading drift detected. Consider recalibration.\r\n");
    }
}
