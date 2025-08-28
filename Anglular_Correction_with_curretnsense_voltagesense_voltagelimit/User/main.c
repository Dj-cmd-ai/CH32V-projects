#include "debug.h"

//====================GPIO designation and variables for Potentiometer input====================
#define ANALOG1_PIN GPIO_Pin_4
#define ANALOG1_PORT GPIOD
uint16_t adcReading1 = 0;
volatile uint8_t adcFlag = 1;

//=====================GPIO designations and variables for Current sensing=======================
#define ANALOG2_PIN GPIO_Pin_2
#define OVERCURRENT_THRESHOLD_COUNT 10  // Number of consecutive overcurrent readings required to trip
uint8_t overcurrentflag = 0;
uint16_t adcReading2 = 0;
uint32_t voltage_mv = 0;
uint32_t current_mA = 0;

//=====================GPIO designations and variables for AC Voltage sensing=======================
#define ANALOG3_PIN GPIO_Pin_3
#define VOLTAGE_THRESHOLD_COUNT_ 10 // Number of consecutive over-voltage readings
uint8_t voltageflag = 0;
uint16_t adcReading3 = 0;
uint32_t DCvoltageadc = 0;
uint32_t dcVoltageReading = 0;
uint16_t Cap_Voltage = 0;
uint16_t AC_Voltage = 0;
uint8_t Vref = 5;

// Voltage divider values
#define resistor_factor 91

/*Variable which decides the over-current limit*/
uint32_t currentthreshold = 0;

/*Variable to count over-current condition triggering */
uint8_t overcurrentCount = 0;  // Counter for consecutive overcurrent readings

/*Variable which decides the over and under voltage limit*/
uint32_t overvoltagethreshold = 270;
uint32_t undervoltagethreshold = 120;

/*Variable to count voltage crossing condition triggering*/
uint8_t voltageCount = 0;

/* Reference voltage and ADC max-value*/
#define VREF_MV 5000   // 5V reference in millivolts (5000 mV)
#define ADC_MAX 1023   // 10-bit ADC max value

/*Averaging variables for current sensing*/
#define MOVING_AVERAGE1_SIZE 10
uint32_t adcHistory[MOVING_AVERAGE1_SIZE] = {0};
uint32_t adcIndex = 0;
uint32_t adcSum = 0;
uint32_t averageadcReading2Value = 0;

// ===================== VOLTAGE AVERAGING VARIABLES =====================
#define VOLTAGE_AVG_SIZE 10
uint32_t voltageHistory[VOLTAGE_AVG_SIZE] = {0};
uint8_t voltageIndex = 0;
uint32_t voltageSum = 0;
uint32_t averageDCvoltageAdc = 0;


/*Buffer for using multiple ADC*/
uint16_t ADCBuffer[3];

/* PWM Timer settings */
#define PWM_ARR 20000
#define PWM_PSC 2-1

//====================Max Auto-reload-register value which decides the frequency of PWM====================
#define TIMER_MAX 20000

// ===================== HALL COMPENSATION CONFIGURATION =====================
// Set to 1 to enable hall position compensation, 0 to disable
#define HALL_COMPENSATION_ENABLED 1

// Fixed delay method settings
#define HALL_DELAY_MICROSECONDS 0   // Adjust this value (0-2000 us typical)

// ===================== TIMER-BASED PRINTING CONFIGURATION =====================
// Timer-based printing variables
volatile uint32_t system_ms_counter = 0;  // Millisecond counter
uint32_t last_print_time = 0;
#define PRINT_INTERVAL_MS 250  // 1000ms = 1 second

// ============================================================================

//=====================Potentiometer scaled PWM VARIABLE=======================
uint32_t scaledValue = 0;

//===========================Interrupt function variables======================
uint8_t interrupt_flag_1 = 0; // Keep for compatibility
uint8_t interrupt_flag_2 = 0; // Keep for compatibility

//===========================Timer function variable declaration================
TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
TIM_OCInitTypeDef TIM_OCInitStructure = {0};

/* PWM Output Mode Definition */
#define PWM_MODE1   0
#define PWM_MODE2   1
#define PWM_MODE PWM_MODE2

//=============================Predefined void functions========================
void ADCflag(void);
void TIM2_SetPWM1(uint16_t ccp);
void TIM2_SetPWM2(uint16_t ccp);

// ===================== SIMPLE FIXED DELAY COMPENSATION =====================
// Use simple delay loops for fixed compensation
void simple_delay_us(uint32_t delay_us)
{
    // Simple delay loop calibrated for 48MHz system clock
    // Each loop iteration takes approximately 4 cycles
    uint32_t cycles = delay_us * 12; // 48MHz / 4 cycles per loop

    for (volatile uint32_t i = 0; i < cycles; i++)
    {
        __asm("nop");
    }
}

// Execute the actual commutation
void execute_commutation(uint8_t commutation_type)
{
    if (commutation_type == 1)
    {
        TIM2_SetPWM1(scaledValue);  // Update PWM on PC1
        TIM2_SetPWM2(0);            // Disable PWM on PC2
    }
    else if (commutation_type == 2)
    {
        TIM2_SetPWM2(scaledValue);  // Update PWM on PC2
        TIM2_SetPWM1(0);            // Disable PWM on PC1
    }
}

// Start compensation with fixed delay
void start_compensation_timer(uint8_t commutation_type)
{
    if (!HALL_COMPENSATION_ENABLED)
    {
        // No compensation, execute immediately
        execute_commutation(commutation_type);
        return;
    }

    // Apply fixed delay if compensation is enabled
    if (HALL_DELAY_MICROSECONDS > 0)
    {
        simple_delay_us(HALL_DELAY_MICROSECONDS);
    }

    // Execute commutation after delay
    execute_commutation(commutation_type);
}

//======================Current sensing function=================
void current_calculation(void)
{
    // *** ADC Moving Average Calculation ***
    adcSum -= adcHistory[adcIndex];  // Remove the oldest value
    adcHistory[adcIndex] = adcReading2;  // Add new value
    adcSum += adcReading2;  // Update sum
    adcIndex = (adcIndex + 1) % MOVING_AVERAGE1_SIZE;  // Circular buffer
    averageadcReading2Value = adcSum / MOVING_AVERAGE1_SIZE;  // Compute average
    // *** Convert ADC to Voltage (Integer Math) ***
    voltage_mv = (averageadcReading2Value * VREF_MV) / ADC_MAX; // Voltage in millivolts
    current_mA = (voltage_mv * 10) / 28;  //for 56 gain :: the divisor would be 28 for 0.05ohm shunt and 17 for 0.03ohm shunt
}

//======================Voltage sensing function=================
void voltage_calculation(void)
{
    // Apply moving average filter to ADC reading
    voltageSum -= voltageHistory[voltageIndex];     // Remove oldest reading
    voltageHistory[voltageIndex] = adcReading3;     // Store new reading
    voltageSum += adcReading3;                      // Add to sum
    voltageIndex = (voltageIndex + 1) % VOLTAGE_AVG_SIZE;  // Update index

    // Calculate average
    averageDCvoltageAdc = voltageSum / VOLTAGE_AVG_SIZE;

    // Use the averaged value instead of raw reading
    DCvoltageadc = averageDCvoltageAdc;

    // Calculate DC and AC voltages
    dcVoltageReading = (DCvoltageadc * Vref * 1000) / 1023;
    Cap_Voltage = (dcVoltageReading * resistor_factor) / 1000;
    AC_Voltage = (Cap_Voltage * 1000) / 1414;
}

// ===================== TIMER-BASED PRINTING FUNCTIONS =====================
// TIM2 overflow interrupt handler for millisecond timing
void TIM2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM2_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

        // Increment millisecond counter (approximate)
        // TIM2 overflows every: 7000 / (48MHz/2) = ~0.29ms
        // For better accuracy, we can use sub-millisecond counting
        static uint16_t sub_ms_counter = 0;
        sub_ms_counter += 292;  // Approximately 0.292ms per TIM2 overflow
        if (sub_ms_counter >= 1000) {
            system_ms_counter++;
            sub_ms_counter -= 1000;
        }
    }
}

// Print comprehensive system status
void print_current_status(void)
{
//    printf("\r\n=== System Status [%ld ms] ===\r\n", system_ms_counter);
//    printf("Current: %ld mA (Threshold: %ld mA)\r\n", current_mA, currentthreshold);
   printf("AC_voltage:- %d \r\n",AC_Voltage );
//    printf("Voltage: %ld mV\r\n", voltage_mv);
//    printf("ADCpot: %d \r\n", adcReading1);
//    printf("ADC Current Raw: %d, Average: %ld\r\n", adcReading2, averageadcReading2Value);
//    printf("Potentiometer: %d -> PWM: %ld\r\n", adcReading1, scaledValue);
//    printf("Overcurrent: %s (Count: %d/%d)\r\n",
//           overcurrentflag ? "ACTIVE" : "Normal",
//           overcurrentCount, OVERCURRENT_THRESHOLD_COUNT);
//    printf("Hall Compensation: %s (%d us)\r\n",
//           HALL_COMPENSATION_ENABLED ? "ON" : "OFF",
//           HALL_DELAY_MICROSECONDS);
//    printf("System Runtime: %ld.%03ld seconds\r\n",
//           system_ms_counter/1000, system_ms_counter%1000);
//    printf("=====================================\r\n");
}

// ===================== ORIGINAL FUNCTIONS =====================

void EXTIO_INT_INIT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource3);
    EXTI_InitStructure.EXTI_Line = EXTI_Line3;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOC, GPIO_PinSource4);
    EXTI_InitStructure.EXTI_Line = EXTI_Line4;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI7_0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void TIM2_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = PWM_ARR;
    TIM_TimeBaseInitStructure.TIM_Prescaler = PWM_PSC;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_CtrlPWMOutputs(TIM2, ENABLE);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);  // Enable update interrupt

    // Configure NVIC for TIM2 interrupt (for millisecond timing)
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  // Lower priority than hall sensors
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIM_Cmd(TIM2, ENABLE);
}

void TIM2_SetPWM1(uint16_t ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#if (PWM_MODE == PWM_MODE1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
#elif (PWM_MODE == PWM_MODE2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC4Init(TIM2, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Disable);
}

void TIM2_SetPWM2(uint16_t ccp)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_PinRemapConfig(GPIO_PartialRemap1_TIM2, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

#if (PWM_MODE == PWM_MODE1)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
#elif (PWM_MODE == PWM_MODE2)
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
#endif

    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = ccp;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Disable);
}

void ADCConfig(void)
{
    ADC_InitTypeDef ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = ANALOG1_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ANALOG1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ANALOG2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ANALOG1_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = ANALOG3_PIN;  // Added third ADC channel
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ANALOG1_PORT, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; //for multi-channel access
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_241Cycles);  // PD3 - Voltage sensing+

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while (ADC_GetResetCalibrationStatus(ADC1));

    ADC_StartCalibration(ADC1);
    while (ADC_GetCalibrationStatus(ADC1));
}

void DMA_Tx_Init(DMA_Channel_TypeDef *DMA_CHx, uint32_t peripheralAddress, uint32_t memoryAddress, uint16_t bufferSize)
{
    DMA_InitTypeDef DMA_InitStructure = {0};

    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    DMA_DeInit(DMA_CHx);
    DMA_InitStructure.DMA_PeripheralBaseAddr = peripheralAddress;
    DMA_InitStructure.DMA_MemoryBaseAddr = memoryAddress;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
    DMA_InitStructure.DMA_BufferSize = bufferSize;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA_CHx, &DMA_InitStructure);
}

void ADCflag(void)
{
    adcReading1 = ADCBuffer[0];
    adcReading2 = ADCBuffer[1];
    adcReading3 = ADCBuffer[2];

    // Apply offset correction
    if (adcReading1 <= 25) {  // Ignore readings less than 25
        adcReading1 = 0;
        overcurrentflag = 0;
        // voltageflag = 0;
    }

    if (adcReading1 < 100)
    {
        currentthreshold = 2000;
    }
    else if (adcReading1 > 100)
    {
        currentthreshold = 2000;
    }

    // Modified overcurrent detection logic to require consecutive readings
    if (current_mA > currentthreshold)
    {
        overcurrentCount++;

        // Only set the overcurrent flag after consecutive readings above threshold
        if (overcurrentCount >= OVERCURRENT_THRESHOLD_COUNT)
        {
            overcurrentflag = 1;
        }
    }
    else if (current_mA < currentthreshold)
    {
        // Reset the counter if current is below threshold
        overcurrentCount = 0;
    }

    if (AC_Voltage > overvoltagethreshold || AC_Voltage < undervoltagethreshold)
    {
        voltageCount++;

        //Only set the voltage flag after consecutive readings
        if (voltageCount >= VOLTAGE_THRESHOLD_COUNT_)
        {
            voltageflag = 1;
        }
    }
    else if (AC_Voltage <= overvoltagethreshold && AC_Voltage >= undervoltagethreshold)
    {
        voltageCount = 0;
        voltageflag = 0;
    }

    if (overcurrentflag == 1 || voltageflag == 1)
    {
        scaledValue = 0;
    }
    else if (overcurrentflag == 0 && voltageflag == 0)
    {
        // Scale adcReading to 0 - 30% of TIMER_MAX
        scaledValue = (adcReading1 * TIMER_MAX * 50) / (1023 * 100);
    }
}

void initial_condition_INIT(void)
{
    if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3) == RESET)
    {
        start_compensation_timer(1); // Start compensation for PWM1
    }
    else if (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4) == RESET)
    {
        start_compensation_timer(2); // Start compensation for PWM2
    }
}

// Modified interrupt handler with fixed compensation
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        // Trigger fixed compensation for PWM1
        start_compensation_timer(1);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
    else if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        // Trigger fixed compensation for PWM2
        start_compensation_timer(2);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

// ===================== CALIBRATION AND DEBUG FUNCTIONS =====================

void print_compensation_status(void)
{
    printf("=== Hall Fixed Compensation Status ===\r\n");
    printf("Compensation Enabled: %s\r\n", HALL_COMPENSATION_ENABLED ? "YES" : "NO");
    printf("Fixed Delay: %d microseconds\r\n", HALL_DELAY_MICROSECONDS);
    printf("Print Interval: %d ms\r\n", PRINT_INTERVAL_MS);
    printf("======================================\r\n");
}

// Test function to manually trigger compensation
void test_compensation(uint8_t commutation_type)
{
    printf("Testing fixed compensation for commutation type %d\r\n", commutation_type);
    start_compensation_timer(commutation_type);
}

// ===================== MAIN FUNCTION =====================

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("Initializing Motor Control with Fixed Hall Compensation...\r\n");

    // Initialize all peripherals
    ADCConfig();
    EXTIO_INT_INIT();
    TIM2_Init();  // This now includes timer interrupt setup
    DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)ADCBuffer, 3);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    // Print initial configuration
    print_compensation_status();

    printf("Motor control system ready!\r\n");
    printf("Hall sensors on PC3 and PC4\r\n");
    printf("PWM outputs on PC1 and PC2\r\n");
    printf("Speed control via ADC on PD4\r\n");
    printf("Current sensing on PD2\r\n");
    printf("Status will be printed every %d ms\r\n", PRINT_INTERVAL_MS);
    printf("Starting main control loop...\r\n\n");

    while (1)
    {
        initial_condition_INIT();
        ADCflag(); // Update ADC reading
        current_calculation();
        voltage_calculation();

        // Timer-based printing every second (precise timing)
        if ((system_ms_counter - last_print_time) >= PRINT_INTERVAL_MS)
        {
            print_current_status();
            last_print_time = system_ms_counter;
        }

        // Handle any pending interrupt flags (for compatibility)
        if (interrupt_flag_1)
        {
            interrupt_flag_1 = 0;
            start_compensation_timer(1);
        }
        else if (interrupt_flag_2)
        {
            interrupt_flag_2 = 0;
            start_compensation_timer(2);
        }

        // Small delay to prevent excessive polling
        for (volatile int i = 0; i < 1000; i++);
    }
}
