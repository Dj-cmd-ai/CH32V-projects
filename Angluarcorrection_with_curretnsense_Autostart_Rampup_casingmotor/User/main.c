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

/*Variable which decides the over-current limit*/
uint32_t currentthreshold = 0;

/*Variable to count over-current condition triggering */
uint8_t overcurrentCount = 0;  // Counter for consecutive overcurrent readings

/* Reference voltage and ADC max-value*/
#define VREF_MV 5000   // 5V reference in millivolts (5000 mV)
#define ADC_MAX 1023   // 10-bit ADC max value

/*Averaging variables for current sensing*/
#define MOVING_AVERAGE1_SIZE 10
uint32_t adcHistory[MOVING_AVERAGE1_SIZE] = {0};
uint32_t adcIndex = 0;
uint32_t adcSum = 0;
uint32_t averageadcReading2Value = 0;

/*Buffer for using multiple ADC*/
uint16_t ADCBuffer[2];

/* PWM Timer settings */
#define PWM_ARR 20000
#define PWM_PSC 2-1

//====================Max Auto-reload-register value which decides the frequency of PWM====================
#define TIMER_MAX 20000

// ===================== HALL COMPENSATION CONFIGURATION =====================
// Set to 1 to enable hall position compensation, 0 to disable
#define HALL_COMPENSATION_ENABLED 1

// Fixed delay method settings
#define HALL_DELAY_MICROSECONDS 0  // Adjust this value (0-2000 us typical)

// ===================== TIMER-BASED PRINTING CONFIGURATION =====================
// Timer-based printing variables
volatile uint32_t system_ms_counter = 0;  // Millisecond counter
uint32_t last_print_time = 0;
#define PRINT_INTERVAL_MS 1000  // 1000ms = 1 second

// ===================== CURRENT-BASED STARTUP DETECTION CONFIGURATION =====================
#define STARTUP_DETECTION_ENABLED 1         // Enable/disable startup detection
#define STARTUP_TEST_PWM_VALUE 12           // Low PWM value for testing rotor lock (very low)
#define STARTUP_LOCK_CURRENT_THRESHOLD 500  // Current threshold to detect locked rotor (mA)
#define STARTUP_LOCK_COUNT_THRESHOLD 5      // Number of consecutive high current readings to confirm lock
#define STARTUP_RETRY_DELAY_MS 500          // Delay between retry attempts
#define STARTUP_TEST_DURATION_MS 1000        // How long to test with low PWM before checking current
#define STARTUP_MAX_RETRIES 10              // Maximum number of retry attempts

// ===================== RAMP-UP CONFIGURATION =====================
#define RAMP_UP_ENABLED 1                   // Enable/disable ramp-up function
#define RAMP_UP_DURATION_MS 5000           // Total time to ramp from test PWM to target PWM (2 seconds)
#define RAMP_UPDATE_INTERVAL_MS 50         // How often to update ramp value (20 times per second)
#define RAMP_MIN_STEP 1                    // Minimum ramp step size
#define RAMP_MAX_STEP 50                   // Maximum ramp step size for fast ramping

// Startup state machine
typedef enum {
    STARTUP_IDLE,              // Normal operation or motor stopped
    STARTUP_TESTING,           // Applying test PWM and monitoring current
    STARTUP_LOCK_DETECTED,     // Rotor lock detected, preparing for retry
    STARTUP_RETRY_DELAY,       // Waiting before next retry attempt
    STARTUP_SUCCESS,           // Startup successful, transitioning to normal operation
    STARTUP_RAMPING,           // Ramping up from test PWM to target PWM
    STARTUP_FAILED             // Max retries exceeded
} startup_detection_state_t;

// Startup control variables
volatile startup_detection_state_t startup_state = STARTUP_IDLE;
volatile uint32_t startup_timer = 0;
volatile uint8_t startup_lock_count = 0;        // Count of consecutive high current readings
volatile uint8_t startup_retry_count = 0;       // Number of retry attempts
volatile uint8_t startup_triggered = 0;
volatile uint8_t last_pot_zero_state = 1;
volatile uint8_t startup_control_active = 0;    // Flag to override normal scaledValue
volatile uint32_t target_scaled_value = 0;      // Store the target pot-based scaled value

// ===================== ZERO DETECTION FILTERING CONFIGURATION =====================
#define ZERO_FILTER_ENABLED 1               // Enable/disable zero detection filtering
#define ZERO_FILTER_COUNT_THRESHOLD 10      // Number of consecutive zero readings required
#define ZERO_FILTER_NONZERO_THRESHOLD 5     // Number of consecutive non-zero readings required
#define ZERO_DETECTION_THRESHOLD 25         // ADC value considered as "zero" (same as existing)

// Zero detection filter variables
volatile uint8_t zero_consecutive_count = 0;     // Count of consecutive zero readings
volatile uint8_t nonzero_consecutive_count = 0;  // Count of consecutive non-zero readings
volatile uint8_t pot_confirmed_zero = 1;         // Confirmed zero state (start assuming zero)
volatile uint8_t pot_confirmed_nonzero = 0;      // Confirmed non-zero state
volatile uint32_t ramp_current_value = 0;       // Current ramped PWM value
volatile uint32_t ramp_start_value = 0;         // Starting PWM value for ramp
volatile uint32_t ramp_target_value = 0;        // Target PWM value for ramp
volatile uint32_t ramp_timer = 0;               // Timer for ramp updates
volatile uint32_t last_ramp_update_time = 0;    // Last time ramp was updated
volatile uint8_t ramp_active = 0;               // Flag indicating ramp is active

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
void ADCflag_with_startup_detection(void);
void TIM2_SetPWM1(uint16_t ccp);
void TIM2_SetPWM2(uint16_t ccp);
void process_zero_detection_filter(void);
uint8_t is_potentiometer_zero_filtered(void);
void handle_startup_detection_sequence(void);
void handle_ramp_up(void);

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

// ===================== RAMP-UP FUNCTIONS =====================

// Initialize ramp-up sequence
void start_ramp_up(uint32_t start_pwm, uint32_t target_pwm)
{
    if (!RAMP_UP_ENABLED) {
        // If ramp is disabled, set target immediately
        scaledValue = target_pwm;
        return;
    }

    printf("=== STARTING RAMP-UP SEQUENCE ===\r\n");
    printf("Ramping from %ld to %ld over %d ms\r\n", start_pwm, target_pwm, RAMP_UP_DURATION_MS);

    ramp_start_value = start_pwm;
    ramp_target_value = target_pwm;
    ramp_current_value = start_pwm;
    ramp_timer = system_ms_counter;
    last_ramp_update_time = system_ms_counter;
    ramp_active = 1;

    // Set initial value
    scaledValue = ramp_current_value;
}

// Calculate ramp step size based on remaining time and distance
uint32_t calculate_ramp_step(uint32_t current_val, uint32_t target_val, uint32_t elapsed_ms, uint32_t total_duration_ms)
{
    if (elapsed_ms >= total_duration_ms) {
        return (target_val > current_val) ? (target_val - current_val) : (current_val - target_val);
    }

    uint32_t remaining_ms = total_duration_ms - elapsed_ms;
    uint32_t remaining_distance = (target_val > current_val) ? (target_val - current_val) : (current_val - target_val);

    if (remaining_ms == 0) return remaining_distance;

    // Calculate step size: distance / (remaining_time / update_interval)
    uint32_t remaining_updates = (remaining_ms + RAMP_UPDATE_INTERVAL_MS - 1) / RAMP_UPDATE_INTERVAL_MS; // Ceiling division
    uint32_t step_size = (remaining_distance + remaining_updates - 1) / remaining_updates; // Ceiling division

    // Constrain step size
    if (step_size < RAMP_MIN_STEP) step_size = RAMP_MIN_STEP;
    if (step_size > RAMP_MAX_STEP) step_size = RAMP_MAX_STEP;

    return step_size;
}

// Handle ramp-up progression
void handle_ramp_up(void)
{
    if (!ramp_active) return;

    uint32_t elapsed_total = system_ms_counter - ramp_timer;
    uint32_t elapsed_since_update = system_ms_counter - last_ramp_update_time;

    // Check if it's time to update the ramp
    if (elapsed_since_update >= RAMP_UPDATE_INTERVAL_MS)
    {
        // Check if ramp duration completed
        if (elapsed_total >= RAMP_UP_DURATION_MS)
        {
            // Ramp complete - set final target value
            ramp_current_value = ramp_target_value;
            scaledValue = ramp_current_value;
            ramp_active = 0;
            printf("Ramp-up complete! Final PWM: %ld\r\n", scaledValue);
            return;
        }

        // Calculate and apply next ramp step
        uint32_t step_size = calculate_ramp_step(ramp_current_value, ramp_target_value, elapsed_total, RAMP_UP_DURATION_MS);

        if (ramp_target_value > ramp_current_value)
        {
            // Ramping up
            ramp_current_value += step_size;
            if (ramp_current_value > ramp_target_value) {
                ramp_current_value = ramp_target_value;
            }
        }
        else
        {
            // Ramping down (shouldn't happen in startup, but handle it)
            if (step_size > ramp_current_value) {
                ramp_current_value = 0;
            } else {
                ramp_current_value -= step_size;
            }
            if (ramp_current_value < ramp_target_value) {
                ramp_current_value = ramp_target_value;
            }
        }

        scaledValue = ramp_current_value;
        last_ramp_update_time = system_ms_counter;

        // Debug output every 10 updates (every 500ms)
        static uint8_t ramp_debug_counter = 0;
        if (++ramp_debug_counter >= 10) {
            // Calculate percentage using integer math (percentage * 10 for one decimal place)
            uint32_t progress_x10 = (ramp_current_value * 1000) / ramp_target_value;
            printf("Ramping: %ld/%ld (%ld.%ld%%) at %ld ms\r\n",
                   ramp_current_value, ramp_target_value,
                   progress_x10 / 10, progress_x10 % 10,
                   elapsed_total);
            ramp_debug_counter = 0;
        }
    }
}

// Check if pot value has changed during ramp and update target
void update_ramp_target_if_pot_changed(void)
{
    if (!ramp_active) return;

    // Calculate current pot-based target
    uint32_t current_pot_target = (adcReading1 * TIMER_MAX * 30) / (1023 * 100);

    // If pot target changed significantly, update ramp target
    uint32_t difference = (current_pot_target > ramp_target_value) ?
                         (current_pot_target - ramp_target_value) :
                         (ramp_target_value - current_pot_target);

    if (difference > 50) // Only update if change is significant (>50 PWM units)
    {
        printf("Pot changed during ramp: updating target from %ld to %ld\r\n",
               ramp_target_value, current_pot_target);
        ramp_target_value = current_pot_target;
        target_scaled_value = current_pot_target;
    }
}

// ===================== ZERO DETECTION FILTERING FUNCTIONS =====================

// Check if current ADC reading is considered zero
uint8_t is_adc_reading_zero(uint16_t adc_value)
{
    return (adc_value <= ZERO_DETECTION_THRESHOLD);
}

// Process zero detection with filtering
void process_zero_detection_filter(void)
{
    if (!ZERO_FILTER_ENABLED) {
        // If filtering disabled, use direct detection
        pot_confirmed_zero = is_adc_reading_zero(adcReading1);
        pot_confirmed_nonzero = !pot_confirmed_zero;
        return;
    }

    uint8_t current_reading_zero = is_adc_reading_zero(adcReading1);

    if (current_reading_zero) {
        // Current reading is zero
        zero_consecutive_count++;
        nonzero_consecutive_count = 0;  // Reset non-zero counter

        // Confirm zero state after enough consecutive readings
        if (zero_consecutive_count >= ZERO_FILTER_COUNT_THRESHOLD) {
            if (!pot_confirmed_zero) {
                printf("Pot ZERO confirmed after %d consecutive readings (ADC: %d)\r\n",
                       zero_consecutive_count, adcReading1);
                pot_confirmed_zero = 1;
                pot_confirmed_nonzero = 0;
            }
            // Cap the counter to prevent overflow
            if (zero_consecutive_count > ZERO_FILTER_COUNT_THRESHOLD) {
                zero_consecutive_count = ZERO_FILTER_COUNT_THRESHOLD;
            }
        }
    } else {
        // Current reading is non-zero
        nonzero_consecutive_count++;
        zero_consecutive_count = 0;  // Reset zero counter

        // Confirm non-zero state after enough consecutive readings
        if (nonzero_consecutive_count >= ZERO_FILTER_NONZERO_THRESHOLD) {
            if (!pot_confirmed_nonzero) {
                printf("Pot NON-ZERO confirmed after %d consecutive readings (ADC: %d)\r\n",
                       nonzero_consecutive_count, adcReading1);
                pot_confirmed_nonzero = 1;
                pot_confirmed_zero = 0;
            }
            // Cap the counter to prevent overflow
            if (nonzero_consecutive_count > ZERO_FILTER_NONZERO_THRESHOLD) {
                nonzero_consecutive_count = ZERO_FILTER_NONZERO_THRESHOLD;
            }
        }
    }
}

// Get filtered zero state (replaces is_potentiometer_zero)
uint8_t is_potentiometer_zero_filtered(void)
{
    return pot_confirmed_zero;
}

// Debug function to print filter status
void print_zero_filter_status(void)
{
    printf("Zero Filter - Raw ADC: %d, Zero: %d/%d, NonZero: %d/%d, State: %s\r\n",
           adcReading1, zero_consecutive_count, ZERO_FILTER_COUNT_THRESHOLD,
           nonzero_consecutive_count, ZERO_FILTER_NONZERO_THRESHOLD,
           pot_confirmed_zero ? "ZERO" : "NON-ZERO");
}

// ===================== CURRENT-BASED STARTUP FUNCTIONS =====================

// Check if potentiometer is at zero (now uses filtered detection)
uint8_t is_potentiometer_zero(void)
{
    return is_potentiometer_zero_filtered();
}

// Detect startup conditions with improved filtering
uint8_t detect_startup_condition(void)
{
    uint8_t current_pot_zero = is_potentiometer_zero_filtered();
    uint8_t trigger = 0;

    // Trigger on pot transition from confirmed zero to confirmed non-zero
    if (last_pot_zero_state && !current_pot_zero)
    {
        trigger = 1;
        printf("Startup trigger: Pot moved from CONFIRMED ZERO to NON-ZERO (ADC: %d)\r\n", adcReading1);
    }

    // Also trigger if system starts with pot already confirmed non-zero
    static uint8_t first_check = 1;
    if (first_check && !current_pot_zero && pot_confirmed_nonzero)
    {
        trigger = 1;
        printf("Startup trigger: System started with pot CONFIRMED NON-ZERO (ADC: %d)\r\n", adcReading1);
        first_check = 0;
    }

    last_pot_zero_state = current_pot_zero;
    return trigger;
}

// Check if current indicates locked rotor
uint8_t check_rotor_lock_current(void)
{
    if (current_mA > STARTUP_LOCK_CURRENT_THRESHOLD)
    {
        startup_lock_count++;
        printf("High current detected: %ld mA (count: %d/%d)\r\n",
               current_mA, startup_lock_count, STARTUP_LOCK_COUNT_THRESHOLD);

        if (startup_lock_count >= STARTUP_LOCK_COUNT_THRESHOLD)
        {
            printf("ROTOR LOCK DETECTED! Current consistently above %d mA\r\n",
                   STARTUP_LOCK_CURRENT_THRESHOLD);
            return 1;  // Lock confirmed
        }
    }
    else
    {
        if (startup_lock_count > 0)
        {
            printf("Current normal: %ld mA (resetting lock count)\r\n", current_mA);
        }
        startup_lock_count = 0;  // Reset if current goes back to normal
    }

    return 0;  // No lock detected
}

// Initialize startup detection sequence
void start_startup_detection(void)
{
    if (!STARTUP_DETECTION_ENABLED) return;

    printf("\r\n=== STARTING CURRENT-BASED STARTUP DETECTION ===\r\n");
    printf("Test PWM: %d, Lock threshold: %d mA, Test duration: %d ms\r\n",
           STARTUP_TEST_PWM_VALUE, STARTUP_LOCK_CURRENT_THRESHOLD, STARTUP_TEST_DURATION_MS);

    // Calculate and store target scaled value based on current pot position
    target_scaled_value = (adcReading1 * TIMER_MAX * 30) / (1023 * 100);
    printf("Target scaled value from pot: %ld\r\n", target_scaled_value);

    // Initialize state machine
    startup_state = STARTUP_TESTING;
    startup_timer = system_ms_counter;
    startup_lock_count = 0;
    startup_retry_count = 0;
    startup_triggered = 1;
    startup_control_active = 1;  // Override normal scaledValue control

    printf("Starting test with PWM value: %d\r\n", STARTUP_TEST_PWM_VALUE);
}

// Handle startup detection state machine
void handle_startup_detection_sequence(void)
{
    if (!STARTUP_DETECTION_ENABLED || startup_state == STARTUP_IDLE) return;

    uint32_t elapsed_time = system_ms_counter - startup_timer;

    switch (startup_state)
    {
        case STARTUP_TESTING:
            // Apply test PWM value
            scaledValue = STARTUP_TEST_PWM_VALUE;

            // Check for rotor lock based on current
            if (check_rotor_lock_current())
            {
                printf("Lock detected during test phase. Retry %d/%d\r\n",
                       startup_retry_count + 1, STARTUP_MAX_RETRIES);
                startup_state = STARTUP_LOCK_DETECTED;
                startup_timer = system_ms_counter;
            }
            else if (elapsed_time >= STARTUP_TEST_DURATION_MS)
            {
                if (startup_lock_count == 0)
                {
                    printf("Test completed - No lock detected! Current: %ld mA\r\n", current_mA);
                    startup_state = STARTUP_SUCCESS;
                    startup_timer = system_ms_counter;
                }
                else
                {
                    // Continue testing if we haven't reached the threshold yet
                    printf("Continuing test... Current readings mixed\r\n");
                }
            }
            break;

        case STARTUP_LOCK_DETECTED:
            // Cut output to zero immediately
            scaledValue = 0;
            printf("Output cut to zero due to lock detection\r\n");
            startup_retry_count++;

            if (startup_retry_count >= STARTUP_MAX_RETRIES)
            {
                printf("MAX RETRIES EXCEEDED! Startup failed.\r\n");
                startup_state = STARTUP_FAILED;
            }
            else
            {
                printf("Preparing for retry %d/%d\r\n", startup_retry_count, STARTUP_MAX_RETRIES);
                startup_state = STARTUP_RETRY_DELAY;
                startup_timer = system_ms_counter;
            }
            break;

        case STARTUP_RETRY_DELAY:
            // Keep output at zero during delay
            scaledValue = 0;

            if (elapsed_time >= STARTUP_RETRY_DELAY_MS)
            {
                printf("Retry delay complete. Attempting startup again...\r\n");
                startup_state = STARTUP_TESTING;
                startup_timer = system_ms_counter;
                startup_lock_count = 0;  // Reset lock detection
            }
            break;

        case STARTUP_SUCCESS:
            printf("Test completed - No lock detected! Current: %ld mA\r\n", current_mA);
            printf("Starting ramp-up from %d to %ld\r\n", STARTUP_TEST_PWM_VALUE, target_scaled_value);
            startup_state = STARTUP_RAMPING;
            startup_timer = system_ms_counter;

            // Start ramp from test PWM to target PWM
            start_ramp_up(STARTUP_TEST_PWM_VALUE, target_scaled_value);
            break;

        case STARTUP_RAMPING:
            // Handle ramp-up progression
            handle_ramp_up();

            // Check if user changed pot during ramp
            update_ramp_target_if_pot_changed();

            // Check if ramp is complete
            if (!ramp_active)
            {
                printf("STARTUP COMPLETE! Motor running at target PWM: %ld\r\n", scaledValue);
                startup_state = STARTUP_IDLE;
                startup_triggered = 0;
                startup_control_active = 0;  // Resume normal pot control
            }
            break;

        case STARTUP_FAILED:
            // Keep output at zero
            scaledValue = 0;
            printf("Startup failed - motor may be mechanically stuck\r\n");
            // Stay in failed state until manual reset or pot returns to confirmed zero
            if (is_potentiometer_zero_filtered())
            {
                printf("Pot returned to CONFIRMED ZERO - resetting startup system\r\n");
                startup_state = STARTUP_IDLE;
                startup_triggered = 0;
                startup_control_active = 0;
                startup_retry_count = 0;
            }
            break;

        default:
            startup_state = STARTUP_IDLE;
            startup_triggered = 0;
            startup_control_active = 0;
            break;
    }
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

// Enhanced status printing with startup detection and ramp-up
void print_status_with_startup_detection(void)
{
    printf("Current: %ld mA, PWM: %ld", current_mA, scaledValue);

    if (startup_state != STARTUP_IDLE)
    {
        const char* state_names[] = {"IDLE", "TESTING", "LOCK_DET", "RETRY_DELAY", "SUCCESS", "RAMPING", "FAILED"};
        printf(", Startup: %s", state_names[startup_state]);

        uint32_t elapsed = system_ms_counter - startup_timer;
        printf(" (%ld ms)", elapsed);

        if (startup_state == STARTUP_TESTING || startup_state == STARTUP_LOCK_DETECTED)
        {
            printf(", LockCnt: %d/%d", startup_lock_count, STARTUP_LOCK_COUNT_THRESHOLD);
        }

        if (startup_state == STARTUP_RAMPING && ramp_active)
        {
            // Calculate ramp progress using integer math (percentage * 10 for one decimal place)
            uint32_t progress_x10 = ((ramp_current_value - ramp_start_value) * 1000) /
                                   (ramp_target_value - ramp_start_value);
            printf(", Ramp: %ld.%ld%%", progress_x10 / 10, progress_x10 % 10);
        }

        if (startup_retry_count > 0)
        {
            printf(", Retry: %d/%d", startup_retry_count, STARTUP_MAX_RETRIES);
        }
    }
    else
    {
        printf(", Status: NORMAL");
    }

    printf("\r\n");
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

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE; //for multi-channel access
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 2;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_241Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_241Cycles);

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

// Modified ADCflag function with startup detection and zero filtering
void ADCflag_with_startup_detection(void)
{
    adcReading1 = ADCBuffer[0];
    adcReading2 = ADCBuffer[1];

    // Process zero detection filter FIRST
    process_zero_detection_filter();

    // Apply offset correction (using raw reading for overcurrent reset)
    if (adcReading1 <= 25) {
        overcurrentflag = 0;  // Reset overcurrent if pot at zero
    }

    // Check for startup trigger (now uses filtered detection)
    if (detect_startup_condition())
    {
        start_startup_detection();
    }

    // Set current threshold based on pot reading
    if (adcReading1 < 100)
    {
        currentthreshold = 2000;
    }
    else if (adcReading1 > 100)
    {
        currentthreshold = 1500;
    }

    // Modified overcurrent detection logic
    if (current_mA > currentthreshold)
    {
        overcurrentCount++;
        if (overcurrentCount >= OVERCURRENT_THRESHOLD_COUNT)
        {
            overcurrentflag = 1;
        }
    }
    else if (current_mA < currentthreshold)
    {
        overcurrentCount = 0;
    }

    // Handle overcurrent condition
    if (overcurrentflag == 1)
    {
        scaledValue = 0;
        // Abort startup sequence if main overcurrent protection triggers
        if (startup_state != STARTUP_IDLE)
        {
            startup_state = STARTUP_FAILED;
            startup_control_active = 0;
            printf("MAIN OVERCURRENT - Startup detection aborted!\r\n");
        }
    }
    else if (overcurrentflag == 0)
    {
        // Only update scaledValue if startup detection is not controlling it
        if (!startup_control_active)
        {
            // Normal pot-controlled scaling (only if pot is confirmed non-zero)
            if (pot_confirmed_nonzero) {
                scaledValue = (adcReading1 * TIMER_MAX * 50) / (1023 * 100);
            } else {
                scaledValue = 0;  // Keep at zero if pot state not confirmed
            }
        }
        // If startup_control_active, scaledValue is managed by startup state machine or ramp function
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

// Hall sensor interrupt handler
void EXTI7_0_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI7_0_IRQHandler(void)
{
    if (EXTI_GetITStatus(EXTI_Line3) != RESET)
    {
        start_compensation_timer(1);
        EXTI_ClearITPendingBit(EXTI_Line3);
    }
    else if (EXTI_GetITStatus(EXTI_Line4) != RESET)
    {
        start_compensation_timer(2);
        EXTI_ClearITPendingBit(EXTI_Line4);
    }
}

// ===================== DEBUG AND TEST FUNCTIONS =====================

// Manual test function
void test_startup_detection(void)
{
    printf("=== MANUAL STARTUP DETECTION TEST ===\r\n");
    printf("Current pot reading: %d\r\n", adcReading1);
    printf("Current motor current: %ld mA\r\n", current_mA);
    start_startup_detection();
}

// Reset startup system (useful for debugging)
void reset_startup_detection(void)
{
    printf("=== RESETTING STARTUP DETECTION SYSTEM ===\r\n");
    startup_state = STARTUP_IDLE;
    startup_triggered = 0;
    startup_control_active = 0;
    startup_retry_count = 0;
    startup_lock_count = 0;
    scaledValue = 0;
    printf("Startup system reset complete\r\n");
}

// Print system configuration
void print_system_configuration(void)
{
    printf("\r\n=== BLDC MOTOR CONTROLLER CONFIGURATION ===\r\n");
    printf("System Clock: %ld Hz\r\n", SystemCoreClock);
    printf("PWM Frequency: %d Hz (ARR=%d, PSC=%d)\r\n",
           (int)(SystemCoreClock / ((PWM_PSC + 1) * (PWM_ARR + 1))), PWM_ARR, PWM_PSC);
    printf("Hall Compensation: %s (%d us delay)\r\n",
           HALL_COMPENSATION_ENABLED ? "ENABLED" : "DISABLED", HALL_DELAY_MICROSECONDS);
    printf("Startup Detection: %s\r\n", STARTUP_DETECTION_ENABLED ? "ENABLED" : "DISABLED");
    if (STARTUP_DETECTION_ENABLED) {
        printf("  - Test PWM Value: %d\r\n", STARTUP_TEST_PWM_VALUE);
        printf("  - Lock Current Threshold: %d mA\r\n", STARTUP_LOCK_CURRENT_THRESHOLD);
        printf("  - Lock Count Threshold: %d readings\r\n", STARTUP_LOCK_COUNT_THRESHOLD);
        printf("  - Test Duration: %d ms\r\n", STARTUP_TEST_DURATION_MS);
        printf("  - Retry Delay: %d ms\r\n", STARTUP_RETRY_DELAY_MS);
        printf("  - Max Retries: %d\r\n", STARTUP_MAX_RETRIES);
    }
    printf("Ramp-Up Function: %s\r\n", RAMP_UP_ENABLED ? "ENABLED" : "DISABLED");
    if (RAMP_UP_ENABLED) {
        printf("  - Ramp Duration: %d ms\r\n", RAMP_UP_DURATION_MS);
        printf("  - Update Interval: %d ms\r\n", RAMP_UPDATE_INTERVAL_MS);
        printf("  - Step Range: %d-%d PWM units\r\n", RAMP_MIN_STEP, RAMP_MAX_STEP);
    }
    printf("Zero Detection Filter: %s\r\n", ZERO_FILTER_ENABLED ? "ENABLED" : "DISABLED");
    if (ZERO_FILTER_ENABLED) {
        printf("  - Zero Confirmation: %d consecutive readings\r\n", ZERO_FILTER_COUNT_THRESHOLD);
        printf("  - Non-Zero Confirmation: %d consecutive readings\r\n", ZERO_FILTER_NONZERO_THRESHOLD);
        printf("  - Zero Threshold: ADC <= %d\r\n", ZERO_DETECTION_THRESHOLD);
    }
    printf("Current Sensing: Moving average of %d samples\r\n", MOVING_AVERAGE1_SIZE);
    printf("Overcurrent Protection: %d consecutive readings\r\n", OVERCURRENT_THRESHOLD_COUNT);
    printf("Status Print Interval: %d ms\r\n", PRINT_INTERVAL_MS);

    printf("===========================================\r\n\n");

}

// Print detailed system status (for debugging)
void print_detailed_status(void)
{
    printf("\r\n=== DETAILED SYSTEM STATUS ===\r\n");
    printf("Runtime: %ld.%03ld seconds\r\n", system_ms_counter/1000, system_ms_counter%1000);
    printf("Potentiometer ADC: %d (scaled: %ld)\r\n", adcReading1, scaledValue);

    // Zero filter status
    if (ZERO_FILTER_ENABLED) {
        printf("Zero Filter: Raw=%d, Zero=%d/%d, NonZero=%d/%d, State=%s\r\n",
               adcReading1, zero_consecutive_count, ZERO_FILTER_COUNT_THRESHOLD,
               nonzero_consecutive_count, ZERO_FILTER_NONZERO_THRESHOLD,
               pot_confirmed_zero ? "CONFIRMED_ZERO" : "CONFIRMED_NONZERO");
    }

    printf("Current ADC: %d (average: %ld)\r\n", adcReading2, averageadcReading2Value);
    printf("Current: %ld mA (threshold: %ld mA)\r\n", current_mA, currentthreshold);
    printf("Voltage: %ld mV\r\n", voltage_mv);
    printf("Overcurrent: %s (count: %d/%d)\r\n",
           overcurrentflag ? "ACTIVE" : "Normal", overcurrentCount, OVERCURRENT_THRESHOLD_COUNT);

    // Hall sensor states
    uint8_t hall3 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_3);
    uint8_t hall4 = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_4);
    printf("Hall Sensors - PC3: %s, PC4: %s\r\n",
           hall3 ? "HIGH" : "LOW", hall4 ? "HIGH" : "LOW");

    // Startup system status
    if (startup_state != STARTUP_IDLE) {
        const char* state_names[] = {"IDLE", "TESTING", "LOCK_DET", "RETRY_DELAY", "SUCCESS", "RAMPING", "FAILED"};
        printf("Startup State: %s\r\n", state_names[startup_state]);
        printf("Startup Control Active: %s\r\n", startup_control_active ? "YES" : "NO");
        printf("Lock Count: %d/%d\r\n", startup_lock_count, STARTUP_LOCK_COUNT_THRESHOLD);
        printf("Retry Count: %d/%d\r\n", startup_retry_count, STARTUP_MAX_RETRIES);
        printf("Target Scaled Value: %ld\r\n", target_scaled_value);

        if (ramp_active) {
            printf("Ramp Status: ACTIVE\r\n");
            printf("Ramp Progress: %ld -> %ld -> %ld\r\n", ramp_start_value, ramp_current_value, ramp_target_value);
            // Calculate progress using integer math (percentage * 10 for one decimal place)
            uint32_t progress_x10 = ((ramp_current_value - ramp_start_value) * 1000) /
                                   (ramp_target_value - ramp_start_value);
            printf("Ramp Completion: %ld.%ld%%\r\n", progress_x10 / 10, progress_x10 % 10);
        }
    } else {
        printf("Startup State: IDLE (Normal Operation)\r\n");
    }
    printf("==============================\r\n\n");
}

// ===================== MAIN FUNCTION =====================

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    USART_Printf_Init(115200);

    printf("\r\n");
    printf("========================================\r\n");
    printf("    BLDC Motor Controller Starting     \r\n");
    printf("     with Current-Based Startup       \r\n");
    printf("========================================\r\n");

    // Print system configuration
    print_system_configuration();

    // Initialize all peripherals
    printf("Initializing peripherals...\r\n");
    ADCConfig();
    printf("  - ADC configured for pot and current sensing\r\n");

    EXTIO_INT_INIT();
    printf("  - Hall sensor interrupts configured (PC3, PC4)\r\n");

    TIM2_Init();
    printf("  - PWM timer configured (PC1, PC2 outputs)\r\n");

    DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)ADCBuffer, 2);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    printf("  - DMA configured for dual ADC\r\n");

    ADC_SoftwareStartConvCmd(ADC1, ENABLE);
    printf("  - ADC conversion started\r\n");

    printf("\r\nSystem ready! Features:\r\n");
    printf("  + Automatic startup detection with current feedback\r\n");
    printf("  + Rotor lock detection and retry mechanism\r\n");
    printf("  + Smooth PWM ramp-up after successful startup\r\n");
    printf("  + Hall sensor compensation\r\n");
    printf("  + Overcurrent protection\r\n");
    printf("  + Moving average current sensing\r\n");
    printf("\r\nOperation:\r\n");
    printf("  - Move potentiometer from 0 to start motor\r\n");
    printf("  - System will auto-detect and handle locked rotor\r\n");
    printf("  - Motor will ramp up smoothly over %d ms\r\n", RAMP_UP_DURATION_MS);
    printf("  - Status printed every %d ms\r\n", PRINT_INTERVAL_MS);
    printf("\r\nStarting main control loop...\r\n");
    printf("========================================\r\n\n");

    // Small delay to let ADC stabilize
    for (volatile int i = 0; i < 100000; i++);

    while (1)
    {
        // Core control functions
        initial_condition_INIT();                    // Handle hall sensor states
        ADCflag_with_startup_detection();           // Process ADC with startup detection
        current_calculation();                      // Calculate current from ADC
        handle_startup_detection_sequence();       // Handle startup state machine

        // Timer-based status printing
        if ((system_ms_counter - last_print_time) >= PRINT_INTERVAL_MS)
        {
//            print_status_with_startup_detection();  // Print enhanced status
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

// ===================== ADDITIONAL UART COMMANDS (Optional) =====================
/*
You can add these functions to be called via UART commands for debugging:

Commands you could implement:
- "status" -> print_detailed_status()
- "config" -> print_system_configuration()
- "test" -> test_startup_detection()
- "reset" -> reset_startup_detection()

Example UART command handler:
void process_uart_command(char* command)
{
    if (strcmp(command, "status") == 0) {
        print_detailed_status();
    } else if (strcmp(command, "config") == 0) {
        print_system_configuration();
    } else if (strcmp(command, "test") == 0) {
        test_startup_detection();
    } else if (strcmp(command, "reset") == 0) {
        reset_startup_detection();
    } else {
        printf("Unknown command: %s\r\n", command);
        printf("Available: status, config, test, reset\r\n");
    }
}
*/
