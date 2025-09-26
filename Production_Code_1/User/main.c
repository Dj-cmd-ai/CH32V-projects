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

// ===================== VOLTAGE-BASED PWM MULTIPLIER SYSTEM =====================
// Voltage range definitions and corresponding multipliers
#define VOLTAGE_RANGE_COUNT 8

// Structure to define voltage ranges and their multipliers
typedef struct {
    uint16_t min_voltage;  // Minimum voltage for this range
    uint16_t max_voltage;  // Maximum voltage for this range
    uint8_t multiplier;    // PWM multiplier (%) for this range
} VoltageRange;

// Define voltage ranges within safety limits
const VoltageRange voltage_ranges[VOLTAGE_RANGE_COUNT] = {
    {120, 140, 90},    // 120-140V: Use 80% multiplier
    {140, 160, 85},    // 140-160V: Use 75% multiplier
    {160, 180, 80},    // 160-180V: Use 70% multiplier
    {180, 200, 75},    // 180-200V: Use 65% multiplier
    {200, 220, 70},    // 200-220V: Use 60% multiplier
    {220, 240, 65},    // 220-240V: Use 55% multiplier
    {240, 260, 60},    // 240-260V: Use 50% multiplier
    {260, 999, 55}     // Above 260V: Use 45% multiplier
};

// Global variables for voltage-based multiplier
uint8_t currentMultiplier = 70;  // Default to 70% (matches original behavior)
uint32_t lastVoltageCheckTime = 0;
#define VOLTAGE_CHECK_INTERVAL_MS 100  // Check every 100ms

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
#define HALL_DELAY_MICROSECONDS 0  // Adjust this value (0-2000 us typical)

// ===================== TIMER-BASED PRINTING CONFIGURATION =====================
// Timer-based printing variables
volatile uint32_t system_ms_counter = 0;  // Millisecond counter
uint32_t last_print_time = 0;
#define PRINT_INTERVAL_MS 250  // 250ms = 4 times per second

// ===================== CURRENT-BASED STARTUP DETECTION CONFIGURATION =====================
#define STARTUP_DETECTION_ENABLED 1         // Enable/disable startup detection
#define STARTUP_TEST_PWM_VALUE 12           // Low PWM value for testing rotor lock (very low)
#define STARTUP_LOCK_CURRENT_THRESHOLD 500  // Current threshold to detect locked rotor (mA)
#define STARTUP_LOCK_COUNT_THRESHOLD 5      // Number of consecutive high current readings to confirm lock
#define STARTUP_RETRY_DELAY_MS 500          // Delay between retry attempts
#define STARTUP_TEST_DURATION_MS 1000        // How long to test with low PWM before checking current
#define STARTUP_MAX_RETRIES 10              // Maximum number of retry attempts

// ===================== IMPROVED RAMP-UP CONFIGURATION =====================
#define RAMP_UP_ENABLED 1                   // Enable/disable ramp-up function
#define RAMP_UP_DURATION_MS 1000           // Fast ramp: 1 second
#define RAMP_UPDATE_INTERVAL_MS 10         // High frequency updates (100 times per second)

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
#define ZERO_FILTER_COUNT_THRESHOLD 25        // Number of consecutive zero readings required(1000 for 3 speed)
#define ZERO_FILTER_NONZERO_THRESHOLD 5     // Number of consecutive non-zero readings required
#define ZERO_DETECTION_THRESHOLD 25         // ADC value considered as "zero"

// Zero detection filter variables
volatile uint16_t zero_consecutive_count = 0;     // Count of consecutive zero readings
volatile uint8_t nonzero_consecutive_count = 0;  // Count of consecutive non-zero readings
volatile uint8_t pot_confirmed_zero = 1;         // Confirmed zero state (start assuming zero)
volatile uint8_t pot_confirmed_nonzero = 0;      // Confirmed non-zero state

// ===================== RAMP VARIABLES =====================
volatile uint32_t ramp_current_value = 0;       // Current ramped PWM value
volatile uint32_t ramp_start_value = 0;         // Starting PWM value for ramp
volatile uint32_t ramp_target_value = 0;        // Target PWM value for ramp
volatile uint32_t ramp_start_time = 0;          // When ramp started
volatile uint32_t last_ramp_update_time = 0;    // Last time ramp was updated
volatile uint8_t ramp_active = 0;               // Flag indicating ramp is active

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

//=============================Function prototypes========================
void ADCflag_with_startup_detection(void);
void TIM2_SetPWM1(uint16_t ccp);
void TIM2_SetPWM2(uint16_t ccp);
void process_zero_detection_filter(void);
uint8_t is_potentiometer_zero_filtered(void);
void handle_startup_detection_sequence(void);
void handle_ramp_up(void);
uint8_t get_multiplier_for_voltage(uint16_t ac_voltage);
void update_multiplier_if_needed(void);

// ===================== VOLTAGE-BASED MULTIPLIER FUNCTIONS =====================
// Function to determine multiplier based on voltage
uint8_t get_multiplier_for_voltage(uint16_t ac_voltage) {
    // Check if voltage is below minimum safety limit
    if (ac_voltage < 120) {
        return 0; // Stop motor below 120V
    }

    // Find the appropriate voltage range
    for (uint8_t i = 0; i < VOLTAGE_RANGE_COUNT; i++) {
        if (ac_voltage >= voltage_ranges[i].min_voltage &&
            ac_voltage < voltage_ranges[i].max_voltage) {
            return voltage_ranges[i].multiplier;
        }
    }

    // Fallback for above 260V
    return 45;
}

// Update multiplier based on voltage
void update_multiplier_if_needed(void) {
    if ((system_ms_counter - lastVoltageCheckTime) >= VOLTAGE_CHECK_INTERVAL_MS) {
        // Get the appropriate multiplier for current voltage
        uint8_t newMultiplier = get_multiplier_for_voltage(AC_Voltage);

        // Update if changed
        if (newMultiplier != currentMultiplier) {
            // printf("Voltage multiplier changed: %d%% -> %d%% (AC_V: %d)\r\n",
            //        currentMultiplier, newMultiplier, AC_Voltage);
            currentMultiplier = newMultiplier;
        }

        // Update the last check time
        lastVoltageCheckTime = system_ms_counter;
    }
}

// ===================== SIMPLE FIXED DELAY COMPENSATION =====================
// Use simple delay loops for fixed compensation
void simple_delay_us(uint32_t delay_us)
{
    // Simple delay loop calibrated for 48MHz system clock
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

// ===================== LINEAR RAMP-UP FUNCTIONS =====================

// Initialize ramp-up sequence
void start_ramp_up(uint32_t start_pwm, uint32_t target_pwm)
{
    if (!RAMP_UP_ENABLED) {
        // If ramp is disabled, set target immediately
        scaledValue = target_pwm;
        return;
    }

    // printf("=== STARTING LINEAR RAMP-UP ===\r\n");
    // printf("Ramping from %ld to %ld over %d ms (Multiplier: %d%%)\r\n",
    //        start_pwm, target_pwm, RAMP_UP_DURATION_MS, currentMultiplier);

    ramp_start_value = start_pwm;
    ramp_target_value = target_pwm;
    ramp_current_value = start_pwm;
    ramp_start_time = system_ms_counter;
    last_ramp_update_time = system_ms_counter;
    ramp_active = 1;

    // Set initial value
    scaledValue = ramp_current_value;
}

// Linear interpolation ramp
void handle_ramp_up(void)
{
    if (!ramp_active) return;

    uint32_t current_time = system_ms_counter;
    uint32_t elapsed_total = current_time - ramp_start_time;
    uint32_t elapsed_since_update = current_time - last_ramp_update_time;

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
            // printf("Linear ramp complete! Final PWM: %ld (Multiplier: %d%%)\r\n",
            //        scaledValue, currentMultiplier);
            return;
        }

        // Linear interpolation: current = start + (target - start) * (elapsed / total_duration)
        uint32_t range = (ramp_target_value > ramp_start_value) ?
                        (ramp_target_value - ramp_start_value) :
                        (ramp_start_value - ramp_target_value);

        uint32_t progress = (elapsed_total * range) / RAMP_UP_DURATION_MS;

        if (ramp_target_value > ramp_start_value) {
            ramp_current_value = ramp_start_value + progress;
        } else {
            ramp_current_value = ramp_start_value - progress;
        }

        // Ensure we don't overshoot
        if (ramp_target_value > ramp_start_value) {
            if (ramp_current_value > ramp_target_value) {
                ramp_current_value = ramp_target_value;
            }
        } else {
            if (ramp_current_value < ramp_target_value) {
                ramp_current_value = ramp_target_value;
            }
        }

        scaledValue = ramp_current_value;
        last_ramp_update_time = current_time;

        // Debug output every 100ms (every 10 updates) - COMMENTED
        // static uint8_t ramp_debug_counter = 0;
        // if (++ramp_debug_counter >= 10) {
        //     uint32_t progress_percent = (elapsed_total * 100) / RAMP_UP_DURATION_MS;
        //     printf("Ramp: %ld/%ld (%ld%%) at %ld ms, Mult: %d%%\r\n",
        //            ramp_current_value, ramp_target_value,
        //            progress_percent, elapsed_total, currentMultiplier);
        //     ramp_debug_counter = 0;
        // }
    }
}

// Check if pot value has changed during ramp and update target
void update_ramp_target_if_pot_changed(void)
{
    if (!ramp_active) return;

    // Calculate current pot-based target using current voltage multiplier
    uint32_t current_pot_target = (adcReading1 * TIMER_MAX * currentMultiplier) / (1023 * 100);

    // If pot target changed significantly, update ramp target
    uint32_t difference = (current_pot_target > ramp_target_value) ?
                         (current_pot_target - ramp_target_value) :
                         (ramp_target_value - current_pot_target);

    if (difference > 50) // Only update if change is significant (>50 PWM units)
    {
        // printf("Pot changed during ramp: updating target from %ld to %ld\r\n",
        //        ramp_target_value, current_pot_target);
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
                // printf("Pot ZERO confirmed after %d consecutive readings (ADC: %d)\r\n",
                //        zero_consecutive_count, adcReading1);
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
                // printf("Pot NON-ZERO confirmed after %d consecutive readings (ADC: %d)\r\n",
                //        nonzero_consecutive_count, adcReading1);
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

// Get filtered zero state
uint8_t is_potentiometer_zero_filtered(void)
{
    return pot_confirmed_zero;
}

// ===================== CURRENT-BASED STARTUP FUNCTIONS =====================

// Check if potentiometer is at zero
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
        // printf("Startup trigger: Pot moved from CONFIRMED ZERO to NON-ZERO (ADC: %d)\r\n", adcReading1);
    }

    // Also trigger if system starts with pot already confirmed non-zero
    static uint8_t first_check = 1;
    if (first_check && !current_pot_zero && pot_confirmed_nonzero)
    {
        trigger = 1;
        // printf("Startup trigger: System started with pot CONFIRMED NON-ZERO (ADC: %d)\r\n", adcReading1);
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
        // printf("High current detected: %ld mA (count: %d/%d)\r\n",
        //        current_mA, startup_lock_count, STARTUP_LOCK_COUNT_THRESHOLD);

        if (startup_lock_count >= STARTUP_LOCK_COUNT_THRESHOLD)
        {
            // printf("ROTOR LOCK DETECTED! Current consistently above %d mA\r\n",
            //        STARTUP_LOCK_CURRENT_THRESHOLD);
            return 1;  // Lock confirmed
        }
    }
    else
    {
        if (startup_lock_count > 0)
        {
            // printf("Current normal: %ld mA (resetting lock count)\r\n", current_mA);
        }
        startup_lock_count = 0;  // Reset if current goes back to normal
    }

    return 0;  // No lock detected
}

// Initialize startup detection sequence
void start_startup_detection(void)
{
    if (!STARTUP_DETECTION_ENABLED) return;

    // printf("\r\n=== STARTING CURRENT-BASED STARTUP DETECTION ===\r\n");
    // printf("Test PWM: %d, Lock threshold: %d mA, Test duration: %d ms\r\n",
    //        STARTUP_TEST_PWM_VALUE, STARTUP_LOCK_CURRENT_THRESHOLD, STARTUP_TEST_DURATION_MS);

    // Calculate and store target scaled value based on current pot position AND voltage multiplier
    target_scaled_value = (adcReading1 * TIMER_MAX * currentMultiplier) / (1023 * 100);
    // printf("Target scaled value from pot: %ld (using %d%% multiplier)\r\n",
    //        target_scaled_value, currentMultiplier);

    // Initialize state machine
    startup_state = STARTUP_TESTING;
    startup_timer = system_ms_counter;
    startup_lock_count = 0;
    startup_retry_count = 0;
    startup_triggered = 1;
    startup_control_active = 1;  // Override normal scaledValue control

    // printf("Starting test with PWM value: %d\r\n", STARTUP_TEST_PWM_VALUE);
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
                // printf("Lock detected during test phase. Retry %d/%d\r\n",
                //        startup_retry_count + 1, STARTUP_MAX_RETRIES);
                startup_state = STARTUP_LOCK_DETECTED;
                startup_timer = system_ms_counter;
            }
            else if (elapsed_time >= STARTUP_TEST_DURATION_MS)
            {
                if (startup_lock_count == 0)
                {
                    // printf("Test completed - No lock detected! Current: %ld mA\r\n", current_mA);
                    startup_state = STARTUP_SUCCESS;
                    startup_timer = system_ms_counter;
                }
                else
                {
                    // Continue testing if we haven't reached the threshold yet
                    // printf("Continuing test... Current readings mixed\r\n");
                }
            }
            break;

        case STARTUP_LOCK_DETECTED:
            // Cut output to zero immediately
            scaledValue = 0;
            // printf("Output cut to zero due to lock detection\r\n");
            startup_retry_count++;

            if (startup_retry_count >= STARTUP_MAX_RETRIES)
            {
                // printf("MAX RETRIES EXCEEDED! Startup failed.\r\n");
                startup_state = STARTUP_FAILED;
            }
            else
            {
                // printf("Preparing for retry %d/%d\r\n", startup_retry_count, STARTUP_MAX_RETRIES);
                startup_state = STARTUP_RETRY_DELAY;
                startup_timer = system_ms_counter;
            }
            break;

        case STARTUP_RETRY_DELAY:
            // Keep output at zero during delay
            scaledValue = 0;

            if (elapsed_time >= STARTUP_RETRY_DELAY_MS)
            {
                // printf("Retry delay complete. Attempting startup again...\r\n");
                startup_state = STARTUP_TESTING;
                startup_timer = system_ms_counter;
                startup_lock_count = 0;  // Reset lock detection
            }
            break;

        case STARTUP_SUCCESS:
            // printf("Test completed - No lock detected! Current: %ld mA\r\n", current_mA);

            // Recalculate target with current voltage multiplier (might have changed during test)
            target_scaled_value = (adcReading1 * TIMER_MAX * currentMultiplier) / (1023 * 100);

            // printf("Starting fast linear ramp from %d to %ld (Mult: %d%%)\r\n",
            //        STARTUP_TEST_PWM_VALUE, target_scaled_value, currentMultiplier);
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
                // printf("STARTUP COMPLETE! Motor running at target PWM: %ld (Mult: %d%%)\r\n",
                //        scaledValue, currentMultiplier);
                startup_state = STARTUP_IDLE;
                startup_triggered = 0;
                startup_control_active = 0;  // Resume normal pot control
            }
            break;

        case STARTUP_FAILED:
            // Keep output at zero
            scaledValue = 0;
            // printf("Startup failed - motor may be mechanically stuck\r\n");
            // Stay in failed state until manual reset or pot returns to confirmed zero
            if (is_potentiometer_zero_filtered())
            {
                // printf("Pot returned to CONFIRMED ZERO - resetting startup system\r\n");
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
    // ADC Moving Average Calculation
    adcSum -= adcHistory[adcIndex];  // Remove the oldest value
    adcHistory[adcIndex] = adcReading2;  // Add new value
    adcSum += adcReading2;  // Update sum
    adcIndex = (adcIndex + 1) % MOVING_AVERAGE1_SIZE;  // Circular buffer
    averageadcReading2Value = adcSum / MOVING_AVERAGE1_SIZE;  // Compute average

    // Convert ADC to Voltage (Integer Math)
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
        static uint16_t sub_ms_counter = 0;
        sub_ms_counter += 292;  // Approximately 0.292ms per TIM2 overflow
        if (sub_ms_counter >= 1000) {
            system_ms_counter++;
            sub_ms_counter -= 1000;
        }
    }
}

// Enhanced status printing (production version with minimal output)
void print_status_with_startup_detection(void)
{
    printf("I: %d mA, PWM: %d, V: %d, M: %d%%",
           current_mA, scaledValue, AC_Voltage, currentMultiplier);

    if (startup_state != STARTUP_IDLE)
    {
        const char* state_names[] = {"IDLE", "TEST", "LOCK", "RETRY", "OK", "RAMP", "FAIL"};
        printf(", S: %s", state_names[startup_state]);

        if (startup_state == STARTUP_RAMPING && ramp_active)
        {
            uint32_t progress_percent = ((system_ms_counter - ramp_start_time) * 100) / RAMP_UP_DURATION_MS;
            if (progress_percent > 100) progress_percent = 100;
            printf("(%d%%)", progress_percent);
        }
    }

    // Add fault flags
    if (overcurrentflag || voltageflag) {
        printf(", FAULT");
    }

    printf("\r\n");
}

// ===================== PERIPHERAL INITIALIZATION FUNCTIONS =====================

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
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

    // Configure NVIC for TIM2 interrupt
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
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

    GPIO_InitStructure.GPIO_Pin = ANALOG3_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(ANALOG1_PORT, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 3;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_Channel_7, 1, ADC_SampleTime_241Cycles);  // PD4 - Potentiometer
    ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_241Cycles);  // PD2 - Current sensing
    ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 3, ADC_SampleTime_241Cycles);  // PD3 - Voltage sensing

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

// ===================== MAIN CONTROL FUNCTION =====================

// Modified ADCflag function with all integrated features
void ADCflag_with_startup_detection(void)
{
    adcReading1 = ADCBuffer[0];
    adcReading2 = ADCBuffer[1];
    adcReading3 = ADCBuffer[2];

    // Process zero detection filter FIRST
    process_zero_detection_filter();

    // Apply offset correction
    if (adcReading1 <= 25) {
        overcurrentflag = 0;  // Reset overcurrent if pot at zero
    }

    // Check for startup trigger
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

    // Overcurrent detection logic
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

    // Voltage protection logic
    if (AC_Voltage > overvoltagethreshold || AC_Voltage < undervoltagethreshold)
    {
        voltageCount++;
        if (voltageCount >= VOLTAGE_THRESHOLD_COUNT_)
        {
            voltageflag = 1;
            if (startup_state != STARTUP_IDLE)
            {
                // printf("VOLTAGE FAULT during startup - Aborting! AC_V: %d V\r\n", AC_Voltage);
                startup_state = STARTUP_FAILED;
                startup_control_active = 0;
            }
        }
    }
    else if (AC_Voltage <= overvoltagethreshold && AC_Voltage >= undervoltagethreshold)
    {
        voltageCount = 0;
        voltageflag = 0;
    }

    // Handle fault conditions
    if (overcurrentflag == 1 || voltageflag == 1)
    {
        scaledValue = 0;
        // Abort startup sequence if any fault protection triggers
        if (startup_state != STARTUP_IDLE)
        {
            startup_state = STARTUP_FAILED;
            startup_control_active = 0;
        }
    }
    else if (overcurrentflag == 0 && voltageflag == 0)
    {
        // Only update scaledValue if startup detection is not controlling it
        if (!startup_control_active)
        {
            // Normal pot-controlled scaling with voltage-based multiplier
            if (pot_confirmed_nonzero) {
                scaledValue = (adcReading1 * TIMER_MAX * currentMultiplier) / (1023 * 100);
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

// ===================== DEBUG FUNCTIONS (ENABLE AS NEEDED) =====================

// Manual test function (uncomment for testing)
// void test_startup_detection(void)
// {
//     printf("=== MANUAL STARTUP DETECTION TEST ===\r\n");
//     printf("Current pot reading: %d\r\n", adcReading1);
//     printf("Current motor current: %ld mA\r\n", current_mA);
//     printf("AC Voltage: %d V (Multiplier: %d%%)\r\n", AC_Voltage, currentMultiplier);
//     start_startup_detection();
// }

// Reset startup system (uncomment for debugging)
// void reset_startup_detection(void)
// {
//     printf("=== RESETTING STARTUP DETECTION SYSTEM ===\r\n");
//     startup_state = STARTUP_IDLE;
//     startup_triggered = 0;
//     startup_control_active = 0;
//     startup_retry_count = 0;
//     startup_lock_count = 0;
//     scaledValue = 0;
//     ramp_active = 0;
//     printf("Startup system reset complete\r\n");
// }

// Print system configuration (uncomment for setup verification)
// void print_system_configuration(void)
// {
//     printf("\r\n=== ADVANCED BLDC MOTOR CONTROLLER CONFIGURATION ===\r\n");
//     printf("System Clock: %ld Hz\r\n", SystemCoreClock);
//     printf("PWM Frequency: %d Hz (ARR=%d, PSC=%d)\r\n",
//            (int)(SystemCoreClock / ((PWM_PSC + 1) * (PWM_ARR + 1))), PWM_ARR, PWM_PSC);
//     printf("Hall Compensation: %s (%d us delay)\r\n",
//            HALL_COMPENSATION_ENABLED ? "ENABLED" : "DISABLED", HALL_DELAY_MICROSECONDS);
//     printf("Startup Detection: %s\r\n", STARTUP_DETECTION_ENABLED ? "ENABLED" : "DISABLED");
//     printf("Linear Ramp-Up: %s (%d ms duration)\r\n", RAMP_UP_ENABLED ? "ENABLED" : "DISABLED", RAMP_UP_DURATION_MS);
//     printf("Zero Detection Filter: %s\r\n", ZERO_FILTER_ENABLED ? "ENABLED" : "DISABLED");
//     printf("Voltage-Based PWM Multiplier: ENABLED\r\n");
//     printf("Current Multiplier: %d%%\r\n", currentMultiplier);
//     printf("Status Print Interval: %d ms\r\n", PRINT_INTERVAL_MS);
//     printf("===========================================\r\n\n");
// }

// ===================== MAIN FUNCTION =====================

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    USART_Printf_Init(115200);

    printf("Advanced BLDC Motor Controller v1.0\r\n");
    printf("SystemClk: %d Hz\r\n", SystemCoreClock);
    printf("Features: Fast Ramp + Voltage Adaptation + Triple Protection\r\n");

    // Initialize all peripherals
    ADCConfig();
    EXTIO_INT_INIT();
    TIM2_Init();
    DMA_Tx_Init(DMA1_Channel1, (u32)&ADC1->RDATAR, (u32)ADCBuffer, 3);
    DMA_Cmd(DMA1_Channel1, ENABLE);
    ADC_SoftwareStartConvCmd(ADC1, ENABLE);

    printf("Initialization complete. Starting control loop...\r\n");

    // Small delay to let ADC stabilize
    for (volatile int i = 0; i < 100000; i++);

    while (1)
    {
        // Core control functions
        initial_condition_INIT();                    // Handle hall sensor states
        ADCflag_with_startup_detection();           // Process ADC with all protections
        current_calculation();                      // Calculate current from ADC
        voltage_calculation();                      // Calculate voltage from ADC
        update_multiplier_if_needed();              // Update voltage-based multiplier
        handle_startup_detection_sequence();       // Handle startup state machine

        // Timer-based status printing (compact format for production)
        if ((system_ms_counter - last_print_time) >= PRINT_INTERVAL_MS)
        {
//            print_status_with_startup_detection();  // Print status
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

// ===================== UART COMMAND HANDLER (OPTIONAL) =====================
/*
Uncomment and modify this section to enable UART command debugging:

void process_uart_command(char* command)
{
    if (strcmp(command, "status") == 0) {
        // Add detailed status function here
    } else if (strcmp(command, "config") == 0) {
        print_system_configuration();
    } else if (strcmp(command, "test") == 0) {
        test_startup_detection();
    } else if (strcmp(command, "reset") == 0) {
        reset_startup_detection();
    } else {
        printf("Commands: status, config, test, reset\r\n");
    }
}
*/
