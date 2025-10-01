/********************************** (C) COPYRIGHT *******************************
 * Complete IR Remote Cooler Control System with 8-Hour Timer
 * Power (0x10): System on/off
 * Speed (0x04): Sequential LEDs Q7->Q1, speed control (SPEED_0 to SPEED_7)
 * Swing (0x02): Toggle PB6, swing control
 * Pump (0x08): Toggle PB7, pump control
 * Timer (0x20): Timer OFF->1H->2H->3H->4H->5H->6H->7H->8H->OFF (cycles 0-8)
 *
 * Timer Steps: 1-7 use binary indicators, Step 8 activates 8-hour timer on PB5
 *******************************************************************************/

#include "debug.h"

/* Global variables */
// Master system control
u8 system_enabled = 0;     // Master system state (0=OFF, 1=ON)

// Speed control
u8 led_step = 0;           // Current LED step (0-7)

// Swing control
u8 pb6_state = 0;          // PB6 current state (0=LOW, 1=HIGH)

// Pump control
u8 pb7_state = 0;          // PB7 current state (0=LOW, 1=HIGH)

// Timer control (extended to include 8-hour timer)
u8 timer_hours = 0;        // Timer setting (0=OFF, 1-7=hours, 8=8h-timer)
volatile u32 timer_seconds = 0;     // Countdown in seconds (1-7 hour timer)
volatile u8 timer_active = 0;       // 1-7 hour timer running flag

// 8-hour timer control (activated when timer_hours = 8)
volatile u32 timer_8h_seconds = 0;  // 8-hour countdown in seconds
volatile u8 timer_8h_active = 0;    // 8-hour timer running flag
u8 pb5_state = 0;          // PB5 current state (0=ready/blink, 1=active/constant)

// Blink states for ready indicators (synchronized)
volatile uint8_t ready_blink_state = 0;
volatile uint16_t blink_counter = 0;

// Dual timing counters
volatile uint16_t ms_100_counter = 0;  // Counter for 100ms intervals
volatile u32 second_counter = 0;       // Counter for 1 second intervals (10 * 100ms)

// IR variables
volatile u32 pulse_width = 0;
u32 pulse_buffer[50];
u8 pulse_count = 0;
u8 receiving = 0;
u32 last_pulse_time = 0;

#define TIMEOUT_MS 3
#define DEBOUNCE_TIME_MS 50

// Timing intervals
#define BLINK_INTERVAL 8        // 8 * 100ms = 800ms blink period
#define MS_100_INTERVAL 100     // 100 * 1ms = 100ms
#define SECOND_INTERVAL 10      // 10 * 100ms = 1000ms (1 second)

/* CUSTOM PROTOCOL TIMING */
#define CUSTOM_START_HIGH_MIN    10000
#define CUSTOM_START_HIGH_MAX    11100
#define CUSTOM_START_LOW_MIN     1600
#define CUSTOM_START_LOW_MAX     1800
#define CUSTOM_BIT_HIGH_MIN      400
#define CUSTOM_BIT_HIGH_MAX      12500
#define CUSTOM_BIT0_LOW_MIN      450
#define CUSTOM_BIT0_LOW_MAX      650
#define CUSTOM_BIT1_LOW_MIN      1600
#define CUSTOM_BIT1_LOW_MAX      1750
#define CUSTOM_TOTAL_BITS        11

/* IR Commands */
#define CMD_POWER  0x10
#define CMD_PUMP   0x08
#define CMD_SWING  0x02
#define CMD_SPEED  0x04
#define CMD_TIMER  0x20

typedef struct {
    u16 data_code;
    u8 device_id;
    u8 command;
} CustomIR_t;

// First HC164 Control pins (main sequence)
#define DATA_PIN    GPIO_Pin_13   // PB13
#define CLOCK_PIN   GPIO_Pin_7    // PA7
#define GPIO_PORT   GPIOB         // For PB13
#define CLOCK_PORT  GPIOA         // For PA7

// Second HC164 Control pins (ready indicators)
#define DATA_PIN_2    GPIO_Pin_14   // PB14
#define CLOCK_PIN_2   GPIO_Pin_0    // PB0
#define GPIO_PORT_2   GPIOB         // Both pins on GPIOB

// Control pins
#define CONTROL_PIN_2 GPIO_Pin_6    // PB6 (swing)
#define CONTROL_PORT_2 GPIOB
#define CONTROL_PIN_3 GPIO_Pin_7    // PB7 (pump)
#define CONTROL_PORT_3 GPIOB
#define CONTROL_PIN_5 GPIO_Pin_5    // PB5 (8h timer)
#define CONTROL_PORT_5 GPIOB

// Timer indicator pins (for 1-7 hour timer)
#define CONTROL_PIN_4A GPIO_Pin_4   // PB4
#define CONTROL_PORT_4A GPIOB
#define CONTROL_PIN_4B GPIO_Pin_3   // PB3
#define CONTROL_PORT_4B GPIOB
#define CONTROL_PIN_4C GPIO_Pin_15  // PA15
#define CONTROL_PORT_4C GPIOA

// Buzzer control pin
#define BUZZER_PIN  GPIO_Pin_15   // PB15
#define BUZZER_PORT GPIOB

// Master switch LED patterns
#define MASTER_LED_OFF  0x20  // System OFF indicator
#define MASTER_LED_ON   0x10  // System ON indicator

void ShiftOut_Byte(uint8_t data);
void Update_Master_LED(void);

// LED sequence patterns for speed indication
const u8 led_patterns[] = {
    0x00,   // Step 0: All OFF (Speed 0)
    0x80,   // Step 1: Q7 ON (Speed 1)
    0xC0,   // Step 2: Q7,Q6 ON (Speed 2)
    0xE0,   // Step 3: Q7,Q6,Q5 ON (Speed 3)
    0xF0,   // Step 4: Q7,Q6,Q5,Q4 ON (Speed 4)
    0xF8,   // Step 5: Q7,Q6,Q5,Q4,Q3 ON (Speed 5)
    0xFC,   // Step 6: Q7,Q6,Q5,Q4,Q3,Q2 ON (Speed 6)
    0xFE,   // Step 7: Q7,Q6,Q5,Q4,Q3,Q2,Q1 ON (Speed 7)
};

/*********************************************************************
 * @fn      Buzzer_Beep
 */
void Buzzer_Beep(u16 duration_ms)
{
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
    Delay_Ms(duration_ms);
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/*********************************************************************
 * @fn      Update_Timer_Indicators
 * @brief   Update PA15, PB3, PB4 based on timer hours (1-7 only)
 */
void Update_Timer_Indicators(u8 hours)
{
    // Reset all indicators first
    GPIO_ResetBits(CONTROL_PORT_4A, CONTROL_PIN_4A);  // PB4 LOW
    GPIO_ResetBits(CONTROL_PORT_4B, CONTROL_PIN_4B);  // PB3 LOW
    GPIO_ResetBits(CONTROL_PORT_4C, CONTROL_PIN_4C);  // PA15 LOW

    if(hours > 0 && hours <= 7) {
        // Binary representation of hours (1-7)
        if(hours & 0x01) GPIO_SetBits(CONTROL_PORT_4C, CONTROL_PIN_4C);  // PA15 = bit 0
        if(hours & 0x02) GPIO_SetBits(CONTROL_PORT_4B, CONTROL_PIN_4B);  // PB3  = bit 1
        if(hours & 0x04) GPIO_SetBits(CONTROL_PORT_4A, CONTROL_PIN_4A);  // PB4  = bit 2
    }
    // Note: 8-hour timer (hours=8) uses PB5, not binary indicators
}

/*********************************************************************
 * @fn      Timer_Update
 * @brief   Update timer countdown (call every second)
 */
void Timer_Update(void)
{
    // Update 1-7 hour timer
    if(timer_active && timer_seconds > 0) {
        timer_seconds--;
        if(timer_seconds == 0) {
            // Timer expired - turn off entire system
            system_enabled = 0;
            timer_hours = 0;
            timer_active = 0;
            led_step = 0;
            pb6_state = 0;
            pb7_state = 0;
            pb5_state = 0;
            timer_8h_active = 0;

            // Send ESP32 Motor OFF command
            printf("MOF\r\n");

            // Turn off all control outputs
            GPIO_ResetBits(CONTROL_PORT_2, CONTROL_PIN_2);  // PB6 OFF
            GPIO_ResetBits(CONTROL_PORT_3, CONTROL_PIN_3);  // PB7 OFF
            GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);  // PB5 OFF
            Update_Timer_Indicators(0);                     // Timer indicators OFF
            ShiftOut_Byte(0x00);                           // First HC164 OFF
            Update_Master_LED();                           // Show system OFF
        }
    }

    // Update 8-hour timer
    if(timer_8h_active && timer_8h_seconds > 0) {
        timer_8h_seconds--;
        if(timer_8h_seconds == 0) {
            // 8-hour timer expired - turn off entire system
            system_enabled = 0;
            timer_hours = 0;
            timer_active = 0;
            timer_8h_active = 0;
            led_step = 0;
            pb6_state = 0;
            pb7_state = 0;
            pb5_state = 0;

            // Send ESP32 Motor OFF command
            printf("MOF\r\n");

            GPIO_ResetBits(CONTROL_PORT_2, CONTROL_PIN_2);
            GPIO_ResetBits(CONTROL_PORT_3, CONTROL_PIN_3);
            GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);
            Update_Timer_Indicators(0);
            ShiftOut_Byte(0x00);
            Update_Master_LED();
        }
    }
}

/*********************************************************************
 * @fn      GPIO_HC164_Config
 */
void GPIO_HC164_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure HC164 pins
    GPIO_InitStructure.GPIO_Pin = DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CLOCK_PIN;
    GPIO_Init(CLOCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DATA_PIN_2 | CLOCK_PIN_2;
    GPIO_Init(GPIO_PORT_2, &GPIO_InitStructure);

    // Configure control pins (including PB5 for 8h timer)
    GPIO_InitStructure.GPIO_Pin = CONTROL_PIN_2 | CONTROL_PIN_3 | CONTROL_PIN_5 | CONTROL_PIN_4A | CONTROL_PIN_4B;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CONTROL_PIN_4C;
    GPIO_Init(CONTROL_PORT_4C, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = BUZZER_PIN;
    GPIO_Init(BUZZER_PORT, &GPIO_InitStructure);

    // Initialize all pins to LOW
    GPIO_ResetBits(GPIO_PORT, DATA_PIN);
    GPIO_ResetBits(CLOCK_PORT, CLOCK_PIN);
    GPIO_ResetBits(GPIO_PORT_2, DATA_PIN_2 | CLOCK_PIN_2);
    GPIO_ResetBits(GPIOB, CONTROL_PIN_2 | CONTROL_PIN_3 | CONTROL_PIN_5 | CONTROL_PIN_4A | CONTROL_PIN_4B);
    GPIO_ResetBits(CONTROL_PORT_4C, CONTROL_PIN_4C);
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/*********************************************************************
 * @fn      ShiftOut_Byte
 */
void ShiftOut_Byte(uint8_t data)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if(data & (1 << i))
            GPIO_SetBits(GPIO_PORT, DATA_PIN);
        else
            GPIO_ResetBits(GPIO_PORT, DATA_PIN);

        Delay_Us(2);
        GPIO_SetBits(CLOCK_PORT, CLOCK_PIN);
        Delay_Us(2);
        GPIO_ResetBits(CLOCK_PORT, CLOCK_PIN);
        Delay_Us(2);
    }
}

/*********************************************************************
 * @fn      ShiftOut_Byte_HC164_2
 */
void ShiftOut_Byte_HC164_2(uint8_t data)
{
    uint8_t i;
    for(i = 0; i < 8; i++)
    {
        if(data & (1 << i))
            GPIO_SetBits(GPIO_PORT_2, DATA_PIN_2);
        else
            GPIO_ResetBits(GPIO_PORT_2, DATA_PIN_2);

        Delay_Us(2);
        GPIO_SetBits(GPIO_PORT_2, CLOCK_PIN_2);
        Delay_Us(2);
        GPIO_ResetBits(GPIO_PORT_2, CLOCK_PIN_2);
        Delay_Us(2);
    }
}

/*********************************************************************
 * @fn      Update_Master_LED
 */
void Update_Master_LED(void)
{
    uint8_t master_pattern;

    if(system_enabled) {
        master_pattern = MASTER_LED_ON;  // 0x10 - System ON
    } else {
        master_pattern = MASTER_LED_OFF; // 0x20 - System OFF
    }

    ShiftOut_Byte_HC164_2(master_pattern);
}

/*********************************************************************
 * @fn      Update_Ready_Indicators
 * @brief   Update all ready indicators with synchronized behavior
 */
void Update_Ready_Indicators(void)
{
    if(!system_enabled) {
        Update_Master_LED();
        return;
    }

    uint8_t pattern = 0x00;

    // Speed indicator (0x80)
    if(led_step == 0) {
        if(ready_blink_state) pattern |= 0x80;
    } else {
        pattern |= 0x80;
    }

    // Swing indicator (0x40)
    if(pb6_state == 0) {
        if(ready_blink_state) pattern |= 0x40;
    } else {
        pattern |= 0x40;
    }

    // Pump indicator (0x08)
    if(pb7_state == 0) {
        if(ready_blink_state) pattern |= 0x08;
    } else {
        pattern |= 0x08;
    }

    // Timer indicator (0x04) - includes both 1-7h and 8h timer
    if(timer_hours == 0) {
        if(ready_blink_state) pattern |= 0x04;
    } else {
        pattern |= 0x04;
    }

    // Add master LED
    pattern |= MASTER_LED_ON;

    ShiftOut_Byte_HC164_2(pattern);

    // Handle 8-hour timer indicator (PB5) - CORRECTED LOGIC
    if(timer_hours == 8) {
        // 8-hour timer IS SELECTED - constant ON
        GPIO_SetBits(CONTROL_PORT_5, CONTROL_PIN_5);
    } else {
        // 8-hour timer is NOT SELECTED - blink to show it's available
        if(ready_blink_state) {
            GPIO_SetBits(CONTROL_PORT_5, CONTROL_PIN_5);
        } else {
            GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);
        }
    }
}

/*********************************************************************
 * @fn      TIM2_Init
 */
void TIM2_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = 72 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_Cmd(TIM2, ENABLE);
}

/*********************************************************************
 * @fn      IR_Init
 */
void IR_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource12);

    EXTI_InitStructure.EXTI_Line = EXTI_Line12;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = EXTI15_10_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

/*********************************************************************
 * @fn      Reset_IR_Buffer
 */
void Reset_IR_Buffer(void)
{
    pulse_count = 0;
    receiving = 0;
    for(int i = 0; i < 50; i++) {
        pulse_buffer[i] = 0;
    }
}

/*********************************************************************
 * @fn      Decode_Custom_Protocol
 */
u8 Decode_Custom_Protocol(CustomIR_t *ir_code)
{
    if(pulse_count < 23 || pulse_count > 25) {
        return 0;
    }

    if(pulse_buffer[0] < CUSTOM_START_HIGH_MIN || pulse_buffer[0] > CUSTOM_START_HIGH_MAX) {
        return 0;
    }
    if(pulse_buffer[1] < CUSTOM_START_LOW_MIN || pulse_buffer[1] > CUSTOM_START_LOW_MAX) {
        return 0;
    }

    u16 data = 0;
    u8 bit_count = 0;

    for(int i = 2; i < pulse_count - 1 && bit_count < CUSTOM_TOTAL_BITS; i += 2) {
        u32 pulse = pulse_buffer[i];
        u32 space = pulse_buffer[i + 1];

        if(pulse >= CUSTOM_BIT_HIGH_MIN && pulse <= CUSTOM_BIT_HIGH_MAX) {
            if(space >= CUSTOM_BIT0_LOW_MIN && space <= CUSTOM_BIT0_LOW_MAX) {
                data = (data << 1) | 0;
                bit_count++;
            } else if(space >= CUSTOM_BIT1_LOW_MIN && space <= CUSTOM_BIT1_LOW_MAX) {
                data = (data << 1) | 1;
                bit_count++;
            }
        }
    }

    if(bit_count < 8) {
        return 0;
    }

    ir_code->data_code = data;
    ir_code->device_id = (data >> 7) & 0x0F;
    ir_code->command = data & 0x7F;

    return 1;
}

/*********************************************************************
 * @fn      Process_Power_Command
 */
void Process_Power_Command(void)
{
    system_enabled = !system_enabled;

    if(system_enabled) {
        printf("MO\r\n");  // ESP32 Motor ON command
        Buzzer_Beep(1000);
    } else {
        printf("MOF\r\n");  // ESP32 Motor OFF command
        Buzzer_Beep(200);

        // Reset all states
        led_step = 0;
        pb6_state = 0;
        pb7_state = 0;
        pb5_state = 0;
        timer_hours = 0;
        timer_active = 0;
        timer_8h_active = 0;

        // Turn off all outputs
        GPIO_ResetBits(CONTROL_PORT_2, CONTROL_PIN_2);
        GPIO_ResetBits(CONTROL_PORT_3, CONTROL_PIN_3);
        GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);
        Update_Timer_Indicators(0);
        ShiftOut_Byte(0x00);
    }

    Update_Master_LED();
}

/*********************************************************************
 * @fn      Process_Speed_Command
 */
void Process_Speed_Command(void)
{
    if(!system_enabled) return;

    Buzzer_Beep(100);

    led_step++;
    if(led_step >= 8) {
        led_step = 0;
        printf("LR\r\n");  // ESP32 Limit Reached command - reset to speed 0
    } else {
        printf("ISP\r\n");  // ESP32 Increase Speed command
    }

    ShiftOut_Byte(led_patterns[led_step]);
}

/*********************************************************************
 * @fn      Process_Swing_Command
 */
void Process_Swing_Command(void)
{
    if(!system_enabled) return;

    Buzzer_Beep(100);

    pb6_state = !pb6_state;

    if(pb6_state) {
        GPIO_SetBits(CONTROL_PORT_2, CONTROL_PIN_2);
    } else {
        GPIO_ResetBits(CONTROL_PORT_2, CONTROL_PIN_2);
    }
    // No serial output for swing control (local only)
}

/*********************************************************************
 * @fn      Process_Pump_Command
 */
void Process_Pump_Command(void)
{
    if(!system_enabled) return;

    Buzzer_Beep(100);

    pb7_state = !pb7_state;

    if(pb7_state) {
        GPIO_SetBits(CONTROL_PORT_3, CONTROL_PIN_3);
    } else {
        GPIO_ResetBits(CONTROL_PORT_3, CONTROL_PIN_3);
    }
    // No serial output for pump control (local only)
}

/*********************************************************************
 * @fn      Process_Timer_Command
 * @brief   Process timer control command (cycles 0->1->2->3->4->5->6->7->8->0)
 */
void Process_Timer_Command(void)
{
    if(!system_enabled) return;

    Buzzer_Beep(100);

    timer_hours++;
    if(timer_hours > 8) timer_hours = 0;  // Extended to include 8-hour timer

    // Handle 1-7 hour timer
    if(timer_hours >= 1 && timer_hours <= 7) {
        timer_seconds = timer_hours * 3600; // Convert hours to seconds
        timer_active = 1;
        timer_8h_active = 0;  // Disable 8-hour timer
        pb5_state = 0;        // Reset 8-hour indicator
        Update_Timer_Indicators(timer_hours);
        GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);  // Turn off PB5
    }
    // Handle 8-hour timer (special case)
    else if(timer_hours == 8) {
        timer_active = 0;     // Disable 1-7 hour timer
        timer_8h_seconds = 8 * 3600;  // 8 hours in seconds
        timer_8h_active = 1;  // Enable 8-hour timer
        Update_Timer_Indicators(0);  // Clear binary indicators (not used for 8h)
        // PB5 will be set HIGH by Update_Ready_Indicators() in main loop
    }
    // Handle timer OFF
    else {
        timer_active = 0;
        timer_8h_active = 0;
        Update_Timer_Indicators(0);  // Clear binary indicators
        // PB5 will blink by Update_Ready_Indicators() to show 8h function available
    }
    // No serial output for timer control (local only)
}

/*********************************************************************
 * @fn      Timer_Update
 * @brief   Update timer countdown (call every second)
 */

/*********************************************************************
 * @fn      Process_IR_Signal
 */
void Process_IR_Signal(void)
{
    if(pulse_count < 5) {
        return;
    }

    CustomIR_t ir_code;

    if(Decode_Custom_Protocol(&ir_code)) {
//        printf("Decoded command: 0x%02X\r\n", ir_code.command);

        switch(ir_code.command) {
            case CMD_POWER:
                Process_Power_Command();
                break;
            case CMD_SPEED:
                Process_Speed_Command();
                break;
            case CMD_SWING:
                Process_Swing_Command();
                break;
            case CMD_PUMP:
                Process_Pump_Command();
                break;
            case CMD_TIMER:
                Process_Timer_Command();
                break;
            default:
                printf("Unknown command: 0x%02X\r\n", ir_code.command);
                break;
        }
    }
}

/*********************************************************************
 * @fn      EXTI15_10_IRQHandler
 */
void EXTI15_10_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void EXTI15_10_IRQHandler(void)
{
    static u32 last_time = 0;

    if(EXTI_GetITStatus(EXTI_Line12) != RESET) {
        u32 current_time = TIM_GetCounter(TIM2);
        u8 pin_state = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12);

        if(last_time != 0) {
            if(current_time >= last_time) {
                pulse_width = current_time - last_time;
            } else {
                pulse_width = (0xFFFF - last_time) + current_time + 1;
            }

            if(pulse_count < 50) {
                pulse_buffer[pulse_count] = pulse_width;
                pulse_count++;
                receiving = 1;
                last_pulse_time = current_time;
            }
        }

        last_time = current_time;

        if(pin_state == 0) {
            EXTI_InitTypeDef EXTI_InitStructure = {0};
            EXTI_InitStructure.EXTI_Line = EXTI_Line12;
            EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
            EXTI_InitStructure.EXTI_LineCmd = ENABLE;
            EXTI_Init(&EXTI_InitStructure);
        } else {
            EXTI_InitTypeDef EXTI_InitStructure = {0};
            EXTI_InitStructure.EXTI_Line = EXTI_Line12;
            EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
            EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
            EXTI_InitStructure.EXTI_LineCmd = ENABLE;
            EXTI_Init(&EXTI_InitStructure);
        }

        EXTI_ClearITPendingBit(EXTI_Line12);
    }
}

/*********************************************************************
 * @fn      main
 */
int main(void)
{
    u32 current_time = 0;
    static u32 last_process_time = 0;

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(9600);

    printf("=== COMPLETE IR REMOTE COOLER CONTROL WITH 8H TIMER ===\r\n");
    printf("Power: 0x10 | Speed: 0x04 | Swing: 0x02 | Pump: 0x08\r\n");
    printf("Timer: 0x20 (cycles: OFF->1H->2H->3H->4H->5H->6H->7H->8H->OFF)\r\n");

    GPIO_HC164_Config();
    TIM2_Init();
    IR_Init();
    Reset_IR_Buffer();

    ShiftOut_Byte(0x00);
    Update_Master_LED();
    Update_Timer_Indicators(0);

    printf("System Ready - All functions + 8H timer available\r\n");

    while(1)
    {
        current_time = TIM_GetCounter(TIM2);

        // Handle IR signal processing (fast 1ms timing)
        if(receiving && pulse_count > 0) {
            u32 time_since_last;
            if(current_time >= last_pulse_time) {
                time_since_last = current_time - last_pulse_time;
            } else {
                time_since_last = (0xFFFF - last_pulse_time) + current_time + 1;
            }

            if(time_since_last > (TIMEOUT_MS * 1000)) {
                u32 time_since_process;
                if(current_time >= last_process_time) {
                    time_since_process = current_time - last_process_time;
                } else {
                    time_since_process = (0xFFFF - last_process_time) + current_time + 1;
                }

                if(time_since_process > (DEBOUNCE_TIME_MS * 1000)) {
                    Process_IR_Signal();
                    last_process_time = current_time;
                }

                Reset_IR_Buffer();
            }
        }

        // Handle 100ms timing intervals
        if(system_enabled) {
            ms_100_counter++;
            if(ms_100_counter >= MS_100_INTERVAL) {  // Every 100ms
                ms_100_counter = 0;

                // Handle blinking at proper 100ms intervals
                blink_counter++;
                if(blink_counter >= BLINK_INTERVAL) {  // Every 800ms
                    ready_blink_state = !ready_blink_state;
                    Update_Ready_Indicators();
                    blink_counter = 0;
                }

                // Handle timer countdown every 1 second (10 * 100ms)
                second_counter++;
                if(second_counter >= SECOND_INTERVAL) {
                    Timer_Update();
                    second_counter = 0;
                }
            }
        }

        Delay_Ms(1);  // Fast 1ms loop for IR responsiveness
    }
}
