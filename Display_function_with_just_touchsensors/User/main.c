/********************************** (C) COPYRIGHT *******************************
 * Combined Penta Touch Input + Master Switch + 74HC164 Control + Buzzer
 * Master switch: PA3 touch controls entire system on/off
 * First touch: Sequential LEDs Q7->Q1, speed control (sends ESP32 commands)
 * Second touch: Toggle PB6, swing control (local only)
 * Third touch: Toggle PB7, pump control (local only)
 * Fourth touch: Timer 1-7 hours, auto system shutdown
 * Fifth touch: Toggle PB5, 8-hour timer mode
 *
 * ESP32 COMMAND PROTOCOL VERSION: Sends MO, MOF, ISP, LR commands
 *******************************************************************************/

#include "debug.h"

/* Global variables */
// Master system control
u8 system_enabled = 0;     // Master system state (0=OFF, 1=ON)
u8 master_touch_ready = 1; // Ready for master touch

// First touch - Speed control
u8 touch_state = 0;        // Current first touch state
u8 touch_ready = 1;        // Ready for next first touch
u8 led_step = 0;           // Current LED step (0-7)

// Second touch - Swing control (LOCAL ONLY)
u8 touch_state_2 = 0;      // Current second touch state
u8 touch_ready_2 = 1;      // Ready for next second touch
u8 pb6_state = 0;          // PB6 current state (0=LOW, 1=HIGH)

// Third touch - Pump control (LOCAL ONLY)
u8 touch_state_3 = 0;      // Current third touch state
u8 touch_ready_3 = 1;      // Ready for next third touch
u8 pb7_state = 0;          // PB7 current state (0=LOW, 1=HIGH)

// Fourth touch - Timer 1-7 hours
u8 touch_state_4 = 0;      // Current fourth touch state
u8 touch_ready_4 = 1;      // Ready for next fourth touch
u8 timer_hours = 0;        // Timer setting (0=OFF, 1-7=hours)
volatile u32 timer_seconds = 0;     // Countdown in seconds
volatile u8 timer_active = 0;       // Timer running flag

// Fifth touch - 8 hour timer
u8 touch_state_5 = 0;      // Current fifth touch state
u8 touch_ready_5 = 1;      // Ready for next fifth touch
u8 pb5_state = 0;          // PB5 current state (0=ready/blink, 1=active/constant)
volatile u32 timer_8h_seconds = 0;  // 8-hour countdown in seconds
volatile u8 timer_8h_active = 0;    // 8-hour timer running flag

// Blink states for ready indicators (synchronized)
volatile uint8_t ready_blink_state = 0;    // Master blink state for synchronization
volatile uint16_t blink_counter = 0;       // Single counter for synchronized blinking

// Timer counter (1 second intervals)
volatile u32 second_counter = 0;

// Debug variables (commented out for production)
// volatile u16 debug_counter = 0;
// #define DEBUG_PRINT_INTERVAL 20

// Touch thresholds
#define TOUCH_THRESHOLD 3090
#define TOUCH_THRESHOLD_2 3500
#define TOUCH_THRESHOLD_3 2940
#define TOUCH_THRESHOLD_4 2660
#define TOUCH_THRESHOLD_5 3140
#define MASTER_TOUCH_THRESHOLD 3360

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

// Timer indicator pins
#define CONTROL_PIN_4A GPIO_Pin_4   // PB4
#define CONTROL_PORT_4A GPIOB
#define CONTROL_PIN_4B GPIO_Pin_3   // PB3
#define CONTROL_PORT_4B GPIOB
#define CONTROL_PIN_4C GPIO_Pin_15  // PA15
#define CONTROL_PORT_4C GPIOA

// Buzzer control pin
#define BUZZER_PIN  GPIO_Pin_15   // PB15
#define BUZZER_PORT GPIOB

// Synchronized blink interval
#define BLINK_INTERVAL 8

// Master switch LED patterns
#define MASTER_LED_OFF  0x20  // System OFF indicator
#define MASTER_LED_ON   0x10  // System ON indicator

// Function prototypes
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
 * @brief   Make buzzer beep for specified duration
 */
void Buzzer_Beep(u16 duration_ms)
{
    GPIO_SetBits(BUZZER_PORT, BUZZER_PIN);
    Delay_Ms(duration_ms);
    GPIO_ResetBits(BUZZER_PORT, BUZZER_PIN);
}

/*********************************************************************
 * @fn      Update_Timer_Indicators
 * @brief   Update PA15, PB3, PB4 based on timer hours (1-7)
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
            led_step = 0;
            pb6_state = 0;
            pb7_state = 0;
            pb5_state = 0;
            timer_8h_active = 0;

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
 * @brief   Configure GPIO pins for HC164 controllers and control pins
 */
void GPIO_HC164_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Configure HC164 pins
    GPIO_InitStructure.GPIO_Pin = DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = CLOCK_PIN;
    GPIO_Init(CLOCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = DATA_PIN_2 | CLOCK_PIN_2;
    GPIO_Init(GPIO_PORT_2, &GPIO_InitStructure);

    // Configure control pins
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
 * @brief   Shift out 8 bits to first 74HC164
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
 * @brief   Shift out 8 bits to second 74HC164
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
 * @brief   Update master switch LED indicator
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

    // Timer indicator (0x04)
    if(timer_hours == 0) {
        if(ready_blink_state) pattern |= 0x04;
    } else {
        pattern |= 0x04;
    }

    // Add master LED
    pattern |= MASTER_LED_ON;

    ShiftOut_Byte_HC164_2(pattern);

    // Handle 8-hour timer indicator (PB5)
    if(pb5_state == 0) {
        if(ready_blink_state) {
            GPIO_SetBits(CONTROL_PORT_5, CONTROL_PIN_5);
        } else {
            GPIO_ResetBits(CONTROL_PORT_5, CONTROL_PIN_5);
        }
    } else {
        GPIO_SetBits(CONTROL_PORT_5, CONTROL_PIN_5);
    }
}

/*********************************************************************
 * @fn      Touch_Key_Init
 * @brief   Initialize touch sensors and peripherals
 */
void Touch_Key_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure={0};
    ADC_InitTypeDef ADC_InitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE );
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    // Configure touch sensor pins as analog inputs
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_HC164_Config();

    ShiftOut_Byte(0x00);
    Update_Master_LED();
    Update_Timer_Indicators(0);

    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_Cmd(ADC1, ENABLE);
    TKey1->CTLR1 |= (1<<26)|(1<<24);
}

/*********************************************************************
 * @fn      Touch_Key_Adc
 * @brief   Read touch sensor ADC value
 */
u16 Touch_Key_Adc(u8 ch)
{
    ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_7Cycles5 );
    TKey1->IDATAR1 = 0x3;
    TKey1->RDATAR = 0x2;
    while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
    return (uint16_t) TKey1->RDATAR;
}

/*********************************************************************
 * @fn      Master_Touch_Process
 * @brief   Process master switch (PA3) - Sends ESP32 commands
 */
void Master_Touch_Process(u16 adc_value)
{
    if(adc_value < MASTER_TOUCH_THRESHOLD && master_touch_ready) {
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
        master_touch_ready = 0;
    }

    if(adc_value >= MASTER_TOUCH_THRESHOLD && !master_touch_ready) {
        master_touch_ready = 1;
    }
}

/*********************************************************************
 * @fn      Touch_Process
 * @brief   Process speed control (PA1) - Sends ESP32 commands
 */
void Touch_Process(u16 adc_value)
{
    if(!system_enabled) return;

    if(adc_value < TOUCH_THRESHOLD && touch_ready) {
        Buzzer_Beep(100);

        led_step++;
        if(led_step >= 8) {
            led_step = 0;
            printf("LR\r\n");  // ESP32 Limit Reached command - reset to speed 0
        } else {
            printf("ISP\r\n");  // ESP32 Increase Speed command
        }

        ShiftOut_Byte(led_patterns[led_step]);
        touch_ready = 0;
    }

    if(adc_value >= TOUCH_THRESHOLD && !touch_ready) {
        touch_ready = 1;
    }
}

/*********************************************************************
 * @fn      Touch_Process_2
 * @brief   Process swing control (PA2) - LOCAL CONTROL ONLY
 */
void Touch_Process_2(u16 adc_value)
{
    if(!system_enabled) return;

    if(adc_value < TOUCH_THRESHOLD_2 && touch_ready_2) {
        Buzzer_Beep(100);

        pb6_state = !pb6_state;

        if(pb6_state) {
            GPIO_SetBits(CONTROL_PORT_2, CONTROL_PIN_2);
        } else {
            GPIO_ResetBits(CONTROL_PORT_2, CONTROL_PIN_2);
        }

        touch_ready_2 = 0;
    }

    if(adc_value >= TOUCH_THRESHOLD_2 && !touch_ready_2) {
        touch_ready_2 = 1;
    }
}

/*********************************************************************
 * @fn      Touch_Process_3
 * @brief   Process pump control (PA4) - LOCAL CONTROL ONLY
 */
void Touch_Process_3(u16 adc_value)
{
    if(!system_enabled) return;

    if(adc_value < TOUCH_THRESHOLD_3 && touch_ready_3) {
        Buzzer_Beep(100);

        pb7_state = !pb7_state;

        if(pb7_state) {
            GPIO_SetBits(CONTROL_PORT_3, CONTROL_PIN_3);
        } else {
            GPIO_ResetBits(CONTROL_PORT_3, CONTROL_PIN_3);
        }

        touch_ready_3 = 0;
    }

    if(adc_value >= TOUCH_THRESHOLD_3 && !touch_ready_3) {
        touch_ready_3 = 1;
    }
}

/*********************************************************************
 * @fn      Touch_Process_4
 * @brief   Process timer control (PA5) - 1-7 hours
 */
void Touch_Process_4(u16 adc_value)
{
    if(!system_enabled) return;

    if(adc_value < TOUCH_THRESHOLD_4 && touch_ready_4) {
        Buzzer_Beep(100);

        timer_hours++;
        if(timer_hours > 7) timer_hours = 0;

        if(timer_hours > 0) {
            timer_seconds = timer_hours * 3600; // Convert hours to seconds
            timer_active = 1;
        } else {
            timer_active = 0;
        }

        Update_Timer_Indicators(timer_hours);
        touch_ready_4 = 0;
    }

    if(adc_value >= TOUCH_THRESHOLD_4 && !touch_ready_4) {
        touch_ready_4 = 1;
    }
}

/*********************************************************************
 * @fn      Touch_Process_5
 * @brief   Process 8-hour timer (PA6)
 */
void Touch_Process_5(u16 adc_value)
{
    if(!system_enabled) return;

    if(adc_value < TOUCH_THRESHOLD_5 && touch_ready_5) {
        Buzzer_Beep(100);

        pb5_state = !pb5_state;

        if(pb5_state) {
            // Start 8-hour timer
            timer_8h_seconds = 8 * 3600; // 8 hours for production
            // timer_8h_seconds = 80; // 80 seconds for testing
            timer_8h_active = 1;
        } else {
            // Stop 8-hour timer
            timer_8h_active = 0;
            timer_8h_seconds = 0;
        }

        touch_ready_5 = 0;
    }

    if(adc_value >= TOUCH_THRESHOLD_5 && !touch_ready_5) {
        touch_ready_5 = 1;
    }
}

/*********************************************************************
 * @fn      main
 * @brief   Main program
 */
int main(void)
{
    u16 ADC_vals[6];

    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(9600);

    printf("CH32V203C6T6 Display Board - ESP32 Protocol\r\n");
    printf("Commands: MO (Motor ON), MOF (Motor OFF)\r\n");
    printf("Commands: ISP (Increase Speed), LR (Limit Reset)\r\n");
    printf("===========================================\r\n");

    Touch_Key_Init();

    while(1)
    {
        // Read all touch sensors
        ADC_vals[0] = Touch_Key_Adc(ADC_Channel_3);  // Master (PA3)
        ADC_vals[1] = Touch_Key_Adc(ADC_Channel_1);  // Speed (PA1)
        ADC_vals[2] = Touch_Key_Adc(ADC_Channel_2);  // Swing (PA2)
        ADC_vals[3] = Touch_Key_Adc(ADC_Channel_4);  // Pump (PA4)
        ADC_vals[4] = Touch_Key_Adc(ADC_Channel_5);  // Timer 1-7h (PA5)
        ADC_vals[5] = Touch_Key_Adc(ADC_Channel_6);  // Timer 8h (PA6)

        // Process all touches
        Master_Touch_Process(ADC_vals[0]);  // Sends MO/MOF
        Touch_Process(ADC_vals[1]);         // Sends ISP/LR
        Touch_Process_2(ADC_vals[2]);       // Local swing control
        Touch_Process_3(ADC_vals[3]);       // Local pump control
        Touch_Process_4(ADC_vals[4]);       // Local timer control
        Touch_Process_5(ADC_vals[5]);       // Local 8h timer

        // Handle synchronized blinking and timer updates
        if(system_enabled) {
            blink_counter++;
            if(blink_counter >= BLINK_INTERVAL) {
                ready_blink_state = !ready_blink_state;
                Update_Ready_Indicators();
                blink_counter = 0;
            }

            // Update timers every ~1 second (10 * 100ms = 1000ms)
            second_counter++;
            if(second_counter >= 10) {
                Timer_Update();
                second_counter = 0;
            }
        }

        Delay_Ms(100);
    }
}
