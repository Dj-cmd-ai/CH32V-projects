/*
 * 74HC164 + Individual LEDs - Custom Blinking and Constant Patterns
 * Control LEDs with different patterns (constant ON, constant OFF, blinking)
 *
 * Hardware Connections:
 * PB14 -> 74HC164 Data (Pin 1 & 2 - A & B inputs tied together)
 * PB0  -> 74HC164 Clock (Pin 8)
 */

#include "debug.h"
#include "ch32v20x.h"

#define DATA_PIN    GPIO_Pin_14   // PB14
#define CLOCK_PIN   GPIO_Pin_0    // PB0
#define GPIO_PORT   GPIOB         // Both pins on GPIOB

#define BLINK_DELAY 1000  // Blink rate in milliseconds

// LED definitions
#define LED_1   0x01  // Qa
#define LED_2   0x02  // Qb
#define LED_3   0x04  // Qc
#define LED_4   0x08  // Qd
#define LED_5   0x10  // Qe
#define LED_6   0x20  // Qf
#define LED_7   0x40  // Qg
#define LED_8   0x80  // Qh

// **CUSTOMIZE YOUR PATTERNS HERE**
#define CONSTANT_ON_LEDS    (LED_6 | LED_1)           // LEDs that stay ON always
#define CONSTANT_OFF_LEDS   (LED_2 | LED_5 )           // LEDs that stay OFF always
#define BLINKING_LEDS       (LED_8 | LED_7 | LED_4 | LED_3)   // LEDs that blink
#define UNUSED_LEDS         ( )                   // LEDs not used (stay OFF)

volatile uint8_t blink_state = 0;

// Function prototypes
void GPIO_Config(void);
void ShiftOut_Byte(uint8_t data);
void Update_LED_Pattern(void);
void Demo_Different_Patterns(void);
void Set_Custom_Pattern(uint8_t constant_on, uint8_t blinking, uint8_t blink_state);

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
    printf("CH32V203C6T6 - Custom LED Patterns\r\n");
    printf("Constant ON: LEDs 1,3\r\n");
    printf("Blinking: LEDs 5,6,7\r\n");
    printf("Constant OFF: LEDs 2,4,8\r\n\r\n");

    // Initialize GPIO
    GPIO_Config();

    printf("Starting custom LED patterns...\r\n\r\n");

    while(1)
    {
        // **CHOOSE YOUR MODE:**

        // MODE 1: Use predefined custom pattern
        Update_LED_Pattern();

        // MODE 2: Demo different patterns (uncomment to use)
        // Demo_Different_Patterns();

        Delay_Ms(BLINK_DELAY);
    }
}

/*********************************************************************
 * @fn      GPIO_Config
 * @brief   Configure GPIO pins for 74HC164 control
 * @return  none
 */
void GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // Enable GPIOB clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB14 (Data) and PB0 (Clock) as push-pull outputs
    GPIO_InitStructure.GPIO_Pin = DATA_PIN | CLOCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIO_PORT, &GPIO_InitStructure);

    // Initialize pins to LOW
    GPIO_ResetBits(GPIO_PORT, DATA_PIN);
    GPIO_ResetBits(GPIO_PORT, CLOCK_PIN);
}

/*********************************************************************
 * @fn      ShiftOut_Byte
 * @brief   Shift out 8 bits to 74HC164
 * @param   data - 8-bit data to shift out (LSB first)
 * @return  none
 */
void ShiftOut_Byte(uint8_t data)
{
    uint8_t i;

    for(i = 0; i < 8; i++)
    {
        // Set data line (LSB first - bit 0 goes to Qa)
        if(data & (1 << i))
        {
            GPIO_SetBits(GPIO_PORT, DATA_PIN);    // Set PB14 HIGH
        }
        else
        {
            GPIO_ResetBits(GPIO_PORT, DATA_PIN);  // Set PB14 LOW
        }

        // Small delay for signal setup
        Delay_Us(2);

        // Clock pulse - rising edge shifts data
        GPIO_SetBits(GPIO_PORT, CLOCK_PIN);      // Set PB0 HIGH
        Delay_Us(2);                             // Clock pulse width
        GPIO_ResetBits(GPIO_PORT, CLOCK_PIN);    // Set PB0 LOW
        Delay_Us(2);                             // Clock recovery time
    }
}

/*********************************************************************
 * @fn      Update_LED_Pattern
 * @brief   Update LEDs with custom pattern (constant + blinking)
 * @return  none
 */
void Update_LED_Pattern(void)
{
    uint8_t pattern = 0x00;

    // Toggle blink state
    blink_state = !blink_state;

    // Add constant ON LEDs
    pattern |= CONSTANT_ON_LEDS;

    // Add blinking LEDs (only when blink_state is ON)
    if(blink_state)
    {
        pattern |= BLINKING_LEDS;
    }

    // Constant OFF and unused LEDs are automatically 0

    printf("Pattern: 0x%02X - Constant ON: 0x%02X, Blinking: 0x%02X (%s)\r\n",
           pattern, CONSTANT_ON_LEDS, BLINKING_LEDS, blink_state ? "ON" : "OFF");

    ShiftOut_Byte(pattern);
}

/*********************************************************************
 * @fn      Set_Custom_Pattern
 * @brief   Set custom LED pattern with specified constant and blinking LEDs
 * @param   constant_on - LEDs that stay constantly ON
 * @param   blinking - LEDs that blink
 * @param   blink_state - current state of blinking LEDs (1=ON, 0=OFF)
 * @return  none
 */
void Set_Custom_Pattern(uint8_t constant_on, uint8_t blinking, uint8_t blink_state)
{
    uint8_t pattern = constant_on;  // Start with constant ON LEDs

    if(blink_state)
    {
        pattern |= blinking;  // Add blinking LEDs when they should be ON
    }

    ShiftOut_Byte(pattern);
}

/*********************************************************************
 * @fn      Demo_Different_Patterns
 * @brief   Demo different LED patterns
 * @return  none
 */
void Demo_Different_Patterns(void)
{
    static uint8_t demo_step = 0;
    static uint8_t step_counter = 0;

    step_counter++;

    switch(demo_step)
    {
        case 0: // All LEDs blink together
            printf("Demo: All LEDs blinking\r\n");
            Set_Custom_Pattern(0x00, 0xFF, blink_state);
            if(step_counter >= 10) { demo_step++; step_counter = 0; }
            break;

        case 1: // Alternating pattern
            printf("Demo: Alternating pattern\r\n");
            if(blink_state)
                Set_Custom_Pattern(0x55, 0x00, 0);  // 0b01010101
            else
                Set_Custom_Pattern(0xAA, 0x00, 0);  // 0b10101010
            if(step_counter >= 10) { demo_step++; step_counter = 0; }
            break;

        case 2: // Only odd LEDs constant, even LEDs blink
            printf("Demo: Odd constant, even blinking\r\n");
            Set_Custom_Pattern(0x55, 0xAA, blink_state);  // Odd ON, even blink
            if(step_counter >= 10) { demo_step++; step_counter = 0; }
            break;

        case 3: // Running pattern
            printf("Demo: Running light\r\n");
            static uint8_t running_pos = 0;
            Set_Custom_Pattern(0x00, (1 << running_pos), 1);
            if(step_counter >= 2) {
                running_pos = (running_pos + 1) % 8;
                step_counter = 0;
                if(running_pos == 0) { demo_step++; }
            }
            break;

        default:
            demo_step = 0;  // Reset demo
            break;
    }

    blink_state = !blink_state;  // Toggle for next time
}
// Git integration test -Thu Aug 28 15:27:58 IST 2025
