/*
 * 74HC164 + 7-Segment Display Control - CH32V203C6T6
 * Display digits 1-9 sequentially
 *
 * Hardware Connections:
 * PB13 -> 74HC164 Data (Pin 1 & 2 - A & B inputs tied together)
 * PA7  -> 74HC164 Clock (Pin 8)
 * Clear pin tied to VCC (always enabled)
 *
 * 74HC164 to 7-Segment mapping (based on your schematic):
 * Qa -> Segment A, Qb -> Segment B, Qc -> Segment C, Qd -> Segment D
 * Qe -> Segment E, Qf -> Segment F, Qg -> Segment G, Qh -> DP (decimal point)
 */

#include "debug.h"
#include "ch32v20x.h"

#define DATA_PIN    GPIO_Pin_13   // PB13
#define CLOCK_PIN   GPIO_Pin_7    // PA7
#define DATA_PORT   GPIOB         // For PB13
#define CLOCK_PORT  GPIOA         // For PA7

#define DISPLAY_DELAY 1000  // Delay between digits in milliseconds

// 7-segment patterns for digits 1-9 (common cathode)
// Bit order: DP G F E D C B A (Qh to Qa)
//            7  6 5 4 3 2 1 0
const uint8_t digit_patterns[11] = {

        0xFC,  // 0: A+B+C+D+E+F = 0x10+0x08+0x04+0x80+0x40+0x20 = 0xFC
        0xFC,  // 0: A+B+C+D+E+F = 0x10+0x08+0x04+0x80+0x40+0x20 = 0xFC
        0x0C,  // 1: B+C = 0x08+0x04 = 0x0C
        0xD9,  // 2: A+B+G+E+D = 0x10+0x08+0x01+0x40+0x80 = 0xD9
        0x9D,  // 3: A+B+G+C+D = 0x10+0x08+0x01+0x04+0x80 = 0x9D
        0x2D,  // 4: F+G+B+C = 0x20+0x01+0x08+0x04 = 0x2D
        0xB5,  // 5: A+F+G+C+D = 0x10+0x20+0x01+0x04+0x80 = 0xB5
        0xF5,  // 6: A+F+G+E+D+C = 0x10+0x20+0x01+0x40+0x80+0x04 = 0xF5
        0x1C,  // 7: A+B+C = 0x10+0x08+0x04 = 0x1C
        0xFD,  // 8: A+B+C+D+E+F+G = 0x10+0x08+0x04+0x80+0x40+0x20+0x01 = 0xFD
        0xBD   // 9: A+B+C+D+F+G = 0x10+0x08+0x04+0x80+0x20+0x01 = 0xBD

};

volatile uint8_t current_digit = 1;

// Function prototypes
void GPIO_Config(void);
void Delay_Ms(uint32_t ms);
void ShiftOut_Byte(uint8_t data);
void Display_Digit(uint8_t digit);
void Clear_Display(void);
void Test_All_Segments(void);

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
    USART_Printf_Init(115200);  // Initialize UART for debug

    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("CH32V203C6T6 - 74HC164 + 7-Segment Display\r\n");
    printf("Displaying digits 1-9 sequentially\r\n");

    // Initialize GPIO
    GPIO_Config();

    printf("GPIO initialized\r\n");

    // Test all segments first
    printf("Testing all segments...\r\n");
    Test_All_Segments();
    Delay_Ms(2000);

    // Clear display
    Clear_Display();
    Delay_Ms(500);

    printf("Starting digit sequence 1-9...\r\n\r\n");

    while(1)
    {

        // Display digits 1 through 9
        for(current_digit = 1; current_digit <= 10; current_digit++)
        {
            Display_Digit(current_digit);

            printf("Displaying digit: %d (Pattern: 0x%02X)\r\n",
                   current_digit, digit_patterns[current_digit]);

            Delay_Ms(DISPLAY_DELAY);
        }

        printf("Sequence complete, restarting...\r\n\r\n");

        // Brief pause before restarting sequence
        Clear_Display();
        Delay_Ms(500);
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

    // Enable GPIOA and GPIOB clocks
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    // Configure PB13 (Data) as push-pull output
    GPIO_InitStructure.GPIO_Pin = DATA_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(DATA_PORT, &GPIO_InitStructure);

    // Configure PA7 (Clock) as push-pull output
    GPIO_InitStructure.GPIO_Pin = CLOCK_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(CLOCK_PORT, &GPIO_InitStructure);

    // Initialize pins to LOW
    GPIO_ResetBits(DATA_PORT, DATA_PIN);
    GPIO_ResetBits(CLOCK_PORT, CLOCK_PIN);
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
            GPIO_SetBits(DATA_PORT, DATA_PIN);    // Set PB13 HIGH
        }
        else
        {
            GPIO_ResetBits(DATA_PORT, DATA_PIN);  // Set PB13 LOW
        }

        // Small delay for signal setup
        Delay_Us(2);

        // Clock pulse - rising edge shifts data
        GPIO_SetBits(CLOCK_PORT, CLOCK_PIN);      // Set PA7 HIGH
        Delay_Us(2);                              // Clock pulse width
        GPIO_ResetBits(CLOCK_PORT, CLOCK_PIN);    // Set PA7 LOW
        Delay_Us(2);                              // Clock recovery time
    }
}

/*********************************************************************
 * @fn      Display_Digit
 * @brief   Display a specific digit (0-9) on 7-segment display
 * @param   digit - digit to display (0-9)
 * @return  none
 */
void Display_Digit(uint8_t digit)
{
    if(digit > 10)
    {
        printf("Error: Invalid digit %d\r\n", digit);
        return;
    }

    uint8_t pattern = digit_patterns[digit];

    // Print segment breakdown for debugging
    printf("Digit %d segments: ", digit);
    if(pattern & 0x10) printf("A ");
    if(pattern & 0x08) printf("B ");
    if(pattern & 0x04) printf("C ");
    if(pattern & 0x80) printf("D ");
    if(pattern & 0x40) printf("E ");
    if(pattern & 0x20) printf("F ");
    if(pattern & 0x01) printf("G ");
    if(pattern & 0x02) printf("DP ");
    printf("\r\n");

    // Send pattern to shift register
    ShiftOut_Byte(pattern);
}

/*********************************************************************
 * @fn      Clear_Display
 * @brief   Turn off all segments
 * @return  none
 */
void Clear_Display(void)
{
    printf("Clearing display\r\n");
    ShiftOut_Byte(0x00);  // All segments OFF
    Delay_Ms(1000);
}

/*********************************************************************
 * @fn      Test_All_Segments
 * @brief   Test all segments by turning them on (displays "8")
 * @return  none
 */
void Test_All_Segments(void)
{
    printf("Testing all segments (displaying '8')\r\n");
    ShiftOut_Byte(0xFD);  // All segments ON except DP
    Delay_Ms(1000);
}

/*********************************************************************
 * @fn      Delay_Ms
 * @brief   Millisecond delay function
 * @param   ms - delay time in milliseconds
 * @return  none
 */
//void Delay_Ms(uint32_t ms)
//{
//    uint32_t i;
//    for(i = 0; i < ms; i++)
//    {
//        Delay_Us(1000);
//    }
//}
