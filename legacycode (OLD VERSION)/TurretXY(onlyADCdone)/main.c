#include <stdint.h>
#include <stm32l432xx.h>
#include <stdio.h>
#include <errno.h>
#include "stm32l4xx.h"
#include "eeng1030_lib.h"
#include "display.h"
#include "spi.h"
#include <sys/unistd.h>  // STD FILE OUT, STD FILE IN

// Function Prototypes
void setup(void);
void initADC(void);
void readADC_All(uint16_t *pot1_val, uint16_t *pot2_val);
int scaleValue(int value);
void initSerial(uint32_t baudrate);
void eputc(char c);

// Variables
volatile uint32_t milliseconds;

// MAIN
int main()
{
    setup();

    while (1)
    {
        uint16_t pot1_val = 0, pot2_val = 0;
        readADC_All(&pot1_val, &pot2_val);
        printf("Pot1: %u, Pot2: %u\n",
              pot1_val,
              pot2_val);

        delay_ms(500); // Print every 500ms
    }
}

// SETUP
void setup(void)
{
    initClocks(); // Setup system clocks
    SysTick->LOAD = 80000-1; // 1ms SysTick interrupt
    SysTick->CTRL = 7; // Enable SysTick
    __asm(" cpsie i "); // Enable global interrupts

    RCC->AHB2ENR |= (1 << 0) | (1 << 1); // Enable GPIOA and GPIOB

    // ADC Pins
    pinMode(GPIOA, 0, 3); // PA0 -> Potentiometer 1
    pinMode(GPIOA, 1, 3); // PA1 -> Potentiometer 2

    initADC();
    initSerial(9600); // UART @ 9600 baud
}

void initADC(void)
{
    RCC->AHB2ENR |= (1 << 13); // ADC clock
    RCC->CCIPR |= (1 << 29) | (1 << 28); // System clock for ADC

    ADC1_COMMON->CCR = (1 << 22) | (1 << 16); // Vref enable, HCLK/1

    ADC1->CR = (1 << 28); // Voltage regulator
    delay_ms(100);
    ADC1->CR |= (1 << 31); // Calibration
    while (ADC1->CR & (1 << 31));

    ADC1->CFGR = 0; // NO CONTINUOUS MODE

    // Set sequence
    ADC1->SQR1 &= ~(0x3FFFFFF << 6);
    ADC1->SQR1 |= (5 << 6) | (6 << 12);
    ADC1->SQR1 &= ~(0xF << 0);
    ADC1->SQR1 |= (1 << 0); // L=1 => two conversions

    ADC1->SMPR1 = (7 << (3 * 5)) | (7 << (3 * 6));

    ADC1->CR |= (1 << 0); // Enable ADC
    while (!(ADC1->ISR & (1 << 0))); // Wait ready
}

void readADC_All(uint16_t *pot1_val, uint16_t *pot2_val)
{
    ADC1->ISR |= (1 << 3); // Clear EOS
    ADC1->CR |= (1 << 2);  // Start ADC conversion manually

    // Wait for first conversion
    while (!(ADC1->ISR & (1 << 2)));
    *pot1_val = ADC1->DR;
    ADC1->ISR |= (1 << 2); // Clear EOC

    // Wait for second conversion
    while (!(ADC1->ISR & (1 << 2)));
    *pot2_val = ADC1->DR;
    ADC1->ISR |= (1 << 2); // Clear EOC
}


// void initADC(void)
// {
//     // Enable ADC Clock
//     RCC->AHB2ENR |= (1 << 13); // Enable ADC1 clock
//     RCC->CCIPR |= (1 << 29) | (1 << 28); // Select system clock for ADC

//     // Common ADC config
//     ADC1_COMMON->CCR = (1 << 22) | (1 << 16); // Enable Vrefint, Clock Prescaler HCLK/1

//     // ADC Voltage Regulator Enable
//     ADC1->CR = (1 << 28); // ADVREGEN = 1
//     delay_ms(100); // Wait for stabilization
//     ADC1->CR |= (1 << 31); // Start Calibration
//     while (ADC1->CR & (1 << 31)); // Wait for calibration to finish

//     // Continuous conversion mode
//     ADC1->CFGR = (1 << 13); // CONT = 1

//     // Setup sequence:
//     ADC1->SQR1 &= ~(0x3FFFFFF << 6); // Clear all sequence bits
//     ADC1->SQR1 |= (5 << 6);   // 1st conversion: Channel 5 (PA0)
//     ADC1->SQR1 |= (6 << 12);  // 2nd conversion: Channel 6 (PA1)

//     ADC1->SQR1 &= ~(0xF << 0); // Clear L bits
//     ADC1->SQR1 |= (1 << 0);    // L = 1 => two conversions total

//     // Sampling times: Max for better accuracy
//     ADC1->SMPR1 = (7 << (3 * 5)) | (7 << (3 * 6)); // Ch5 and Ch6 sample time = 640.5 ADC clk cycles

//     // Enable ADC
//     ADC1->CR |= (1 << 0); // ADEN
//     while (!(ADC1->ISR & (1 << 0))); // Wait for ADC Ready
// }

// void readADC_All(uint16_t *pot1_val, uint16_t *pot2_val)
// {
//     ADC1->ISR |= (1 << 3); // Clear End of Sequence (EOS) flag
//     ADC1->CR |= (1 << 2);  // Start conversion

//     // Read first conversion
//     while (!(ADC1->ISR & (1 << 2))); // Wait for End of Conversion (EOC)
//     *pot1_val = ADC1->DR;            // Read PA0 (ADC Channel 5)
//     ADC1->ISR |= (1 << 2);            // Clear EOC

//     // Read second conversion
//     while (!(ADC1->ISR & (1 << 2))); // Wait for End of Conversion (EOC)
//     *pot2_val = ADC1->DR;            // Read PA1 (ADC Channel 6)
//     ADC1->ISR |= (1 << 2);            // Clear EOC
// }


// Scale 0–4095 to 1–100
int scaleValue(int value)
{
    int minValue = 0;
    int maxValue = 4095;
    int minScale = 1;
    int maxScale = 100;

    if (value < minValue) value = minValue;
    if (value > maxValue) value = maxValue;

    int scaledValue = ((value - minValue) * (maxScale - minScale) / (maxValue - minValue)) + minScale;
    return scaledValue;
}

// Setup UART Serial
void initSerial(uint32_t baudrate)
{
    RCC->AHB2ENR |= (1 << 0); // Enable GPIOA
    pinMode(GPIOA, 2, 2); // PA2 AF mode
    selectAlternateFunction(GPIOA, 2, 7); // AF7 for USART2_TX

    RCC->APB1ENR1 |= (1 << 17); // Enable USART2

    const uint32_t CLOCK_SPEED = 80000000;
    uint32_t BaudRateDivisor = CLOCK_SPEED / baudrate;

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = 0;
    USART2->CR3 = (1 << 12); // Disable overrun detection
    USART2->BRR = BaudRateDivisor;
    USART2->CR1 = (1 << 3); // Enable transmitter
    USART2->CR1 |= (1<<0);
}

// Redirect printf to UART
int _write(int file, char *data, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) // STDOUT_FILENO, STDERR_FILENO
    {
        errno=EBADF;
        return -1;
    }
    while (len--){
        eputc(*data);
        data++;
    }
    return 0;
}

void eputc(char c)
{
    while (!(USART2->ISR & (1 << 7))); // TXE must be 1
    USART2->TDR = c;
}

// void eputc(char c)
// {
//     while ((USART2->ISR & (1 << 6)) == 0); // Wait for TXE
//     USART2->TDR = c;
// }

// SysTick for delay
void SysTick_Handler(void)
{
    milliseconds++;
}
