#include <stdint.h>
#include <stm32l432xx.h>
#include <stdio.h>
#include <errno.h>
#include "eeng1030_lib.h"
#include "display.h"
#include "spi.h"
#include <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO
#include <math.h> // Used for angle calculations involving sin and cosine

/*  Code for display.c and spi.c were updated with new functions and variables.
    display.c Code:
    - void printNumberSHORT(uint16_t Number, uint16_t x, uint16_t y, uint16_t ForeColour, uint16_t BackColour) 
      Displays digits without leading zeroes

    - void drawSemiCircle(uint16_t x0, uint16_t y0, uint16_t radius, uint16_t Colour);
      Draws a semi-circle around a centerpoint

    spi.c Code:
    - extern SPI_TypeDef *ACTIVE_SPI
      Pointer that defaults to SPI1

    - void waitForSPIReady(void);
      Waits for SPI BSY flag to be set to 0. It is used to ensure that sets of SPI transfers are complete before continuing
*/

/*  
    FUNCTIONS
*/

// Setup
void setup(void); // Initialize all peripherals

// ADC
void initADC(void); // Initialize ADC to use channels 5 and 15
void readADC_All(uint16_t *pot1_val, uint16_t *pot2_val); // Reads ADC at PA8 and PA9 and stores value

// USART
void initSerial(uint32_t baudrate); // Start USART (necessary for printf)
void eputc(char c); // Part of UART writing logic (necessary for printf)

// PWM
void initTIM1_PWM_Servo(void); // PWM Timer
uint16_t pot_to_servo(uint16_t pot_value); // Converts ADC to correct PWM duty cycle

// TIM2 Timer Capture
void initTIM2_Capture(void); // Timer Capture

// EXTI5 Interrupt
void initScrollButton(void); // Initialize EXTI5 to PB5

// Display
void displayFunctions(void); // Display mode logic
void dispStats(void); // Used to display measurement characters in display

// Tick delay
void delay(volatile uint32_t dly); 

// Servo initial calibration
void servo_startup_ramp_stepper(void); // Calibrates servos towards 90 degrees at startup

/* 
    GLOBAL VARIABLES
*/ 

// SysTick variables
volatile uint32_t milliseconds;

// ADC rolling average variables
#define AVERAGE_WINDOW 8 // Number of ADC readings to average from
uint16_t pot1_samples[AVERAGE_WINDOW];
uint16_t pot2_samples[AVERAGE_WINDOW];
uint32_t pot1_sum = 0;
uint32_t pot2_sum = 0;
uint8_t sample_idx = 0;
uint16_t prev_pot1_avg = 0;
uint16_t prev_pot2_avg = 0;
#define DISABLE_ROLLING_AVERAGE 0 // 0 -> Enabled, 1 -> Disabled

// ADC deadzonevariables
#define DEADZONE_MIN 1 // Deadzone minimum ADC count
#define DEADZONE_MAX 2 // 
#define DISABLE_DEADZONE_FOR_TESTING 0 // 0 -> Enabled, 1 -> Disabled

#define SERVO_PWM_MIN 30  // corresponds to ~0.6ms pulse
#define SERVO_PWM_MAX 118 // corresponds to ~2.4ms pulse

#define POT_ADC_MIN 0     // min ADC count for potentiometer
#define POT_ADC_MAX 4000  // max ADC count for potentiometer

// Timer Capture Variables
volatile float pwm1_freq = 0;
volatile float pwm1_duty = 0;
volatile uint8_t pwm1_ready = 0;
volatile float pwm2_freq = 0;
volatile float pwm2_duty = 0;
volatile uint8_t pwm2_ready = 0;

// Display Variables
#define SCREEN_HEIGHT 80
#define SCREEN_WIDTH 160
volatile uint8_t displayMode = 0; // 0 = angles, 1 = scrolling, 2 = static 
volatile uint16_t x_pos1 = 0; // Inital x-position for servo x
volatile uint16_t x_pos2 = 80; // Initial x-position for servo y
uint16_t saved_widthHigh1 = 20;
uint16_t saved_widthLow1 = 20;
uint16_t saved_widthHigh2 = 20;
uint16_t saved_widthLow2 = 20;
uint16_t waveform_width_per_period = 40;

// Ramp Servos Flag
volatile uint8_t calibrated = 0;

int main()
{
    ACTIVE_SPI = SPI1; // Set to SPI1
    setup();
    fillRectangle(0, 0, 160, 80, RGBToWord(0, 0, 0)); // Clear screen
    // Display initializing for 200 ms to show display works before starting the loop
    printText("Initializing...", 20, 40, RGBToWord(255,255,255), RGBToWord(0,0,0));
    delay_ms(200);
    fillRectangle(0, 0, 160, 80, RGBToWord(0, 0, 0)); // Clear screen
    x_pos1 = 0;
    x_pos2 = 80;

    while (1)
    {
        // Reset servos to 90 degrees upon startup.
        if (!calibrated)
            {
                servo_startup_ramp_stepper();
            }
        uint16_t pot1_now = 0;
        uint16_t pot2_now = 0;
    
        readADC_All(&pot1_now, &pot2_now); // Read ADC chan 5 and chan 15

        // Rolling average smoothing
        pot1_sum -= pot1_samples[sample_idx]; // Remove the oldest samples
        pot2_sum -= pot2_samples[sample_idx];

        pot1_samples[sample_idx] = pot1_now; // Replace the now empty index with newest sample
        pot2_samples[sample_idx] = pot2_now;

        pot1_sum += pot1_now; // Add new sample into the sum
        pot2_sum += pot2_now;

        sample_idx = (sample_idx + 1) % AVERAGE_WINDOW; // Move index to next sample and reset if greater than the average window

        // Debug rolling average toggle
        #if DISABLE_ROLLING_AVERAGE
        uint16_t pot1_avg = pot1_now; // Use direct ADC values
        uint16_t pot2_avg = pot2_now;
        #else
        uint16_t pot1_avg = pot1_sum / AVERAGE_WINDOW; // Use the rolling average ADC values
        uint16_t pot2_avg = pot2_sum / AVERAGE_WINDOW;
        #endif

        // Deadzone Logic
        // Find absolute difference between current average and previous average ADC values
        // Note: This can be used without rolling average if it is disabled
        uint16_t diff1 = (pot1_avg > prev_pot1_avg) ? (pot1_avg - prev_pot1_avg) : (prev_pot1_avg - pot1_avg);
        uint16_t diff2 = (pot2_avg > prev_pot2_avg) ? (pot2_avg - prev_pot2_avg) : (prev_pot2_avg - pot2_avg);

        // If absolute difference is greater than 50, use minimum deadzone.
        // If greater than 50, use maximum deadzone minus the diff/10 to transition smoothly between min and max
        uint16_t dynamic_deadzone1 = (diff1 > 50) ? DEADZONE_MIN : DEADZONE_MAX - (diff1 / 10);
        uint16_t dynamic_deadzone2 = (diff2 > 50) ? DEADZONE_MIN : DEADZONE_MAX - (diff2 / 10);

        //
        if (dynamic_deadzone1 > DEADZONE_MAX) dynamic_deadzone2 = DEADZONE_MAX; // Ensure maximum deadzone
        if (dynamic_deadzone2 > DEADZONE_MAX) dynamic_deadzone2 = DEADZONE_MAX;
        if (dynamic_deadzone1 < DEADZONE_MIN) dynamic_deadzone1 = DEADZONE_MIN; // Ensure minimum deadzone
        if (dynamic_deadzone2 < DEADZONE_MIN) dynamic_deadzone2 = DEADZONE_MIN;

        // Debugging setting for deadzone (skips deadzone if statement)
        #if DISABLE_DEADZONE_FOR_TESTING
        TIM1->CCR1 = pot_to_servo(pot1_avg);
        TIM1->CCR2 = pot_to_servo(pot2_avg);
        prev_pot1_avg = pot1_avg;
        prev_pot2_avg = pot2_avg;
        #else // Normal setting
        if (diff1 > dynamic_deadzone1) // Only write to TIM1 channel 1 if the change in ADC count is greater than the dynamic deadzone
        {
            TIM1->CCR1 = pot_to_servo(pot1_avg);
            prev_pot1_avg = pot1_avg;
        }
        if (diff2 > dynamic_deadzone2) // Same but for TIM1 channel 2
        {
            TIM1->CCR2 = pot_to_servo(pot2_avg);
            prev_pot2_avg = pot2_avg;
        }
        #endif

        // Turn on the display logic
        displayFunctions();

        // Print ADC values
        printf("Pot1: %u | Pot2: %u\n", pot1_avg, pot2_avg);
        delay_ms(50); // Small delay for readability
    }
}

// Setup peripherals
void setup(void)
{
    initClocks();
    SysTick->LOAD = 80000-1; // 1ms SysTick
    SysTick->CTRL = 7;
    __asm(" cpsie i "); // Enable global interrupts

    RCC->AHB2ENR |= (1 << 0) | (1 << 1); // GPIOA and GPIOB

    initSerial(9600);
    initADC();
    initTIM1_PWM_Servo();
    initTIM2_Capture();
    initScrollButton();
    init_display(); // ST7735 display
    fillRectangle(0, 0, 160, 80, RGBToWord(0, 0, 0)); // Clear screen
}

// Function to convert PWM ticks to angle
uint16_t pwmToAngle(uint16_t pwm_ticks)
{
    // 30 ticks = 0 degrees
    // 118 ticks = 180 degrees (decided this through tuning)
    if (pwm_ticks < SERVO_PWM_MIN) pwm_ticks = SERVO_PWM_MIN;
    if (pwm_ticks > SERVO_PWM_MAX) pwm_ticks = SERVO_PWM_MAX;
    return (pwm_ticks - 30) * 180 / (118 - 30);
}

// ADC setup
void initADC(void)
{
    RCC->AHB2ENR |= (1 << 13); // Enable ADC clock
    RCC->CCIPR |= (1 << 29) | (1 << 28); // Select System Clock from CCIPR

    ADC1_COMMON->CCR = (1 << 22) | (1 << 16); // Set VREFEN and prescaler

    ADC1->CR = (1 << 28); // Turn on ADC voltage regulator
    delay_ms(100); // Let it stabilize
    ADC1->CR |= (1 << 31); // Calibrate ADC
    while (ADC1->CR & (1 << 31)); // Wait for calibration to finish

    ADC1->CFGR = 0; // Reset ADC configuration register

    // Set PA0 = CH5, PB0 = CH8
    ADC1->SQR1 &= ~(0x3FFFFFF << 6); // Clear previous
    ADC1->SQR1 |= (5 << 6) | (15 << 12); // Set channel 5 as Rank 1 conversion, and channel 15 as Rank 2
    ADC1->SQR1 &= ~(0xF << 0); // Set length to 1 (this is for 2 conversions 1+1)
    ADC1->SQR1 |= (1 << 0);

    // Longer sampling time to ensure accuracy
    ADC1->SMPR1 = (6 << (3 * 5)) | (6 << (3 * (15-10))); // Longer sampling time for channels 5 and 15

    ADC1->CR |= (1 << 0); // Enable ADC
    while (!(ADC1->ISR & (1 << 0))); // Wait for ADC to be ready
}

// Read ADC channel 5 and channel 15
void readADC_All(uint16_t *pot1_val, uint16_t *pot2_val)
{
    ADC1->ISR |= (1 << 3); // Clear EOS flag
    ADC1->CR |= (1 << 2); // Start conversion

    while (!(ADC1->ISR & (1 << 2))); // Wait for conversion
    *pot1_val = ADC1->DR; // Read first channel (PA0 Channel 5)
    ADC1->ISR |= (1 << 2); // Clear EOC flag

    while (!(ADC1->ISR & (1 << 2))); // Wait for conversion to complete
    *pot2_val = ADC1->DR; // Read second channel (PB0 Channel 15)
    ADC1->ISR |= (1 << 2); // Clear EOC flag
}

// PWM Output for Servos
void initTIM1_PWM_Servo(void)
{
    RCC->APB2ENR |= (1 << 11); // Enable TIM1 clock

    pinMode(GPIOA,8,2); // Set PA8 to alternative function mode
    pinMode(GPIOA,9,2); // Set PA9 to alternative function mode
    selectAlternateFunction(GPIOA,8,1); // Assign TIM1 alternate function to PA8 (CH1)
    selectAlternateFunction(GPIOA,9,1); // and PA9 (CH2)

    TIM1->PSC = 1599; // Prescale System clock (80 MHz / 1600 = 50 kHz)
    // Need 50 Hz for servo motor control
    TIM1->ARR = 999; // ARR PWM period (1000 ticks at 50 Hz = 20 ms -> 50 Hz)
    TIM1->CCMR1 |= (6 << 4) | (6 << 12); // Set output compare mode to PMW Mode 1 for CH1 and CH2
    TIM1->CCMR1 |= (1 << 3) | (1 << 11); // Enable output preload for both channels
    TIM1->CCER |= (1 << 0) | (1 << 4); // Enable output on CH1 and CH2
    TIM1->BDTR |= (1 << 15); // Set Main Output Enable (necessary for advanced control timers)
    TIM1->CR1 |= (1 << 7) | (1 << 0); // Set auto-reload preload enable and enable the timer
}

// Converts ADC value to the PWM ticks
uint16_t pot_to_servo(uint16_t pot_value)
{
    // Clip ADC value to a consistent range to reduce jittering
    if (pot_value < POT_ADC_MIN) pot_value = POT_ADC_MIN;
    if (pot_value > POT_ADC_MAX) pot_value = POT_ADC_MAX;

    // Scale ADC range to PWM range
    uint32_t pwm_ticks = SERVO_PWM_MIN +
        ((uint32_t)(pot_value - POT_ADC_MIN) * (SERVO_PWM_MAX - SERVO_PWM_MIN)) /
        (POT_ADC_MAX - POT_ADC_MIN);

    // Ensure PWM ticks are within servo mechanical limits (30-119 ticks is 0-180 degrees)
    if (pwm_ticks < SERVO_PWM_MIN) pwm_ticks = SERVO_PWM_MIN;
    if (pwm_ticks > SERVO_PWM_MAX) pwm_ticks = SERVO_PWM_MAX;

    return (uint16_t)pwm_ticks;
}

// Ensures that the motors start at 90 degrees upon startup
void servo_startup_ramp_stepper(void)
{
    static uint8_t initializedCalibration = 0;
    static int16_t start1;
    static int16_t start2;
    static uint16_t target = 74;   // Target 90 degrees (1.5ms pulse width)
    static uint32_t last_ramp_time = 0;
    static uint8_t startUpFlag = 0; // Flag to mark whether to set TIM1 CCRx to default value or not

    if(!startUpFlag) // Ensure that TIM1 CCRx has a value
    {
        // Default values
        TIM1->CCR1 = 60;
        TIM1->CCR2 = 60;
        // Flag to mark default values are provided
        startUpFlag = 1;
    }

    // If initialize calibration flag is not set
    if (!initializedCalibration)
    {
        start1 = TIM1->CCR1; // Calibrate Servo Y
        start2 = TIM1->CCR2; // Calibrate Servo X
        initializedCalibration = 1; // Set Flag
    }
    // Only ramp up every 10 ms to avoid servos moving too quickly
    if ((milliseconds - last_ramp_time) >= 10)
    {
        last_ramp_time = milliseconds;

        // Ramp start1 toward target
        if (start1 < target)
            start1++;
        else if (start1 > target)
            start1--;

        // Ramp start2 toward target
        if (start2 < target)
            start2++;
        else if (start2 > target)
            start2--;

        TIM1->CCR1 = start1;
        TIM1->CCR2 = start2;

        // Check if both channels reached target
        if (start1 == target && start2 == target)
        {
            calibrated = 1; // Set Flag
        }
    }
}

// --- UART Serial ---
void initSerial(uint32_t baudrate)
{
    RCC->AHB2ENR |= (1 << 0);
    pinMode(GPIOA,2,2);
    selectAlternateFunction(GPIOA,2,7);
    RCC->APB1ENR1 |= (1 << 17);

    USART2->CR1 = 0;
    USART2->CR2 = 0;
    USART2->CR3 = (1 << 12);
    USART2->BRR = (80000000 / baudrate);
    USART2->CR1 |= (1 << 3) | (1 << 0);
}

void eputc(char c)
{
    while (!(USART2->ISR & (1 << 7)));
    USART2->TDR = c;
}

int _write(int file, char *ptr, int len)
{
    if ((file != STDOUT_FILENO) && (file != STDERR_FILENO)) {
        errno=EBADF;
        return -1;
    }
    for (int i = 0; i < len; i++) {
        eputc(ptr[i]);
    }
    return len;
}

// --- TIM2 capture for Servo PWM signals ---
void initTIM2_Capture(void)
{
    RCC->AHB2ENR |= (1 << 0) | (1 << 1); // GPIOA + GPIOB
    RCC->APB1ENR1 |= (1 << 0); // TIM2 clock

    // PA5 = TIM2_CH1
    pinMode(GPIOA,5,2);
    selectAlternateFunction(GPIOA,5,1); // AF1

    // PB3 = TIM2_CH2
    pinMode(GPIOB,3,2);
    selectAlternateFunction(GPIOB,3,1); // AF1

    // TIM2 setup
    TIM2->CCMR1 &= ~(0x3 << 0);
    TIM2->CCMR1 |= (0x1 << 0); // CC1S=01

    TIM2->CCMR1 &= ~(0x3 << 8);
    TIM2->CCMR1 |= (0x1 << 8); // CC2S=01

    TIM2->CCER |= (1 << 0) | (1 << 4); // CC1E + CC2E

    TIM2->DIER |= (1 << 1) | (1 << 2); // CC1IE + CC2IE

    TIM2->PSC = 79; // 1MHz
    TIM2->ARR = 0xFFFFFFFF;

    TIM2->CR1 |= (1 << 0); // Enable timer

    NVIC_EnableIRQ(TIM2_IRQn);
}

// TIM2 interrupt handler (Timer capture)
void TIM2_IRQHandler(void)
{
    static uint32_t rise1_prev = 0, rise1_curr = 0, fall1 = 0; // Timestamp of previous rising, current rising, and current falling edge
    static uint32_t rise2_prev = 0, rise2_curr = 0, fall2 = 0; // Same but for channel 2
    static uint8_t state1 = 0, state2 = 0; // State keeps track if waiting for rising or falling edge

    // Channel 1
    if (TIM2->SR & (1 << 1)) // Check if CC1IF interrupt flag is set
    {
        uint32_t captured = TIM2->CCR1; // Store timer value at input edge

        if (state1 == 0) // First rising edge
        {
            rise1_prev = captured; // Store as previous rising edge
            TIM2->CCER |= (1 << 1); // Set CC1P, capture next falling edge
            state1 = 1; // Ready for next state
        }
        else if (state1 == 1) // Falling edge
        {
            fall1 = captured;  // Store falling edge
            TIM2->CCER &= ~(1 << 1); // Clear CC1P, capture next rising edge
            state1 = 2; // Ready for next state
        }
        else if (state1 == 2) // Rising edge
        {
            rise1_curr = captured; // Store rising edge

            // Note: These calculations below have an overflow handler. Checks if current edge is newer than previous edge,
            // If older take sum them together and take them away from the maximum value for the calculation instead

            // Calculate period (time between two rising edges)
            uint32_t period = (rise1_curr >= rise1_prev) ? (rise1_curr - rise1_prev) : (0xFFFFFFFF - rise1_prev + rise1_curr);
            // Calculate pulse (time between rising edge and falling edge)
            uint32_t pulse = (fall1 >= rise1_prev) ? (fall1 - rise1_prev) : (0xFFFFFFFF - rise1_prev + fall1);

            if (period > 0) // If there is a signal
            {
                pwm1_freq = 1000000.0f / (float)period; // Calculate frequency
                pwm1_duty = ((float)pulse / (float)period) * 100.0f; // Calculate duty
                pwm1_ready = 1; // Set timer capture calculation flag
            }

            rise1_prev = rise1_curr; // Store current rising edge as previous rising edge
            TIM2->CCER |= (1 << 1); // Set CC1P for next falling edge
            state1 = 1; // Enter state 1 again
        }

        TIM2->SR &= ~(1 << 1); // Clear CC1IF interrupt flag
    }

    // Channel 2
    if (TIM2->SR & (1 << 2)) // Check if CC2IF interrupt flag is set
    {
        uint32_t captured = TIM2->CCR2; // Store timer value at input edge

        if (state2 == 0) // First rising edge
        {
            rise2_prev = captured; // Store as previous edge
            TIM2->CCER |= (1 << 5); // Set CC2P, capture next falling edge
            state2 = 1; // Ready for next state
        }
        else if (state2 == 1) // Falling edge
        {
            fall2 = captured; // Store falling edge
            TIM2->CCER &= ~(1 << 5); // Clear CC2P, capture next rising edge
            state2 = 2;
        }
        else if (state2 == 2) // Rising edge
        {
            rise2_curr = captured; // Store rising edge

            // Note: These calculations below have an overflow handler. Checks if current edge is newer than previous edge,
            // If older take sum them together and take them away from the maximum value for the calculation instead

            // Calculate period (time between two rising edges)
            uint32_t period = (rise2_curr >= rise2_prev) ? (rise2_curr - rise2_prev) : (0xFFFFFFFF - rise2_prev + rise2_curr);
            // Calculate pulse (time between rising edge and falling edge)
            uint32_t pulse = (fall2 >= rise2_prev) ? (fall2 - rise2_prev) : (0xFFFFFFFF - rise2_prev + fall2);

            if (period > 0) // If there is a signal
            {
                pwm2_freq = 1000000.0f / (float)period; // Calculate frequency
                pwm2_duty = ((float)pulse / (float)period) * 100.0f; // Calculate duty
                pwm2_ready = 1; // Set timer capture calculation flag
            }

            rise2_prev = rise2_curr; // Store previous rising edge to current rising edge
            TIM2->CCER |= (1 << 5); // Set CC2P for next falling edge
            state2 = 1; // Enter state 1 again
        }

        TIM2->SR &= ~(1 << 2); // Clear CC2IF interrupt flag
    }
}

// PB5 Display Cycle Button Initializer
void initScrollButton(void)
{
    pinMode(GPIOB,5,0); // PB5 digital input
    enablePullUp(GPIOB,5); // Pull-up enabled to stop accidental interrupts

    RCC->APB2ENR |= (1 << 0); // Enable SYSCFG
    SYSCFG->EXTICR[1] &= ~(0xF << 4); // Clear 4 bits for EXTI5
    SYSCFG->EXTICR[1] |= (1 << 4); // Set EXTI5 to PB5

    EXTI->FTSR1 |= (1 << 5); // Trigger on falling edge
    EXTI->IMR1 |= (1 << 5); // Unmask interrupt

    NVIC_EnableIRQ(EXTI9_5_IRQn); // Enable external interrupts
}

void EXTI9_5_IRQHandler(void)
{
    if ((GPIOB->IDR & (1 << 5)) == 0) // If pressed
    {
        delay(10000); // debounce delay
        if ((GPIOB->IDR & (1 << 5)) == 0) // If pressed still after delay
        {
            displayMode++; // Increment displayMode
            if (displayMode > 2) // Cycle through 0, 1, 2, and then reset back to 0
                displayMode = 0;
        }
    }
    EXTI->PR1 |= (1 << 5); // Clear EXTI5 interrupt flag
}

// Tick Delay
void delay(volatile uint32_t dly)
{
    while (dly--);
}

// System Counter
void SysTick_Handler(void)
{
    milliseconds++;
}

// Displays different display modes depending on the display mode flag 
// (0 = aiming angle, 1 = static wave, 2 = scrolling wave)
void displayFunctions(void)
{
    if (displayMode == 2) // Scrolling wave mode
    {
        waveform_width_per_period = 20; // Length of waveform
    }
    else if (displayMode == 1) // Static wave mode
    {
        waveform_width_per_period = 40; // Length of waveform
    }

    if (pwm1_ready) // Check if timer capture calculation flag is ready (pwm1_ready = 1)
    {
        pwm1_ready = 0; // Set flag to 0
        saved_widthHigh1 = (pwm1_duty * waveform_width_per_period * 10) / 100; // Save width of active high
        // If width > than set, use set max width
        if (saved_widthHigh1 > waveform_width_per_period) saved_widthHigh1 = waveform_width_per_period;
        saved_widthLow1 = waveform_width_per_period - saved_widthHigh1; // Width of low is equal to width of max - width of high
    }

    if (pwm2_ready) // Check if timer capture calculation flag is ready (pwm2_ready = 1)
    {
        pwm2_ready = 0; // Set flag to 0
        saved_widthHigh2 = (pwm2_duty * waveform_width_per_period * 10) / 100; // Save width of active high
        // If width > than set, use set max width
        if (saved_widthHigh2 > waveform_width_per_period) saved_widthHigh2 = waveform_width_per_period;
        saved_widthLow2 = waveform_width_per_period - saved_widthHigh2; // Width of low is equal to width of max - width of high
    }

    // Note: waitForSPIReady() ensures that the SPI transfer is fully complete
    if (displayMode == 2) // Scrolling wave mode
    {
        // Servo X
        // Draw vertical white line from (x_pos1, 20) to (x_pos1, 50) to mark low to high
        drawLine(x_pos1, 20, x_pos1, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Draw horizontal green line (high) from (x_pos1, 20) to (x_pos1 + saved_widthHigh1, 20) to mark high pulse width line
        drawLine(x_pos1, 20, x_pos1 + saved_widthHigh1, 20, RGBToWord(0, 255, 0));
        waitForSPIReady();

        // Assign and increment x_pos1 with saved_widthHigh1
        x_pos1 += saved_widthHigh1;

        // Draw vertical white line from (x_pos1, 20) to (x_pos1, 50) to mark high to low
        drawLine(x_pos1, 20, x_pos1, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Draw horizontal red line (low) from (x_pos1, 50) to (x_pos1 + saved_widthLow1, 50) to mark low pulse width line
        drawLine(x_pos1, 50, x_pos1 + saved_widthLow1, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        // Assign and increment x_pos1 with saved_widthLow1
        x_pos1 += saved_widthLow1;

        if (x_pos1 >= 80) // If x_pos1 reaches the end of its scope on the display
        {
            x_pos1 = 0; // Reset servo X's x_pos1 to its initial value
            fillRectangle(0, 15, 80, 50, RGBToWord(0, 0, 0)); // Only clear the wave
            waitForSPIReady();
        }

        // Servo Y
        // Draw vertical white line from (x_pos2, 20) to (x_pos2, 50) to mark low to high
        drawLine(x_pos2, 20, x_pos2, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Draw horizontal green line (high) from (x_pos2, 20) to (x_pos2 + saved_widthHigh2, 20) to mark high pulse width line
        drawLine(x_pos2, 20, x_pos2 + saved_widthHigh2, 20, RGBToWord(0, 255, 0));
        waitForSPIReady();

        // Assign and increment x_pos2 with saved_widthHigh2
        x_pos2 += saved_widthHigh2;

        // Draw vertical white line from (x_pos2, 20) to (x_pos2, 50) to mark high to low
        drawLine(x_pos2, 20, x_pos2, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Draw horizontal red line (low) from (x_pos2, 50) to (x_pos2 + saved_widthLow2, 50) to mark low pulse width line
        drawLine(x_pos2, 50, x_pos2 + saved_widthLow2, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        // Assign and increment x_pos2 with saved_widthLow2
        x_pos2 += saved_widthLow2;

        if (x_pos2 >= 160) // If x-pos2 reaches the end of its scope on the display
        {
            x_pos2 = 80; // Reset servo Y's x-pos2 to its initial value
            fillRectangle(80, 15, 80, 50, RGBToWord(0, 0, 0)); // Only clear the wave
            waitForSPIReady();
        }

        dispStats(); // Show angle and duty
    }
    else if (displayMode == 1) // Static wave mode
    {
        // Clear semi-circles from aiming angle
        drawSemiCircle(40, 45, 30, RGBToWord(0, 0, 0));
        drawSemiCircle(120, 45, 30, RGBToWord(0, 0, 0));

        // Only clear where the waves are shown
        fillRectangle(0, 15, 80, 50, RGBToWord(0, 0, 0));
        waitForSPIReady();
        fillRectangle(80, 15, 80, 50, RGBToWord(0, 0, 0));
        waitForSPIReady();

        // Servo X static
        uint16_t center_start1 = 20; // Initial x coordinate for Servo X static wave

        // Horizontal red line from (center_start1, 50) to (centre_start1 + 10, 50)
        drawLine(center_start1, 50, center_start1 + 10, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        // Vertical white line from (center_start1 + 10, 50) to (center_start1 + 10, 20)
        drawLine(center_start1 + 10, 50, center_start1 + 10, 20, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Horizontal green line from (center_start1 + 10, 20) to (center_start1 + 10 + saved+widthHigh1, 20)
        drawLine(center_start1 + 10, 20, center_start1 + 10 + saved_widthHigh1, 20, RGBToWord(0, 255, 0));
        waitForSPIReady();

        // Vertical white line from (center_start1 + 10 saved_widthHigh1, 20) to (center_start1 + 10 + saved_widthHigh1)
        drawLine(center_start1 + 10 + saved_widthHigh1, 20, center_start1 + 10 + saved_widthHigh1, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Horizontal red line from (center_start1 + 10 + saved_widthHigh1, 50) to (center_start1 + 10 + saved_widthHigh1 + 10, 50)
        drawLine(center_start1 + 10 + saved_widthHigh1, 50, center_start1 + 10 + saved_widthHigh1 + 10, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        // Servo Y static
        uint16_t center_start2 = 90; // Initial x-coordinate for Servo Y static wave

        // Horizontal red line from (center_start2, 50) to (centre_start2 + 10, 50)
        drawLine(center_start2, 50, center_start2 + 10, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        // Vertical white line from (center_start2 + 10, 50) to (center_start2 + 10, 20)
        drawLine(center_start2 + 10, 50, center_start2 + 10, 20, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Horizontal green line from (center_start2 + 10, 20) to (center_start2 + 10 + saved_widthHigh2, 20)
        drawLine(center_start2 + 10, 20, center_start2 + 10 + saved_widthHigh2, 20, RGBToWord(0, 255, 0));
        waitForSPIReady();

        // Vertical white line from (center_start2 + 10 saved_widthHigh2, 20) to (center_start2 + 10 + saved_widthHigh2)
        drawLine(center_start2 + 10 + saved_widthHigh2, 20, center_start2 + 10 + saved_widthHigh2, 50, RGBToWord(255, 255, 255));
        waitForSPIReady();

        // Horizontal red line from (center_start2 + 10 + savedwidthHigh2, 50) to (center_start2 + 10 + saved_widthHigh2 + 10, 50)
        drawLine(center_start2 + 10 + saved_widthHigh2, 50, center_start2 + 10 + saved_widthHigh2 + 10, 50, RGBToWord(255, 0, 0));
        waitForSPIReady();

        dispStats();
    }
    else if (displayMode == 0) // Aiming angle mode
    {
        // Write titles for each graph for servo X and servo Y
        printText("Servo X", 20, 0, RGBToWord(255, 255, 0), RGBToWord(0, 0, 0));
        printText("Servo Y", 100, 0, RGBToWord(255, 255, 0), RGBToWord(0, 0, 0));
        waitForSPIReady();

        // Only clear the aiming dials
        fillRectangle(0, 15, SCREEN_WIDTH, 105, RGBToWord(0, 0, 0));
        waitForSPIReady();

        // Servo X Dial
        drawSemiCircle(40, 45, 30, RGBToWord(255, 255, 255)); // Draw semi-circle
        waitForSPIReady();

        uint16_t angle1 = pwmToAngle(TIM1->CCR2); // Convert PWM from channel 2 to angle
        uint16_t needleX1 = 40 + (int)(30 * cosf((180 + angle1) * 3.14159f / 180.0f)); // Use angle to map line endpoints
        uint16_t needleY1 = 45 + (int)(30 * sinf((180 + angle1) * 3.14159f / 180.0f));

        drawLine(40, 45, needleX1, needleY1, RGBToWord(0, 255, 0)); // Draw a line between those endpoints
        waitForSPIReady();

        // Servo Y Dial
        drawSemiCircle(120, 45, 30, RGBToWord(255, 255, 255)); // Draw semi-circle
        waitForSPIReady();

        uint16_t angle2 = pwmToAngle(TIM1->CCR1); // Convert PWM from channel 1 to angle
        uint16_t needleX2 = 120 + (int)(30 * cosf((180 + angle2) * 3.14159f / 180.0f)); // Use angle to map line endpoints
        uint16_t needleY2 = 45 + (int)(30 * sinf((180 + angle2) * 3.14159f / 180.0f));

        drawLine(120, 45, needleX2, needleY2, RGBToWord(0, 255, 0)); // Draw a line between those endpoints
        waitForSPIReady();

        dispStats(); // Show angle and duty
    }
}

// Used to display angle of servos and duty cycle percentage
void dispStats(void)
{
    // Servo X
    printText("A:", 2, 62, RGBToWord(255,255,255), RGBToWord(0,0,0)); // "Angle:"
    printNumberSHORT((uint16_t)pwmToAngle(TIM1->CCR2), 18, 62, RGBToWord(255,255,255), RGBToWord(0,0,0));
    printText(" deg", 50, 62, RGBToWord(255,255,255), RGBToWord(0,0,0));

    printText("D:", 2, 72, RGBToWord(255,255,255), RGBToWord(0,0,0)); // "Duty:"
    printNumberSHORT((uint16_t)pwm1_duty, 18, 72, RGBToWord(255,255,255), RGBToWord(0,0,0));
    printText(" %", 50, 72, RGBToWord(255,255,255), RGBToWord(0,0,0));

    // Servo Y
    printText("A:", 82, 62, RGBToWord(255,255,255), RGBToWord(0,0,0)); // "Angle:"
    printNumberSHORT((uint16_t)pwmToAngle(TIM1->CCR1), 98, 62, RGBToWord(255,255,255), RGBToWord(0,0,0));
    printText(" deg", 130, 62, RGBToWord(255,255,255), RGBToWord(0,0,0));

    printText("D:", 82, 72, RGBToWord(255,255,255), RGBToWord(0,0,0)); // "Duty:"
    printNumberSHORT((uint16_t)pwm2_duty, 98, 72, RGBToWord(255,255,255), RGBToWord(0,0,0));
    printText(" %", 130, 72, RGBToWord(255,255,255), RGBToWord(0,0,0));
}