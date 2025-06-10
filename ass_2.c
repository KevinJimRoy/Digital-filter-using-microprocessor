#include "MKL25Z4.h"
#include <stdint.h>

#define BAUD_RATE 115200
#define SAMPLE_COUNT 1001

// butterworth values from MATLAB (for 50 Hz cutoff @ 1000 Hz sampling)
int16_t b[3] = {5,10,5};      
int16_t a[2] = {-401,164};       

uint8_t x[3] = {0};                 // Input samples
int16_t y[3] = {0}; 
// Output samples

void RED_LED_on(void) { //for debuging
    // Enable clock for Port B
    SIM->SCGC5 |= 0x00000400;

    // Configure PTB18 as GPIO
    PORTB->PCR[18] = PORT_PCR_MUX(1);

    // Set PTB18 as output
    FPTB->PDDR |= (1 << 18);

    // Drive PTB18 LOW (active-low LED ON)
    FPTB->PCOR = (1 << 18);
}

void UART0_init(void) {
    SIM->SCGC4 |= 0x00000400;
    SIM->SOPT2 |= SIM_SOPT2_UART0SRC(1);
    SIM->SCGC5 |= 0x00000f00;

    PORTA->PCR[1] = PORT_PCR_MUX(2); // RX
    PORTA->PCR[2] = PORT_PCR_MUX(2); // TX

    UART0->C2 = 0;
    UART0->BDH = 0;
    UART0->BDL = (uint8_t)(SystemCoreClock / (16 * BAUD_RATE));
    UART0->C4 = 0;
    UART0->C2 = UART_C2_TE_MASK | UART_C2_RE_MASK;
}

uint8_t UART0_receive(void) {
    while (!(UART0->S1 & UART_S1_RDRF_MASK));
		RED_LED_on();
    return UART0->D;
}

void UART0_transmit(uint8_t data) {
    while (!(UART0->S1 & UART_S1_TDRE_MASK));
    UART0->D = data;
}

void DAC0_init(void) {
    SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK;
    DAC0->C0 = DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK;
}

void DAC0_write(uint16_t val) {
    DAC0->DAT[0].DATH = (val >> 8) & 0x0F;
    DAC0->DAT[0].DATL = val & 0xFF;
}

int main(void) {
    UART0_init();
    DAC0_init();

    for (int n = 0; n < SAMPLE_COUNT; n++) {
        // old samples
        x[2] = x[1]; x[1] = x[0];
        y[2] = y[1]; y[1] = y[0];

        // Read new input sample
        x[0] = UART0_receive();

        // butterworth eq
        int32_t output = 0;
        output = b[0] * x[0] + b[1] * x[1] + b[2] * x[2] - a[0] * y[1] - a[1] * y[2];
        output = output >> 8;  // DivideA by 256 (Q8)

        // Clamp output to 8-bit range
        if (output < 0) output = 0;
        if (output > 255) output = y[0];

        y[0] = output;

        UART0_transmit((uint8_t)y[0]);     // Send filtered sample
        DAC0_write(y[0] << 4);         // 8-bit to 12-bit DAC scaling
   }
	
    while (1); // Idle
}