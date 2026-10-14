// Error reporting: chosen by the first UART character.
//  'n' NULL pointer write, 'h' interrupt enabled without a handler,
//  'w' watchdog left running (program resets), 'r' return from main.
#include "rslk_hw.h"

volatile uint32_t *volatile bad_ptr = 0;

int main(void) {
    static volatile uint32_t boots;
    Clock_Init48MHz();
    UART0_Init();
    UART0_OutString("boot\r\n");
    char c = UART0_InChar();
    if (c == 'n') {
        *bad_ptr = 0x1234;                  // write to address 0 (flash) -> fault
    } else if (c == 'h') {
        TIMER_A2->CTL = 0x0210;             // SMCLK up mode
        TIMER_A2->CCR[0] = 1000;
        TIMER_A2->CCTL[0] = 0x0010;         // CCIE, but no TA2_0_IRQHandler defined
        NVIC->ISER[0] = 1 << 12;
        EnableInterrupts();
        while (1) {}
    } else if (c == 'w') {
        WDT_A->CTL = 0x5A00 | 0x04;         // start the watchdog (SMCLK / 2^15) and never feed it
        while (1) {}
    }
    boots++;
    return 0;
}
