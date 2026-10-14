// TExaS-style logic analyzer stream (as inc/TExaS.c does it) mixed with UART text,
// plus variables of several C types for the watch list.
#include "rslk_hw.h"

void (*PeriodicTask2)(void);                  // same names as inc/TExaS.c
void LogicAnalyzer_P7(void) { EUSCI_A0->TXBUF = P7->IN | 0x80; }
void Scope(void) { EUSCI_A0->TXBUF = 0; }

void T32_INT1_IRQHandler(void) {
    TIMER32_1->INTCLR = 1;
    (*PeriodicTask2)();
}

volatile uint8_t sample_bits;
volatile int16_t temperature = -42;
volatile float gain = 1.5f;
volatile uint16_t table[4] = {10, 20, 30, 40};
enum State { IDLE, RUNNING, DONE };
volatile enum State state = RUNNING;

int main(void) {
    static volatile uint32_t ticks;
    Clock_Init48MHz();
    UART0_Init();
    SysTick_Init();
    UART0_OutString("before\r\n");            // plain text before the analyzer starts
    while ((EUSCI_A0->IFG & 0x02) == 0) {}
    P7->DIR = 0x05;                           // P7.0 and P7.2 are outputs we toggle
    P7->OUT = 0;
    PeriodicTask2 = LogicAnalyzer_P7;
    TIMER32_1->LOAD = 48000000 / 10000 - 1;   // 10 kHz, like PeriodicTask2_Init
    TIMER32_1->INTCLR = 1;
    TIMER32_1->CONTROL = 0xE2;
    NVIC->ISER[0] = 1u << 25;
    EnableInterrupts();
    for (ticks = 0; ticks < 50; ticks++) {     // P7.0 square wave: 1 ms high, 1 ms low
        P7->OUT ^= 0x01;
        sample_bits = P7->OUT;
        SysTick_Wait(48000);
    }
    NVIC->ICER[0] = 1u << 25;                 // stop the analyzer (PeriodicTask2_Stop)
    state = DONE;
    SysTick_Wait(4800);
    UART0_OutString("after\r\n");
    return 0;
}
