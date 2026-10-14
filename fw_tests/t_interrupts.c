// Interrupt rates and priorities: TA1 at 1 kHz, Timer32 at 250 Hz, SysTick at 100 Hz,
// all running for exactly one second measured by TA2 in continuous mode.
// Also uses floating point inside an ISR to check FPU context save/restore.
#include "rslk_hw.h"

volatile uint32_t ta1_count, systick_count, t32_count, ta2_overflows;
volatile float isr_acc;

void TA1_0_IRQHandler(void) {
    TIMER_A1->CCTL[0] &= ~0x0001;
    ta1_count++;
    isr_acc = isr_acc * 0.5f + 1.25f;        // FP work inside an interrupt
}
void SysTick_Handler(void) { systick_count++; }
void T32_INT1_IRQHandler(void) { TIMER32_1->INTCLR = 0; t32_count++; }
void TA2_N_IRQHandler(void) {
    if (TIMER_A2->IV == 0x0E) ta2_overflows++;  // TAIFG via TA2IV
}

int main(void) {
    Clock_Init48MHz();
    UART0_Init();
    // TA1: SMCLK/4 = 3 MHz, CCR0 = 2999 -> 1 kHz
    TIMER_A1->CTL = 0x0280;
    TIMER_A1->CCTL[0] = 0x0010;
    TIMER_A1->CCR[0] = 2999;
    TIMER_A1->EX0 = 0;
    NVIC->IP[10] = 0x40;
    NVIC->ISER[0] = 1 << 10;
    // SysTick: 480000 cycles -> 100 Hz, priority 2
    SysTick->CTRL = 0; SysTick->LOAD = 480000 - 1; SysTick->VAL = 0;
    SCB->SHP[11] = 0x40;
    // Timer32_1: MCLK/1, periodic, 48e6/250 = 192000
    TIMER32_1->LOAD = 192000 - 1;
    TIMER32_1->CONTROL = 0xE2;              // enable, periodic, IE, 32-bit
    NVIC->ISER[0] = 1 << 25;
    // TA2: ACLK (32768 Hz) continuous, TAIFG every 2 s; use the counter to time 1 s
    TIMER_A2->CTL = 0x0124;                 // ACLK, continuous, TACLR
    TIMER_A2->CTL |= 0x0002;                // TAIE
    NVIC->ISER[0] = 1 << 13;
    float main_acc = 0.0f;
    TIMER_A1->CTL |= 0x0014;                // up mode + clear
    SysTick->CTRL = 0x07;
    EnableInterrupts();
    uint16_t start = TIMER_A2->R;
    while ((uint16_t)(TIMER_A2->R - start) < 32768) {
        main_acc = main_acc + 0.001f;        // FP work in main concurrently with the ISR
    }
    DisableInterrupts();
    UART0_OutString("ta1="); UART0_OutUDec(ta1_count);
    UART0_OutString(" systick="); UART0_OutUDec(systick_count);
    UART0_OutString(" t32="); UART0_OutUDec(t32_count);
    UART0_OutString(" isr_acc="); UART0_OutUDec((uint32_t)(isr_acc * 1000));
    UART0_OutString(" main_ok="); UART0_OutUDec(main_acc > 0.0f ? 1 : 0);
    UART0_OutString("\r\n");
    while (1) { WaitForInterrupt(); }
}
