// Drive forward, measure wheel speed with TA3 input capture (tachometer),
// stop on a bump-switch edge interrupt (PORT4), report which switch hit.
#include "rslk_hw.h"

volatile uint16_t right_first, right_period, left_first, left_period;
volatile int32_t right_steps, left_steps;
volatile uint8_t bump_hit;

void TA3_0_IRQHandler(void) {               // right encoder A on P10.4 / CCR0
    uint16_t now = TIMER_A3->CCR[0];
    TIMER_A3->CCTL[0] &= ~0x0001;
    right_period = now - right_first;
    right_first = now;
    right_steps += (P5->IN & 0x01) ? 1 : -1;
}
void TA3_N_IRQHandler(void) {               // left encoder A on P10.5 / CCR1
    uint16_t now = TIMER_A3->CCR[1];
    TIMER_A3->CCTL[1] &= ~0x0001;
    left_period = now - left_first;
    left_first = now;
    left_steps += (P5->IN & 0x04) ? 1 : -1;
}
void PORT4_IRQHandler(void) {
    uint8_t flags = P4->IFG & 0xED;
    P4->IFG &= ~flags;
    if (!bump_hit) bump_hit = Bump_Read();
    Motor_Stop();
}

int main(void) {
    Clock_Init48MHz();
    UART0_Init();
    Motor_Init();
    Bump_Init();
    // encoder B inputs
    P5->SEL0 &= ~0x05; P5->SEL1 &= ~0x05; P5->DIR &= ~0x05;
    // bump edge interrupts (falling edge = pressed)
    P4->IES |= 0xED; P4->IFG &= ~0xED; P4->IE |= 0xED;
    NVIC->ISER[1] = 1 << (38 - 32);
    // TA3 capture on rising edges, SMCLK/1 = 12 MHz, continuous
    P10->SEL0 |= 0x30; P10->SEL1 &= ~0x30; P10->DIR &= ~0x30;
    TIMER_A3->CTL = 0x0200;
    TIMER_A3->CCTL[0] = 0x4910;
    TIMER_A3->CCTL[1] = 0x4910;
    TIMER_A3->EX0 = 0;
    NVIC->ISER[0] = (1 << 14) | (1 << 15);
    TIMER_A3->CTL |= 0x0024;
    EnableInterrupts();
    Motor_Forward(7500, 7500);              // 50 % duty
    for (int i = 0; i < 12 && !bump_hit; i++) {
        Clock_Delay1ms(250);
        uint32_t rpm_r = right_period ? 2000000 / right_period : 0;   // 12e6*60/360 / period
        uint32_t rpm_l = left_period ? 2000000 / left_period : 0;
        UART0_OutString("rpm_l="); UART0_OutUDec(rpm_l);
        UART0_OutString(" rpm_r="); UART0_OutUDec(rpm_r);
        UART0_OutString(" steps_l="); UART0_OutSDec(left_steps);
        UART0_OutString(" steps_r="); UART0_OutSDec(right_steps);
        UART0_OutString("\r\n");
    }
    while (!bump_hit) {}
    UART0_OutString("bump="); UART0_OutUDec(bump_hit);
    UART0_OutString(" steps_l="); UART0_OutSDec(left_steps);
    UART0_OutString("\r\n");
    Motor_Backward(4000, 4000);
    Clock_Delay1ms(300);
    Motor_Stop();
    UART0_OutString("backed_up steps_l="); UART0_OutSDec(left_steps);
    UART0_OutString("\r\n");
    return 0;
}
