// Minimal register-level RSLK MAX drivers for the simulator's test programs.
// They mirror the TI-RSLK (Valvano) driver API so the tests exercise the
// same register patterns as the real lab code.
#ifndef RSLK_HW_H
#define RSLK_HW_H
#include <stdint.h>
#include "msp.h"

static uint32_t ClockFrequency = 3000000;

// ---------------------------------------------------------------- clock
static void Clock_Init48MHz(void) {
    // PJ.2/PJ.3 HFXT pins
    PJ->SEL0 |= 0x0C; PJ->SEL1 &= ~0x0C;
    // core voltage level 1 for 48 MHz
    while (PCM->CTL1 & 0x00000100) {}
    PCM->CTL0 = 0x695A0001;
    while (PCM->CTL1 & 0x00000100) {}
    // two flash wait states
    FLCTL->BANK0_RDCTL = (FLCTL->BANK0_RDCTL & ~0x0000F000) | 0x00001000;
    FLCTL->BANK1_RDCTL = (FLCTL->BANK1_RDCTL & ~0x0000F000) | 0x00001000;
    CS->KEY = 0x695A;
    CS->CTL2 = (CS->CTL2 & ~0x00700000) | 0x00610000;   // HFXT 40-48 MHz, on
    while (CS->IFG & 0x02) { CS->CLRIFG = 0x02; }
    // MCLK = HFXT, HSMCLK = HFXT/2, SMCLK = HFXT/4, ACLK = REFO
    CS->CTL1 = 0x20100255;
    CS->KEY = 0;
    ClockFrequency = 48000000;
}

__attribute__((naked, noinline)) void delay(uint32_t n) {   // calibrated busy loop (as in Clock.c)
    __asm volatile ("1: subs r0, r0, #1\n"
                    "   bne 1b\n"
                    "   bx lr");
}
__attribute__((noinline)) void Clock_Delay1ms(uint32_t n) {
    while (n) { delay(ClockFrequency / 9162); n--; }
}

// ---------------------------------------------------------------- SysTick
static void SysTick_Init(void) {
    SysTick->CTRL = 0; SysTick->LOAD = 0x00FFFFFF; SysTick->VAL = 0; SysTick->CTRL = 0x05;
}
static void SysTick_Wait(uint32_t delay) {
    if (delay <= 1) return;
    SysTick->LOAD = delay - 1;
    SysTick->VAL = 0;
    while ((SysTick->CTRL & 0x00010000) == 0) {}
}
static void SysTick_Wait10ms(uint32_t n) { while (n--) SysTick_Wait(480000); }

// ---------------------------------------------------------------- UART0 (polled, 115200 baud from 12 MHz SMCLK)
static void UART0_Init(void) {
    EUSCI_A0->CTLW0 = 0x0001;
    EUSCI_A0->CTLW0 = 0x00C1;
    EUSCI_A0->BRW = 104;
    EUSCI_A0->MCTLW &= ~0xFFF1;
    P1->SEL0 |= 0x0C; P1->SEL1 &= ~0x0C;
    EUSCI_A0->CTLW0 &= ~0x0001;
    EUSCI_A0->IE &= ~0x000F;
}
static void UART0_OutChar(char c) {
    while ((EUSCI_A0->IFG & 0x02) == 0) {}
    EUSCI_A0->TXBUF = c;
}
static void UART0_OutString(const char *s) { while (*s) UART0_OutChar(*s++); }
static void UART0_OutUDec(uint32_t n) {
    char buf[12]; int i = 0;
    do { buf[i++] = '0' + n % 10; n /= 10; } while (n);
    while (i) UART0_OutChar(buf[--i]);
}
static void UART0_OutSDec(int32_t n) {
    if (n < 0) { UART0_OutChar('-'); n = -n; }
    UART0_OutUDec((uint32_t)n);
}
static char UART0_InChar(void) {
    while ((EUSCI_A0->IFG & 0x01) == 0) {}
    return (char)EUSCI_A0->RXBUF;
}

// ---------------------------------------------------------------- LaunchPad
static void LaunchPad_Init(void) {
    P1->SEL0 &= ~0x13; P1->SEL1 &= ~0x13;
    P1->DIR &= ~0x12; P1->DIR |= 0x01; P1->REN |= 0x12; P1->OUT |= 0x12;
    P2->SEL0 &= ~0x07; P2->SEL1 &= ~0x07; P2->DIR |= 0x07; P2->OUT &= ~0x07;
}
static uint8_t LaunchPad_Input(void) { return ((~P1->IN & 0x10) >> 3) | ((~P1->IN & 0x02) >> 1); }

// ---------------------------------------------------------------- motors (TA0 PWM, up/down mode, period 15000)
static void Motor_Init(void) {
    P3->SEL0 &= ~0xC0; P3->SEL1 &= ~0xC0; P3->DIR |= 0xC0; P3->OUT &= ~0xC0;   // nSLEEP
    P5->SEL0 &= ~0x30; P5->SEL1 &= ~0x30; P5->DIR |= 0x30; P5->OUT &= ~0x30;   // DIR
    P2->DIR |= 0xC0; P2->SEL0 |= 0xC0; P2->SEL1 &= ~0xC0;                      // TA0.3, TA0.4
    TIMER_A0->CCTL[0] = 0x0080;
    TIMER_A0->CCR[0] = 15000;
    TIMER_A0->EX0 = 0;
    TIMER_A0->CCTL[3] = 0x0040; TIMER_A0->CCR[3] = 0;
    TIMER_A0->CCTL[4] = 0x0040; TIMER_A0->CCR[4] = 0;
    TIMER_A0->CTL = 0x02F0;          // SMCLK, /8, up/down
}
static void Motor_Stop(void) { P3->OUT &= ~0xC0; TIMER_A0->CCR[3] = 0; TIMER_A0->CCR[4] = 0; }
static void Motor_Forward(uint16_t left, uint16_t right) {
    P5->OUT &= ~0x30; TIMER_A0->CCR[4] = left; TIMER_A0->CCR[3] = right; P3->OUT |= 0xC0;
}
static void Motor_Backward(uint16_t left, uint16_t right) {
    P5->OUT |= 0x30; TIMER_A0->CCR[4] = left; TIMER_A0->CCR[3] = right; P3->OUT |= 0xC0;
}
static void Motor_Left(uint16_t left, uint16_t right) {      // spin left: left back, right forward
    P5->OUT = (P5->OUT & ~0x30) | 0x10; TIMER_A0->CCR[4] = left; TIMER_A0->CCR[3] = right; P3->OUT |= 0xC0;
}
static void Motor_Right(uint16_t left, uint16_t right) {
    P5->OUT = (P5->OUT & ~0x30) | 0x20; TIMER_A0->CCR[4] = left; TIMER_A0->CCR[3] = right; P3->OUT |= 0xC0;
}

// ---------------------------------------------------------------- reflectance (QTRX)
__attribute__((noinline)) void Clock_Delay1us(uint32_t n) {
    volatile uint32_t k = (382 * n) / 100;
    while (k) { k--; }
}
static void Reflectance_Init(void) {
    P5->SEL0 &= ~0x08; P5->SEL1 &= ~0x08; P5->DIR |= 0x08; P5->OUT &= ~0x08;
    P9->SEL0 &= ~0x04; P9->SEL1 &= ~0x04; P9->DIR |= 0x04; P9->OUT &= ~0x04;
    P7->SEL0 = 0; P7->SEL1 = 0; P7->DIR = 0;
}
static uint8_t Reflectance_Read(uint32_t time) {
    uint8_t result;
    P5->OUT |= 0x08; P9->OUT |= 0x04;
    P7->DIR = 0xFF; P7->OUT = 0xFF;
    Clock_Delay1us(10);
    P7->DIR = 0x00;
    Clock_Delay1us(time);
    result = P7->IN;
    P5->OUT &= ~0x08; P9->OUT &= ~0x04;
    return result;
}
static int32_t Reflectance_Position(uint8_t data) {
    static const int32_t W[8] = {334, 238, 142, 48, -48, -142, -238, -334};
    int32_t sum = 0, count = 0;
    for (int i = 0; i < 8; i++) if (data & (1 << i)) { sum += W[i]; count++; }
    return count ? sum / count : 0;
}

// ---------------------------------------------------------------- bump switches
static void Bump_Init(void) {
    P4->SEL0 &= ~0xED; P4->SEL1 &= ~0xED; P4->DIR &= ~0xED; P4->REN |= 0xED; P4->OUT |= 0xED;
}
static uint8_t Bump_Read(void) {
    uint8_t in = ~P4->IN;
    return (in & 0x01) | ((in & 0x0C) >> 1) | ((in & 0xE0) >> 2);
}

static inline void EnableInterrupts(void) { __asm volatile ("cpsie i"); }
static inline void DisableInterrupts(void) { __asm volatile ("cpsid i"); }
static inline void WaitForInterrupt(void) { __asm volatile ("wfi"); }

#endif
