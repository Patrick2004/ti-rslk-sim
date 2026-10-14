// ADC14 sequence conversion of the three IR sensors, bit-band LED toggling,
// and a software PWM (GPIO toggling, like the Lab 3 "simple motors" code).
#include "rslk_hw.h"

#define LED1_BB (*((volatile uint8_t *)(0x42098040)))     // P1OUT bit 0 via bit-band

static void ADC_Init3(void) {
    ADC14->CTL0 &= ~0x00000002;
    while (ADC14->CTL0 & 0x00010000) {}
    ADC14->CTL0 = 0x04223390;          // SHP, SMCLK, sequence, SHT=32, MSC, ON
    ADC14->CTL1 = 0x00000030;          // 14-bit
    ADC14->MCTL[0] = 17;               // A17 right
    ADC14->MCTL[1] = 12;               // A12 centre
    ADC14->MCTL[2] = 0x80 | 16;        // A16 left, end of sequence
    ADC14->IER0 = 0; ADC14->IER1 = 0;
    P4->SEL0 |= 0x02; P4->SEL1 |= 0x02;
    P9->SEL0 |= 0x03; P9->SEL1 |= 0x03;
    ADC14->CTL0 |= 0x00000002;
}

int main(void) {
    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();
    ADC_Init3();
    for (int i = 0; i < 3; i++) {
        while (ADC14->CTL0 & 0x00010000) {}
        ADC14->CTL0 |= 0x00000001;
        while ((ADC14->IFGR0 & 0x04) == 0) {}
        uint32_t r = ADC14->MEM[0], c = ADC14->MEM[1], l = ADC14->MEM[2];
        UART0_OutString("adc_l="); UART0_OutUDec(l);
        UART0_OutString(" adc_c="); UART0_OutUDec(c);
        UART0_OutString(" adc_r="); UART0_OutUDec(r);
        UART0_OutString("\r\n");
        LED1_BB ^= 1;
        Clock_Delay1ms(50);
    }
    UART0_OutString("led1="); UART0_OutUDec(P1->OUT & 1);
    UART0_OutString("\r\n");
    // software PWM: 25 % duty at 100 Hz on the motor enable pins (no timer)
    P3->SEL0 &= ~0xC0; P3->SEL1 &= ~0xC0; P3->DIR |= 0xC0; P3->OUT |= 0xC0;
    P5->SEL0 &= ~0x30; P5->SEL1 &= ~0x30; P5->DIR |= 0x30; P5->OUT &= ~0x30;
    P2->SEL0 &= ~0xC0; P2->SEL1 &= ~0xC0; P2->DIR |= 0xC0;
    SysTick_Init();
    for (int n = 0; n < 200; n++) {         // 2 s
        P2->OUT |= 0xC0;
        SysTick_Wait(120000);               // 2.5 ms high
        P2->OUT &= ~0xC0;
        SysTick_Wait(360000);               // 7.5 ms low
    }
    P3->OUT &= ~0xC0;
    UART0_OutString("softpwm done\r\n");
    return 0;
}
