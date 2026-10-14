// Closed-loop line follower (proportional control on the reflectance position),
// in the style of the RSLK line-following labs.  Prints laps/position reports.
#include "rslk_hw.h"

#define BASE 4000
#define KP   12

int main(void) {
    Clock_Init48MHz();
    UART0_Init();
    LaunchPad_Init();
    Motor_Init();
    Reflectance_Init();
    uint32_t lost = 0, loops = 0;
    while (1) {
        uint8_t data = Reflectance_Read(1000);
        if (data == 0) {
            lost++;
            Motor_Forward(2000, 2000);       // coast straight briefly when the line is lost
        } else {
            int32_t pos = Reflectance_Position(data);    // + means the line is to the right
            int32_t u = KP * pos;
            int32_t l = BASE + u, r = BASE - u;
            if (l < 0) l = 0;
            if (r < 0) r = 0;
            if (l > 14000) l = 14000;
            if (r > 14000) r = 14000;
            Motor_Forward((uint16_t)l, (uint16_t)r);
            P2->OUT = (P2->OUT & ~0x07) | (pos > 100 ? 0x01 : pos < -100 ? 0x04 : 0x02);
        }
        loops++;
        if (loops % 500 == 0) {
            UART0_OutString("loops="); UART0_OutUDec(loops);
            UART0_OutString(" lost="); UART0_OutUDec(lost);
            UART0_OutString("\r\n");
        }
        Clock_Delay1ms(2);
    }
}
