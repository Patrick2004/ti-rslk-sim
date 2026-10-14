// Clock switch, polled UART output and SysTick polling delays.
#include "rslk_hw.h"

int main(void) {
    Clock_Init48MHz();
    UART0_Init();
    SysTick_Init();
    UART0_OutString("hello\r\n");
    for (int i = 0; i < 5; i++) {
        SysTick_Wait10ms(10);                 // 100 ms
        UART0_OutString("tick ");
        UART0_OutUDec(i);
        UART0_OutString("\r\n");
    }
    // echo one line typed by the user
    UART0_OutString("type> ");
    char c;
    while ((c = UART0_InChar()) != '\r') UART0_OutChar(c);
    UART0_OutString("\r\ndone\r\n");
    return 0;
}
