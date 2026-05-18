#ifdef LINUX

#include "platform/linux.h"

CAN_Linux g_can("vcan0");

void SystemClock_Config(void) {}

void Error_Handler(void) {
    while (1) {}
}

#endif /* LINUX */
