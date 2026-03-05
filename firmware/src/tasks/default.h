#pragma once
#if F4
#include "stm32f4xx_hal.h"
#include "platform/stm_f4.h"
#endif
#include "cmsis_os.h"
#include <cstdio>

namespace task{
    class Default{
        public:
           void run();

        private: 
            void StartDefault();
            static void StartDefaultEntry(void *argument);
            osThreadId_t defaultTaskHandle;

            const osThreadAttr_t defaultTask_attributes = {
                "defaultTask",          // name
                0,                    // attr_bits
                nullptr,              // cb_mem
                0,                    // cb_size
                nullptr,              // stack_mem
                256 * 4,              // stack_size
                osPriorityNormal,     // priority
                0,                    // tz_module
                0                     // reserved
            };

    };
}

