#include "default.h"

namespace task {
    
void Default::run() {
    defaultTaskHandle = osThreadNew(&Default::StartDefaultEntry, this, &defaultTask_attributes);
}

void Default::StartDefaultEntry(void *argument) {
    auto *self = static_cast<Default*>(argument);
    if (self) {
        self->StartDefault();
    }
}



void Default::StartDefault()
{
    uint32_t time = HAL_GetTick();
    for (;;)
    {
        time = HAL_GetTick();
        printf("defaulting %lu \n", time);

      osDelay(1000);
    }
}

}