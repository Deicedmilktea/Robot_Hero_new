#include "referee_task.h"

void UI_task(void const *argument)
{
    for (;;)
    {
        UITask();
        osDelay(1);
    }
}