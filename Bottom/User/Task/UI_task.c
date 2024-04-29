#include "referee_task.h"

void UI_task(void const *argument)
{
    while (1)
    {
        UITask();
        osDelay(1);
    }
}