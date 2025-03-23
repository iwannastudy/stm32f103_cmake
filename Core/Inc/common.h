#ifndef __COMMON_H__
#define __COMMON_H__

#include "usart.h"
// #include "FreeRTOS.h"
// #include "task.h"
// #include "queue.h"
// #include "event_groups.h"
// #include "semphr.h"

#define SYSTEM_CTRL                 6
#define DEAMON_TASK                 5
#define USER_INTERACT               4
#define SENSORS_MODULES             3
#define DISPLAY_UI                  2
#define LOWEST_TASK                 1

#define CHECKPOINTA(format, ...)    UartPrintf(""format"\r\n", ##__VA_ARGS__)

#ifdef DBG_B
#define CHECKPOINTB(format, ...)    
                                    UartPrintf(""__FILE__", line:%d, %s(): "format"\r\n", __LINE__, __func__, ##__VA_ARGS__)
#else
#define CHECKPOINTB(...)
#endif

#define ERRORPOINT(format, ...)     UartPrintf("%s(): ERROR:"format"\r\n", __func__, ##__VA_ARGS__)

typedef enum
{
    EV_NET_DEA_RUN   = 0x0001,
    EV_UI_RUN        = 0x0002,
    EV_TASK_RUN_3    = 0x0004,
    EV_TASK_RUN_4    = 0x0008,
    EV_NETWORK_OK    = 0x0010,
    EV_CH4_RECVED    = 0x0020,
    EV_BENG_WORKDONE = 0x0040,
    EV_DATA_SAVED    = 0x0080,
    EV_BT_RECVED     = 0x0100,
    AIR_CHECKED      = 0x0200,
    EV_MAX           = 0x800000
}event_bits;

// extern SemaphoreHandle_t printfMutex;
// extern EventGroupHandle_t evtGroup;


#endif
