#ifndef __COMMON_H__
#define __COMMON_H__

#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

// system control
#define SYS_CTRL_PORIRITY           (configMAX_PRIORITIES-1)

// user interaction
#define USER_INTRA_PORIORITY        (SYS_CTRL_PORIRITY-1)

// sensors and modules
#define SENS_MODS_PORIORITY         (USER_INTRA_PORIORITY-1)

// tasks that no requirement for real-time
#define NORMAL_PORIORIT             (SENS_MODS_PORIORITY-1)

// tasks that with the lowest
#define LOWEST_PORIORIT             1

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

// extern EventGroupHandle_t evtGroup;


#endif
