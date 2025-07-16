#ifndef __COMMON_H__
#define __COMMON_H__

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "semphr.h"

#include "usbd_cdc_if.h"

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

#ifdef DBG_A
#define CHECKPOINTA(format, ...)    UsbPrintf(""format"\r\n", ##__VA_ARGS__)
#else
#define CHECKPOINTA(...)
#endif

#ifdef DBG_B
#define CHECKPOINTB(format, ...)    
                                    UsbPrintf(""__FILE__", line:%d, %s(): "format"\r\n", __LINE__, __func__, ##__VA_ARGS__)
#else
#define CHECKPOINTB(...)
#endif

#define ERRORPOINT(format, ...)     UsbPrintf("%s(): ERROR:"format"\r\n", __func__, ##__VA_ARGS__)



#endif
