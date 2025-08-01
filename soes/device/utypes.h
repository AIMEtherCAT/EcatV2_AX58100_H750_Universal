#ifndef __UTYPES_H__
#define __UTYPES_H__

#include "cc.h"

/* Object dictionary storage */

typedef struct
{
   /* Identity */

   uint32_t serial;

   /* Inputs */

   uint8_t slave_status;
   uint64_t slave2master[10];

   /* Outputs */

   uint8_t master_status;
   uint64_t master2slave[10];

   /* Parameters */

   uint16_t sdo_len;

   /* Manufacturer specific data */

   struct
   {
      uint16_t Sync_mode;
      uint32_t CycleTime;
      uint16_t Sync_modes_supported;
      uint32_t Minimum_Cycle_Time;
      uint16_t CycleTimeTooSmallCnt;
   } SyncMgrParam1;
   struct
   {
      uint16_t Sync_mode;
      uint32_t CycleTime;
      uint16_t Sync_modes_supported;
      uint32_t Minimum_Cycle_Time;
      uint16_t CycleTimeTooSmallCnt;
   } SyncMgrParam2;

} _Objects;

extern _Objects Obj;

#endif /* __UTYPES_H__ */
