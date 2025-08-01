#include "esc_coe.h"
#include "utypes.h"
#include <stddef.h>

#ifndef HW_REV
#define HW_REV "1.0"
#endif

#ifndef SW_REV
#define SW_REV "1.0"
#endif

static const char acName1000[] = "Device Type";
static const char acName1008[] = "Manufacturer Device Name";
static const char acName1009[] = "Manufacturer Hardware Version";
static const char acName100A[] = "Manufacturer Software Version";
static const char acName1018[] = "Identity Object";
static const char acName1018_00[] = "Max SubIndex";
static const char acName1018_01[] = "Vendor ID";
static const char acName1018_02[] = "Product Code";
static const char acName1018_03[] = "Revision Number";
static const char acName1018_04[] = "Serial Number";
static const char acName1600[] = "master_status";
static const char acName1600_00[] = "Max SubIndex";
static const char acName1600_01[] = "master_status";
static const char acName1601[] = "master2slave";
static const char acName1601_00[] = "Max SubIndex";
static const char acName1601_01[] = "m2s1";
static const char acName1601_02[] = "m2s2";
static const char acName1601_03[] = "m2s3";
static const char acName1601_04[] = "m2s4";
static const char acName1601_05[] = "m2s5";
static const char acName1601_06[] = "m2s6";
static const char acName1601_07[] = "m2s7";
static const char acName1601_08[] = "m2s8";
static const char acName1601_09[] = "m2s9";
static const char acName1601_0A[] = "m2s10";
static const char acName1A00[] = "slave_status";
static const char acName1A00_00[] = "Max SubIndex";
static const char acName1A00_01[] = "slave_status";
static const char acName1A01[] = "slave2master";
static const char acName1A01_00[] = "Max SubIndex";
static const char acName1A01_01[] = "s2m1";
static const char acName1A01_02[] = "s2m2";
static const char acName1A01_03[] = "s2m3";
static const char acName1A01_04[] = "s2m4";
static const char acName1A01_05[] = "s2m5";
static const char acName1A01_06[] = "s2m6";
static const char acName1A01_07[] = "s2m7";
static const char acName1A01_08[] = "s2m8";
static const char acName1A01_09[] = "s2m9";
static const char acName1A01_0A[] = "s2m10";
static const char acName1C00[] = "Sync Manager Communication Type";
static const char acName1C00_00[] = "Max SubIndex";
static const char acName1C00_01[] = "Communications Type SM0";
static const char acName1C00_02[] = "Communications Type SM1";
static const char acName1C00_03[] = "Communications Type SM2";
static const char acName1C00_04[] = "Communications Type SM3";
static const char acName1C12[] = "Sync Manager 2 PDO Assignment";
static const char acName1C12_00[] = "Max SubIndex";
static const char acName1C12_01[] = "PDO Mapping";
static const char acName1C12_02[] = "PDO Mapping";
static const char acName1C13[] = "Sync Manager 3 PDO Assignment";
static const char acName1C13_00[] = "Max SubIndex";
static const char acName1C13_01[] = "PDO Mapping";
static const char acName1C13_02[] = "PDO Mapping";
static const char acName1C32[] = "SyncMgrParam1";
static const char acName1C32_00[] = "Max SubIndex";
static const char acName1C32_01[] = "Sync mode";
static const char acName1C32_02[] = "CycleTime";
static const char acName1C32_04[] = "Sync modes supported";
static const char acName1C32_05[] = "Minimum Cycle Time";
static const char acName1C32_0C[] = "CycleTimeTooSmallCnt";
static const char acName1C33[] = "SyncMgrParam2";
static const char acName1C33_00[] = "Max SubIndex";
static const char acName1C33_01[] = "Sync mode";
static const char acName1C33_02[] = "CycleTime";
static const char acName1C33_04[] = "Sync modes supported";
static const char acName1C33_05[] = "Minimum Cycle Time";
static const char acName1C33_0C[] = "CycleTimeTooSmallCnt";
static const char acName6000[] = "slave_status";
static const char acName6001[] = "slave2master";
static const char acName6001_00[] = "Max SubIndex";
static const char acName6001_01[] = "s2m1";
static const char acName6001_02[] = "s2m2";
static const char acName6001_03[] = "s2m3";
static const char acName6001_04[] = "s2m4";
static const char acName6001_05[] = "s2m5";
static const char acName6001_06[] = "s2m6";
static const char acName6001_07[] = "s2m7";
static const char acName6001_08[] = "s2m8";
static const char acName6001_09[] = "s2m9";
static const char acName6001_0A[] = "s2m10";
static const char acName7000[] = "master_status";
static const char acName7001[] = "master2slave";
static const char acName7001_00[] = "Max SubIndex";
static const char acName7001_01[] = "m2s1";
static const char acName7001_02[] = "m2s2";
static const char acName7001_03[] = "m2s3";
static const char acName7001_04[] = "m2s4";
static const char acName7001_05[] = "m2s5";
static const char acName7001_06[] = "m2s6";
static const char acName7001_07[] = "m2s7";
static const char acName7001_08[] = "m2s8";
static const char acName7001_09[] = "m2s9";
static const char acName7001_0A[] = "m2s10";
static const char acName8000[] = "sdo_len";

const _objd SDO1000[] =
{
  {0x0, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1000, 0x00001389, NULL},
};
const _objd SDO1008[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 208, ATYPE_RO, acName1008, 0, "58100_H750_UniversalModule"},
};
const _objd SDO1009[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName1009, 0, HW_REV},
};
const _objd SDO100A[] =
{
  {0x0, DTYPE_VISIBLE_STRING, 24, ATYPE_RO, acName100A, 0, SW_REV},
};
const _objd SDO1018[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1018_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_01, 0, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_02, 0x03, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_03, 0, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1018_04, 0x00000000, &Obj.serial},
};
const _objd SDO1600[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1600_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1600_01, 0x70000008, NULL},
};
const _objd SDO1601[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1601_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_01, 0x70010140, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_02, 0x70010240, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_03, 0x70010340, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_04, 0x70010440, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_05, 0x70010540, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_06, 0x70010640, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_07, 0x70010740, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_08, 0x70010840, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_09, 0x70010940, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1601_0A, 0x70010A40, NULL},
};
const _objd SDO1A00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A00_00, 1, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A00_01, 0x60000008, NULL},
};
const _objd SDO1A01[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1A01_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_01, 0x60010140, NULL},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_02, 0x60010240, NULL},
  {0x03, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_03, 0x60010340, NULL},
  {0x04, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_04, 0x60010440, NULL},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_05, 0x60010540, NULL},
  {0x06, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_06, 0x60010640, NULL},
  {0x07, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_07, 0x60010740, NULL},
  {0x08, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_08, 0x60010840, NULL},
  {0x09, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_09, 0x60010940, NULL},
  {0x0A, DTYPE_UNSIGNED32, 32, ATYPE_RO, acName1A01_0A, 0x60010A40, NULL},
};
const _objd SDO1C00[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_00, 4, NULL},
  {0x01, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_01, 1, NULL},
  {0x02, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_02, 2, NULL},
  {0x03, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_03, 3, NULL},
  {0x04, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C00_04, 4, NULL},
};
const _objd SDO1C12[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C12_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_01, 0x1600, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C12_02, 0x1601, NULL},
};
const _objd SDO1C13[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C13_00, 2, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_01, 0x1A00, NULL},
  {0x02, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C13_02, 0x1A01, NULL},
};
const _objd SDO1C32[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C32_00, 12, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C32_01, 0x01, &Obj.SyncMgrParam1.Sync_mode},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName1C32_02, 30000, &Obj.SyncMgrParam1.CycleTime},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C32_04, 0x4000, &Obj.SyncMgrParam1.Sync_modes_supported},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName1C32_05, 7000, &Obj.SyncMgrParam1.Minimum_Cycle_Time},
  {0x0C, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C32_0C, 0, &Obj.SyncMgrParam1.CycleTimeTooSmallCnt},
};
const _objd SDO1C33[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName1C33_00, 12, NULL},
  {0x01, DTYPE_UNSIGNED16, 16, ATYPE_RW, acName1C33_01, 0x22, &Obj.SyncMgrParam2.Sync_mode},
  {0x02, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName1C33_02, 30000, &Obj.SyncMgrParam2.CycleTime},
  {0x04, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C33_04, 0x4000, &Obj.SyncMgrParam2.Sync_modes_supported},
  {0x05, DTYPE_UNSIGNED32, 32, ATYPE_RW, acName1C33_05, 7000, &Obj.SyncMgrParam2.Minimum_Cycle_Time},
  {0x0C, DTYPE_UNSIGNED16, 16, ATYPE_RO, acName1C33_0C, 0, &Obj.SyncMgrParam2.CycleTimeTooSmallCnt},
};
const _objd SDO6000[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_TXPDO, acName6000, 0, &Obj.slave_status},
};
const _objd SDO6001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName6001_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_01, 0, &Obj.slave2master[0]},
  {0x02, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_02, 0, &Obj.slave2master[1]},
  {0x03, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_03, 0, &Obj.slave2master[2]},
  {0x04, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_04, 0, &Obj.slave2master[3]},
  {0x05, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_05, 0, &Obj.slave2master[4]},
  {0x06, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_06, 0, &Obj.slave2master[5]},
  {0x07, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_07, 0, &Obj.slave2master[6]},
  {0x08, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_08, 0, &Obj.slave2master[7]},
  {0x09, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_09, 0, &Obj.slave2master[8]},
  {0x0A, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_TXPDO, acName6001_0A, 0, &Obj.slave2master[9]},
};
const _objd SDO7000[] =
{
  {0x0, DTYPE_UNSIGNED8, 8, ATYPE_RO | ATYPE_RXPDO, acName7000, 0, &Obj.master_status},
};
const _objd SDO7001[] =
{
  {0x00, DTYPE_UNSIGNED8, 8, ATYPE_RO, acName7001_00, 10, NULL},
  {0x01, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_01, 0, &Obj.master2slave[0]},
  {0x02, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_02, 0, &Obj.master2slave[1]},
  {0x03, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_03, 0, &Obj.master2slave[2]},
  {0x04, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_04, 0, &Obj.master2slave[3]},
  {0x05, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_05, 0, &Obj.master2slave[4]},
  {0x06, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_06, 0, &Obj.master2slave[5]},
  {0x07, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_07, 0, &Obj.master2slave[6]},
  {0x08, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_08, 0, &Obj.master2slave[7]},
  {0x09, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_09, 0, &Obj.master2slave[8]},
  {0x0A, DTYPE_UNSIGNED64, 64, ATYPE_RO | ATYPE_RXPDO, acName7001_0A, 0, &Obj.master2slave[9]},
};
const _objd SDO8000[] =
{
  {0x0, DTYPE_UNSIGNED16, 16, ATYPE_RW | ATYPE_RXPDO | ATYPE_TXPDO, acName8000, 0, &Obj.sdo_len},
};

const _objectlist SDOobjects[] =
{
  {0x1000, OTYPE_VAR, 0, 0, acName1000, SDO1000},
  {0x1008, OTYPE_VAR, 0, 0, acName1008, SDO1008},
  {0x1009, OTYPE_VAR, 0, 0, acName1009, SDO1009},
  {0x100A, OTYPE_VAR, 0, 0, acName100A, SDO100A},
  {0x1018, OTYPE_RECORD, 4, 0, acName1018, SDO1018},
  {0x1600, OTYPE_RECORD, 1, 0, acName1600, SDO1600},
  {0x1601, OTYPE_RECORD, 10, 0, acName1601, SDO1601},
  {0x1A00, OTYPE_RECORD, 1, 0, acName1A00, SDO1A00},
  {0x1A01, OTYPE_RECORD, 10, 0, acName1A01, SDO1A01},
  {0x1C00, OTYPE_ARRAY, 4, 0, acName1C00, SDO1C00},
  {0x1C12, OTYPE_ARRAY, 2, 0, acName1C12, SDO1C12},
  {0x1C13, OTYPE_ARRAY, 2, 0, acName1C13, SDO1C13},
  {0x1C32, OTYPE_RECORD, 5, 0, acName1C32, SDO1C32},
  {0x1C33, OTYPE_RECORD, 5, 0, acName1C33, SDO1C33},
  {0x6000, OTYPE_VAR, 0, 0, acName6000, SDO6000},
  {0x6001, OTYPE_ARRAY, 10, 0, acName6001, SDO6001},
  {0x7000, OTYPE_VAR, 0, 0, acName7000, SDO7000},
  {0x7001, OTYPE_ARRAY, 10, 0, acName7001, SDO7001},
  {0x8000, OTYPE_VAR, 0, 0, acName8000, SDO8000},
  {0xffff, 0xff, 0xff, 0xff, NULL, NULL}
};
