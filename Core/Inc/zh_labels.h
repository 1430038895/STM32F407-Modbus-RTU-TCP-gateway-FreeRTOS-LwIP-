/* Auto-generated point-name -> Chinese label (unicode code points) */
#ifndef __ZH_LABELS_H
#define __ZH_LABELS_H
#include <stdint.h>
#include <string.h>
static const uint16_t L_baud[] = {0x6CE2,0x7279,0x7387};
static const uint16_t L_station[] = {0x7AD9,0x70B9,0x53F7};
static const uint16_t L_decimals[] = {0x5C0F,0x6570,0x4F4D};
static const uint16_t L_K[] = {0x004B,0x503C};
static const uint16_t L_flow_total[] = {0x7D2F,0x79EF,0x6D41,0x91CF};
static const uint16_t L_flow_total_lo[] = {0x7D2F,0x79EF,0x6D41,0x91CF,0x4F4E};
static const uint16_t L_flow_total_dec[] = {0x7D2F,0x79EF,0x6D41,0x91CF,0x5C0F};
static const uint16_t L_flow_total_pulse[] = {0x7D2F,0x79EF,0x8109,0x51B2};
static const uint16_t L_flow_temp[] = {0x4E34,0x65F6,0x6D41,0x91CF};
static const uint16_t L_flow_temp_lo[] = {0x4E34,0x65F6,0x6D41,0x91CF,0x4F4E};
static const uint16_t L_flow_temp_dec[] = {0x4E34,0x65F6,0x6D41,0x91CF,0x5C0F};
static const uint16_t L_flow_temp_pulse[] = {0x4E34,0x65F6,0x8109,0x51B2};
static const uint16_t L_flow_instant[] = {0x77AC,0x65F6,0x6D41,0x91CF};
static const uint16_t L_flow_instant_pulse[] = {0x77AC,0x65F6,0x8109,0x51B2};
static const uint16_t L_ma_4ma[] = {0x0034,0x006D,0x0041,0x4E0B,0x9650};
static const uint16_t L_ma_20ma[] = {0x0032,0x0030,0x006D,0x0041,0x4E0A,0x9650};
static const uint16_t L_press_A[] = {0x538B,0x529B,0x0041};
static const uint16_t L_press_B[] = {0x538B,0x529B,0x0042};
static const uint16_t L_pressure_kpa[] = {0x538B,0x529B,0x006B,0x0050,0x0061};
static const uint16_t L_wind_speed[] = {0x98CE,0x901F};
static const uint16_t L_wind_dir[] = {0x98CE,0x5411};
static const uint16_t L_air_temp[] = {0x6C14,0x6E29};
static const uint16_t L_humidity[] = {0x6E7F,0x5EA6};
static const uint16_t L_pressure[] = {0x6C14,0x538B};
static const uint16_t L_rain_min[] = {0x5206,0x949F,0x96E8,0x91CF};
static const uint16_t L_rain_hour[] = {0x5C0F,0x65F6,0x96E8,0x91CF};
static const uint16_t L_rain_day[] = {0x5929,0x96E8,0x91CF};
static const uint16_t L_rain_total[] = {0x7D2F,0x8BA1,0x96E8,0x91CF};
static const uint16_t L_dev_addr[] = {0x8BBE,0x5907,0x5730,0x5740};
static const uint16_t L_temp[] = {0x6E29,0x5EA6};
static const uint16_t L_temp_c[] = {0x6E29,0x5EA6,0x2103};
static const uint16_t L_pm1[] = {0x0050,0x004D,0x0031};
static const uint16_t L_pm25[] = {0x0050,0x004D,0x0032,0x002E,0x0035};
static const uint16_t L_pm10[] = {0x0050,0x004D,0x0031,0x0030};
static const uint16_t L_co2[] = {0x0043,0x004F,0x0032};
static const uint16_t L_hcho[] = {0x7532,0x919B};
static const uint16_t L_voc[] = {0x0056,0x004F,0x0043};
static const uint16_t L_ntc_res[] = {0x004E,0x0054,0x0043,0x7535,0x963B};
static const uint16_t *zh_label_lookup(const char *name, uint8_t *len)
{
    if (!strcmp(name, "baud")) { *len = 3; return L_baud; }
    if (!strcmp(name, "station")) { *len = 3; return L_station; }
    if (!strcmp(name, "decimals")) { *len = 3; return L_decimals; }
    if (!strcmp(name, "K")) { *len = 2; return L_K; }
    if (!strcmp(name, "flow_total")) { *len = 4; return L_flow_total; }
    if (!strcmp(name, "flow_total_lo")) { *len = 5; return L_flow_total_lo; }
    if (!strcmp(name, "flow_total_dec")) { *len = 5; return L_flow_total_dec; }
    if (!strcmp(name, "flow_total_pulse")) { *len = 4; return L_flow_total_pulse; }
    if (!strcmp(name, "flow_temp")) { *len = 4; return L_flow_temp; }
    if (!strcmp(name, "flow_temp_lo")) { *len = 5; return L_flow_temp_lo; }
    if (!strcmp(name, "flow_temp_dec")) { *len = 5; return L_flow_temp_dec; }
    if (!strcmp(name, "flow_temp_pulse")) { *len = 4; return L_flow_temp_pulse; }
    if (!strcmp(name, "flow_instant")) { *len = 4; return L_flow_instant; }
    if (!strcmp(name, "flow_instant_pulse")) { *len = 4; return L_flow_instant_pulse; }
    if (!strcmp(name, "ma_4ma")) { *len = 5; return L_ma_4ma; }
    if (!strcmp(name, "ma_20ma")) { *len = 6; return L_ma_20ma; }
    if (!strcmp(name, "press_A")) { *len = 3; return L_press_A; }
    if (!strcmp(name, "press_B")) { *len = 3; return L_press_B; }
    if (!strcmp(name, "pressure_kpa")) { *len = 5; return L_pressure_kpa; }
    if (!strcmp(name, "wind_speed")) { *len = 2; return L_wind_speed; }
    if (!strcmp(name, "wind_dir")) { *len = 2; return L_wind_dir; }
    if (!strcmp(name, "air_temp")) { *len = 2; return L_air_temp; }
    if (!strcmp(name, "humidity")) { *len = 2; return L_humidity; }
    if (!strcmp(name, "pressure")) { *len = 2; return L_pressure; }
    if (!strcmp(name, "rain_min")) { *len = 4; return L_rain_min; }
    if (!strcmp(name, "rain_hour")) { *len = 4; return L_rain_hour; }
    if (!strcmp(name, "rain_day")) { *len = 3; return L_rain_day; }
    if (!strcmp(name, "rain_total")) { *len = 4; return L_rain_total; }
    if (!strcmp(name, "dev_addr")) { *len = 4; return L_dev_addr; }
    if (!strcmp(name, "temp")) { *len = 2; return L_temp; }
    if (!strcmp(name, "temp_c")) { *len = 3; return L_temp_c; }
    if (!strcmp(name, "pm1")) { *len = 3; return L_pm1; }
    if (!strcmp(name, "pm25")) { *len = 5; return L_pm25; }
    if (!strcmp(name, "pm10")) { *len = 4; return L_pm10; }
    if (!strcmp(name, "co2")) { *len = 3; return L_co2; }
    if (!strcmp(name, "hcho")) { *len = 2; return L_hcho; }
    if (!strcmp(name, "voc")) { *len = 3; return L_voc; }
    if (!strcmp(name, "ntc_res")) { *len = 5; return L_ntc_res; }
    *len = 0; return 0;
}
#endif