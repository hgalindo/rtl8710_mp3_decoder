/*
 *  Copyright (C) 
 *
 */

#ifndef __RTL_COMMON_H__
#define __RTL_COMMON_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "c_types.h"

enum {
        NULL_MODE = 0,
        STATION_MODE,
        SOFTAP_MODE,
        STATIONAP_MODE,
        MAX_MODE
};

struct station_config {
    uint8 ssid[32];
    uint8 password[64];
    uint8 bssid_set;
    uint8 bssid[6];
};


#endif
