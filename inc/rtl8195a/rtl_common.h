/*
 *  Copyright (C) 2013 -2014  Espressif System
 *
 */

#ifndef __ESP_COMMON_H__
#define __ESP_COMMON_H__

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "c_types.h"
#include "eagle_soc.h"
#include "gpio_register.h"
#include "ets_sys.h"
#include "pin_mux_register.h"
#include "spi_register.h"
#include "uart_register.h"

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
