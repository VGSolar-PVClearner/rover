#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_ESC_2BLD6010_ENABLED
#if !defined(HAL_BUILD_AP_PERIPH) && BOARD_FLASH_SIZE > 1024
#define AP_ESC_2BLD6010_ENABLED 1
#else
#define AP_ESC_2BLD6010_ENABLED 0
#endif
#endif
