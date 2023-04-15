#pragma once
#ifdef HW_0_9_X
#include <hw_configs/hw_0_9.h>
#elif HW_0_10_X || HW_0_11_X || HW_0_12_X
#include <hw_configs/hw_0_10_11_12.h>
#elif HW_0_13_X
#include <hw_configs/hw_0_13.h>
#else
#error No hardware version defined
#endif
