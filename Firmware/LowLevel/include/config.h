#ifndef _CONFIG_H
#define _CONFIG_H

#define BATT_ABS_MAX 28.7f
#define BATT_ABS_Min 21.7f

#define BATT_FULL BATT_ABS_MAX - 0.3f
#define BATT_EMPTY BATT_ABS_Min + 0.3f

#define TILT_EMERGENCY_MILLIS 2500  // Time for a single wheel to be lifted in order to count as emergency (0 disable). This is to filter uneven ground.
#define LIFT_EMERGENCY_MILLIS 100   // Time for both wheels to be lifted in order to count as emergency (0 disable). This is to filter uneven ground.

#define V_CHARGE_MAX 30.0f  // Default YF-C500 max. charge voltage
#define I_CHARGE_MAX 1.5f   // Default YF-C500 max. charge current

#define V_CHARGE_ABS_MAX 40.0f  // Absolute max. limited by D2/D3 Schottky
#define I_CHARGE_ABS_MAX 5.0f   // Absolute max. limited by D2/D3 Schottky

#endif  // _CONFIG_H