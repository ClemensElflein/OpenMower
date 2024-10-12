#ifndef _CONFIG_H
#define _CONFIG_H

#define BATT_ABS_MAX 28.7f
#define BATT_ABS_MIN 21.7f

#define BATT_EMPTY BATT_ABS_MIN + 0.3f
#define BATT_FULL BATT_ABS_MAX - 0.3f

#define V_BATT_CUTOFF 29.0f

#define TILT_EMERGENCY_MILLIS 2500  // Time for a single wheel to be lifted in order to count as emergency (0 disable). This is to filter uneven ground.
#define LIFT_EMERGENCY_MILLIS 100   // Time for both wheels to be lifted in order to count as emergency (0 disable). This is to filter uneven ground.

#define V_CHARGE_MAX 30.0f  // Default YF-C500 max. charge voltage
#define I_CHARGE_MAX 1.5f   // Default YF-C500 max. charge current

#define V_CHARGE_ABS_MAX 40.0f  // Absolute max. limited by D2/D3 Schottky
#define I_CHARGE_ABS_MAX 5.0f   // Absolute max. limited by D2/D3 Schottky

#define RAIN_THRESHOLD 700U  // (Yet) Stock-CoverUI exclusive raw ADC value. Is the measured value is lower, it rains

#define SOUND_VOLUME 80    // Volume (0-100%)
#define LANGUAGE 'e', 'n'  // ISO 639-1 (2-char) language code (en, de, ...)

#define CONFIG_FILENAME "/openmower.cfg"  // Where our llhl_config get saved in LittleFS (flash)

#endif  // _CONFIG_H