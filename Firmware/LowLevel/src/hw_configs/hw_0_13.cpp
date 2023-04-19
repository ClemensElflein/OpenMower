#include "hw_0_13.h"

void hw_assign_pins() {
    // Assign the correct pins to the IMU's hardware SPI.
    SPI.setCS(PIN_IMU_CS);
    SPI.setSCK(PIN_IMU_SCK);
    SPI.setRX(PIN_IMU_MISO);
    SPI.setTX(PIN_IMU_MOSI);
}
