#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "MMC5603.cpp"

int main() {
    stdio_init_all();
    sleep_ms(200);
    MMC5603::init();
    MMC5603::setBasicSettings(MMC5603::BASIC_MODES::FAST_UPDATE);
    sleep_ms(1000);
    while (1) {
        MMC5603::getAllAxis();
        MMC5603::printActiveAxis();
        MMC5603::getTemp();
        MMC5603::printTemp();

        sleep_ms(50);
    }
    return 0;
}