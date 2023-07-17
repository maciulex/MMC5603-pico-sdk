#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "MMC5603.cpp"

int main() {
    stdio_init_all();

    MMC5603::init();
    MMC5603::setBasicSettings(MMC5603::BASIC_MODES::FAST_UPDATE);

    while (1) {
        MMC5603::getAll();
        MMC5603::printActiveAxis();
        MMC5603::printTemp();

        sleep_ms(250);
    }
    return 0;
}