# ADXL345_RP2040
A simple C++ library for Raspberry Pi Pico to control the nice ADXL345 accelerometer from Analog Devices.
Based on https://github.com/adafruit/Adafruit_ADXL345 with added support for enabling interrupts.
Only I²C interface is supported.
To use motion sensing functions interrupts, you will need to set detection tresholds.
Helper functions to set tresholds for free fall detection are provided.
Code is very much work-in-progress, functions for other interrupts are missing.

Official datasheet: https://www.analog.com/media/en/technical-documentation/data-sheets/ADXL345.pdf

Example:
```c++
#include <stdio.h>
#include "ADXL345.h"

ADXL345 accelerometer;
int interrupt_pin = 0;

void accelerometer_interrupt_handler(uint gpio, uint32_t events) {
    printf("interrupt! reasons: 0x%x\n",
     accelerometer.getInterruptSources()); // get interrupt reasons and clear latched motion interrupts
}

int main()
{
    stdio_init_all();

    accelerometer = ADXL345();
    accelerometer.begin(ADXL345_DEFAULT_ADDRESS, // I2C address
                    i2c_default, // Pico I2C bus (0 is default)
                    PICO_DEFAULT_I2C_SDA_PIN, // SDA pin (4 is default)
                    PICO_DEFAULT_I2C_SCL_PIN); // SCL pin (5 is default)

    accelerometer.setRange(ADXL345_RANGE_16_G); // set 16 g range
    accelerometer.setFreefallInterruptTime(350); // 350 ms minimum
    accelerometer.setFreefallInterruptTreshold(600); // below 600 mg
    accelerometer.setInterrupt(ADXL345_INT_FREE_FALL, true); // enable free fall interrupt

    gpio_set_irq_enabled_with_callback(interrupt_pin, GPIO_IRQ_EDGE_RISE, true, &accelerometer_interrupt_handler);

    while (true) {
        printf("X: %d Y: %d Z: %d\n", 
            // note: these are raw values, to get real gravitational force, you can use accelerometer.getEvent()
            accelerometer.getX(),
            accelerometer.getY(),
            accelerometer.getZ()
        );

        sleep_ms(1000);
    }
}
```

CMakeLists.txt:
```cmake
...
add_library(pico_ADXL345 ADXL345.h ADXL345.cpp)
target_link_libraries(pico_ADXL345 pico_stdlib hardware_i2c)

target_link_libraries(your_project
        ...
        pico_ADXL345
)
```