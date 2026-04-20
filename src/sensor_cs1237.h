#ifndef __SENSOR_CS1237_H
#define __SENSOR_CS1237_H

#include "sensor_bulk.h" // struct sensor_bulk

// CS1237 ADC sensor for probe/weight applications
struct cs1237_sensor {
    struct sensor_bulk sb;
    uint8_t oid;
    uint8_t homing;
    uint32_t trigger_clock;
    uint32_t rest_ticks;
    uint32_t threshold;
    uint32_t last_adc;
    uint8_t trsync_oid;
    uint8_t trigger_reason;
    uint8_t error_reason;
    struct gpio_in dout;
    struct gpio_out sclk;
};

#endif // sensor_cs1237.h
