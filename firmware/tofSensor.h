
#include "mbed.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"


#define RANGE_SENSOR1 0
#define RANGE_SENSOR2 1


void init_sensor(uint8_t device_ind);

uint32_t take_measurement(uint8_t device_ind);

void reset_i2c_chn();
