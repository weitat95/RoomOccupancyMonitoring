
#include "mbed.h"
#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

void init_sensor();

uint32_t take_measurement();
