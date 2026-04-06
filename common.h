#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <Arduino.h>
#include "bme68x.h"
#include <unistd.h>

#include "Wire.h"
#include <SPI.h>


int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf);
void bme68x_check_rslt(const char api_name[], int8_t rslt);

