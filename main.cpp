#include <Arduino.h>
#include "bme68x.h"
#include <stdint.h>
#include <unistd.h>
#include "common.h"

//#define SAMPLE_COUNT 10



struct bme68x_dev      bme;
struct bme68x_conf     conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data     data;

uint8_t  mac[6];
uint16_t sample_count = 1;



void setup() {
   
    // Read and print board ID
    //esp_read_mac(mac, ESP_MAC_WIFI_STA);
    //Serial.printf("Board ID: %02X%02X%02X%02X%02X%02X\n",
    //             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // Init I2C and wire callbacks into the dev struct
    bme68x_interface_init(&bme, BME68X_I2C_INTF);

    // Init the sensor
    int8_t rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    // Sensor config
    conf.filter = BME68X_FILTER_OFF;
    conf.odr    = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    //bme68x_check_rslt("bme68x_set_conf", rslt);

    // Heater config
    heatr_conf.enable    = BME68X_ENABLE;
    heatr_conf.heatr_temp = 300;
    heatr_conf.heatr_dur  = 100;
    rslt = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &heatr_conf, &bme);
    //bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    delay(5000); // wait for sensor to stabilize
    Serial.println("Sample, TimeStamp(ms), Temperature(C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status");
}



void loop() {
    //if (sample_count > SAMPLE_COUNT) return; // stop after 300 samples

    int8_t rslt;
    uint8_t n_fields;

    // Trigger one measurement
    rslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    // Wait for measurement to complete
    uint32_t del_period = bme68x_get_meas_dur(BME68X_FORCED_MODE, &conf, &bme)
                        + (heatr_conf.heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);

    // Read data
    rslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);

    if (n_fields) {
        Serial.printf("%u, %lu, %.2f, %.2f, %.2f, %.2f, 0x%x\n",
                      sample_count,
                      (uint32_t)millis(),
                      data.temperature,
                      data.pressure,
                      data.humidity,
                      data.gas_resistance,
                      data.status);
        sample_count++;
    }
}