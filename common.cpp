/**
 * Copyright (C) 2023 Bosch Sensortec GmbH. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#define BME68X_CS_PIN 5

#include "common.h"

/******************************************************************************/
/*!                 Macro definitions                                         */
/*! BME68X shuttle board ID */
#define BME68X_SHUTTLE_ID 0x93

/******************************************************************************/
/*!                Static variable definition                                 */
static uint8_t dev_addr;

/******************************************************************************/
/*!                User interface functions                                   */

static SPISettings bme_spi_settings(4000000, MSBFIRST, SPI_MODE0);

/*!
 * I2C read function map to COINES platform
 */
// BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t *)intf_ptr;

//     (void)intf_ptr;

//     return coines_read_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, reg_data, (uint16_t)len);
// }

BME68X_INTF_RET_TYPE bme68x_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_adr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(device_adr);
    Wire.write(reg_addr);
    Wire.endTransmission(false);

    Wire.requestFrom(device_adr, (uint8_t)len);
    for (uint32_t i = 0; i < len; i++)
    {
        reg_data[i] = Wire.read();
    }

    return 0;
}

/*!
 * I2C write function map to COINES platform
 */
// BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
// {
//     uint8_t device_addr = *(uint8_t *)intf_ptr;

//     (void)intf_ptr;

//     return coines_write_i2c(COINES_I2C_BUS_0, device_addr, reg_addr, (uint8_t *)reg_data, (uint16_t)len);
// }

BME68X_INTF_RET_TYPE bme68x_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;

    Wire.beginTransmission(device_addr);
    Wire.write(reg_addr);
    for (uint32_t i = 0; i < len; i++)
    {
        Wire.write(reg_data[i]);
    }
    Wire.endTransmission();

    return 0;
}

/*!
 * SPI read function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;

    SPI.beginTransaction(bme_spi_settings);
    digitalWrite(BME68X_CS_PIN, LOW); 

    SPI.transfer(reg_addr | 0x80); 
    for (uint32_t i = 0; i < len; i++)
    {
        reg_data[i] = SPI.transfer(0x00); 

    digitalWrite(BME68X_CS_PIN, HIGH); 
    SPI.endTransaction();

    return 0;
    }
}

/*!
 * SPI write function map to COINES platform
 */
BME68X_INTF_RET_TYPE bme68x_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t len, void *intf_ptr)
{
    uint8_t device_addr = *(uint8_t *)intf_ptr;

    SPI.beginTransaction(bme_spi_settings);
    digitalWrite(BME68X_CS_PIN, LOW);
    SPI.transfer(reg_addr & 0x7F);
    for (uint32_t i = 0; i < len; i++)
    {
        SPI.transfer(reg_data[i]);
    }
    digitalWrite(BME68X_CS_PIN, HIGH);  
    SPI.endTransaction();

    return 0;
}

/*!
 * Delay function map to COINES platform
 */
void bme68x_delay_us(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;
    delayMicroseconds(period);
}

void bme68x_check_rslt(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
    case BME68X_OK:

        /* Do nothing */
        break;
    case BME68X_E_NULL_PTR:
        printf("API name [%s]  Error [%d] : Null pointer\r\n", api_name, rslt);
        break;
    case BME68X_E_COM_FAIL:
        printf("API name [%s]  Error [%d] : Communication failure\r\n", api_name, rslt);
        break;
    case BME68X_E_INVALID_LENGTH:
        printf("API name [%s]  Error [%d] : Incorrect length parameter\r\n", api_name, rslt);
        break;
    case BME68X_E_DEV_NOT_FOUND:
        printf("API name [%s]  Error [%d] : Device not found\r\n", api_name, rslt);
        break;
    case BME68X_E_SELF_TEST:
        printf("API name [%s]  Error [%d] : Self test error\r\n", api_name, rslt);
        break;
    case BME68X_W_NO_NEW_DATA:
        printf("API name [%s]  Warning [%d] : No new data found\r\n", api_name, rslt);
        break;
    default:
        printf("API name [%s]  Error [%d] : Unknown error code\r\n", api_name, rslt);
        break;
    }
}

int8_t bme68x_interface_init(struct bme68x_dev *bme, uint8_t intf)
{
    int8_t rslt = BME68X_OK;
    uint8_t shuttle_id;
    /* Bus configuration : I2C */
    if (intf == BME68X_I2C_INTF)
    {
        printf("I2C Interface\n");
        dev_addr = BME68X_I2C_ADDR_HIGH;
        bme->read = bme68x_i2c_read;
        bme->write = bme68x_i2c_write;
        bme->intf = BME68X_I2C_INTF;
        Serial.begin(115200);
        while (!Serial)
            delay(10); // wait for serial monitor to open
        Wire.begin(21, 22);
        Wire.setClock(400000);
    }
    /* Bus configuration : SPI */
    else if (intf == BME68X_SPI_INTF)
    {
        printf("SPI Interface\n");
        bme->read = bme68x_spi_read;
        bme->write = bme68x_spi_write;
        bme->intf = BME68X_SPI_INTF;

        SPI.begin();
    }
    else
    {
        rslt = BME68X_E_NULL_PTR;
    }

    delay(100);
    bme->delay_us = bme68x_delay_us;
    bme->intf_ptr = &dev_addr;
    bme->amb_temp = 25; /* The ambient temperature in deg C is used for defining the heater temperature */

    return rslt;
}
