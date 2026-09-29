#include <Arduino.h>
#include "bme68x.h"
#include <stdint.h>
#include "common.h"
#include <ArduinoJson.h>
#include "esp_system.h"
#include <Preferences.h>

struct bme68x_dev        bme;
struct bme68x_conf       conf;
struct bme68x_heatr_conf heatr_conf;
struct bme68x_data       data[3];

uint32_t sample_count  = 0;
int      current_label = 0;
bool     stop_sampling = false;

uint8_t  scanning_count = 0;
bool     is_sleeping    = false;
uint8_t  last_gas_index = 255;
bool     first_row      = true;
uint8_t  mac[6];
int      bootCounter;
String   seedStr;

#define BUTTON_PIN 13
// #define STOP_PIN   12    

void IRAM_ATTR onButtonPress()   { current_label = 1; }
void IRAM_ATTR onButtonRelease() { current_label = 0; }
// void IRAM_ATTR onStopPress() { stop_sampling = true; }  
const char* configJson = R"({
  "configHeader": {
    "dateCreated_ISO": "2026-04-06T20:18:03.212Z",
    "appVersion": "3.2.0",
    "boardType": "board_8",
    "boardMode": "burn_in",
    "boardLayout": "grouped"
  },
  "configBody": {
    "heaterProfiles": [
      {
        "id": "heater_354",
        "timeBase": 140,
        "temperatureTimeVectors": [
          [
            320,
            5
          ],
          [
            100,
            2
          ],
          [
            100,
            10
          ],
          [
            100,
            30
          ],
          [
            200,
            5
          ],
          [
            200,
            5
          ],
          [
            200,
            5
          ],
          [
            320,
            5
          ],
          [
            320,
            5
          ],
          [
            320,
            5
          ]
        ]
      }
    ],
    "dutyCycleProfiles": [
      {
        "id": "duty_5_10",
        "numberScanningCycles": 5,
        "numberSleepingCycles": 10
      }
    ],
    "sensorConfigurations": [
      {
        "sensorIndex": 0,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 1,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 2,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 3,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 4,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 5,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 6,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      },
      {
        "sensorIndex": 7,
        "heaterProfile": "heater_354",
        "dutyCycleProfile": "duty_5_10"
      }
    ]
  }
})";

uint16_t heatr_temp_prof[10];
uint16_t heatr_dur_prof[10];
int      scanningCycles;
int      sleepingCycles;
uint16_t shared_heatr_dur;
uint8_t  profile_len;
const char* heater_id;
uint16_t timeBase;

#define BME68X_VALID_DATA  UINT8_C(0xB0)
#define SAMPLE_COUNT       UINT8_C(50)

void printFileHeader() {
    Serial.println("{");
    Serial.println("\"configHeader\": {");
    Serial.printf("  \"dateCreated_ISO\": \"2026-04-06T20:18:03.212Z\",\n");
    Serial.printf("  \"appVersion\": \"3.2.0\",\n");
    Serial.printf("  \"boardType\": \"adafruit_feather_esp32_v2\",\n");
    Serial.printf("  \"boardMode\": \"burn_in\",\n");
    Serial.printf("  \"boardLayout\": \"grouped\"\n");
    Serial.println("},");
    Serial.println("\"configBody\": {");
    Serial.println("  \"heaterProfiles\": [{");
    Serial.println("    \"id\": \"heater_354\",");
    Serial.println("    \"timeBase\": 140,");
    Serial.println("    \"temperatureTimeVectors\": [");
    for (int i = 0; i < 10; i++)
        Serial.printf("      [%d, %d]%s\n", heatr_temp_prof[i], heatr_dur_prof[i], i < 9 ? "," : "");
    Serial.println("    ]");
    Serial.println("  }],");
    Serial.println("  \"dutyCycleProfiles\": [{");
    Serial.printf("    \"id\": \"duty_5_10\",\n");
    Serial.printf("    \"numberScanningCycles\": %d,\n", scanningCycles);
    Serial.printf("    \"numberSleepingCycles\": %d\n",  sleepingCycles);
    Serial.println("  }],");
    Serial.println("  \"sensorConfigurations\": [");
    Serial.println("    {\"sensorIndex\": 0, \"heaterProfile\": \"heater_354\", \"dutyCycleProfile\": \"duty_5_10\"}");
    Serial.println("  ]");
    Serial.println("},");
    Serial.println("\"rawDataHeader\": {");
    Serial.printf("  \"counterPowerOnOff\": %d,\n",  bootCounter);
    Serial.printf("  \"seedPowerOnOff\": \"%s\",\n", seedStr.c_str());
    Serial.println("  \"counterFileLimit\": 1,");
    Serial.println("  \"dateCreated\": \"0\",");
    Serial.println("  \"dateCreated_ISO\": \"0\",");
    Serial.println("  \"firmwareVersion\": \"1.0.0\",");
    Serial.printf("  \"boardId\": \"%02X%02X%02X%02X%02X%02X\"\n",
                  mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    Serial.println("},");
    Serial.println("\"rawDataBody\": {");
    Serial.println("  \"dataColumns\": [");
    Serial.println("    {\"name\": \"Sensor Index\",             \"unit\": \"\",              \"format\": \"integer\", \"key\": \"sensor_index\",               \"colId\": 1},");
    Serial.println("    {\"name\": \"Sensor ID\",                \"unit\": \"\",              \"format\": \"integer\", \"key\": \"sensor_id\",                  \"colId\": 2},");
    Serial.println("    {\"name\": \"Time Since PowerOn\",       \"unit\": \"Milliseconds\",  \"format\": \"integer\", \"key\": \"timestamp_since_poweron\",    \"colId\": 3},");
    Serial.println("    {\"name\": \"Real time clock\",          \"unit\": \"Unix Timestamp\",\"format\": \"integer\", \"key\": \"real_time_clock\",           \"colId\": 4},");
    Serial.println("    {\"name\": \"Temperature\",              \"unit\": \"DegreesCelcius\",\"format\": \"float\",   \"key\": \"temperature\",               \"colId\": 5},");
    Serial.println("    {\"name\": \"Pressure\",                 \"unit\": \"Hectopascals\",  \"format\": \"float\",   \"key\": \"pressure\",                  \"colId\": 6},");
    Serial.println("    {\"name\": \"Relative Humidity\",        \"unit\": \"Percent\",       \"format\": \"float\",   \"key\": \"relative_humidity\",         \"colId\": 7},");
    Serial.println("    {\"name\": \"Resistance Gassensor\",     \"unit\": \"Ohms\",          \"format\": \"float\",   \"key\": \"resistance_gassensor\",      \"colId\": 8},");
    Serial.println("    {\"name\": \"Heater Profile Step Index\",\"unit\": \"\",              \"format\": \"integer\", \"key\": \"heater_profile_step_index\", \"colId\": 9},");
    Serial.println("    {\"name\": \"Scanning Mode Enabled\",    \"unit\": \"\",              \"format\": \"boolean\", \"key\": \"scanning_enabled\",          \"colId\": 10},");
    Serial.println("    {\"name\": \"Scanning Cycle Index\",     \"unit\": \"\",              \"format\": \"integer\", \"key\": \"scanning_cycle_index\",      \"colId\": 11},");
    Serial.println("    {\"name\": \"Label Tag\",                \"unit\": \"\",              \"format\": \"integer\", \"key\": \"label_tag\",                 \"colId\": 12},");
    Serial.println("    {\"name\": \"Error Code\",               \"unit\": \"\",              \"format\": \"integer\", \"key\": \"error_code\",                \"colId\": 13}");
    Serial.println("  ],");
    Serial.println("  \"dataBlock\": [");
}

void closeFile() {
    Serial.println("\n  ]");
    Serial.println("}");
    Serial.println("}");
}

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    // boot counter
    Preferences prefs;
    prefs.begin("bme_app", false);
    bootCounter = prefs.getInt("boot_count", 0) + 1;
    prefs.putInt("boot_count", bootCounter);
    prefs.end();

    // random seed
    randomSeed(esp_random());
    const char chars[] = "abcdefghijklmnopqrstuvwxyz0123456789";
    for (int i = 0; i < 16; i++) seedStr += chars[random(0, 36)];

    // MAC address
    esp_read_mac(mac, ESP_MAC_WIFI_STA);

    // label button
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonPress,   FALLING);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), onButtonRelease, RISING);

    // stop button 
    // pinMode(STOP_PIN, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(STOP_PIN), onStopPress, FALLING);

  
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, configJson);
    if (err) {
        Serial.print("JSON parse failed: ");
        Serial.println(err.c_str());
        return;
    }

    const char* boardType = doc["configHeader"]["boardType"];
    const char* boardMode = doc["configHeader"]["boardMode"];
    const char* dateISO   = doc["configHeader"]["dateCreated_ISO"];
    Serial.printf("Board Type: %s, Mode: %s, Date Created: %s\n", boardType, boardMode, dateISO);

    JsonObject hp          = doc["configBody"]["heaterProfiles"][0];
    heater_id              = hp["id"];
    timeBase               = hp["timeBase"];
    JsonArray tempTimeVectors = hp["temperatureTimeVectors"];
    profile_len            = tempTimeVectors.size();

    for (uint8_t i = 0; i < profile_len; i++) {
        heatr_temp_prof[i] = tempTimeVectors[i][0];
        heatr_dur_prof[i]  = tempTimeVectors[i][1];
    }

    JsonObject dcp     = doc["configBody"]["dutyCycleProfiles"][0];
    const char* duty_id = dcp["id"];
    scanningCycles     = dcp["numberScanningCycles"];
    sleepingCycles     = dcp["numberSleepingCycles"];
    Serial.printf("Duty cycle: %s, scan=%d sleep=%d\n", duty_id, scanningCycles, sleepingCycles);

    bme68x_interface_init(&bme, BME68X_I2C_INTF);

    int8_t rslt = bme68x_init(&bme);
    bme68x_check_rslt("bme68x_init", rslt);

    conf.filter  = BME68X_FILTER_OFF;
    conf.odr     = BME68X_ODR_NONE;
    conf.os_hum  = BME68X_OS_16X;
    conf.os_pres = BME68X_OS_1X;
    conf.os_temp = BME68X_OS_2X;
    rslt = bme68x_set_conf(&conf, &bme);
    bme68x_check_rslt("bme68x_set_conf", rslt);

    heatr_conf.enable           = BME68X_ENABLE;
    heatr_conf.heatr_temp_prof  = heatr_temp_prof;
    heatr_conf.heatr_dur_prof   = heatr_dur_prof;
    heatr_conf.profile_len      = 10;
    heatr_conf.shared_heatr_dur = (uint16_t)(timeBase - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme) / 1000));
    rslt = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &heatr_conf, &bme);
    bme68x_check_rslt("bme68x_set_heatr_conf", rslt);

    rslt = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
    bme68x_check_rslt("bme68x_set_op_mode", rslt);

    printFileHeader();
}

void loop() {
    // stop button disabled — uncomment below to re-enable
    // if (stop_sampling) { closeFile(); while (true) delay(1000); }

    uint8_t n_fields;
    int8_t rslt;

    uint32_t del_period = bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &conf, &bme)
                        + (heatr_conf.shared_heatr_dur * 1000);
    bme.delay_us(del_period, bme.intf_ptr);

    if (is_sleeping) {
        bme68x_set_op_mode(BME68X_SLEEP_MODE, &bme);
        uint32_t sum_mults = 0;
        for (int i = 0; i < profile_len; i++) sum_mults += heatr_dur_prof[i];
        delay((uint32_t)sleepingCycles * sum_mults * 140);
        is_sleeping = false;
        bme68x_set_op_mode(BME68X_PARALLEL_MODE, &bme);
        return;
    }

    rslt = bme68x_get_data(BME68X_PARALLEL_MODE, data, &n_fields, &bme);
    bme68x_check_rslt("bme68x_get_data", rslt);

    for (uint8_t i = 0; i < n_fields; i++) {
        if (data[i].status & BME68X_NEW_DATA_MSK) {
            if (!first_row) Serial.print(",\n");
            first_row = false;

            int error_code = 0;
            if (!(data[i].status & BME68X_GASM_VALID_MSK)) error_code = 1;
            if (!(data[i].status & BME68X_HEAT_STAB_MSK))  error_code = 1;

            Serial.printf("    [%d, %lu, %lu, 0, %.6f, %.6f, %.6f, %.6f, %d, %d, %d, %d, %d]",
                0,
                (uint32_t)(ESP.getEfuseMac() & 0xFFFFFFFF),
                (unsigned long)millis(),
                data[i].temperature,
                data[i].pressure / 100.0f,
                data[i].humidity,
                data[i].gas_resistance,
                data[i].gas_index,
                is_sleeping ? 0 : 1,
                scanning_count,
                current_label,
                error_code
            );

            if (data[i].gas_index == 0 && last_gas_index == 9) {
                scanning_count++;
                if (scanning_count >= scanningCycles) {
                    scanning_count = 0;
                    is_sleeping    = true;
                }
            }
            last_gas_index = data[i].gas_index;
            sample_count++;
        }
    }
}