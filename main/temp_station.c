/* ULP riscv DS18B20 1wire temperature sensor example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <string.h>
#include "esp_sleep.h"
#include "esp_err.h"
#include "esp_log.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "ulp_riscv.h"
#include "ulp_main.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ssd1306.h"
#include "ds18b20.h"
#include "ds18b20_types.h"
#include "onewire_bus.h"
#include "config.h"
#include "../components/ra01s/ra01s.h"

#define TAG "GPIO"

static ssd1306_handle_t ssd1306_dev = NULL;
static char textBuffers[MAX_OLED_LINES][21];
static int nrTextItems = 0;
char text[25] = "";

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

static void init_ulp_program(void);

static void addOledString(char *text, bool redraw, bool reset)
{

  if (reset)
  {
    nrTextItems = 0;
  }
  if (nrTextItems < MAX_OLED_LINES)
  {
    nrTextItems++;
  }
  // Shift items
  else
  {
    for (int i = 0; i < MAX_OLED_LINES - 1; i++)
    {
      strcpy(textBuffers[i], textBuffers[i + 1]);
    }
  }

  char truncated[21] = {0};
  strncpy(truncated, text, 20);
  strcpy(textBuffers[nrTextItems - 1], truncated);

  if (redraw)
  {
    ssd1306_clear_screen(ssd1306_dev, 0x00);
  }
  for (int i = 0; i < nrTextItems; i++)
  {
    ssd1306_draw_string(ssd1306_dev, 0, i * OLED_TEXTSIZE, (const uint8_t *)textBuffers[i], OLED_TEXTSIZE, 1);
  }

  if (redraw)
  {
    ssd1306_refresh_gram(ssd1306_dev);
  }
}

int32_t getTemperature(uint8_t idx)
{
  if (idx >= MAX_DS18B20)
  {
    return INT32_MIN;
  }

  int32_t *arrayPtr = (int32_t *)&ulp_ds18b20_temperature;

  return arrayPtr[idx];
}

void setRom(uint8_t idx, uint64_t value)
{
  if (idx > MAX_DS18B20)
    return;

  uint64_t *arrayPtr = (uint64_t *)&ulp_ds18b20_roms;

  arrayPtr[idx] = value;
}

uint64_t getRom(uint8_t idx)
{
  if (idx > MAX_DS18B20)
    return 0;

  return ((uint64_t *)&ulp_ds18b20_roms)[idx];
}

void initLora()
{
  // Init LoRa
  LoRaInit();

  int8_t txPowerInDbm = 14;

  float tcxoVoltage = 3.3;     // don't use TCXO
  bool useRegulatorLDO = true; // use only LDO in all modes
  // LoRaDebugPrint(true);

  if (LoRaBegin(868000000, txPowerInDbm, tcxoVoltage, useRegulatorLDO) != 0)
  {
    ESP_LOGE(TAG, "Does not recognize the module");
    addOledString("Lora not recognized", true, false);

    while (1)
    {
      vTaskDelay(1);
    }
  }
}

void configLora()
{
  uint8_t spreadingFactor = 7;
  uint8_t bandwidth = 4;
  uint8_t codingRate = 1;
  uint16_t preambleLength = 8;
  uint8_t payloadLen = 0;
  bool crcOn = true;
  bool invertIrq = false;
  LoRaConfig(spreadingFactor, bandwidth, codingRate, preambleLength, payloadLen, crcOn, invertIrq);
}

void app_main(void)
{

  initLora();
  configLora();

#if OLED_ENABLED
  // Init OLED
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = (gpio_num_t)I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = (gpio_num_t)I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  conf.clk_flags = I2C_SCLK_SRC_FLAG_FOR_NOMAL;

  i2c_param_config(I2C_MASTER_NUM, &conf);
  i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
  ssd1306_dev = ssd1306_create(I2C_MASTER_NUM, SSD1306_I2C_ADDRESS);
  ssd1306_clear_screen(ssd1306_dev, 0x00);
  ssd1306_refresh_gram(ssd1306_dev);
#endif

  esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();

  if (cause == ESP_SLEEP_WAKEUP_COCPU_TRAP_TRIG)
  {
#if OLED_LOGGING
    sprintf(text, "TRAP TRIG wakeup!");
    addOledString(text, true, false);
    sprintf(text, "Something went wrong.");
    addOledString(text, true, false);
    while (1)
    {
      vTaskDelay(1);
    }
#endif

    vTaskDelay(pdMS_TO_TICKS(5000));
    esp_restart();
  }
  /* not a wakeup from ULP, load the firmware */
  if (cause != ESP_SLEEP_WAKEUP_ULP)
  {
    uint32_t device_count = 0;
    uint64_t rom_ids[MAX_DS18B20] = {0};

    for (int i = 0; i < NUM_DS18B20_GPIO; i++)
    {
      uint8_t local_device_count = 0;
      gpio_num_t gpio;
      switch (i)
      {
      case 0:
        gpio = SENSOR_1WIRE_GPIO1;
        break;
      case 1:
        gpio = SENSOR_1WIRE_GPIO2;
        break;
      case 2:
        gpio = SENSOR_1WIRE_GPIO3;
        break;
      default:
        gpio = SENSOR_1WIRE_GPIO4;
        break;
      }

      // Searching for DS18B10 sensors
      // install 1-wire bus
      onewire_bus_handle_t bus = NULL;
      onewire_bus_config_t bus_config = {
          .bus_gpio_num = gpio,
      };
      onewire_bus_rmt_config_t rmt_config = {
          .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
      };
      ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

      ds18b20_device_handle_t ds18b20s[MAX_DS18B20_PER_GPIO];
      onewire_device_iter_handle_t iter = NULL;
      onewire_device_t next_onewire_device;
      esp_err_t search_result = ESP_OK;

      printf("Searching GPIO %d", i + 1);

      // create 1-wire device iterator, which is used for device search
      ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
      do
      {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK)
        { // found a new device, let's check if we can upgrade it to a DS18B20
          ds18b20_config_t ds_cfg = {};
          // check if the device is a DS18B20, if so, return the ds18b20 handle
          if (ds18b20_new_device(&next_onewire_device, &ds_cfg, &ds18b20s[local_device_count]) == ESP_OK)
          {
            uint8_t globalIndex = i * MAX_DS18B20_PER_GPIO + local_device_count;
            ESP_LOGI(TAG, "Found a DS18B20[%d], address: %016llX", globalIndex, next_onewire_device.address);
            rom_ids[globalIndex] = next_onewire_device.address;

            sprintf(text, "%x %x", (uint8_t)((rom_ids[globalIndex] >> 56) & 0xFF), (uint8_t)((rom_ids[globalIndex] >> 48) & 0xFF));
            addOledString(text, true, false);

            local_device_count++;
            if (local_device_count == MAX_DS18B20_PER_GPIO)
            {
              break;
            }
          }
        }
        else
        {
          break;
        }

      } while (search_result != ESP_ERR_NOT_FOUND);

      // Set resolution
      for (int k = 0; k < local_device_count; k++)
      {
        ESP_ERROR_CHECK(ds18b20_set_resolution(ds18b20s[k], DS18B20_RESOLUTION));
      }
      device_count += local_device_count;

      ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    }

    sprintf(text, "%ld device(s) found", device_count);
    addOledString(text, true, false);

    if (device_count == 0)
    {

      addOledString("Restarting ESP...", true, false);
      vTaskDelay(pdMS_TO_TICKS(5000));

      esp_restart();
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    init_ulp_program();
    vTaskDelay(pdMS_TO_TICKS(100));

    for (int k = 0; k < MAX_DS18B20; k++)
    {
      setRom(k, rom_ids[k]);
    }
  }
  /* Wakeup from backup timer */
  else if (cause == ESP_SLEEP_WAKEUP_TIMER)
  {
    addOledString("Timer wakeup", true, false);
    while (1)
    {
      vTaskDelay(1);
    }
    esp_restart();
  }
  /* ULP Risc-V detected a change in temperatures */
  else if (cause == ESP_SLEEP_WAKEUP_ULP)
  {
#if OLED_LOGGING
    for (int i = 0; i < MAX_DS18B20; i++)
    {

      if (getRom(i) != 0)
      {
        sprintf(text, "%x %x: %.1f", (uint8_t)((getRom(i) >> 56) & 0xFF), (uint8_t)((getRom(i) >> 48) & 0xFF), getTemperature(i) * 0.0625);
        addOledString(text, true, false);
      }
    }

    sprintf(text, "RAM left %ld", esp_get_free_heap_size());
    addOledString(text, true, false);
    sprintf(text, "tstack: %d", uxTaskGetStackHighWaterMark(NULL));
    addOledString(text, true, false);
    ESP_LOGI(TAG, "RAM left %ld", esp_get_free_heap_size());
    ESP_LOGI(TAG, "tstack: %d", uxTaskGetStackHighWaterMark(NULL));

    vTaskDelay(pdMS_TO_TICKS(1000));
#endif

    // Create buffer for node id + sensor ids + measurements
    uint8_t buffer[sizeof(uint16_t) + MAX_DS18B20 * 2 * sizeof(int16_t)];
    uint8_t bufferSize = 2;

    buffer[0] = LORA_NODE_ID & 0xFF;
    buffer[1] = LORA_NODE_ID >> 8 & 0xFF;

    uint8_t *offset = buffer + sizeof(uint16_t);

    // Copy last 2 bytes of sensor ids and measured values into buffer
    for (int i = 0; i < MAX_DS18B20; i++)
    {
      if (getRom(i) == 0 || getTemperature(i) == INT32_MIN)
      {
        continue;
      }

      // Copy last 2 byte of sensors rom Id to buffer
      *(int16_t *)offset = (getRom(i) >> 48) & 0xFFFF;
      offset += sizeof(uint16_t);
      // Copy last measured value (in 10th °) to buffer
      *(int16_t *)offset = (int16_t)((int16_t)(getTemperature(i))) * 0.625;
      offset += sizeof(uint16_t);
      bufferSize += 4;
    }

    if (LoRaSend(buffer, bufferSize, SX126x_TXMODE_SYNC) == false)
    {
      ESP_LOGE(pcTaskGetName(NULL), "LoRaSend fail");

#if OLED_LOGGING

      addOledString("Lora Send fail", true, false);
      while (1)
      {
        vTaskDelay(1);
      }
#endif
    }

    int lost = GetPacketLost();
    if (lost != 0)
    {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
#if OLED_LOGGING
      sprintf(text, "%d packets lost", lost);
      addOledString("Lora Send fail", true, false);
      while (1)
      {
        vTaskDelay(pdMS_TO_TICKS(1));
      }
#endif
    }
  }

#if OLED_SLEEP
  ssd1306_sleep(ssd1306_dev);
#endif
  // Lora Sleep
  SetSleep(false);

  // Enable wakeup by ULP (normal way)
  // Also enable a timer wakeup to reset the esp in case nothing happened for 2.5x polling time
  ESP_ERROR_CHECK(esp_sleep_enable_ulp_wakeup());
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(DS18B20_BACKUP_TIMER * 1000 * 1000));
  vTaskDelay(pdMS_TO_TICKS(500));
  esp_deep_sleep_start();
}

static void init_ulp_program(void)
{
  esp_err_t err = ulp_riscv_load_binary(ulp_main_bin_start, (ulp_main_bin_end - ulp_main_bin_start));
  ESP_ERROR_CHECK(err);

  // Polling period
  ulp_set_wakeup_period(0, DS18B20_POLL_RATE_SECONDS * 1000 * 1000);

  /* Start the program */
  err = ulp_riscv_run();
  ESP_ERROR_CHECK(err);
}
