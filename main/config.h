#define LORA_NODE_ID 0x02

// DS18B20
#define SENSOR_1WIRE_GPIO1 GPIO_NUM_12
#define SENSOR_1WIRE_GPIO2 GPIO_NUM_15
#define SENSOR_1WIRE_GPIO3 GPIO_NUM_16
#define SENSOR_1WIRE_GPIO4 0

#define NUM_DS18B20_GPIO 3
#define MAX_DS18B20_PER_GPIO 3

#define MAX_DS18B20 NUM_DS18B20_GPIO *MAX_DS18B20_PER_GPIO
#define MIN_CHANGE_FOR_WAKEUP 2 // 2*0,0625 = min 0,125° change                         // 0 = wake up always
#define DS18B20_RESOLUTION DS18B20_RESOLUTION_12B
#define DS18B20_POLL_RATE_SECONDS 60
#define DS18B20_BACKUP_TIMER 1800

// OLED Display SSD1306
#define OLED_ENABLED 1
#define OLED_LOGGING OLED_ENABLED & 0
#define OLED_SLEEP OLED_ENABLED & 1
#define I2C_MASTER_SCL_IO 17      /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18      /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUM_1  /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000 /*!< I2C master clock frequency */
#define MAX_OLED_LINES 5
#define OLED_TEXTSIZE 12
