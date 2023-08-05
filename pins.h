
// GP25
#define PIN_LED   PICO_DEFAULT_LED_PIN

//==== Microphones (x3) -- I2S

// Clock and Word-Select shared for all devices
// Then 3x Data pins for addressing 6x I2S microphones (2 channels per 3x pins)

#define PIN_I2S_CLK    2
#define PIN_I2S_WS     3
//skip GP4,GP5 in case we want to use I2C0 (SDA/SCL)

// All INPUT pins (data 0,1,2) must be consequtive GPIO's for PIO to address.
#define PIN_I2S_DATA0  6
#define PIN_I2S_DATA1  7
#define PIN_I2S_DATA2  8

#define PIN_TEST       11


//==== OV7670 Camera -- parallel interface


//==== TFT LCD -- SPI



//==== SD Card -- SPI



//==== Battery Monitor -- ADC
#define PIN_PIN_BATTERY_ADC 28
