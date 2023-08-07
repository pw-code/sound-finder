
// GP25
#define PIN_LED   PICO_DEFAULT_LED_PIN

//==== Microphones (x3) -- I2S

// Clock and Word-Select shared for all devices
// Then 3x Data pins for addressing 6x I2S microphones (2 channels per 3x pins)

#define PIN_I2S_CLK    0
#define PIN_I2S_WS     1
// All INPUT pins (data 0,1,2) must be consequtive GPIO's for PIO to address.
#define PIN_I2S_DATA0  2
#define PIN_I2S_DATA1  3
#define PIN_I2S_DATA2  4


//==== OV7670 Camera -- parallel interface

// Inputs (data, pixel clock)
// These must be consequtive GPIO's for one PIO program to read 
#define PIN_OV7670_D0     5
#define PIN_OV7670_D1     6
#define PIN_OV7670_D2     7
#define PIN_OV7670_D3     8
#define PIN_OV7670_D4     9
#define PIN_OV7670_D5     10
#define PIN_OV7670_D6     11
#define PIN_OV7670_D7     12
#define PIN_OV7670_HREF   13
#define PIN_OV7670_VSYNC  14
#define PIN_OV7670_PCLK   15

// PWM pin. Fit these around the SPI & I2C pins
#define PIN_OV7670_XCLK   22

// Data/Command interface (I2C)  (Using I2C0)
#define PIN_OV7670_SIO_D  20  /*I2C0 SDA*/
#define PIN_OV7670_SIO_C  21  /*I2C0 SCL*/

//==== SPI (TFT LCD & SD Card) -- SPI 0
#define PIN_SPI_SCK       18
#define PIN_SPI_MOSI      19
#define PIN_SPI_MISO      16

#define PIN_LCD_DC        17

#define PIN_LCD_CS        26
#define PIN_SDCARD_CS     27


// Test pin (for logic analser to 'see' program states)
#define PIN_TEST          28
// TODO: Consider using GP25 (LED) which is available on TP5 (test point 5 on rear)
