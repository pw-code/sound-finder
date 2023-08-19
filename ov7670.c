#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/sem.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/pwm.h"

#include "pins.h"
#include "video.h"

#include "ov7670reg.h"

#include "ov7670.pio.h"

//Validate pins.h and ov7670.pio agree on fixed GPIO pin assignments
#if PIN_OV7670_HREF != ov7670_pio_href_pin
# error ("Mismatched OV7670 HREF pin")
#endif
#if PIN_OV7670_VSYNC != ov7670_pio_vsync_pin
# error ("Mismatched OV7670 VSYNC pin")
#endif
#if PIN_OV7670_PCLK != ov7670_pio_pclk_pin
# error ("Mismatched OV7670 PCLK pin")
#endif

PIO ov7670_pio;

uint dma_pixel_chan_0;
uint dma_pixel_chan_1;

i2c_inst_t *ov7670_i2c;

struct semaphore video_sem;

// OV7670 Initialisation sequence

typedef struct {
  uint8_t reg;   ///< Register address
  uint8_t value; ///< Value to store
} OV7670_command;

static const OV7670_command
    OV7670_init[] = {
        // Initialisation sequences from Adafruit: https://github.com/adafruit/Adafruit_OV7670/blob/master/src/ov7670.c

    // https://gist.github.com/bsantraigi/a0d8678fbd5759468bc57b42d92e2d99
        { OV7670_REG_TSLB, 0x04 },	/* OV */
//        { OV7670_REG_COM7, 0 },	/* VGA */
        {OV7670_REG_COM7, OV7670_COM7_SIZE_VGA | OV7670_COM7_RGB},
//        {OV7670_REG_COM7, OV7670_COM7_SIZE_QVGA | OV7670_COM7_RGB},
        {OV7670_REG_RGB444, 0},
        {OV7670_REG_COM15, OV7670_COM15_RGB565 | OV7670_COM15_R00FF},

        { OV7670_REG_COM3, OV7670_COM3_DCWEN },
        { OV7670_REG_COM14, 0x19 },
        { 0x72, 0x11 },
        { 0x73, 0xf1 },

        { OV7670_REG_HSTART, 0x16 },
        { OV7670_REG_HSTOP, 0x04 },
        { OV7670_REG_HREF, 0xa4 },
        { OV7670_REG_VSTART, 0x02 },
        { OV7670_REG_VSTOP, 0x7a },
        { OV7670_REG_VREF, 0x0a },

        /* Mystery scaling numbers */
        { 0x70, 0x3a }, { 0x71, 0x35 },
//        { 0x72, 0x11 }, { 0x73, 0xf0 },
        { 0xa2,/* 0x02 changed to 1*/1 }, { OV7670_REG_COM10, 0x0 },
        /* Gamma curve values */
        { 0x7a, 0x20 }, { 0x7b, 0x10 },
        { 0x7c, 0x1e }, { 0x7d, 0x35 },
        { 0x7e, 0x5a }, { 0x7f, 0x69 },
        { 0x80, 0x76 }, { 0x81, 0x80 },
        { 0x82, 0x88 }, { 0x83, 0x8f },
        { 0x84, 0x96 }, { 0x85, 0xa3 },
        { 0x86, 0xaf }, { 0x87, 0xc4 },
        { 0x88, 0xd7 }, { 0x89, 0xe8 },
        /* AGC and AEC parameters.  Note we start by disabling those features,
        then turn them only after tweaking the values. */
        { OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP },
        { OV7670_REG_GAIN, 0 }, { OV7670_REG_AECH, 0 },
        { OV7670_REG_COM4, 0x40 }, /* magic reserved bit */
        { OV7670_REG_COM17, 0x00 },
        { OV7670_REG_COM9, 0x18 }, /* 4x gain + magic rsvd bit */
        { OV7670_REG_BD50MAX, 0x05 }, { OV7670_REG_BD60MAX, 0x07 },
        { OV7670_REG_AEW, 0x95 }, { OV7670_REG_AEB, 0x33 },
        { OV7670_REG_VPT, 0xe3 }, { OV7670_REG_HAECC1, 0x78 },
        { OV7670_REG_HAECC2, 0x68 }, { 0xa1, 0x03 }, /* magic */
        { OV7670_REG_HAECC3, 0xd8 }, { OV7670_REG_HAECC4, 0xd8 },
        { OV7670_REG_HAECC5, 0xf0 }, { OV7670_REG_HAECC6, 0x90 },
        { OV7670_REG_HAECC7, 0x94 },
        { OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_AGC | OV7670_COM8_AEC },
        { 0x30, 0 }, { 0x31, 0 },//disable some delays
        /* Almost all of these are magic "reserved" values.  */
        { OV7670_REG_COM5, 0x61 }, { OV7670_REG_COM6, 0x4b },
        { 0x16, 0x02 }, { OV7670_REG_MVFP, 0x07 },
        { 0x21, 0x02 }, { 0x22, 0x91 },
        { 0x29, 0x07 }, { 0x33, 0x0b },
        { 0x35, 0x0b }, { 0x37, 0x1d },
        { 0x38, 0x71 }, { 0x39, 0x2a },
        { OV7670_REG_COM12, 0x78 }, { 0x4d, 0x40 },
        { 0x4e, 0x20 }, { OV7670_REG_GFIX, 0 },
        /*{0x6b, 0x4a},*/{ 0x74, 0x10 },
        { 0x8d, 0x4f }, { 0x8e, 0 },
        { 0x8f, 0 }, { 0x90, 0 },
        { 0x91, 0 }, { 0x96, 0 },
        { 0x9a, 0 }, { 0xb0, 0x84 },
        { 0xb1, 0x0c }, { 0xb2, 0x0e },
        { 0xb3, 0x82 }, { 0xb8, 0x0a },

        /* More reserved magic, some of which tweaks white balance */
        { 0x43, 0x0a }, { 0x44, 0xf0 },
        { 0x45, 0x34 }, { 0x46, 0x58 },
        { 0x47, 0x28 }, { 0x48, 0x3a },
        { 0x59, 0x88 }, { 0x5a, 0x88 },
        { 0x5b, 0x44 }, { 0x5c, 0x67 },
        { 0x5d, 0x49 }, { 0x5e, 0x0e },
        { 0x6c, 0x0a }, { 0x6d, 0x55 },
        { 0x6e, 0x11 }, { 0x6f, 0x9e }, /* it was 0x9F "9e for advance AWB" */
        { 0x6a, 0x40 },
        { OV7670_REG_BLUE, 0x60 },
        { OV7670_REG_RED, 0x40 },
        { OV7670_REG_GGAIN, 0x80 },
        { OV7670_REG_COM8, OV7670_COM8_FASTAEC | OV7670_COM8_AECSTEP | OV7670_COM8_AGC | OV7670_COM8_AEC | OV7670_COM8_AWB },

        /* Matrix coefficients */
        { 0x4f, 0x80 }, { 0x50, 0x80 },
        { 0x51, 0 },    { 0x52, 0x22 },
        { 0x53, 0x5e }, { 0x54, 0x80 },
        { 0x58, 0x9e },

        { OV7670_REG_COM16, OV7670_COM16_AWBGAIN }, { OV7670_REG_EDGE, 0 },
        { 0x75, 0x05 }, { OV7670_REG_REG76, 0xe1 },
        { 0x4c, 0 },     { 0x77, 0x01 },
        { OV7670_REG_COM13, /*0xc3*/0x48 }, { 0x4b, 0x09 },
        { 0xc9, 0x60 },		/*{OV7670_REG_COM16, 0x38},*/
        { 0x56, 0x40 },

        { 0x34, 0x11 }, { OV7670_REG_COM11, OV7670_COM11_EXP | OV7670_COM11_HZAUTO },
        { 0xa4, 0x82/*Was 0x88*/ }, { 0x96, 0 },
        { 0x97, 0x30 }, { 0x98, 0x20 },
        { 0x99, 0x30 }, { 0x9a, 0x84 },
        { 0x9b, 0x29 }, { 0x9c, 0x03 },
        { 0x9d, 0x4c }, { 0x9e, 0x3f },
        { 0x78, 0x04 },

        /* Extra-weird stuff.  Some sort of multiplexor register */
        { 0x79, 0x01 }, { 0xc8, 0xf0 },
        { 0x79, 0x0f }, { 0xc8, 0x00 },
        { 0x79, 0x10 }, { 0xc8, 0x7e },
        { 0x79, 0x0a }, { 0xc8, 0x80 },
        { 0x79, 0x0b }, { 0xc8, 0x01 },
        { 0x79, 0x0c }, { 0xc8, 0x0f },
        { 0x79, 0x0d }, { 0xc8, 0x20 },
        { 0x79, 0x09 }, { 0xc8, 0x80 },
        { 0x79, 0x02 }, { 0xc8, 0xc0 },
        { 0x79, 0x03 }, { 0xc8, 0x40 },
        { 0x79, 0x05 }, { 0xc8, 0x30 },
        { 0x79, 0x26 },


        {OV7670_REG_EDGE, 0x02}, //Edge enhancement factor 

        //{OV7670_REG_COM11, 0xFF}, //night mode all options on : automatic
        //{OV7670_REG_COM11, OV7670_COM11_NIGHT | OV7670_COM11_EXP | OV7670_COM11_HZAUTO | (OV7670_COM11_NMFR & 0xFF) | OV7670_COM11_BAND},

        // 3.3 Frame rate adjustment for 13 Mhz input clock
        // 14.3fps, PCLK = 13Mhz
        {0x11, 0x02}, //was 0x01, slowed down to reduce fps and leave more video overlay processing time
        {0x6b, 0x4a},
        {0x2a, 0x00},
        {0x2b, 0x00},
        {0x92, 0x46},
        {0x93, 0x00},
        {0x3b, 0x0a},
        // 4.2 Night Mode with Auto Frame Rate
        // 14.3fps ~ 3.6fps night mode for 50Hz light environment
        {0x3b, 0xca},
        {0x11, 0x02}, //was 0x01, slowed down to reduce fps and leave more video overlay processing time

#undef OV7670_SHOW_TEST_PATTERN
#ifdef OV7670_SHOW_TEST_PATTERN
        // Colour bars
        // {OV7670_REG_SCALING_XSC, 0x3A}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x35 | 0x80}, // Enable pattern
        {OV7670_REG_SCALING_XSC, 0x00}, // Enable pattern
        {OV7670_REG_SCALING_YSC, 0x80}, // Enable pattern
        // Shifting 1
        // {OV7670_REG_SCALING_XSC, 0x80}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x00}, // Enable pattern
        // Fade to grey
        // {OV7670_REG_SCALING_XSC, 0x80}, // Enable pattern
        // {OV7670_REG_SCALING_YSC, 0x80}, // Enable pattern
#else
        // {OV7670_REG_SCALING_XSC, 0},
        // {OV7670_REG_SCALING_YSC, 0},
#endif

        // Colour balance
        // {OV7670_REG_GAIN, 0x00},
        // {OV7670_REG_RED, 0x40},
        // {OV7670_REG_BLUE, 0x80},
        // {OV7670_REG_GGAIN, 0xF0},
        // {OV7670_REG_RBIAS, 0x82},
        // {OV7670_REG_BBIAS, 0x82},
        // {OV7670_REG_GbBIAS, 0x82},

        {OV7670_REG_EDGE, 0x02}, //Edge enhancement factor 

        {0xFF, 0xFF},       // End-of-data marker
};

//====================================================================================================

void pixel_dma_handler() {
    // which DMA channel triggered IRQ 1?
    if (dma_channel_get_irq1_status(dma_pixel_chan_0)) {

        //channel 0
        last_video_buf = 0;
        // clear the interrupt
        dma_channel_acknowledge_irq1(dma_pixel_chan_0);
        // Reset buffer, ready for next time we are triggered/chained
        dma_channel_set_write_addr(dma_pixel_chan_0, video_buffer[0], false);

    } else {

        //channel 1
        last_video_buf = 1;
        // clear the interrupt
        dma_channel_acknowledge_irq1(dma_pixel_chan_1);
        // Reset buffer, ready for next time we are triggered/chained
        dma_channel_set_write_addr(dma_pixel_chan_1, video_buffer[1], false);
    }

    sem_release(&video_sem);
}


static void init_pixel_dma(PIO pio) {
    // We want to use DMA to gather all the pixel data in alternate row buffers
    dma_pixel_chan_0 = dma_claim_unused_channel(true);
    dma_pixel_chan_1 = dma_claim_unused_channel(true);

    dma_channel_config cfg0 = dma_channel_get_default_config(dma_pixel_chan_0);
    channel_config_set_transfer_data_size(&cfg0, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg0, false);
    channel_config_set_write_increment(&cfg0, true);
    channel_config_set_dreq(&cfg0, pio_get_dreq(pio, sm_ov7670, false));
    channel_config_set_chain_to(&cfg0, dma_pixel_chan_1); //chain to 1
    dma_channel_configure(dma_pixel_chan_0, &cfg0,
        video_buffer[0],        // Destination pointer
        &pio->rxf[sm_ov7670],   // Source pointer, lower 16bits of the RX FIFO
        1 + VIDEO_COLUMNS,      // Number of transfers
        false                   // Start later
    );

    dma_channel_config cfg1 = dma_channel_get_default_config(dma_pixel_chan_1);
    channel_config_set_transfer_data_size(&cfg1, DMA_SIZE_16);
    channel_config_set_read_increment(&cfg1, false);
    channel_config_set_write_increment(&cfg1, true);
    channel_config_set_dreq(&cfg1, pio_get_dreq(pio, sm_ov7670, false));
    channel_config_set_chain_to(&cfg1, dma_pixel_chan_0); //back to 0
    dma_channel_configure(dma_pixel_chan_1, &cfg1,
        video_buffer[1],        // Destination pointer
        &pio->rxf[sm_ov7670],   // Source pointer, lower 16bits of the RX FIFO
        1 + VIDEO_COLUMNS,      // Number of transfers
        false                   // Start later
    );

    // Tell the DMA to raise IRQ 1 when a channel finishes a block (audio.c uses IRQ 0)
    dma_channel_set_irq1_enabled(dma_pixel_chan_0, true);
    dma_channel_set_irq1_enabled(dma_pixel_chan_1, true);

    irq_set_exclusive_handler(DMA_IRQ_1, pixel_dma_handler);
    irq_set_enabled(DMA_IRQ_1, true);

    //start chain
    dma_channel_start(dma_pixel_chan_0);
}

//====================================================================================================

static void ov7670_write_reg(uint8_t reg, uint8_t val) {
	i2c_write_blocking(ov7670_i2c, OV7670_ADDR, (uint8_t[]){ reg, val }, 2, false);
}

static uint8_t ov7670_read_reg(uint8_t reg) {
    uint8_t data;
	i2c_write_blocking(ov7670_i2c, OV7670_ADDR, &reg, 1, false);
	i2c_read_blocking(ov7670_i2c, OV7670_ADDR, &data, 1, false);
    return data;
}

/**
 * @brief Init OV7670 camera input using I2C and PIO
 */
void ov7670_init(PIO pio, i2c_inst_t *i2c) {
    ov7670_pio = pio;
    ov7670_i2c = i2c;
    
    sem_init(&video_sem, 0, 1);

    // OV7670 needs XCLK before it will function. Typical speed 24MHz
    gpio_set_function(PIN_OV7670_XCLK, GPIO_FUNC_PWM);
    uint pwm_slice = pwm_gpio_to_slice_num(PIN_OV7670_XCLK);
    uint pwm_chan = pwm_gpio_to_channel(PIN_OV7670_XCLK);
    //const float div = (float)clock_get_hz(clk_sys) / (24 * 1000 * 1000); //24Mhz 
    const float div = (float)clock_get_hz(clk_sys) / (13 * 1000 * 1000); //13Mhz as we have datasheet info for that, and 24mhz is too fast for our video drawing code to process
    pwm_set_clkdiv(pwm_slice, div/2);
    // Set period of 2 cycles 
    pwm_set_wrap(pwm_slice, 1);
    pwm_set_chan_level(pwm_slice, pwm_chan, 1); //1 cycle high
    pwm_set_enabled(pwm_slice, true);

    // non-PIO pins that are read by PIO as WAIT GPIO
    gpio_init(PIN_OV7670_HREF);
    gpio_set_dir(PIN_OV7670_HREF, GPIO_IN);
    gpio_init(PIN_OV7670_VSYNC);
    gpio_set_dir(PIN_OV7670_VSYNC, GPIO_IN);
    gpio_init(PIN_OV7670_PCLK);
    gpio_set_dir(PIN_OV7670_PCLK, GPIO_IN);

    // Load OV7670 pixel loader
    ov7670_program_load(pio, PIN_OV7670_VSYNC, PIN_OV7670_D0);

    init_pixel_dma(pio);


    // Initialise camera settings
    ov7670_write_reg(OV7670_REG_COM7, OV7670_COM7_RESET);
    sleep_ms(1); //tReset
    const OV7670_command *cmd = OV7670_init;
    while (cmd->reg != 0xFF) {
        ov7670_write_reg(cmd->reg, cmd->value);
        cmd++;
    }
}

void ov7670_diag() {
    printf("OV7670 PID %02x\n", ov7670_read_reg(OV7670_REG_PID));
    printf("OV7670 VER %02x\n", ov7670_read_reg(OV7670_REG_VER));
}

/**
 * @brief Wait for a line of ov7670 screen data
 */
void ov7670_wait_for_buffer() {
    sem_acquire_blocking(&video_sem);

    // // Wait for either of the video DMA IRQs to trigger (should happen pretty soon at 30fps)
    // while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) == 0) {
    //     tight_loop_contents();
    // }
    // // Then wait for it to be serviced (so we can trust last_video_buf is up-to-date)
    // while ((dma_hw->intr & ((1u << dma_pixel_chan_0) | (1u << dma_pixel_chan_1))) != 0) {
    //     tight_loop_contents();
    // }  
}