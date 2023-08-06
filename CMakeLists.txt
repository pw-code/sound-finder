cmake_minimum_required(VERSION 3.13)

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(sound_finder)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# rest of your project

add_executable(sound-finder
    main.c
    audio.h
    audio.c
    offsets.h
    offsets.c
    pins.h
    video.h
    video.c
    ov7670reg.h
    ili9341.h
)

pico_generate_pio_header(sound-finder ${CMAKE_CURRENT_LIST_DIR}/i2s.pio)

pico_generate_pio_header(sound-finder ${CMAKE_CURRENT_LIST_DIR}/ov7670.pio)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(sound-finder 
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_dma
    hardware_i2c
    hardware_spi
    hardware_pwm
    )

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(sound-finder)

# add url via pico_set_program_url
# just useful data that shows in the image if examined with picotool (eg point to a git repo)
#pico_set_program_url(sound-finder https://???)

# printf to usb/serial/both? (1=on, 0=off)
pico_enable_stdio_usb(sound-finder 1)
pico_enable_stdio_uart(sound-finder 0) #We are using the UART pins 

# support upload and monitoring from "make upload" and "make monitor"
add_custom_target(upload 
    COMMAND picotool load -u sound-finder.bin -f
    DEPENDS sound-finder
)

set(tty_port "/dev/ttyACM0")
add_custom_target(monitor 
    COMMAND bash -c "echo waiting for ${tty_port}; while [ ! -r ${tty_port} ]; do sleep 1; done; minicom -o -D ${tty_port}"
    VERBATIM
)