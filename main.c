#include <stdio.h>
#include "pico/stdlib.h"

const int LED_PIN = PICO_DEFAULT_LED_PIN;

int main() {
    stdio_init_all();

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    for (int i=0; ;++i) {
        gpio_put(LED_PIN, 1);
        printf("Hello, world! %d\n", i);
        sleep_ms(50);

        gpio_put(LED_PIN, 0);
        sleep_ms(2000);
    }
    
    return 0;
}