#include <stdio.h>

#include "pico/stdlib.h"

void main(void) {
	stdio_init_all();
	int i = 0;

	gpio_init(PICO_DEFAULT_LED_PIN);
	gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

	for ( ;; ) {
		gpio_put(PICO_DEFAULT_LED_PIN, i & 1);
		printf("Hello %d\n", i++);
		sleep_ms(1000);
	}
}
