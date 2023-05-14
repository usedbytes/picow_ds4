// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

extern void bt_main(void);

void main(void) {
	stdio_init_all();

	sleep_ms(1000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);

	for ( ;; ) {
		sleep_ms(1000);
		printf("Core 0\n");
	}
}
