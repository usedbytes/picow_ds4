// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "bt_hid.h"

void main(void) {
	stdio_init_all();

	sleep_ms(1000);
	printf("Hello\n");

	multicore_launch_core1(bt_main);
	// Wait for init (should do a handshake with the fifo here?)
	sleep_ms(1000);

	struct bt_hid_state state;
	for ( ;; ) {
		sleep_ms(20);
		bt_hid_get_latest(&state);
		printf("buttons: %04x, l: %d,%d, r: %d,%d, l2,r2: %d,%d hat: %d\n",
				state.buttons, state.lx, state.ly, state.rx, state.ry,
				state.l2, state.r2, state.hat);
	}
}
