// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2023 Brian Starkey <stark3y@gmail.com>

// Setup and run the bluetooth stack, will never return
// i.e. start this on Core 1 with multicore_launch_core1()
void bt_main(void);

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t l2;
	uint8_t r2;
	uint8_t hat;
	uint8_t pad;
};

// Get the latest controller state
void bt_hid_get_latest(struct bt_hid_state *dst);
