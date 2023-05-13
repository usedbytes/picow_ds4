/**
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

// ===========================================================================
// ===========================================================================
// ===========================================================================

/*
 * Derived from the btstack hid_host_demo:
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Modifications Copyright (C) 2021 Brian Starkey <stark3y@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL MATTHIAS
 * RINGWALD OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at
 * contact@bluekitchen-gmbh.com
 *
 */
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>

#include "btstack_run_loop.h"
#include "btstack_config.h"
#include "btstack.h"

#define BTN_BIT_A       0
#define BTN_BIT_B       1
#define BTN_BIT_X       2
#define BTN_BIT_Y       3
#define BTN_BIT_L1      4
#define BTN_BIT_L2      5
#define BTN_BIT_L3      6
#define BTN_BIT_R1      7
#define BTN_BIT_R2      8
#define BTN_BIT_R3      9
#define BTN_BIT_START   10
#define BTN_BIT_SELECT  11
#define BTN_BIT_HEART   12
#define BTN_BIT_STAR    13

struct bt_hid_state {
	uint16_t buttons;
	uint8_t lx;
	uint8_t ly;
	uint8_t rx;
	uint8_t ry;
	uint8_t hat;
	uint8_t pad;
};

struct bt_hid_task_params {
	//QueueHandle_t state_queue;
};

#define MAX_ATTRIBUTE_VALUE_SIZE 300

// SN30 Pro
//static const char * remote_addr_string = "E4:17:D8:EE:73:0E";
// Real DS4
static const char * remote_addr_string = "00:22:68:DB:D3:66";

static struct bt_hid_task_params bt_hid_task_params;

static bd_addr_t remote_addr;
static bd_addr_t connected_addr;
static btstack_packet_callback_registration_t hci_event_callback_registration;

// SDP
static uint8_t hid_descriptor_storage[MAX_ATTRIBUTE_VALUE_SIZE];

static uint16_t hid_host_cid = 0;
static bool     hid_host_descriptor_available = false;
static hid_protocol_mode_t hid_host_report_mode = HID_PROTOCOL_MODE_REPORT;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void hid_host_setup(void){
	// Initialize L2CAP
	l2cap_init();

	// Initialize HID Host
	hid_host_init(hid_descriptor_storage, sizeof(hid_descriptor_storage));
	hid_host_register_packet_handler(packet_handler);

	// Allow sniff mode requests by HID device and support role switch
	gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_SNIFF_MODE | LM_LINK_POLICY_ENABLE_ROLE_SWITCH);

	// try to become master on incoming connections
	hci_set_master_slave_policy(HCI_ROLE_MASTER);

	// register for HCI events
	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);
}

/*
 * Usage page: 0x1, 0x39, 0xf       HAT
 * Usage page: 0x1, 0x30, 0x80      X (Lx)
 * Usage page: 0x1, 0x31, 0x80      Y (Ly)
 * Usage page: 0x1, 0x32, 0x80      Z (Rx)
 * Usage page: 0x1, 0x35, 0x80      Rz (Ry)
 * Usage page: 0x2, 0xc4, 0x0       Accelerator (R2)
 * Usage page: 0x2, 0xc5, 0x0       Brake (L2)
 * Usage page: 0x9, 0x1, 0x0        A (Circle)
 * Usage page: 0x9, 0x2, 0x0        B (Cross)
 * Usage page: 0x9, 0x3, 0x0
 * Usage page: 0x9, 0x4, 0x0        X (Triangle)
 * Usage page: 0x9, 0x5, 0x0        Y (Square)
 * Usage page: 0x9, 0x6, 0x0
 * Usage page: 0x9, 0x7, 0x0        L1
 * Usage page: 0x9, 0x8, 0x0        R1
 * Usage page: 0x9, 0x9, 0x0
 * Usage page: 0x9, 0xa, 0x0
 * Usage page: 0x9, 0xb, 0x0        Select
 * Usage page: 0x9, 0xc, 0x0        Start
 * Usage page: 0x9, 0xd, 0x0        Heart
 * Usage page: 0x9, 0xe, 0x0        L3
 * Usage page: 0x9, 0xf, 0x0        R3
 * Usage page: 0x9, 0x10, 0x0
*/
#define USAGE_PAGE_DESKTOP  0x1
#define USAGE_PAGE_SIM      0x2
#define USAGE_PAGE_BUTTON   0x9

#define DESKTOP_USAGE_HAT 0x39
#define DESKTOP_USAGE_LX  0x30
#define DESKTOP_USAGE_LY  0x31
#define DESKTOP_USAGE_RX  0x32
#define DESKTOP_USAGE_RY  0x35
#define SIM_USAGE_R2      0xc4
#define SIM_USAGE_L2      0xc5

const struct bt_hid_state default_state = {
	.buttons = 0,
	.lx = 0x80,
	.ly = 0x80,
	.rx = 0x80,
	.ry = 0x80,
	.hat = 0xf,
	.pad = 0x0,
};

static void hid_host_handle_interrupt_report(const uint8_t * report, uint16_t report_len){
	// check if HID Input Report
	if (report_len < 1) {
		return;
	}

	if (*report != 0xa1) {
		return;
	}

	report++;
	report_len--;

	btstack_hid_parser_t parser;
	btstack_hid_parser_init(&parser,
			hid_descriptor_storage_get_descriptor_data(hid_host_cid),
			hid_descriptor_storage_get_descriptor_len(hid_host_cid),
			HID_REPORT_TYPE_INPUT, report, report_len);

	struct bt_hid_state state = { 0 };

	while (btstack_hid_parser_has_more(&parser)){
		uint16_t usage_page;
		uint16_t usage;
		int32_t  value;
		btstack_hid_parser_get_field(&parser, &usage_page, &usage, &value);

		//printf("Page: 0x%x, Usage: 0x%x, Value: 0x%x\n", usage_page, usage, value);

		switch (usage_page) {
		case USAGE_PAGE_DESKTOP:
			switch (usage) {
			case DESKTOP_USAGE_HAT:
				state.hat = value;
				break;
			case DESKTOP_USAGE_LX:
				state.lx = value;
				break;
			case DESKTOP_USAGE_LY:
				state.ly = value;
				break;
			case DESKTOP_USAGE_RX:
				state.rx = value;
				break;
			case DESKTOP_USAGE_RY:
				state.ry = value;
				break;
			}
			break;
		case USAGE_PAGE_SIM:
			switch (usage) {
			case SIM_USAGE_L2:
				state.buttons |= (value ? (1 << BTN_BIT_L2) : 0);
				break;
			case SIM_USAGE_R2:
				state.buttons |= (value ? (1 << BTN_BIT_R2) : 0);
				break;
			}
			break;
		case USAGE_PAGE_BUTTON:
			switch (usage) {
			case 0x1:
				state.buttons |= (value ? (1 << BTN_BIT_A) : 0);
				break;
			case 0x2:
				state.buttons |= (value ? (1 << BTN_BIT_B) : 0);
				break;
			case 0x4:
				state.buttons |= (value ? (1 << BTN_BIT_X) : 0);
				break;
			case 0x5:
				state.buttons |= (value ? (1 << BTN_BIT_Y) : 0);
				break;
			case 0x7:
				state.buttons |= (value ? (1 << BTN_BIT_L1) : 0);
				break;
			case 0x8:
				state.buttons |= (value ? (1 << BTN_BIT_R1) : 0);
				break;
			case 0xb:
				state.buttons |= (value ? (1 << BTN_BIT_SELECT) : 0);
				break;
			case 0xc:
				state.buttons |= (value ? (1 << BTN_BIT_START) : 0);
				break;
			case 0xd:
				state.buttons |= (value ? (1 << BTN_BIT_HEART) : 0);
				break;
			case 0xe:
				state.buttons |= (value ? (1 << BTN_BIT_L3) : 0);
				break;
			case 0xf:
				state.buttons |= (value ? (1 << BTN_BIT_R3) : 0);
				break;
			}
			break;
		}
	}

	//xQueueSend(task_params->state_queue, &state, portMAX_DELAY);
}

static void bt_hid_disconnected(bd_addr_t addr)
{
	hid_host_cid = 0;
	hid_host_descriptor_available = false;
	gap_drop_link_key_for_bd_addr(addr);

	//xQueueSend(task_params->state_queue, &default_state, portMAX_DELAY);
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);

	uint8_t   event;
	uint8_t   hid_event;
	bd_addr_t event_addr;
	uint8_t   status;

	if (packet_type != HCI_EVENT_PACKET) {
		return;
	}

	event = hci_event_packet_get_type(packet);
	switch (event) {
	case BTSTACK_EVENT_STATE:
		// On boot, we try a manual connection
		if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
			printf("Starting hid_host_connect (%s)", bd_addr_to_str(remote_addr));
			status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
			if (status != ERROR_CODE_SUCCESS){
				printf("hid_host_connect command failed: 0x%02x", status);
			}
		}
		break;
	case HCI_EVENT_PIN_CODE_REQUEST:
		printf("Pin code request. Responding '0000'");
		hci_event_pin_code_request_get_bd_addr(packet, event_addr);
		gap_pin_code_response(event_addr, "0000");
		break;
	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		printf("SSP User Confirmation Request: %d", little_endian_read_32(packet, 8));
		break;
	case HCI_EVENT_HID_META:
		hid_event = hci_event_hid_meta_get_subevent_code(packet);
		switch (hid_event) {
		case HID_SUBEVENT_INCOMING_CONNECTION:
			hid_subevent_incoming_connection_get_address(packet, event_addr);
			printf("Accepting connection from %s", bd_addr_to_str(event_addr));
			hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
			break;
		case HID_SUBEVENT_CONNECTION_OPENED:
			status = hid_subevent_connection_opened_get_status(packet);
			hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
			if (status != ERROR_CODE_SUCCESS) {
				printf("Connection to %s failed: 0x%02x", bd_addr_to_str(event_addr), status);
				bt_hid_disconnected(event_addr);
				return;
			}
			hid_host_descriptor_available = false;
			hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
			printf("Connected to %s", bd_addr_to_str(event_addr));
			bd_addr_copy(connected_addr, event_addr);
			break;
		case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
			status = hid_subevent_descriptor_available_get_status(packet);
			if (status == ERROR_CODE_SUCCESS){
				hid_host_descriptor_available = true;
				printf("HID descriptor available");
			} else {
				printf("Couldn't process HID Descriptor");
			}
			break;
		case HID_SUBEVENT_REPORT:
			if (hid_host_descriptor_available){
				hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			} else {
				printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			}
			break;
		case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
			status = hid_subevent_set_protocol_response_get_handshake_status(packet);
			if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL){
				printf("Protocol handshake error: 0x%02x", status);
				break;
			}
			hid_protocol_mode_t proto = hid_subevent_set_protocol_response_get_protocol_mode(packet);
			switch (proto) {
			case HID_PROTOCOL_MODE_BOOT:
				printf("Negotiated protocol: BOOT");
				break;
			case HID_PROTOCOL_MODE_REPORT:
				printf("Negotiated protocol: REPORT");
				break;
			default:
				printf("Negotiated unknown protocol: 0x%x", proto);
				break;
			}
			break;
		case HID_SUBEVENT_CONNECTION_CLOSED:
			printf("HID connection closed: %s", bd_addr_to_str(connected_addr));
			bt_hid_disconnected(connected_addr);
			break;
		default:
			printf("Unknown HID subevent: 0x%x", hid_event);
			break;
		}
		break;
	default:
		printf("Unknown HCI event: 0x%x", event);
		break;
	}
}

void bt_hid_task()
{
	// Looks like btstack doesn't give us a way to pass this to the
	// packet handler without a global.
	//task_params = &bt_hid_task_params;

	//btstack_init();

	hid_host_setup();

	sscanf_bd_addr(remote_addr_string, remote_addr);
	bt_hid_disconnected(remote_addr);

	hci_power_control(HCI_POWER_ON);

	btstack_run_loop_execute();

	// Run loop should never return, but just incase
	//vTaskDelete(NULL);
}

// ===========================================================================
// ===========================================================================
// ===========================================================================

void main(void) {
	stdio_init_all();

	sleep_ms(1000);
	printf("Hello over uart\n");

	if (cyw43_arch_init()) {
		printf("Wi-Fi init failed");
		return;
	}

	// main()
	bt_hid_task();

	while (true) {
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
		sleep_ms(1000);
		cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
		sleep_ms(1000);
	}
}
