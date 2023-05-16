/*
 * Derived from the btstack hid_host_demo:
 * Copyright (C) 2017 BlueKitchen GmbH
 *
 * Modifications Copyright (C) 2021-2023 Brian Starkey <stark3y@gmail.com>
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
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/async_context.h"

#include "btstack_run_loop.h"
#include "btstack_config.h"
#include "btstack.h"
#include "classic/sdp_server.h"

#include "bt_hid.h"

#define MAX_ATTRIBUTE_VALUE_SIZE 512

// SN30 Pro
//static const char * remote_addr_string = "E4:17:D8:EE:73:0E";
// Real DS4
//static const char * remote_addr_string = "00:22:68:DB:D3:66";
// Knockoff DS4
//static const char * remote_addr_string = "A5:15:66:8E:91:3B";
// Brian C Knockoff DS4
static const char * remote_addr_string = "8C:41:F2:D0:32:43";

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

	sdp_init();

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

const struct bt_hid_state default_state = {
	.buttons = 0,
	.lx = 0x80,
	.ly = 0x80,
	.rx = 0x80,
	.ry = 0x80,
	.l2 = 0x80,
	.r2 = 0x80,
	.hat = 0x8,
};

struct bt_hid_state latest;

struct __attribute__((packed)) input_report_17 {
	uint8_t report_id;
	uint8_t pad[2];

	uint8_t lx, ly;
	uint8_t rx, ry;
	uint8_t buttons[3];
	uint8_t l2, r2;

	uint16_t timestamp;
	uint16_t temperature;
	uint16_t gyro[3];
	uint16_t accel[3];
	uint8_t pad2[5];
	uint8_t status[2];
	uint8_t pad3;
};

static void hid_host_handle_interrupt_report(const uint8_t *packet, uint16_t packet_len){
	static struct bt_hid_state last_state = { 0 };

	// Only interested in report_id 0x11
	if (packet_len < sizeof(struct input_report_17) + 1) {
		return;
	}

	if ((packet[0] != 0xa1) || (packet[1] != 0x11)) {
		return;
	}

	//printf_hexdump(packet, packet_len);

	struct input_report_17 *report = (struct input_report_17 *)&packet[1];

	// Note: This assumes that we're protected by async_context's
	// single-threaded-ness
	latest = (struct bt_hid_state){
		// Somewhat arbitrary packing of the buttons into a single 16-bit word
		.buttons = ((report->buttons[0] & 0xf0) << 8) | ((report->buttons[2] & 0x3) << 8) | (report->buttons[1]),

		.lx = report->lx,
		.ly = report->ly,
		.rx = report->rx,
		.ry = report->ry,
		.l2 = report->l2,
		.r2 = report->r2,

		.hat = (report->buttons[0] & 0xf),
	};

	// TODO: Parse out battery, touchpad, sixaxis, timestamp, temperature(?!)
	// Sensors will also need calibration

}

void bt_hid_get_latest(struct bt_hid_state *dst)
{
	async_context_t *context = cyw43_arch_async_context();
	async_context_acquire_lock_blocking(context);
	memcpy(dst, &latest, sizeof(*dst));
	async_context_release_lock(context);
}

static void bt_hid_disconnected(bd_addr_t addr)
{
	hid_host_cid = 0;
	hid_host_descriptor_available = false;

	memcpy(&latest, &default_state, sizeof(latest));
}

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
	UNUSED(channel);
	UNUSED(size);

	uint8_t   event;
	uint8_t   hid_event;
	bd_addr_t event_addr;
	uint8_t   status;
	uint8_t reason;

	if (packet_type != HCI_EVENT_PACKET) {
		return;
	}

	event = hci_event_packet_get_type(packet);
	switch (event) {
	case BTSTACK_EVENT_STATE:
		// On boot, we try a manual connection
		if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
			printf("Starting hid_host_connect (%s)\n", bd_addr_to_str(remote_addr));
			status = hid_host_connect(remote_addr, hid_host_report_mode, &hid_host_cid);
			if (status != ERROR_CODE_SUCCESS){
				printf("hid_host_connect command failed: 0x%02x\n", status);
			}
		}
		break;
	case HCI_EVENT_CONNECTION_COMPLETE:
		status = hci_event_connection_complete_get_status(packet);
		printf("Connection complete: %x\n", status);
		break;
	case HCI_EVENT_DISCONNECTION_COMPLETE:
		status = hci_event_disconnection_complete_get_status(packet);
		reason = hci_event_disconnection_complete_get_reason(packet);
		printf("Disconnection complete: status: %x, reason: %x\n", status, reason);
		break;
	case HCI_EVENT_MAX_SLOTS_CHANGED:
		status = hci_event_max_slots_changed_get_lmp_max_slots(packet);
		printf("Max slots changed: %x\n", status);
		break;
	case HCI_EVENT_PIN_CODE_REQUEST:
		printf("Pin code request. Responding '0000'\n");
		hci_event_pin_code_request_get_bd_addr(packet, event_addr);
		gap_pin_code_response(event_addr, "0000");
		break;
	case HCI_EVENT_USER_CONFIRMATION_REQUEST:
		printf("SSP User Confirmation Request: %d\n", little_endian_read_32(packet, 8));
		break;
	case HCI_EVENT_HID_META:
		hid_event = hci_event_hid_meta_get_subevent_code(packet);
		switch (hid_event) {
		case HID_SUBEVENT_INCOMING_CONNECTION:
			hid_subevent_incoming_connection_get_address(packet, event_addr);
			printf("Accepting connection from %s\n", bd_addr_to_str(event_addr));
			hid_host_accept_connection(hid_subevent_incoming_connection_get_hid_cid(packet), hid_host_report_mode);
			break;
		case HID_SUBEVENT_CONNECTION_OPENED:
			status = hid_subevent_connection_opened_get_status(packet);
			hid_subevent_connection_opened_get_bd_addr(packet, event_addr);
			if (status != ERROR_CODE_SUCCESS) {
				printf("Connection to %s failed: 0x%02x\n", bd_addr_to_str(event_addr), status);
				bt_hid_disconnected(event_addr);
				return;
			}
			hid_host_descriptor_available = false;
			hid_host_cid = hid_subevent_connection_opened_get_hid_cid(packet);
			printf("Connected to %s\n", bd_addr_to_str(event_addr));
			bd_addr_copy(connected_addr, event_addr);
			break;
		case HID_SUBEVENT_DESCRIPTOR_AVAILABLE:
			status = hid_subevent_descriptor_available_get_status(packet);
			if (status == ERROR_CODE_SUCCESS){
				hid_host_descriptor_available = true;

				uint16_t dlen = hid_descriptor_storage_get_descriptor_len(hid_host_cid);
				printf("HID descriptor available. Len: %d\n", dlen);

				// Send FEATURE 0x05, to switch the controller to "full" report mode
				hid_host_send_get_report(hid_host_cid, HID_REPORT_TYPE_FEATURE, 0x05);
			} else {
				printf("Couldn't process HID Descriptor, status: %d\n", status);
			}
			break;
		case HID_SUBEVENT_REPORT:
			if (hid_host_descriptor_available){
				hid_host_handle_interrupt_report(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			} else {
				printf("No hid host descriptor available\n");
				printf_hexdump(hid_subevent_report_get_report(packet), hid_subevent_report_get_report_len(packet));
			}
			break;
		case HID_SUBEVENT_SET_PROTOCOL_RESPONSE:
			status = hid_subevent_set_protocol_response_get_handshake_status(packet);
			if (status != HID_HANDSHAKE_PARAM_TYPE_SUCCESSFUL){
				printf("Protocol handshake error: 0x%02x\n", status);
				break;
			}
			hid_protocol_mode_t proto = hid_subevent_set_protocol_response_get_protocol_mode(packet);
			switch (proto) {
			case HID_PROTOCOL_MODE_BOOT:
				printf("Negotiated protocol: BOOT\n");
				break;
			case HID_PROTOCOL_MODE_REPORT:
				printf("Negotiated protocol: REPORT\n");
				break;
			default:
				printf("Negotiated unknown protocol: 0x%x\n", proto);
				break;
			}
			break;
		case HID_SUBEVENT_CONNECTION_CLOSED:
			printf("HID connection closed: %s\n", bd_addr_to_str(connected_addr));
			bt_hid_disconnected(connected_addr);
			break;
		case HID_SUBEVENT_GET_REPORT_RESPONSE:
			{
				status = hid_subevent_get_report_response_get_handshake_status(packet);
				uint16_t dlen =  hid_subevent_get_report_response_get_report_len(packet);
				printf("GET_REPORT response. status: %d, len: %d\n", status, dlen);
			}
			break;
		default:
			printf("Unknown HID subevent: 0x%x\n", hid_event);
			break;
		}
		break;
	default:
		//printf("Unknown HCI event: 0x%x\n", event);
		break;
	}
}

#define BLINK_MS 250
static btstack_timer_source_t blink_timer;
static void blink_handler(btstack_timer_source_t *ts)
{
	static bool on = 0;

	if (hid_host_cid != 0) {
		on = true;
	} else {
		on = !on;
	}

	cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, !!on);

	btstack_run_loop_set_timer(&blink_timer, BLINK_MS);
	btstack_run_loop_add_timer(&blink_timer);
}

void bt_main(void) {
	if (cyw43_arch_init()) {
		printf("Wi-Fi init failed\n");
		return;
	}

	gap_set_security_level(LEVEL_2);

	blink_timer.process = &blink_handler;
	btstack_run_loop_set_timer(&blink_timer, BLINK_MS);
	btstack_run_loop_add_timer(&blink_timer);

	hid_host_setup();
	sscanf_bd_addr(remote_addr_string, remote_addr);
	bt_hid_disconnected(remote_addr);

	hci_power_control(HCI_POWER_ON);

	btstack_run_loop_execute();
}
