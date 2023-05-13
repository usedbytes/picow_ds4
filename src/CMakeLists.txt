set(BTSTACK_ROOT ${PICO_SDK_PATH}/lib/btstack)
set(BTSTACK_EXAMPLE_PATH ${BTSTACK_ROOT}/example)
set(BTSTACK_3RD_PARTY_PATH ${BTSTACK_ROOT}/3rd-party)
set(BT_EXAMPLE_COMMON_DIR "${CMAKE_CURRENT_LIST_DIR}")

add_executable(bt_hid
	main.c
)

pico_enable_stdio_uart(bt_hid 1)
pico_enable_stdio_semihosting(bt_hid 0)

target_include_directories(bt_hid PRIVATE
	${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(bt_hid
	pico_stdlib
	pico_cyw43_arch_none
        pico_btstack_classic
        pico_btstack_ble
        pico_btstack_cyw43
)

pico_add_extra_outputs(bt_hid)
