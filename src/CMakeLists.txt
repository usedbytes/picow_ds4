
add_executable(main
	main.c
)

pico_enable_stdio_uart(main 1)
pico_enable_stdio_semihosting(main 0)

target_link_libraries(main
	pico_stdlib
)

pico_add_extra_outputs(main)
