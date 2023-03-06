define rl
	monitor reset init
	load $arg0
	echo Running...\n
	continue
	end

target extended-remote localhost:3333
monitor arm semihosting enable
monitor log_output /dev/null
