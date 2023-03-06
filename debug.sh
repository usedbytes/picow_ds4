#! /bin/bash

DIR="${BASH_SOURCE%/*}"

nohup -- ${DIR}/openocd.sh ${1} &

OPENOCD_PID=$!

# Wait for openocd to start
sleep 1

${DIR}/gdb.sh ${1}

# Terminate openocd
pkill -TERM -P ${OPENOCD_PID}
