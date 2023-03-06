#! /bin/bash

DIR="${BASH_SOURCE%/*}"

arm-none-eabi-gdb -x ${DIR}/cmds.gdb ${1}
