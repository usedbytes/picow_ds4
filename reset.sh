#! /bin/bash

openocd -f interface/picoprobe.cfg -f target/rp2040.cfg -s tcl -c "init; reset; exit"
