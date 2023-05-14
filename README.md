# DualShock 4 on Pico-W

This is a pretty crude project which brings up a Sony DualShock 4 (PS4)
controller on Pico-W.

It uses a hard-coded controller MAC address (see `src/bt_hid.c`), that needs
to match your controller.

On first boot, make sure the controller is in "pairing" mode (hold "Share" and
the PS button, so the light is doing quick double-flashes).

Once paired, it's enough to just press the PS/Home button to reconnect.

## Building

The DS4 has a large HID descriptor, which doesn't fit in upstream BTStack's
SDP buffer, so this project submodules a fork of BTStack wich just makes that
larger.

So, clone this repository, then:

```
cd picow_ds4
git submodule update --init --recursive
mkdir build
cd build
cmake -DPICO_BOARD=pico_w -DPICO_SDK_PATH=/your/path/to/pico-sdk ../  
make
```
