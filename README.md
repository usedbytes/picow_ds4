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

The `.uf2` file will be `build/src/picow_ds4.uf2`

# Known Issues

`pico-sdk` implements its own `btstack` makefile (see
`$PICO_SDK_PATH/src/rp2_common/pico_btstack/CMakeLists.txt`), which means a
particular `pico-sdk` version is only technically compatible with a single
`btstack` version. See https://github.com/raspberrypi/pico-sdk/issues/1553 and
#1.

If you run in to build issues which look related to `btstack`, it could be
because the version this project depends on has diverged from your version
of `pico-sdk` again.

This repo is known to build OK against `pico-sdk` version
`6a7db34ff63345a7badec79ebea3aaef1712f374` (tag: 1.5.1).

There's two good ways to fix this, which maybe I'll do one day:

1. Upstream a configurable `MAX_ATTRIBUTE_VALUE_SIZE` to `btstack`, so that the
   `btstack` submodule could be removed here.
2. Change `btstack`+`pico-sdk` to put the make rules inside `btstack` instead,
   so that there's no implicit dependency between the two.
