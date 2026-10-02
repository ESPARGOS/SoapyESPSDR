# SoapyESPSDR

<img src="assets/espargos-logo.png" width="40%" align="right" alt="ESPARGOS logo">


[ESP-SDR](https://github.com/ESPARGOS/esp-sdr) + [SoapyESPSDR](https://github.com/ESPARGOS/SoapyESPSDR) enable continuous I/Q streaming with ESP32-S31 at up to **20 MSa/s** over USB or **40 MSa/s over Ethernet**.

SoapyESPSDR is a [SoapySDR](https://github.com/pothosware/SoapySDR) plugin that lets
you use an ESP32-S31 as a radio receiver in Gqrx, GNU Radio, and other
SoapySDR-compatible applications.

[ESP-WebSDR](https://espargos.net/espsdr/app/) lets you explore signals in your
browser. SoapyESPSDR adds continuous I/Q streaming to desktop applications:
listen to signals in Gqrx, record samples, or build your own demodulator in
GNU Radio. Transmission is not supported.

**Experimental:** The driver and streaming firmware are under development.
Signal quality, including the remaining DC peak, still needs improvement.

## What you need

- Linux and an **ESP32-S31 Function-CoreBoard** running ESP-SDR's
  **`esp32s31-stream`** firmware. The ordinary `esp32s31` browser-viewer firmware
  uses a different protocol. See the [firmware setup guide](https://github.com/ESPARGOS/esp-sdr/blob/main/docs/s31-streaming.md).
- The board's native **high-speed USB** connector or **Gigabit Ethernet**.
  The UART and USB Serial/JTAG connectors are for flashing/debugging.
  Ethernet must be gigabit along the entire path; shared USB bandwidth can
  also limit the sample rate.

## Build and install

On Debian/Ubuntu:

```sh
sudo apt install build-essential cmake pkg-config libsoapysdr-dev soapysdr-tools \
  libcurl4-openssl-dev libjsoncpp-dev libusb-1.0-0-dev
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
sudo cmake --install build
```

To use the build without installing, set both variables in the application’s
shell. They isolate it from any older installed driver; unset them to restore
normal system-wide driver discovery.

```sh
export SOAPY_SDR_ROOT="$PWD/build"
export SOAPY_SDR_PLUGIN_PATH="$PWD/build"
```

For USB access without root, add this to
`/etc/udev/rules.d/70-espsdr.rules`, run `sudo udevadm control --reload-rules`,
and reconnect the board:

```text
SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="4531", MODE="0660", TAG+="uaccess"
```

## Connect

For Ethernet, find the board's DHCP address in your router or UART boot log.
**Replace `192.168.1.100` below with that address**. Open it in a browser for the
board's control/status page.

| Connection | Gqrx device string (choose **Other**) |
| --- | --- |
| USB | `soapy=0,driver=espsdr,usb=1` |
| Ethernet | `soapy=0,driver=espsdr,host=192.168.1.100` |

Use a sample rate of `20000000` to start. Check discovery or connectivity with:

```sh
SoapySDRUtil --find="driver=espsdr"
SoapySDRUtil --probe="driver=espsdr,usb=1"
SoapySDRUtil --probe="driver=espsdr,host=192.168.1.100"
```

Optional device arguments: `usb_serial=<serial>` selects a USB board;
`interface=eth0` selects a network interface; `http_port=80` sets the control
port. Avoid placing host Wi-Fi and Ethernet on the same receiver subnet.
Only one connection can receive at a time. During reception, change settings
through the SDR application so its processing stays in sync. Network control
is unauthenticated; use a trusted local network.

## Capabilities and limitations

- **Tuning:** 2300–2800 MHz in 1 MHz steps; manual gain is a PHY table index,
  not calibrated dB.
- **Sample rates:** 4, 8, 16, 20 MSa/s; Ethernet also supports 40 MSa/s.
- **Samples:** native signed 8-bit I and 8-bit Q. `CS8`, `CS16`, and `CF32`
  are supported; wider formats do not add ADC precision.
- **Analog bandwidth:** 13–54 MHz; zero follows the sample rate. Lower sample
  rates do not add an anti-alias filter, so out-of-band signals can alias.
- **Timing:** boot-relative timestamps and loss counters; no synchronized
  multi-board clock.

**Known issue — DC offset:** A residual bias in the I/Q samples can create a
spurious peak at the center of the spectrum, even with correction enabled.

## Tests

The C++ tests and Python localhost tests need no radio. The latter additionally
require `python3-soapysdr` and `python3-numpy`:

```sh
ctest --test-dir build --output-on-failure
SOAPY_SDR_ROOT="$PWD/build" SOAPY_SDR_PLUGIN_PATH="$PWD/build" python3 tests/loopback_test.py
```

To check reception with a connected board, the loss monitor reports missing
samples, overflows, and timeouts, and fails on empty or lossy reception:

```sh
./build/espsdr_loss_monitor --device driver=espsdr,usb=1 --rate 20000000 --seconds 60
```

Use `driver=espsdr,host=<board-IP>` to check Ethernet.

## License

GPL-3.0-or-later; see [LICENSE](LICENSE).
