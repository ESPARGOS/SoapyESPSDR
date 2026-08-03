# SoapyESPSDR

<a href="https://espargos.net/"><img src="assets/espargos-logo.png" alt="ESPARGOS" width="40%" align="right"></a>

SoapyESPSDR is a receive-only SoapySDR driver for ESP-SDR by
[ESPARGOS](https://espargos.net/).

ESP-SDR is firmware for the ESP32-S31 Function-CoreBoard that provides an I/Q
receiver over either Gigabit Ethernet or the native high-speed USB interface.
SoapyESPSDR makes that receiver available to applications that support
SoapySDR, including Gqrx, GNU Radio, and SDR++.

On Ethernet the driver uses HTTP control and UDP I/Q; on USB it uses the
firmware's vendor control and bulk-IQ endpoints. It checks firmware and
transport sequence numbers and reports any missing samples as SoapySDR
overflow events.

![ESP-SDR receiving the 2.4 GHz band in Gqrx](assets/gqrx-esp-sdr.png)

*ESP-SDR receiving the 2.4 GHz band in Gqrx.*

## Requirements

You need:

- An ESP32-S31 Function-CoreBoard running ESP-SDR firmware
- For Ethernet, the board and computer on the same IPv4 network
- SoapySDR 0.8 development files
- libcurl, jsoncpp, and zlib development files
- CMake and a C++17 compiler

The board normally obtains its address through DHCP and is available as
`esp-sdr.local`. If mDNS is unavailable on your computer, use the board's IPv4
address instead.

Only one computer can receive the I/Q stream at a time. ESP-SDR does not use
authentication and should only be used on a trusted network.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

To use the driver without installing it, point SoapySDR at the build directory:

```sh
export SOAPY_SDR_PLUGIN_PATH="$PWD/build"
```

Confirm that SoapySDR can find the receiver:

```sh
SoapySDRUtil --find="driver=espsdr,host=esp-sdr.local"
SoapySDRUtil --probe="driver=espsdr,host=esp-sdr.local"
```

Replace `esp-sdr.local` with an IPv4 address when necessary, for example:

```sh
SoapySDRUtil --probe="driver=espsdr,host=192.168.0.139"
```

## USB transport

When the board's native high-speed USB port is connected, the same control
and streaming interface is available without a network. USB devices are
discovered automatically:

```sh
SoapySDRUtil --find="driver=espsdr"
SoapySDRUtil --probe="driver=espsdr,usb=1"
```

Select a specific board with `usb_serial=<serial>` (the serial is the base
MAC address, shown by `--find`). The firmware is built for either USB or
Ethernet transport, and only one sample stream runs at a time.

The `CS8` application format (USB only) returns the top 8 of each 10 sample
bits. USB normally carries full-precision IQC1 frames and converts them to the
format requested by the application. The production receiver has a fixed
2 MSa/s complex rate. Raw USB device access without root requires a udev rule
for VID `303a`, for example:

```text
SUBSYSTEM=="usb", ATTRS{idVendor}=="303a", MODE="0664", GROUP="plugdev", TAG+="uaccess"
```

Install the module into the system SoapySDR module directory with:

```sh
sudo cmake --install build
```

After installation, `SOAPY_SDR_PLUGIN_PATH` is no longer needed.

## Using Gqrx

Open the Gqrx device configuration window, select **Other**, and enter:

```text
soapy=0,driver=espsdr,host=esp-sdr.local
```

With an explicit address, the string looks like:

```text
soapy=0,driver=espsdr,host=192.168.0.139
```

For native USB, select this specific board by serial:

```text
soapy=0,driver=espsdr,usb_serial=30eda0f3f840
```

Apply a measured board-reference correction in ppm in the same string. For the
bench board characterized against a PlutoSDR, use:

```text
soapy=0,driver=espsdr,host=192.168.0.139,frequency_correction_ppm=8.272
```

The corresponding USB string is:

```text
soapy=0,driver=espsdr,usb_serial=30eda0f3f840,frequency_correction_ppm=8.272
```

Select 2 MSa/s in Gqrx. If the module is not installed system-wide, launch Gqrx
with:

```sh
SOAPY_SDR_PLUGIN_PATH=/home/florian/prgm/esp32/SoapyESPSDR/build gqrx
```

## Supported controls and formats

- `CS16` native I/Q samples and `CF32` converted samples
- Center frequencies from 2300 to 2800 MHz, with 1 kHz tuning resolution
- Signed frontend frequency correction from -100 to +100 ppm
- Fixed 2 MSa/s complex sample rate
- Automatic or manual receive gain
- Manual receive gain from 0 to 76 dB in 1 dB steps
- Open/widest or 13–54 MHz analog receive-filter bandwidth

Manual gain selects the ESP32-S31 PHY's calibrated receive-gain table. The
firmware publishes the available range, unit, and step through its status API,
and SoapyESPSDR reports those values to applications.

The transport preserves frame ordering and reports missing source chunks,
firmware drops, acquisition overruns, and host-queue loss as overflow events.
The production PARLIO receiver is full-duty and does not hand the RF source
through the TCM snapshot aperture.

The analog bandwidth control uses the firmware's Custom/20 MHz digital channel
path. A bandwidth of zero selects the open/widest response; nonzero values are
rounded to whole MHz and must be between 13 and 54 MHz. ESP-SDR does not support
the 40 MHz digital channel mode. Bluetooth-width routes remain expert-only
because the filter response verified with the internal TX loop did not carry
antenna-side RF when used as a continuous production source.

For characterization, the device settings `rx_filter_override`,
`rx_filter_mode`, `rx_filter_dcap`, and `adc_source_sel` expose the firmware's
raw expert filter and dump-mux controls. Leave `rx_filter_override=0` for the
calibrated bandwidth API.

## Monitoring sample loss

The build also produces `espsdr_loss_monitor`, a command-line example that
continuously receives samples and reports loss once per configured interval:

```sh
export SOAPY_SDR_PLUGIN_PATH="$PWD/build"
./build/espsdr_loss_monitor \
    --host esp-sdr.local \
    --rate 2000000 \
    --cycle-total 1 \
    --cycle-stream 1 \
    --seconds 30 \
    --interval 1
```

For the connected USB board, replace `--host esp-sdr.local` with
`--device usb_serial=30eda0f3f840`.

The output includes:

- received and missing I/Q frames per second;
- interval and cumulative loss percentages;
- SoapySDR overflow events;
- firmware capture drops;
- invalid UDP datagrams;
- host receive-queue drops;
- received UDP datagrams per second.

Each I/Q frame contains 1,024 complex samples. Loss percentages are calculated
from the firmware source-frame sequence. Expected discontinuities caused by
changing radio settings are excluded from the loss counters.

## Device arguments

The SoapySDR device string accepts these arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `driver` | `espsdr` | Selects this SoapySDR driver. |
| `host` | `esp-sdr.local` | ESP-SDR hostname or IPv4 address. |
| `http_port` | `80` | Firmware HTTP control port. |
| `udp_port` | `0` | Local UDP port; zero selects an available ephemeral port. |
| `rx_buffer_bytes` | `33554432` | Requested operating-system UDP receive-buffer size. |
| `usb` | unset | Select the first attached ESP-SDR USB device when set to `1`. |
| `usb_serial` | unset | Select one attached ESP-SDR by its lowercase MAC serial. |
| `cycle_total` | `1` | Legacy/sparse-backend total chunks per duty-cycle period. |
| `cycle_stream` | `1` | Legacy/sparse-backend streamed chunks per period. |
| `frequency_correction_ppm` | `0` | Board-specific signed oscillator correction; positive means the ESP LO runs high. |

## Limitations

- Receive only.
- One streaming client at a time.
- No hardware timestamps or timed streaming.
- Absolute gain and sensitivity can vary between boards and with frequency.
- The production receiver currently exposes only 2 MSa/s complex IQ.
