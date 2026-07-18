# SoapyESPSDR

<a href="https://espargos.net/"><img src="assets/espargos-logo.png" alt="ESPARGOS" width="40%" align="right"></a>

SoapyESPSDR is a receive-only SoapySDR driver for ESP-SDR by
[ESPARGOS](https://espargos.net/).

ESP-SDR is firmware for the ESP32-S31 Function-CoreBoard that provides a
network-connected I/Q receiver over the board's Gigabit Ethernet interface.
SoapyESPSDR makes that receiver available to applications that support
SoapySDR, including Gqrx, GNU Radio, and SDR++.

The driver uses HTTP to configure the radio and UDP to receive I/Q samples. It
checks the firmware sequence numbers for missing samples and reports loss to
the application as SoapySDR overflow events.

![ESP-SDR receiving the 2.4 GHz band in Gqrx](assets/gqrx-esp-sdr.png)

*ESP-SDR receiving the 2.4 GHz band in Gqrx at 20 MSa/s.*

## Requirements

You need:

- An ESP32-S31 Function-CoreBoard running ESP-SDR firmware
- The board and computer on the same IPv4 network
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
MAC address, shown by `--find`). Only one sample stream runs at a time;
starting a stream over USB replaces an Ethernet stream and vice versa.

The `CS8` stream format (USB only) selects a compressed int8 wire format
carrying the top 8 of each 10 sample bits, doubling the sustainable sample
rate. Continuous, lossless USB streaming is supported up to 8.89 MSa/s
(decimation 9) with `CS16`/`CF32` and up to 16 MSa/s (decimation 5) with
`CS8`; higher rates (including 20 MSa/s `CS8`, ~88 % delivered) stream
best-effort with accurate loss counters, and the duty-cycle settings work
as on Ethernet. Raw USB device
access without root requires a udev rule for VID `303a`, for example:

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

Select 8 or 16 MSa/s for continuous, lossless streaming. At higher sample
rates, use the **Duty cycle** device settings to reduce the network data
rate. If the module is not installed system-wide, launch Gqrx from the terminal
in which `SOAPY_SDR_PLUGIN_PATH` was exported.

## Supported controls and formats

- `CS16` native I/Q samples and `CF32` converted samples
- Center frequencies from 2300 to 2800 MHz, with 1 kHz tuning resolution
- Sample rates from 8 to 80 MSa/s, corresponding to integer software
  decimation factors from 10 to 1
- Automatic or manual receive gain
- Manual receive gain from 0 to 76 dB in 1 dB steps
- Open/widest or 13–54 MHz analog receive-filter bandwidth

Manual gain selects the ESP32-S31 PHY's calibrated receive-gain table. The
firmware publishes the available range, unit, and step through its status API,
and SoapyESPSDR reports those values to applications.

ESP-SDR supports continuous, lossless streaming at 8 and 16 MSa/s. Higher rates
are available for captures with a reduced duty cycle; missing samples caused by
network or receiver overload are reported as overflows.

The `cycle_total` and `cycle_stream` settings control capture duty cycle in
units of 1,024-sample chunks. For example, `cycle_total=10,cycle_stream=3`
streams three contiguous chunks followed by seven unstreamed chunks, for a 30%
duty cycle. Both default to 1 for continuous reception. Deliberately unstreamed
chunks are not reported as packet loss.

Set these values in the device string so the duty cycle is configured as the
device is opened:

```text
soapy=0,driver=espsdr,host=esp-sdr.local,cycle_total=10,cycle_stream=3
```

The following duty-cycle settings support continuous reception of every
selected chunk. The average streamed rate accounts for the deliberately
unstreamed part of each cycle.

| Sample rate | Software decimation | `cycle_total` | `cycle_stream` | Duty cycle | Average streamed rate |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 8 MSa/s | 10 | 1 | 1 | 100% | 8 MSa/s |
| 8.889 MSa/s | 9 | 1 | 1 | 100% | 8.889 MSa/s |
| 10 MSa/s | 8 | 1 | 1 | 100% | 10 MSa/s |
| 11.429 MSa/s | 7 | 1 | 1 | 100% | 11.429 MSa/s |
| 13.333 MSa/s | 6 | 1 | 1 | 100% | 13.333 MSa/s |
| 16 MSa/s | 5 | 1 | 1 | 100% | 16 MSa/s |
| 20 MSa/s | 4 | 2 | 1 | 50% | 10 MSa/s |
| 26.667 MSa/s | 3 | 3 | 1 | 33.333% | 8.889 MSa/s |
| 40 MSa/s | 2 | 4 | 1 | 25% | 10 MSa/s |
| 80 MSa/s | 1 | 8 | 1 | 12.5% | 10 MSa/s |

These cycle geometries were validated end-to-end with zero firmware drops,
missing selected chunks, host queue drops, or SoapySDR overflow events. Cycle
geometry matters in addition to average data rate: use the listed values or a
lower duty cycle, and monitor the loss sensors when using a different geometry.

The analog bandwidth control uses the firmware's Custom/20 MHz digital channel
path. A bandwidth of zero selects the open/widest analog response; nonzero
values are rounded to whole MHz and must be between 13 and 54 MHz. ESP-SDR does
not support the 40 MHz digital channel mode.

## Monitoring sample loss

The build also produces `espsdr_loss_monitor`, a command-line example that
continuously receives samples and reports loss once per configured interval:

```sh
export SOAPY_SDR_PLUGIN_PATH="$PWD/build"
./build/espsdr_loss_monitor \
    --host esp-sdr.local \
    --rate 8000000 \
    --cycle-total 1 \
    --cycle-stream 1 \
    --seconds 30 \
    --interval 1
```

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
| `cycle_total` | `1` | Total chunks per duty-cycle period. |
| `cycle_stream` | `1` | Contiguous streamed chunks at the start of each period. |

## Limitations

- Receive only.
- One network client at a time.
- No hardware timestamps or timed streaming.
- Absolute gain and sensitivity can vary between boards and with frequency.
- Sample rates above 16 MSa/s generally require a reduced duty cycle when their
  full data rate exceeds the host or network path's capacity. Applications
  should monitor the overflow and loss sensors for the chosen cycle geometry.
