# SoapyESPSDR

SoapyESPSDR is a receive-only SoapySDR driver for ESP-SDR.

ESP-SDR is firmware for the ESP32-S31 Function-CoreBoard that provides a
network-connected I/Q receiver over the board's Gigabit Ethernet interface.
SoapyESPSDR makes that receiver available to applications that support
SoapySDR, including Gqrx, GNU Radio, and SDR++.

The driver uses HTTP to configure the radio and UDP to receive I/Q samples. It
checks the firmware sequence numbers for missing samples and reports loss to
the application as SoapySDR overflow events.

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

Select 8 or 16 MSa/s for continuous, lossless streaming. If the module is not
installed system-wide, launch Gqrx from the terminal in which
`SOAPY_SDR_PLUGIN_PATH` was exported.

## Supported controls and formats

- `CS16` native I/Q samples and `CF32` converted samples
- Center frequencies from 2300 to 2800 MHz, with 1 kHz tuning resolution
- Sample rates of 8, 16, and 20 MSa/s
- Automatic or manual receive gain
- Manual RX Gain hardware codes from 0 to 127
- Open/widest or 13–54 MHz analog receive-filter bandwidth

The RX Gain value is a hardware gain code, not a calibrated value in dB.

ESP-SDR supports continuous, lossless streaming at 8 and 16 MSa/s. At 20 MSa/s,
streaming is best-effort and missing samples are reported as overflows.

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

## Limitations

- Receive only.
- One network client at a time.
- No hardware timestamps or timed streaming.
- RX Gain codes are not calibrated in dB.
- 20 MSa/s is best-effort rather than guaranteed lossless operation.
