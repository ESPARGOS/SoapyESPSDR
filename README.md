# SoapyESPSDR

<a href="https://espargos.net/"><img src="assets/espargos-logo.png" alt="ESPARGOS" width="40%" align="right"></a>

SoapyESPSDR is a SoapySDR transceiver driver for ESP-SDR by
[ESPARGOS](https://espargos.net/).

ESP-SDR is firmware for the ESP32-S31 Function-CoreBoard that provides native
I/Q receive and arbitrary finite-burst I/Q transmit over Gigabit Ethernet and
the native high-speed USB interface. SoapyESPSDR makes it available to
applications that support SoapySDR, including Gqrx, GNU Radio, and SDR++.

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
- libcurl, jsoncpp, zlib, and libusb-1.0 development files
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

On a multi-homed host, `interface=<name>` binds HTTP control and both UDP
directions without changing system routes:

```sh
SoapySDRUtil --probe="driver=espsdr,host=192.168.0.139,interface=enp0s13f0u1u1"
```

An explicit `host` always selects Ethernet even when USB is attached.

## USB transport

When the board's native high-speed USB port is connected, the same control
and streaming interface is available without a network. USB devices are
discovered automatically:

```sh
SoapySDRUtil --find="driver=espsdr"
SoapySDRUtil --probe="driver=espsdr,usb=1"
```

Select a specific board with `usb_serial=<serial>` (the serial is the base
MAC address, shown by `--find`). One combined firmware image keeps Ethernet
and USB control available together; the most recent RX stream start owns the
half-duplex sample engine.

USB RX uses compact native `IQC8` at the selected RX rate. USB TX arms an exact-size
device allocation, transfers packed IQ10 words in 60 KiB bulk chunks, and
commits ownership to the replay engine without a second device-side waveform
copy. The present implementation is validated lossless at 8 MSa/s RX; a
16 MSa/s request reaches about 9.5 MSa/s and reports overflows/gaps, so use
Gigabit Ethernet for lossless 16 MSa/s. Raw USB device access without root
requires a udev rule for VID `303a`,
for example:

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

Select 16 MSa/s for the widest alias-free view and best characterized mode.
Lower continuous rates are available when host or transport capacity is more
important; see the direct-subsampling note below. If the module is not
installed system-wide, launch Gqrx with:

```sh
SOAPY_SDR_PLUGIN_PATH=/home/florian/prgm/esp32/SoapyESPSDR/build gqrx
```

## Supported controls and formats

- `CS8`, `CS16`, and `CF32` RX and TX sample formats
- Center frequencies from 2300 to 2800 MHz, with 1 kHz tuning resolution
- Signed frontend frequency correction from -100 to +100 ppm
- Continuous complex RX at 16 MSa/s divided by an integer from 1 through 10:
  16, 8, 5.333, 4, 3.2, 2.667, 2.286, 2, 1.778, and 1.6 MSa/s
- TX replay rates of 80, 40, 26.667, 20, 8, 6.667, 4, and 3.333 MSa/s
- Automatic or manual receive gain
- Manual receive gain from 0 to 69 dB in 1 dB steps on the tested board
- Calibrated relative TX gain from 0 to 19.57 dB
- Open/widest or 13–54 MHz analog receive-filter bandwidth
- Fixed 20 MHz TX channel bandwidth

Manual gain selects the ESP32-S31 PHY's calibrated receive-gain table. The
firmware publishes the available range, unit, and step through its status API,
and SoapyESPSDR reports those values to applications.

The transport preserves frame ordering and reports missing source chunks,
firmware drops, acquisition overruns, and host-queue loss as overflow events.
The production PARLIO receiver is full-duty and does not hand the RF source
through the TCM snapshot aperture.

The normal TX gain API is a measured relative-power control, not the raw PHY
field. It accepts 0--19.57 dB and selects the nearest point from a conservative
monotonic table characterized over the air at 2.38 GHz. The underlying six-bit
RFTX2 value rises within eight-code banks but falls at their boundaries, so it
is not a linear gain number. Expert experiments may use the `tx_gain_code`
device setting directly; an uncalibrated raw code makes `getGain(TX)` return
NaN until a calibrated gain is selected. Codes beyond the characterized table
can draw substantially more current and are not implied safe by the driver.

Frequency correction changes RF, not just configuration readback. At a
2.38 GHz requested center (2.379 GHz internal modem LO), HackRF measurements
of contiguous 4 MSa/s bursts at -20, 0, and +20 ppm measured a TX slope of
-2400.96 Hz/ppm versus -2379 Hz/ppm ideal (0.92% difference). Thus positive
correction lowers TX RF, consistent with compensating a board reference that
runs high; RX baseband moves in the complementary direction. The three final
captures had 64.7--65.9 dB tone SNR and +1.043--1.820 us scheduled-start error.

The rates below 16 MSa/s divide the hardware sampling clock. They are direct
subsampling modes, not digitally filtered decimation: signals and noise above
the selected Nyquist band alias into the output. The narrowest calibrated
analog RX bandwidth is 13 MHz, so it cannot serve as an anti-alias filter for
the lower rates. Use 16 MSa/s when spectral fidelity across the selected band
matters, or provide external/front-end band limiting when using a lower rate.

The analog bandwidth control uses the firmware's Custom/20 MHz digital channel
path. A bandwidth of zero selects the open/widest response; nonzero values are
rounded to whole MHz and must be between 13 and 54 MHz. ESP-SDR does not support
the 40 MHz digital channel mode. Bluetooth-width routes remain expert-only
because the filter response verified with the internal TX loop did not carry
antenna-side RF when used as a continuous production source.

RX frames carry a flagged low 32-bit microsecond hardware timestamp while the
status API supplies the full boot-relative clock. The driver seeds and unwraps
the roughly 71-minute wire-counter rollover, exposes `hasHardwareTime()` and
`getHardwareTime()`, and sets `SOAPY_SDR_HAS_TIME` on every successful RX read.
Partial reads add the exact sample offset, and aggregation stops at retune or
sparse-capture gaps. The hardware clock is read-only; scheduled/timed TX
uses this same clock domain.

The TX bandwidth is deliberately advertised as fixed at 20 MHz. The S31 PHY's
nominal HT40 register path was tested over the air with a HackRF at identical
gain, receiver tuning, waveform, and replay rate. It changed received power by
only -0.05 dB at a +12 MHz offset and +0.002 dB at +15 MHz, so it did not
measurably widen the direct-IQ transmit response and is not exposed as a
capability.

For characterization, the device settings `rx_filter_override`,
`rx_filter_mode`, `rx_filter_dcap`, and `adc_source_sel` expose the firmware's
raw expert filter and dump-mux controls. Normal applications should leave
these settings alone; the driver selects the required production path when a
stream is activated.

Software AGC state is available through ordinary Soapy sensors:

- `rx_agc_active` is a typed boolean;
- `rx_agc_current_gain` reports the active calibrated RX gain in dB;
- `rx_agc_robust_peak` reports the most recent robust component peak;
- `rx_agc_gain_changes` counts gain-table changes in the current AGC session.

These values are encoded in otherwise reserved IQ-header bits and cached by
the host decoder. Reading them during RX therefore adds no HTTP traffic and
does not compete with a full-rate Ethernet stream. The gain and active state
are updated per decoded frame; the peak is intentionally instantaneous, so a
slow polling application may not observe a short overload that nevertheless
caused the gain-change counter to advance.

## Monitoring sample loss

The build also produces `espsdr_loss_monitor`, a command-line example that
continuously receives samples and reports loss once per configured interval:

```sh
export SOAPY_SDR_PLUGIN_PATH="$PWD/build"
./build/espsdr_loss_monitor \
    --host esp-sdr.local \
    --rate 16000000 \
    --cycle-total 1 \
    --cycle-stream 1 \
    --seconds 30 \
    --interval 1
```

For a connected USB board, replace `--host esp-sdr.local` with
`--device usb_serial=30eda0f3f840`.

The output includes:

- received and missing I/Q frames per second;
- interval and cumulative loss percentages;
- SoapySDR overflow events;
- firmware capture drops;
- invalid UDP datagrams;
- duplicate and reordered UDP datagrams;
- observed UDP sequence gaps and late packets that recover those gaps;
- host receive-queue drops;
- received UDP datagrams per second.

Loss percentages are calculated from both transport and firmware source-frame
sequences. Expected discontinuities caused by changing radio settings are
excluded from the loss counters.

## Device arguments

The SoapySDR device string accepts these arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `driver` | `espsdr` | Selects this SoapySDR driver. |
| `host` | `esp-sdr.local` | ESP-SDR hostname or IPv4 address. |
| `interface` | unset | Bind Ethernet HTTP, RX UDP, and TX UDP to this host interface; useful with overlapping routes. |
| `http_port` | `80` | Firmware HTTP control port. |
| `udp_port` | `0` | Local UDP port; zero selects an available ephemeral port. |
| `rx_buffer_bytes` | `33554432` | Requested operating-system UDP receive-buffer size. |
| `usb` | unset | Select the first attached ESP-SDR USB device when set to `1`. |
| `usb_serial` | unset | Select one attached ESP-SDR by its lowercase MAC serial. |
| `cycle_total` | device state | Total chunks per duty-cycle period; an explicit argument overrides the current firmware setting. |
| `cycle_stream` | device state | Streamed chunks per period; an explicit argument overrides the current firmware setting. |
| `frequency_correction_ppm` | `0` | Board-specific signed oscillator correction; positive means the ESP LO runs high. |

## TX streaming model and limitations

ESP32-S31 replay is currently a synchronous finite-burst TX path, not an
unbounded FIFO. The Soapy layer aggregates consecutive `writeStream()` calls
without a boundary flag, so applications may provide ordinary small buffers
without turning every call into a separate RF/configuration burst. It flushes
on `SOAPY_SDR_END_BURST`, `SOAPY_SDR_ONE_PACKET`, the 1,048,512-sample batch
limit, or `deactivateStream()`. Thus one logical burst may contain 1 to
1,048,512 complex samples. Bursts up to 16,383 samples are contiguous and have
measured near-100% RF duty; larger bursts use up to 64 hardware replay segments
and contain refill gaps. `SOAPY_SDR_END_BURST` also produces an `END_BURST`
event through `readStreamStatus()`. Unflagged buffered samples are accepted
immediately but do not reach RF until a flush boundary, which is normal FIFO
behavior but adds up to one batch of latency. RX and TX handles may be created
together, but simultaneous activation is rejected because the shared RF path
is half-duplex.

For a timed burst, put `SOAPY_SDR_HAS_TIME` and the absolute hardware time on
the first nonempty `writeStream()` fragment and place `SOAPY_SDR_END_BURST` on
the last. The host uploads and the firmware prepares the RF path in advance,
then waits only at the final replay-enable gate. An expired deadline returns
`SOAPY_SDR_TIME_ERROR` without buffering samples. This includes a deadline
that was valid on entry but expires during USB/Ethernet waveform staging;
firmware suppresses it before RF preparation and reports
`tx_replay_deadline_missed=true`, zero replay segments, and zero actual start
time. A transmitted burst's `readStreamStatus()` event carries
`END_BURST | HAS_TIME` and the firmware's observed start time. The hardware
clock is intentionally read-only.

Bench measurements at 2.38 GHz provide useful scale for this contract:

- A 16,383-sample burst measured 99.83% RF duty and about 158--163 ms
  synchronous Ethernet `writeStream()` latency.
- Thirty-four refined-scheduler bursts started 0.915--1.998 us after their
  requested deadlines. This included fragmented writes, seven maximum
  64-segment batches, and 4/80 MSa/s endpoint rates. Two HackRF captures of
  nominal 500 ms-ahead bursts measured 28.5 dB temporal SNR and 72.9--75.7 dB
  image rejection; firmware reported +1.000 and +1.671 us start error.
- A later 300 ms-ahead HackRF regression started +1.225 us late with 28.5 dB
  temporal SNR and 68.5 dB image rejection. Its companion 1 ms-ahead expiry
  probe returned `TIME_ERROR` and saw no RF block 15 dB above the floor. A
  subsequent just-in-time RF-rearm run started +1.177 us late with 28.78 dB
  temporal SNR and 75.09 dB image rejection; the corresponding expiry probe
  again emitted no detectable late burst.
- Persistent AXI-GDMA aperture refill plus one-time source-cache publication
  raised ten consecutive maximum 1,048,512-sample calls to 69.90--70.09% RF
  duty (about 13.98--14.02 MSa/s effective at a nominal 20 MSa/s), with
  individual runs reaching 71.23%, versus 62.17% / 12.43 MSa/s for CPU
  refill. Pluto recaptured the exact maximum at 37.36 dB burst-to-noise and
  31.33 dB image rejection. Steady-state host latency in the later ten-run
  series was 0.322--0.327 s with no UDP error or reset.
- The capture firmware now retains PARLIO across same-geometry RX/TX
  ownership changes. Fifty consecutive maximum-size TX-to-RX cycles completed
  with no reset or transport error and a stable 13,312-byte largest internal
  heap block; 4/16/8/16 MSa/s geometry changes were independently lossless.
- Pluto two-tone captures measured the maximum burst at 20,000,114.8 Sa/s
  (+5.7 ppm) and separated that clock error from the roughly +18 kHz RF CFO.
- All eight advertised TX rates were observed over the air. Pluto measured
  37.1--38.5 dB burst-to-noise and 19.6--33.7 dB image rejection; an
  independent HackRF capture measured 25.0 dB image rejection.

Other current limitations:

- One active sample stream/client at a time.
- No absolute calibrated dBm TX power. The relative TX gain table is an OTA
  calibration of this bench board at 2.38 GHz; output and safe limits can vary
  with board supply and frequency.
- The 2.3--2.8 GHz PLL tuning range is not a flat RF-performance claim. A
  fixed-gain HackRF bench-link sweep fell roughly 11 dB by 2.70 GHz and 30 dB
  by 2.80 GHz relative to 2.30 GHz and showed a narrower residual notch near
  2.46 GHz. The measurement includes both antennas and the HackRF front end,
  so it is useful operating guidance rather than absolute output calibration.
- Absolute RX gain and sensitivity can vary between boards and frequency.
- Sustained lossless Ethernet RX is validated at 16 MSa/s on a healthy host link. All
  ten RX clock divisors streamed without loss in bounded tests; 8 MSa/s also
  delivered 479,150,080 CS16 samples in 60.0 seconds with no transport,
  firmware, host-queue, or Soapy overflow errors. CS8, CS16, and CF32 each
  passed separate 8 MSa/s live tests. Lower modes retain the direct-subsampling
  aliasing limitation described above. A 20 MSa/s experiment
  exceeded the combined S31 PARLIO/PSRAM/GMAC path even though the raw Gigabit
  link budget was sufficient.
- Native USB enumerates at 480 Mbit/s and is validated for Soapy control,
  lossless 8 MSa/s RX, finite 20 MSa/s TX bursts, and repeated RX/TX switching.
  Its current endpoint path reaches about 9.5 MSa/s under a 16 MSa/s request,
  with honest overflow/gap reporting; use Ethernet for lossless 16 MSa/s.
- On this bench, `enp0s13f0u1u1` is the host's normal internet/LAN uplink
  through an RTL8153 in a USB-C dock; ESP Ethernet traffic reaches the host
  through that LAN. `eth0` is PlutoSDR's emulated USB Ethernet, not the ESP
  link. Concurrent HackRF USB traffic can coincide with severe ESP UDP loss
  while the RTL8153 remains negotiated at 1 Gbit/s and reports no PHY errors.
  Do not disconnect or reset `enp0s13f0u1u1` automatically: that interrupts
  the user's normal network. Treat concurrent HackRF/ESP loss as a host-bench
  limitation and use the sequence sensors to reject affected measurements.

Independent RF smoke tests are available in the firmware tree as
`testing/hackrf_soapy_rx_probe.py`, `testing/hackrf_soapy_tx_probe.py`, and
`testing/hackrf_soapy_tx_gain_sweep.py`,
`testing/hackrf_soapy_tx_frequency_sweep.py`,
`testing/hackrf_soapy_tx_correction_sweep.py`,
`testing/hackrf_soapy_rx_correction_sweep.py`,
`testing/hackrf_timed_tx_expiry_probe.py`, and
`testing/pluto_soapy_tx_probe.py`. The Pluto TX probe accepts
`--second-tone-hz` to estimate sample-clock error and carrier offset
independently.
