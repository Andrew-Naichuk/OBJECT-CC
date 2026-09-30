# OBJECT

**A cycling computer for people who enjoy building things as much as riding them.**

OBJECT puts your speed, distance and ride stats on a 3.2″ display, records a GPS track to a microSD card, and lets you download it to your phone over Bluetooth. It is built around a **Seeed XIAO nRF52840 Sense**, a wheel magnet and reed switch, and a handful of readily available modules.

The aim is simple: readable numbers on the handlebars, one button, and a record of where you went. This is a working DIY project still being refined; see [current limitations](#current-limitations) before relying on it as your only ride recorder.

## Out on the bike

- **Speed and distance from the wheel.** A reed switch counts one pulse per revolution. Wheel speed works independently of GPS reception.
- **The essentials at a glance.** Current speed, distance, elapsed time, estimated moving time, maximum speed and average speed.
- **A track to take home.** The GPS supplies position, altitude and UTC time for GPX recording on the microSD card.
- **One button.** A short press changes the backlight; double short-press while stopped opens phone transfer; a four-second hold finishes the ride.
- **Phone downloads.** Double-press to arm transfer, then connect to `OBJECT-001` through the [ride-transfer page](https://object.nav-tech.workers.dev) and download a GPX file.
- **Trip recovery.** Alternating checkpoints restore counters after a restart when a valid record is available. Abrupt power loss during a write can still lose the newest points, but the previous checkpoint remains.

The trip starts with the first wheel pulse. GPS recording begins when the firmware has the required fix, position and time data. Wheel stats are also written to a `.SUM` sidecar when you finish a ride.

## Hardware


| Part                                    | Job                                                                      |
| --------------------------------------- | ------------------------------------------------------------------------ |
| Seeed XIAO nRF52840 Sense               | Microcontroller, Bluetooth and USB                                       |
| 3.2″ ILI9341 IPS SPI display, 240 × 320 | Ride display and onboard microSD slot; LCDWiki MSP3222/MSP3223 interface |
| microSD card                            | Store GPS tracks and trip checkpoints                                    |
| Reed switch and wheel magnet            | One pulse per wheel revolution                                           |
| HGLRC M100 Pro GPS                      | Position, altitude and UTC time                                          |
| Momentary pushbutton                    | Backlight control and new-ride action                                    |
| Adafruit bq25185 charger, **PID 6091**  | Battery charging and power-path management                               |
| 2000 mAh, single-cell LiPo              | Portable power                                                           |
| Schottky diode                          | Prevents XIAO USB power feeding back into the charger LOAD rail          |
| 7 kΩ resistor                           | Holds the display reset input high                                       |


The Schottky diode's exact part number has not yet been documented. No boost converter is fitted in the wiring described here.

## Wiring

Module-level assembly schematic showing the external connections. The modules' internal circuitry is not expanded.

OBJECT assembly electrical schematic

[Open the SVG schematic](docs/schematic.svg) to zoom in or print it.

### Power

`**VSYS` means the bq25185 LOAD rail. It is a variable supply, not regulated 5 V.** Display and GPS power branch off before the Schottky diode.


| From                                 | To                                                      |
| ------------------------------------ | ------------------------------------------------------- |
| LiPo positive / negative             | Charger **BATT+ / BATT−**, observing connector polarity |
| Charger **LOAD+ / VSYS**             | Schottky **anode**                                      |
| Schottky **cathode**, the banded end | XIAO **5V / VBUS** pin                                  |
| Charger **LOAD+ / VSYS**             | Display **VCC**                                         |
| Charger **LOAD+ / VSYS**             | GPS **5V** input                                        |
| Charger **LOAD− / GND**              | XIAO, display, GPS, reed switch and button grounds      |


The battery connects to the external charger; the XIAO's battery pads are unused. Charge through the **bq25185 USB-C port**. The XIAO's USB-C port is used for programming and serial diagnostics; it does not charge the external battery through this isolated LOAD connection.

Adafruit #6091 supplies at most 4.5 V at LOAD and can fall toward 3.0 V as the battery discharges. The M100 Pro is specified for **3.6–5.5 V**, so reliable GPS operation below 3.6 V is not guaranteed. The display accepts 3.3 V or 5 V input, with 5 V recommended for full brightness. A regulated supply is a potential future improvement, not part of this build.

The charger defaults to **1 A charging**. Check the particular battery's permitted charging current and polarity before connecting it. The XIAO's low-battery behavior also needs testing with the fitted diode's voltage drop.

References: [Adafruit #6091](https://www.adafruit.com/product/6091), [HGLRC M100 Pro](https://www.hglrc.com/products/hglrc-m100-pro-gps), [LCDWiki display documentation](https://www.lcdwiki.com/3.2inch_IPS_SPI_Module_ILI9341).

### Display and microSD


| Display pin  | Connection                                   |
| ------------ | -------------------------------------------- |
| VCC          | Charger **LOAD+ / VSYS**                     |
| GND          | Common GND                                   |
| LCD_CS       | XIAO **D1**                                  |
| LCD_RS / D-C | XIAO **D2**                                  |
| LCD_RST      | **7 kΩ resistor to XIAO 3V3**; no reset GPIO |
| SDI / MOSI   | XIAO **D10**                                 |
| SCK          | XIAO **D8**                                  |
| SDO / MISO   | XIAO **D9**                                  |
| LED          | XIAO **D3**, PWM backlight control           |
| SD_CS        | XIAO **D5**                                  |


The display and microSD share the SPI bus, with separate chip-select pins. On this LCDWiki module, **LED is a control input to an onboard MOSFET**: D3 controls brightness rather than supplying the backlight current. Touch pins are unused.

### GPS

`Serial1`, **115200 baud, 8N1**. Firmware attempts to configure 1 Hz position updates.


| GPS pin | Connection               |
| ------- | ------------------------ |
| 5V      | Charger **LOAD+ / VSYS** |
| GND     | Common GND               |
| TX      | XIAO **D7 / RX**         |
| RX      | XIAO **D6 / TX**         |


GPS compass pins are unused.

### Wheel sensor and button

- **Reed switch:** between **D0** and **GND**. One falling edge counts one wheel revolution.
- **Button:** normally open, between **D4** and **GND**.

Both inputs use the XIAO's internal pull-ups. No external pull-up resistors are required by this wiring.

## Set it up for your bike

`WHEEL_CIRC_MM` in [the main sketch](xiao_oled/xiao_oled.ino) is currently **2155 mm**, the build's starting value for a 700 × 32C tire.

For better distance accuracy, use your measured wheel rollout rather than assuming every tire with the same size marking has the same circumference. Update the value and upload the firmware after changing it.

## Reading the screen

The portrait layout keeps current speed largest, with supporting information underneath:

1. **OBJECT and GPS status:** satellite count or “Searching”.
2. **Dot-matrix speed:** km/h, with average speed above it.
3. **24-dot gauge:** normally speed, at 2 km/h per dot; temporarily shows hold-to-save or download progress.
4. **Ride stats:** Distance, Time, Moving and Max.
5. **Footer:** altitude and a status such as `Recording`, `Write failed`, `Transfer`, `Phone` or `No card`.

**Time** is elapsed time since the trip began, including stops while powered on. Time spent powered off is not added after recovery. **Moving** accumulates from valid wheel-pulse intervals; **Avg** divides wheel distance by that moving time once a few revolutions and enough moving time have been recorded.

## Controls


| Action                                | Result                                                                        |
| ------------------------------------- | ----------------------------------------------------------------------------- |
| Press and release before 2 seconds    | Cycle backlight: **bright → dim → off**                                       |
| Double short-press while stopped      | Open a ~90 s Bluetooth transfer window (footer shows `Transfer`)              |
| Hold for 4 seconds while stopped      | Archive the GPS track if it has points, write a ride summary, reset counters  |
| Release during the new-ride countdown | Cancel the action; leave the backlight unchanged                              |


Stop and wait for the speed reading to reach zero before holding for a new ride. The countdown appears after two seconds. Turning the backlight off does **not** stop recording or turn off the device.

If the ride has no GPS points, the device shows **No GPS track**, clears counters, and still writes a `.SUM` sidecar when wheel data exists. With no card available, the long press resets RAM stats and displays **Stats reset**.

## Ride files


| File           | Contents                                                                                         |
| -------------- | ------------------------------------------------------------------------------------------------ |
| `CURRENT.GPX`  | Current GPS track                                                                                |
| `TRIP_A.DAT` / `TRIP_B.DAT` | Alternating CRC-checked checkpoints (sequence + counters + track position)          |
| `YYMMDDHH.GPX` | First-choice archive name, based on available GPS UTC time when saving                           |
| `YYMMDDnn.GPX` | Collision fallback; `nn` is 24–99 (sequence), not a clock hour                                   |
| `RIDEnnnn.GPX` | Numbered fallback when a date-based name is unavailable                                          |
| `*.SUM`        | Sidecar ride summary: revolutions, distance, moving/elapsed time, max speed, point count, UTC end |


Points are attempted no more frequently than once per second while moving, after the trip starts, when GPS data passes the current checks (including u-blox `gnssFixOK`). Stop and resume add endpoint points; drift while parked is not logged. Subsequent moving points also require advancing GPS time and a coordinate change of at least `0.000018°` on either latitude or longitude.

GPX exports contain coordinates, timestamps and altitude when available. Wheel-derived totals are stored in the matching `.SUM` sidecar. Checkpoints are for recovery only and are not offered over Bluetooth.

## Download a ride to your phone

1. Power on OBJECT and keep it near your phone.
2. While stopped, **double short-press** the button so the footer shows `Transfer`.
3. Open the [ride-transfer page](https://object.nav-tech.workers.dev).
4. Tap **Connect**, choose **OBJECT-001**, then select a GPX file.
5. The page checks the transferred file's CRC before requesting a browser download.

You can also download `CURRENT.GPX` without finishing the ride. The device sends a snapshot ending at the point where the transfer began. The page's **Current ride** badge identifies that filename; it is not a live recording-health indicator.

For the documented browser setup, use **Chrome on Android**, or [Bluefy on iPhone](https://apps.apple.com/app/bluefy-web-ble-browser/id1492822185), since Safari does not expose Web Bluetooth. To host your own copy, serve [tools/index.html](tools/index.html) over **HTTPS**.

LIST/GET require an open transfer window on the device. If a transfer stalls, the page aborts the firmware transfer; you can retry while the window is still open.

## Current limitations

| Area                | What to know                                                                                                                                        |
| ------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| Battery operation   | GPS supply can fall below its specified minimum. Full-discharge operation and runtime have not been established here.                               |
| BLE pairing         | Transfer uses a physical button window, not LE Secure Connections bonding. A nearby person cannot list tracks unless the window is open.            |
| Summary format      | `.SUM` is a plain-text sidecar for this project; third-party GPX apps will not read those wheel stats automatically.                                |


## Firmware

Main sketch: [xiao_oled/xiao_oled.ino](xiao_oled/xiao_oled.ino).

For a first check of a new board, [xiao_blink/xiao_blink.ino](xiao_blink/xiao_blink.ino) blinks the onboard LED and prints a serial heartbeat.

### Libraries

Install through Arduino Library Manager:

- Adafruit ILI9341
- Adafruit GFX Library
- Adafruit BusIO

Use the **Seeeduino nRF52** board core. This project's setup uses its bundled SdFat and Bluefruit libraries; do **not** install SdFat 2.3.x separately.

### Build and upload

Board identifier: `Seeeduino:nrf52:xiaonRF52840Sense`.

```bash
arduino-cli board list
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled
arduino-cli upload -p <PORT> --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled
```

Replace `<PORT>` with your board's port and upload only after compilation succeeds. Serial diagnostics use **115200 baud**.

## Repository layout

```text
xiao_oled/     Cycling computer firmware
xiao_blink/    LED blink and serial hardware check
tools/         Web Bluetooth ride-transfer page
docs/          Assembly electrical schematic (SVG)
AGENTS.md      Notes for automated agents
```

## License

[PolyForm Noncommercial License 1.0.0](LICENSE). Personal, hobby and other qualifying noncommercial use is covered by the license. Commercial use requires a separate license from the copyright holder.