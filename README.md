# OBJECT

A DIY cycling computer built on the **Seeed XIAO nRF52840 Sense**. It shows live ride metrics on a 3.2″ IPS display, logs GPS tracks to a microSD card, and lets you pull finished rides onto your phone over Bluetooth.

## What it does

- **Wheel speed** from a reed switch (one pulse per revolution). Distance, max, and average are derived from that; average uses moving time only, so stops do not dilute it.
- **GPS** via an HGLRC M100 Pro (u-blox). Status, satellite count, and altitude appear on screen; position + time are written into a GPX track while you ride.
- **Persistent trip state** on the SD card (`TRIP.DAT` + `CURRENT.GPX`). Power-cycle mid-ride and it resumes where you left off.
- **Ride archive**: hold the button 4 seconds while stopped to finalize the track, rename it on the card, and start a fresh ride.
- **BLE download**: the board advertises as `OBJECT-001`. Open `https://object.nav-tech.workers.dev`, connect from a phone, and save any `*.GPX` file from the card.

## Hardware


| Piece                          | Role                              |
| ------------------------------ | --------------------------------- |
| Seeed XIAO nRF52840 Sense      | MCU, Bluetooth, USB               |
| 3.2″ ILI9341 SPI TFT (240×320) | Ride display + shared SPI microSD |
| Reed switch + magnet           | Wheel revolution sensor           |
| HGLRC M100 Pro GPS             | Position / altitude / UTC time    |
| Tact button                    | Backlight cycle / new ride        |


### Wiring

**Display**


| Module          | XIAO                           |
| --------------- | ------------------------------ |
| VCC             | 3V3                            |
| GND             | GND                            |
| LCD_CS          | D1                             |
| LCD_RS (D/C)    | D2                             |
| LCD_RST         | 7kΩ → 3.3V (held out of reset) |
| MOSI            | D10                            |
| SCK             | D8                             |
| MISO            | D9                             |
| LED (backlight) | D3                             |
| SD_CS           | D5                             |


**Reed switch** — one side to **D0**, other to **GND** (internal pull-up; falling edge = pulse).

**Button** — one side to **D4**, other to **GND**.

**GPS (Serial1, 115200 8N1)**


| GPS | XIAO    |
| --- | ------- |
| GND | GND     |
| TX  | D7 (RX) |
| RX  | D6 (TX) |
| 5V  | 5V      |


Wheel circumference is set for **700×32C (ISO 32-622) → 2155 mm**. Change `WHEEL_CIRC_MM` in `xiao_oled/xiao_oled.ino` for a different tire.

## On-device UI

Monochrome portrait layout, matched to the ride-transfer page:

1. **OBJECT** wordmark and GPS state (satellite count or “Searching”)
2. Large **dot-matrix speed**, average above the gauge
3. **24-dot gauge** — speed (2 km/h per dot), new-ride hold progress, or BLE send %
4. Rows: **Distance**, **Time**, **Moving**, **Max**
5. Footer: **Altitude** and status (`Recording`, `Phone`, or `No card`)

## Controls


| Action                 | Effect                              |
| ---------------------- | ----------------------------------- |
| Short press (< 2 s)    | Backlight: bright → dim → off       |
| Hold 4 s while stopped | Save ride to SD and start a new one |


During the hold, the caption shows a countdown and the gauge fills. Flash messages confirm *Ride saved*, *Stats reset*, or *Save failed*.

## Ride files


| File                             | Purpose                                                  |
| -------------------------------- | -------------------------------------------------------- |
| `CURRENT.GPX`                    | Active track (appended while moving with a live GPS fix) |
| `TRIP.DAT`                       | CRC-checked trip counters for resume after reboot        |
| `YYMMDDHH.GPX` or `RIDEnnnn.GPX` | Archived rides after a successful new-ride hold          |


Track points are written about once per second when you have moved ~2 m and GPS time has advanced. `TRIP.DAT` is not offered over BLE.

## Phone transfer

1. Serve `tools/index.html` over **HTTPS** (Web Bluetooth requires a secure context).
2. Wake the computer, keep it nearby, tap **Connect**, and pick **OBJECT-001**.
3. Tap a ride in the list to download the GPX (CRC-checked before save).

**Browsers:** Chrome on Android works. Safari has no Web Bluetooth — use [Bluefy](https://apps.apple.com/app/bluefy-web-ble-browser/id1492822185) on iPhone.

## Firmware

Main sketch: `[xiao_oled/xiao_oled.ino](xiao_oled/xiao_oled.ino)`

There is also a minimal blink sketch under `xiao_blink/` for board bring-up.

### Libraries

Install from Arduino Library Manager:

- Adafruit ILI9341
- Adafruit GFX Library
- Adafruit BusIO

`SdFat` and Bluefruit ship with the **Seeeduino nRF52** core — do **not** install SdFat 2.3.x separately.

### Build & upload

Board: `Seeeduino:nrf52:xiaonRF52840Sense`

```bash
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled
arduino-cli upload -p <PORT> --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled
```

Confirm the port with `arduino-cli board list` if upload fails.

## Repo layout

```
xiao_oled/     Cycling computer firmware
xiao_blink/    LED blink hardware check
tools/         Web Bluetooth ride-transfer page
AGENTS.md      Notes for automated agents (compile/upload)
```

## License

[PolyForm Noncommercial License 1.0.0](LICENSE) — free for personal, hobby, educational, and other noncommercial use. Commercial use requires a separate license from the copyright holder.

