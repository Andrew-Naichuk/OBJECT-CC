# Screen emulator

Regenerates the README screenshots in [`docs/screens`](../../docs/screens) from the firmware itself.

`emulator.cpp` compiles the **unmodified** [`xiao_oled/xiao_oled.ino`](../../xiao_oled/xiao_oled.ino) on your computer. The display is drawn by the real Adafruit GFX library and fonts into a 240 × 320 framebuffer. The hardware around it is simulated:

- a virtual millisecond clock
- a wheel magnet on the reed switch
- a u-blox GPS sending NAV-PVT (or NMEA GGA)
- the button and battery divider on A4
- an in-memory microSD card, which can be told to fail
- a phone that connects, confirms and downloads over BLE

Each scenario boots from cold, rides through a situation and captures the screen at set moments. The same firmware always produces the same pixels.

## Run it

```bash
python tools/screens/regenerate.py
```

On Windows, `py tools\screens\regenerate.py` works too.

The script:

1. Finds Adafruit GFX. It uses the copy Arduino installed, or clones version 1.12.6 into `tools/screens/build/`.
2. Compiles the emulator.
3. Runs every scenario and writes the native shots to `tools/screens/build/raw/`, with an `INDEX.md` table describing each one.
4. Builds the README images and writes only the ones whose pixels changed.

It then lists what changed, and warns if the README points at an image that nothing generates.

| Option | Use |
| --- | --- |
| `--check` | Write nothing; exit 1 if `docs/screens` is out of date |
| `--raw-only` | Only produce the native shots in `build/raw/` |
| `--scenario NAME` | Run one scenario (repeatable). Only produces raw shots |
| `--gfx PATH` | Use a specific `Adafruit_GFX_Library` folder |
| `--cxx PATH` | Use a specific C++ compiler |

Set `OBJECT_EMU_LOG=1` to print the firmware's serial log while scenarios run.

### Requirements

- Python 3.8+ with Pillow: `python -m pip install pillow`
- A C++17 compiler on `PATH`:
  - **Windows:** [MSYS2](https://www.msys2.org/) with `pacman -S mingw-w64-ucrt-x86_64-gcc`, then add `C:\msys64\ucrt64\bin` to `PATH`. LLVM's `clang++` also works.
  - **macOS:** `xcode-select --install`
  - **Linux:** the distro's `g++`.
- Git, only if Adafruit GFX is not installed through Arduino.

`arduino-cli` and the Seeeduino core are not needed; nothing here builds for the board.

## Files

| Path | Role |
| --- | --- |
| `regenerate.py` | Build, run and compose; the list of README images and their sources |
| `emulator.cpp` | Virtual hardware and the scenarios that capture each screen |
| `stubs/` | Host versions of `Arduino.h`, SdFat, Bluefruit and the ILI9341 driver |
| `build/` | Compiler output, GFX clone and raw shots (ignored by git) |

## Change what is captured

**Firmware changed what a screen looks like.** Run the script and commit the updated `docs/screens/*.png`.

**A new screen state.** In `emulator.cpp`, reach the state inside a scenario and call `shot("NN_name", "what it shows")`. Then add the README image to the right table in `regenerate.py`:

- `SCREENS` for a full screen
- `BANDS` for a strip
- `AS_SEEN` for a screen darkened by its backlight setting

Reference the image from `README.md` and run the script.

**Layout moved.** The anatomy callouts in `regenerate.py` (`ANATOMY`) are rows in panel pixels, taken from the layout constants in the sketch. Update them when `CAPTION_Y`, `HERO_Y`, `GAUGE_Y`, `ROWS_Y` or `FOOTER_Y` change.

**The emulator stops compiling.** The firmware probably calls an Arduino, SdFat or Bluefruit function the stubs do not have yet. Add it to the matching file in `stubs/`; each stub only implements what the sketch uses.

## What is approximated

- **Panel colours:** the shots assume `invertDisplay(true)` cancels the IPS panel's native inversion, so they show the palette constants from the sketch.
- **Backlight:** dim and off do not change the framebuffer. `backlight-dim.png` and `backlight-off.png` scale brightness by the PWM duty as a rough picture of what the eye sees.
- **Timing:** SPI drawing takes no virtual time.
- **Battery:** the voltage is fixed per scenario (4.02 V by default, 3.86 V in `no_card`, 3.68 V in `resume`).
- **Save failed:** reached by making the simulated card refuse the archive rename.
