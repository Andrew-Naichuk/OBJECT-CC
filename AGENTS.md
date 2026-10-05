# Agent notes

## XIAO OLED sketch — always upload

After any change finished always compile and upload to the connected board. Do not wait to be asked.

```powershell
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense "c:\Development\object-1\xiao_oled"
arduino-cli upload -p COM7 --fqbn Seeeduino:nrf52:xiaonRF52840Sense "c:\Development\object-1\xiao_oled"
```

- Board: Seeed XIAO nRF52840 Sense (`Seeeduino:nrf52:xiaonRF52840Sense`)
- Upload port is usually `COM7` (confirm with `arduino-cli board list` if upload fails)
- On PowerShell, run compile then upload sequentially (`&&` may not work); only upload if compile succeeds

### On macOS

`arduino-cli` is not on the agent shell's PATH; it lives at `~/bin/arduino-cli`. The Seeed core's post-build step runs `python`, which macOS does not have (only `/usr/bin/python3`), so the build fails with `exec: "python": executable file not found` or `exit status 72`. A symlink named `python` does not work because Apple's `/usr/bin/python3` launcher dispatches on its invoked name; use a wrapper script:

```bash
mkdir -p /tmp/pyshim
[ -x /tmp/pyshim/python ] || { printf '#!/bin/sh\nexec /usr/bin/python3 "$@"\n' > /tmp/pyshim/python && chmod +x /tmp/pyshim/python; }
export PATH=/tmp/pyshim:$PATH
cd /Users/andriinaichuk/Desktop/DEV/object-1
~/bin/arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled \
  && ~/bin/arduino-cli upload -p /dev/cu.usbmodem2101 --fqbn Seeeduino:nrf52:xiaonRF52840Sense xiao_oled
```

- Upload port is usually `/dev/cu.usbmodem2101`; confirm with `~/bin/arduino-cli board list` (pick the row whose board name is "Seeed XIAO nRF52840 Sense", not the other `usbmodem` device)
- Upload takes about 45 seconds; it is done when it prints `Device programmed.`

## README screens — regenerate after display changes

The images in `docs/screens/` are generated from the firmware by `tools/screens/`. Never edit them by hand.

After any change to `xiao_oled/xiao_oled.ino` that can affect what the display shows (layout, text, fonts, colours, icons, units, prompts, new states), run:

```powershell
py tools\screens\regenerate.py
```

On macOS or Linux, run `python3 tools/screens/regenerate.py`.

- It compiles the sketch on this computer, runs every scenario and rewrites only the images whose pixels changed. It prints the list.
- Open `tools/screens/build/raw/INDEX.md` and look at the changed shots before committing them with the firmware change.
- `--check` exits 1 if `docs/screens` is out of date; use it to verify without writing.
- If the emulator fails to compile, the sketch is using an Arduino, SdFat or Bluefruit call the stubs do not have. Add it in `tools/screens/stubs/`.
- To document a new screen state, follow "Change what is captured" in `tools/screens/README.md`.
- If the README text describes something the new screens contradict, update the text in the same change.

## Firmware OTA — CI publishes on push

Do not build or copy OTA images by hand. A push to `main` that touches `xiao_oled/` runs `.github/workflows/firmware.yml`, which stamps `FW_VERSION`, compiles the sketch and commits `docs/firmware/` (`firmware.bin`, `firmware.dat`, `manifest.json`). Local USB flashes still report `dev` and still need the compile-and-upload step above. The hub fetches the published manifest from GitHub Pages.
