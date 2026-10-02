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
