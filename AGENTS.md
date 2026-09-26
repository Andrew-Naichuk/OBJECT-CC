# Agent notes

## XIAO OLED sketch — always upload

After any change finished always compile and upload to the connected board. Do not wait to be asked.

```powershell
arduino-cli compile --fqbn Seeeduino:nrf52:xiaonRF52840Sense "c:\Development\MC\xiao_oled"
arduino-cli upload -p COM7 --fqbn Seeeduino:nrf52:xiaonRF52840Sense "c:\Development\MC\xiao_oled"
```

- Board: Seeed XIAO nRF52840 Sense (`Seeeduino:nrf52:xiaonRF52840Sense`)
- Upload port is usually `COM7` (confirm with `arduino-cli board list` if upload fails)
- On PowerShell, run compile then upload sequentially (`&&` may not work); only upload if compile succeeds
