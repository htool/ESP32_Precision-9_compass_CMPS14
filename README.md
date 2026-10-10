# ESP32 HWT901B compass (SensESP)

Magnetic heading, attitude and rate of turn from a **WitMotion HWT901B-TTL** on a **[Sailor Hat ESP32 (SH-ESP32)](https://docs.hatlabs.fi/sh-esp32/)**, published to Signal K over Wi‑Fi and as NMEA 2000 PGNs on the onboard isolated CAN bus.

Amazon product: [HWT901B-TTL (B07WF7JKY3)](https://www.amazon.nl/gp/product/B07WF7JKY3).

Board docs: [hardware / pinout](https://docs.hatlabs.fi/sh-esp32/hardware/). SensESP 3 is ESP32-only. Firmware is OTA-flashable after the first USB upload.

## Hardware (SH-ESP32)

Power the SH-ESP32 from the boat’s **8–32 V** power connector, USB (bench), or NMEA 2000 (with voltage-link pads if you use bus power). The IMU is wired on the **GPIO header**; use the board’s **3.3 V** pads for sensor VCC (there is no 5 V rail on the header — HWT901B-TTL accepts 3.3–5 V).

| Function | SH-ESP32 | Notes |
| --- | --- | --- |
| HWT901B TX → ESP32 RX | **GPIO 18** | Free (Ethernet MDIO optional only) |
| HWT901B RX ← ESP32 TX | **GPIO 19** | Free (Ethernet TXD0 optional only) |
| HWT901B VCC | **3V3** | Additional voltage pads (3× GND / 3× 3V3) |
| HWT901B GND | **GND** | Same pads / common ground |
| CAN TX / RX | **32 / 34** | Onboard isolated transceiver (jumpered) |
| Blue LED | GPIO 2 | SensESP / status |
| USB serial | GPIO 1 / 3 | Programming + monitor (CH340) |

Do **not** use GPIO 16/17 for the IMU — those are the default I2C pins. Do not use 32/34 for UART — those are CAN.

Default UART baud is **9600** (factory HWT901B-TTL). If you changed baud in WitMotion software, set `kImuBaud` in `src/main.cpp` to match.

### Aansluitschema

```
HWT901B-TTL                 SH-ESP32 (docs.hatlabs.fi/sh-esp32)
───────────                 ──────────────────────────────────
VCC             ──────────►  3V3  (additional voltage pads)
GND             ──────────►  GND
TX              ──────────►  GPIO 18  (GPIO header → ESP32 RX)
RX              ◄──────────  GPIO 19  (GPIO header → ESP32 TX)

Boat 12/24 V                SH-ESP32 power connector (8–32 V)
NMEA 2000 drop              SH-ESP32 CAN connector (DeviceNet/M12)
```

Cross TX/RX: sensor **TX** → ESP32 **RX** (18), sensor **RX** ← ESP32 **TX** (19). Wire on the GPIO header; no solder-jumper changes needed for 18/19.

Mount the sensor level, arrow/label forward if possible. If the cable plug faces the stern, leave **Invert yaw** enabled in the web UI (default).

## Signal K paths

| Path | Unit | Source |
| --- | --- | --- |
| `navigation.headingMagnetic` | rad | Angle frame yaw + offset |
| `navigation.attitude` | rad (roll/pitch/yaw) | Angle frame |
| `navigation.rateOfTurn` | rad/s | Gyro Z |

## NMEA 2000 (optional)

After the first received N2K frame, the firmware transmits:

| PGN | Content | Rate |
| --- | --- | --- |
| 127250 | Vessel Heading (magnetic) | ~10 Hz |
| 127251 | Rate of Turn | ~10 Hz |
| 127257 | Attitude (pitch/roll) | ~20 Hz |

Device claims as “Precision-9 Compass” (same product info style as the old CMPS14 sketch).

## Calibration (before boat install)

1. Connect the HWT901B to a PC with the included USB‑TTL adapter.
2. Use [WitMotion Windows software](https://wiki.wit-motion.com/) to:
   - Calibrate accelerometer (level surface)
   - Calibrate magnetometer (figure‑8 / slow rotation, away from iron)
   - Optionally set return rate to 10 Hz and content to Acc + Gyro + Angle (+ Mag)
3. Save configuration on the sensor (`save` in WitMotion UI).
4. Disconnect USB and wire the sensor to the SH-ESP32 as above.

Heading offset for the boat (installation misalignment) is set later in the SensESP web UI.

## Build and first flash (USB)

Install [PlatformIO](https://platformio.org/) (VS Code / Cursor extension or `pio` CLI).

```bash
cd ~/src/Arduino/ESP32_Precision-9_compass_WitMotion
cp wifi_secrets.h.example wifi_secrets.h   # edit SSID/password (gitignored)
./pio run -e shesp32 -t upload             # ./pio = PlatformIO ≥6 wrapper
./pio device monitor
```

`wifi_secrets.h` lives at the project root and is compiled in (`-include`), not under `src/`. Optional `WIFI_SSID_2` / `WIFI_PASSWORD_2` are tried as failover via SensESP `set_wifi_clients()`. With secrets the device joins in **STA-only** mode. Without the file, a config AP named `wit-compass` (password `thisisfine`) starts so you can set Wi‑Fi in the SensESP web UI.

Default Signal K server is `192.168.3.1:3000` (boatnet). Change it in the web UI if needed.

If Signal K is HTTPS/TLS, flash `shesp32_espidf` instead of `shesp32`.

Serial log should show `HWT901B linked` once angle frames arrive. Status page: `http://wit-compass.local/` (or the device IP).

## OTA (remote flash)

OTA password comes from `OTA_PASSWORD` in `wifi_secrets.h`. First image must be USB-flashed so the OTA partition table is present.

```bash
# after the device is on Wi-Fi (check serial or http://wit-compass.local/)
python3 ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i DEVICE_IP -p 3232 --auth="$OTA_PASSWORD" \
  -f .pio/build/shesp32/firmware.bin
```

Or uncomment `upload_protocol = espota` / `upload_port` / `upload_flags --auth=...` in `platformio.ini` and `./pio run -e shesp32 -t upload`.

## Config in the web UI

- **Heading offset (°)** — add after yaw conversion (−180…180)
- **Invert yaw** — `heading = 360 − yaw` when the cable faces aft (on by default)
- SK path names if you need non-default paths

## Legacy sketch

The previous Arduino IDE / CMPS14 N2K-only sketch is in `legacy/ESP32_Precision-9_compass_WitMotion.ino` (not built by PlatformIO).
