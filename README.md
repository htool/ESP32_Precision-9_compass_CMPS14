# ESP32 HWT901B compass (SensESP)

Magnetic heading, attitude and rate of turn from a **WitMotion HWT901B-TTL** on a **[Sailor Hat ESP32 (SH-ESP32)](https://docs.hatlabs.fi/sh-esp32/)**, published to Signal K over Wi‑Fi and as NMEA 2000 PGNs on the onboard isolated CAN bus.

Amazon product: [HWT901B-TTL (B07WF7JKY3)](https://www.amazon.nl/gp/product/B07WF7JKY3).

Board docs: [hardware / pinout](https://docs.hatlabs.fi/sh-esp32/hardware/). SensESP 3 is ESP32-only. Firmware is OTA-flashable after the first USB upload.

## Hardware (SH-ESP32)

Power the SH-ESP32 from the boat’s **8–32 V** power connector (preferred), USB (bench), or NMEA 2000 with the **voltage-link** pads joined.

**Supply:** use a stiff **12 V ≥ ~2 A** feed on the 8–32 V connector (0.8 A wall warts brown out when Wi‑Fi associates). N2K NET‑S alone is LEN-limited and usually too weak. USB + 12 V together is fine; plugging USB into a PC also **resets** the ESP.

**Wi‑Fi / RF (current defaults):** STA **19.5 dBm**, **PS_NONE**, CPU **240 MHz**. Earlier draw cuts (11 dBm TX, modem sleep, 160 MHz) caused weak-link `AUTH_EXPIRE` and one-ping drops. N2K TX idle/active rate saving is separate and unchanged. Peaks still need a stiff 12 V supply.

The IMU is wired on the **GPIO header**; use the board’s **3.3 V** pads for sensor VCC (there is no 5 V rail on the header — HWT901B-TTL accepts 3.3–5 V). Wide‑voltage / RS232 housings need Protected Voltage (~12 V) plus a MAX3232 if levels are RS232.

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

UART baud is **57600** (same as the USB cube viewer after cal). On boot the firmware asks a factory-default (9600) sensor to switch and save that rate. If you already set baud in WitMotion software, keep `kImuBaud` in `src/main.cpp` in sync.

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

Cross TX/RX: sensor **TX** → ESP32 **RX** (18), sensor **RX** ← ESP32 **TX** (19). Wire on the GPIO header; no solder-jumper changes needed for 18/19. If linked stays false, try **Swap UART RX/TX pins** in the HWT901B config (applies immediately). Status shows **UART RX activity**, **RX B/s**, **valid frames/s**, and **TX bytes** (power alone cannot be sensed — RX traffic is the proxy for “sensor on”).

**Recommended mount (vertical):** cable/wire **down**, sticker/label face toward the
**bow**, **−X toward starboard** (WitMotion: rotate 90° about X, then
`DIRECTION=vertical`). Set that in the cube viewer (**Vertical mount**) or
WitMotion MiniIMU — it is saved on the sensor flash.

| Face / end | Points |
| --- | --- |
| Cable (−Y) | Down (keel) |
| Opposite cable (+Y) | Up |
| Sticker (+Z) | Bow |
| −X end | Starboard |

Factory starting profile (fresh SPIFFS / **Reset mount profile to factory**):
**Vertical mount** on, **Invert roll / pitch / yaw** all on, **mount** attitude
offsets 0 (matches the desk-verified USB cube defaults for −X=starboard). Heel
follows WitMotion pitch, trim follows WitMotion roll. Fine-tune **mount** yaw /
roll / pitch offsets after the housing is in the bracket — do **not** zero while
the sensor is lying flat; |heel|/|trim| ≫ 45° tips the bow ~90°. Leave **B&G
fine-adjust** at 0 so the MFD still shows zeros.

Upload a cube viewer profile (`tools/calibrations/vertical_*.json`) at
[`/mount_profile`](http://wit-compass.local/mount_profile) (GET/PUT
[`/api/mount_profile.json`](http://wit-compass.local/api/mount_profile.json)).
Large heel/trim offsets in the JSON are cleared automatically. Restart after
changing Vertical mount so DIRECTION is rewritten on the sensor.

## Output mode (choose one)

In the SensESP web UI → **Compass output** (restart after change):

| Mode | Compass data |
| --- | --- |
| **1. NMEA 2000 only** | On the CAN bus; **no** Signal K deltas |
| **2. Signal K only** (default) | Over Wi‑Fi to Signal K; **no** N2K compass TX |

Wi‑Fi starts at boot in both modes (OTA / web UI) and is held open. Magic PGN /
`ota_update.sh` can soft-disconnect or reconnect STA. Only compass **publishing**
is gated by the mode switch.

### Signal K paths (mode 2)

| Path | Unit | Source |
| --- | --- | --- |
| `navigation.headingMagnetic` | rad | Angle frame yaw + offsets |
| `navigation.attitude` | rad (roll/pitch/yaw) | Angle frame |
| `navigation.rateOfTurn` | rad/s | Gyro Z |

### NMEA 2000 (mode 1)

After the first received N2K frame, standard + B&G Precision-9 style PGNs:

| PGN | Content | Rate / role |
| --- | --- | --- |
| 127250 | Vessel Heading (magnetic) | **active_hz** (config; often 3 or 10 Hz) |
| 127251 | Rate of Turn | same period as heading |
| 127257 | Attitude (pitch/roll) | same period as heading |
| **130824** | B&G pitch/roll performance keys | same period |
| **130845** | B&G fine-adjust get/set (heading, heel, trim) | on request from MFD |
| **130850 / 130851** | B&G calibration start/stop + ack | starts/stops COG deviation swing |
| **129026** (RX) | COG / SOG rapid update | used to build the 0.5° deviation map |
| **127258** (RX) | Magnetic variation | converts true COG → magnetic when present |

Each TX cycle sends **127250 + 127251 + 127257 + 130824** with the **same SID**,
then increments SID by **1** (wraps 1…253). Older firmware alternated heading /
attitude and made SID jump by 2 while cutting 127250 rate in half.

### Attitude / heading offsets (two layers)

| Layer | Keys | Used by | Applied |
| --- | --- | --- | --- |
| **Mount** (install) | `yaw/roll/pitch_offset_deg`, invert_*, cube profile, COG mean fold | Web UI / `/mount_profile` | Before invert (+ deviation for heading) |
| **B&G fine-adjust** | `bang_heading/heel/trim_offset_deg` | PGN **130845** only | After mount (+ deviation for heading) |

Mount offsets are **invisible to B&G** — the MFD reads/writes only `bang_*`
(defaults **0**), so after install the boat can nudge a few degrees without
seeing the large install correction.

### Deviation map (COG ↔ compass)

Underway, turn steadily with SOG ≥ 1 kn and start a swing from the MFD compass
cal (PGN 130850) or the web **Control** buttons (**Start / Stop COG deviation
swing**). The firmware records COG vs magnetic heading into a **0.5°** table
(720 bins), extracts the mean error into the **mount yaw offset** (minimizes
constant sensor offset; still invisible to B&G), and keeps the residual curve in
the map.

- Status page group **Deviation**: enabled, bins filled, RMS, live COG−compass
- Config card **Deviation map (COG)** + chart page [`/deviation`](http://wit-compass.local/deviation)
- Download / upload JSON: [`/api/deviation.json`](http://wit-compass.local/api/deviation.json) (GET download, PUT upload)
- N2K listen for COG stays active even in Signal K–only output mode

**NMEA 2000 identity** (web UI, restart required): preferred CAN address, ISO NAME fields (unique number / serial, manufacturer code, device class & function, industry group, device/system instance), and product info (model serial, product code, model ID, software/model version, load equivalency, certification) plus installation description strings. Defaults match the old Precision-9 sketch (address 24, unique 1048678, …). Control tab: **Reset N2K identity to defaults** (reboots).

### Power saving & Wi‑Fi control

**TX rate** follows navigation (configurable wake PGN). **Wi‑Fi** is owned by
`OfflineNetworkProvisioner` + `PowerManager` (not SensESP’s reconnect loop, which
used to call `WiFi.begin()` mid-handshake → `ESP_ERR_WIFI_CONN` / `AUTH_EXPIRE`).

| State | Compass N2K TX | Wi‑Fi |
| --- | --- | --- |
| Active (wake PGN seen) | **active_hz** (config, default 10; boat often 3) | unchanged |
| Idle (no wake PGN for timeout) | **idle_hz** (default 1) | unchanged |
| Boot | — | **On** (hold-open; no timed auto-off) |
| Magic PGN WIFI **on** | — | Associate / stay open |
| Magic PGN WIFI **off** | — | **Soft-off:** STA disconnect only |

**Do not use `WiFi.mode(WIFI_OFF)`.** That tears down lwIP; SensESP then dies and
**N2K TX stops** with it. Soft-off keeps the driver + lwIP so heading keeps
publishing. `ota_update.sh` defaults to **leaving Wi‑Fi on** after flash for the
same reason.

Default wake PGN (TX rate only): **126992** (System Time). Set source address to
pin to a specific device, or PGN `0` = any. Control tab: **Reset power saving to
defaults**.

**Magic Wi‑Fi PGN:** proprietary single-frame **61184**, manufacturer **275**,
data tag `WIFI`, command byte `1`=on / `0`=off (optional minutes in byte 7):

```bash
python3 tools/n2k_wifi_control.py on --iface can0 --minutes 15
python3 tools/n2k_wifi_control.py off --iface can0
```

## Calibration (before boat install)

Official WitMotion procedure ([HWT901B manuals](https://witmotion-sensor.com/pages/faq),
[WIT Standard Protocol](https://wit-motion.gitbook.io/witmotion-sdk/wit-standard-protocol/wit-standard-communication-protocol)):

1. Connect the HWT901B to a PC with the USB‑TTL adapter (TX↔RX crossed).
2. Either run `python3 tools/witmotion_cube.py …` → **Calibrate…**, or use
   WitMotion Windows MiniIMU under `vendor/witmotion-pc-software/`:
   - **Accel** — level table, still (`CALSW=1`). Never do this on a rocking boat.
   - **Mag** — spherical fit (`CALSW=0x07`): slow free 3D tumbling 1½–3 min,
     ≥20 cm from iron (laptop speakers count). Dual-plane mode is `CALSW=0x09`.
   - Keep **9-axis** (`AXIS6=0`). 6-axis integrates gyro and *will* drift.
3. Save to flash. Confirm heading stays put when still, then tracks when you turn.
4. Wire the sensor to the SH-ESP32.

**Boat install (soft/hard iron):** Mag cal only removes *constant* bias. Mount away
from steel, speakers, and DC cables; set **mount yaw offset** in the SensESP UI
(or align against a known good heading on the bus); build a GPS deviation table
if residual error exceeds ~±5°. Re-run mag cal at the mount if needed — not
accel. Good marine write-up:
[open-boat-projects.org — HWT901B for Signal K](https://open-boat-projects.org/en/magnetischer-kompass-fuer-signalk/).

## Build and first flash (USB)

Install [PlatformIO](https://platformio.org/) (VS Code / Cursor extension or `pio` CLI).

```bash
cd ~/src/Arduino/ESP32_Precision-9_compass_WitMotion
cp wifi_secrets.h.example wifi_secrets.h   # edit SSID/password (gitignored)
./pio run -e shesp32 -t upload             # ./pio = PlatformIO ≥6 wrapper
./pio device monitor
```

`wifi_secrets.h` lives at the project root and is compiled in (`-include`), not
under `src/`. Optional `WIFI_SSID_2` / `WIFI_PASSWORD_2` are a standby SSID: boot
scans and prefers the strongest usable AP (RSSI ≥ about **−80 dBm**; weaker APs
often `AUTH_EXPIRE` on this antenna). With secrets the device joins in
**STA-only** mode. Without the file, a config AP named `wit-compass` (password
`thisisfine`) starts so you can set Wi‑Fi in the SensESP web UI.

Default Signal K server is `192.168.3.1:3000` (boatnet). Change it in the web UI if needed.

If Signal K is HTTPS/TLS, flash `shesp32_espidf` instead of `shesp32`.

Serial log should show `HWT901B linked` once angle frames arrive. Status page:
`http://wit-compass.local/` (or the device IP, e.g. `192.168.2.24` on the boat LAN).

## Tools (USB calibration / 3D cube)

With the HWT901B on a USB‑TTL adapter (TX↔RX crossed, common GND):

```bash
python3 tools/witmotion_cube.py --port /dev/ttyUSB0 --baud 57600
```

Use **Calibrate…** for guided accel + magnetometer calibration (same `CALSW` UART
commands as the Windows software). Official Windows installer zips live under
`vendor/witmotion-pc-software/` (see README there; not committed — large binaries).

**Save profile** / **Load profile** store viewer offsets + `HX/HY/HZOFFSET` under
`tools/calibrations/` as `horizontal_latest.json` or `vertical_latest.json`
(whichever DIRECTION is active). The viewer banner and 3D chip show the mode;
red “DIRECTION mismatch” means the sensor pose does not match the register.

Headless horizontal snapshot:

```bash
python3 tools/witmotion_cube.py --port /dev/ttyUSB0 --baud 57600 --save-horizontal-profile
```

For boat-mount mag cal: click **Vertical mount** first (banner = VERTICAL), then
**Calibrate…**. Accel still uses a flat/horizontal step; mag returns to vertical.

## OTA (remote flash)

OTA password comes from `OTA_PASSWORD` in `wifi_secrets.h`. First image must be
USB-flashed so the OTA partition table is present.

Wi‑Fi is on at boot (hold-open). Preferred wrapper (builds + espota; **leaves
Wi‑Fi on** by default so a post-flash magic OFF cannot kill N2K):

```bash
./ota_update.sh                         # wit-compass.local, can0
./ota_update.sh 192.168.2.24            # explicit IP
HOST=192.168.2.24 ./ota_update.sh --skip-build   # flash existing bin only
```

From a host that has a good LAN path to the device (e.g. **boatnet** on
`192.168.2.0/24`), bind espota’s callback interface with `-I`:

```bash
# on boatnet, after scp of firmware.bin into the project tree:
cd ~/src/Arduino/ESP32_Precision-9_compass_WitMotion
AUTH=$(sed -n 's/.*OTA_PASSWORD[[:space:]]*"\([^"]*\)".*/\1/p' wifi_secrets.h)
python3 ~/.platformio/packages/framework-arduinoespressif32/tools/espota.py \
  -i 192.168.2.24 -I 192.168.2.1 -p 3232 -t 90 \
  --auth="$AUTH" -f .pio/build/shesp32/firmware.bin -r
```

Or uncomment in `platformio.ini`:

```ini
upload_protocol = espota
upload_port = 192.168.2.24
upload_flags =
    --auth=YOUR_OTA_PASSWORD
```

then `./pio run -e shesp32 -t upload` (Wi‑Fi must already be on). Under the hood
this is the same `espota.py`.

To force Wi‑Fi off after OTA (soft-off only — safe for N2K):  
`LEAVE_WIFI_ON=0 ./ota_update.sh …` or `python3 tools/n2k_wifi_control.py off`.

## Config in the web UI

- **Compass output** — NMEA 2000 only or Signal K only (restart required)
- **Yaw / roll / pitch offset (°)** — internal **mount** offsets (invisible to B&G)
- **Invert roll / pitch / yaw** — applied after mount offsets
- **B&G heading / heel / trim fine-adjust (°)** — PGN 130845 layer (MFD sees these)
- **Power saving** — wake PGN, idle/active Hz, Wi‑Fi on-duration (hold-open ignores timed off)
- SK path names if you need non-default paths (Signal K mode)

API: `GET`/`PUT` `http://DEVICE/api/config/hwt901b` (and `/power`, `/output`, …).

## Legacy sketch

The previous Arduino IDE / CMPS14 N2K-only sketch is in `legacy/ESP32_Precision-9_compass_WitMotion.ino` (not built by PlatformIO).
