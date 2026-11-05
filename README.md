# J1939 CAN Sniffer – Live Vehicle Speed + .trc Replay

A desktop application that connects your laptop to a J1939 CAN network, displays live message traces **with human-readable PG names** (from the SAE J1939 Digital Annex JUL2026), lets you filter by PGN, shows **live vehicle speed / engine RPM**, and **opens & replays Peak Systems `.trc` log files**.

Repository: https://github.com/Vedant-Bharate/j1939-can-sniffer

## Features

- **Live sniffing** via `python-can` (SocketCAN, PCAN, Vector, Ixxat, CANalyst, or Virtual for demo)
- Every extended frame shown with:
  - Timestamp
  - 29-bit CAN ID
  - PGN + **official PG Label** (e.g. “Cruise Control/Vehicle Speed 1”)
  - Source Address
  - Raw data (hex)
  - Decoded key signals when present
- Large live displays for:
  - **Wheel-Based Vehicle Speed** (SPN 84 / PGN 65265)
  - **Engine Speed** (SPN 190 / PGN 61444)
  - **Actual Engine % Torque** (SPN 513)
- Filter by PGN number or name
- **Open & review Peak `.trc` files** (v1.x and v2.x formats supported via python-can’s `TRCReader`)
- **Timed replay** of `.trc` files at 0.25× – 50× speed
- Built-in simulator (Virtual interface) that emulates a vehicle accelerating / decelerating

## Quick Start (Demo – no hardware)

```bash
pip install python-can
python j1939_can_sniffer.py
```

1. Leave **Interface = virtual**
2. Click **▶ Start Live**
3. Watch vehicle speed and RPM climb and fall
4. Type `65265` or `speed` in the filter box

## Open / Replay a .trc file

1. Click **📂 Open .trc** and select a Peak Systems trace file
2. The table is immediately filled so you can browse and filter
3. Click **▶ Replay .trc** to play the messages back in (accelerated) real time
4. Adjust the “Speed ×” combo (0.25 … 50) before starting replay

## Real CAN Hardware

| Setting   | Typical value              |
|-----------|----------------------------|
| Interface | `socketcan` (Linux) / `pcan` / `vector` |
| Channel   | `can0` or `PCAN_USBBUS1`   |
| Bitrate   | `250000` or `500000`       |

Linux SocketCAN example:
```bash
sudo ip link set can0 type can bitrate 250000
sudo ip link set up can0
```

## Key Signals

| SPN | Name                      | PGN   | PG Name                        | Scaling               | Position |
|-----|---------------------------|-------|--------------------------------|-----------------------|----------|
| 84  | Wheel-Based Vehicle Speed | 65265 | Cruise Control/Vehicle Speed 1 | 0.00390625 km/h/bit   | bytes 2-3 |
| 190 | Engine Speed              | 61444 | Electronic Engine Controller 1 | 0.125 rpm/bit         | bytes 4-5 |
| 513 | Actual Engine – % Torque  | 61444 | Electronic Engine Controller 1 | 1 %/bit, offset –125  | byte 3   |

These are the standard signals used by fleet management, tachographs and ADAS for vehicle speed on heavy-duty vehicles.

## Files

| File                  | Purpose |
|-----------------------|---------|
| `j1939_can_sniffer.py`| Main application |
| `pgn_map.json`        | ~3200 PGN → official name mappings extracted from SAE J1939DA JUL2026 |
| `README.md`           | This file |

## Requirements

```bash
pip install python-can
```

Python 3.10+ recommended. Tkinter is used for the GUI (usually already present on most systems).

## License

MIT – feel free to use and extend.

---

Data source: SAE J1939™DA Digital Annex (JUL 2026 release).
