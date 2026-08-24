# Tilt-Compensated Compass + GPS — Eventide Module

Eventide module for the SparkFun/Robot Electronics CMPS12 tilt-compensated compass (I2C) and the ATGM336H NMEA GPS receiver.

## What it does

### Compass
- Polls the CMPS12 over I2C using `tcc_i2c.py`.
- Exposes an HTTP API for the dashboard:
  - `GET /api/telemetry` — latest bearing, pitch, roll, raw IMU data, temperature, and calibration state.
  - `GET /api/health` — whether a reading has been received.
- Streams line-delimited JSON over the `stream` Unix socket (`/tmp/tcc_stream.sock` by default).
- Default widgets:
  - **Compass Attitude** (`orientation3d`) — 3D attitude indicator driven by bearing/pitch/roll.
  - **Compass Telemetry** (`telemetry`) — polled readout of bearing, attitude, IMU, temperature, and calibration levels.

### GPS
- Reads standard NMEA sentences from the configured serial port using `gps_service.py`.
- Interprets fix status from GGA/RMC sentences and surfaces human-readable JSON.
- Exposes an HTTP API for the dashboard:
  - `GET /api/telemetry` — full GPS state (lat/lon/altitude, speed, course, fix quality, satellites, DOP, etc.).
  - `GET /api/track` — minimal track update for the map widget (`lat`, `lon`, `heading`, `target`).
  - `GET /api/health` — whether a GPS lock has been achieved.
- Default widgets:
  - **GPS Map** (`map`) — Leaflet map showing the current GPS position and heading.
  - **GPS Telemetry** (`telemetry`) — polled readout of fix, position, speed, and satellite info.

## Files

- `eventide-module.json` — Eventide manifest (programs, sockets, UI).
- `tcc_service.py` — Compass service; sensor reader + HTTP API + Unix-socket stream.
- `tcc_i2c.py` — CMPS12 I2C driver (used by `tcc_service.py`).
- `tcc_serial.py` — Standalone serial driver for the CMPS12 (not used by this module).
- `gps_service.py` — GPS service; serial NMEA reader + HTTP API.
- `requirements.txt` — Python dependencies.

## Install

Install from the Eventide dashboard's MODULES tab by pointing it at this repository, or zip the folder and use **ZIP FILE**.

## Arguments

| Argument          | Default        | Description                                  |
| ----------------- | -------------- | -------------------------------------------- |
| `--bus-number`    | `1`            | I2C bus number for the CMPS12.               |
| `--address`       | `0xC0`         | CMPS12 8-bit I2C address (hex string).       |
| `--poll-interval` | `0.02`         | Compass sensor poll interval in seconds.     |
| `--stream-interval`| `0.1`         | Compass Unix-socket stream interval in sec.  |
| `--port`          | `/dev/ttyAMA0` | Serial port for the ATGM336H GPS.            |
| `--baud`          | `9600`         | Baud rate for the ATGM336H GPS.              |

## Sockets

- `http` (TCP, auto-allocated port) — compass API, proxied at `/proxy/tilt-compensated-compass/http/...`.
- `stream` (Unix, `/tmp/tcc_stream.sock`) — compass line-delimited JSON telemetry stream.
- `gps_http` (TCP, auto-allocated port) — GPS API, proxied at `/proxy/tilt-compensated-compass/gps_http/...`.

## Local sanity check

```bash
python3 -m json.tool eventide-module.json > /dev/null && echo "manifest OK"
python3 -m py_compile tcc_service.py gps_service.py tcc_i2c.py && echo "syntax OK"
```
