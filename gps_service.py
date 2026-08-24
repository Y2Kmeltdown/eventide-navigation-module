#!/usr/bin/env python3
"""
gps_service.py — Eventide service for an NMEA GPS receiver (e.g. ATGM336H).

Opens a serial port, parses standard NMEA sentences, and exposes the current
fix/state over HTTP for the dashboard's map and telemetry widgets.

Run directly for local testing:
    python3 gps_service.py --port /dev/ttyAMA0 --baud 9600 --http-port 8081
"""

import argparse
import asyncio
import json
import signal
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

import pynmea2
import serial
from aiohttp import web


class GPSService:
    """Serial NMEA reader + HTTP API for map/telemetry widgets."""

    def __init__(
        self,
        port: str,
        baud: int,
        timeout: float = 1.0,
        recordings_dir: str | None = None,
    ):
        self.port = port
        self.baud = baud
        self.timeout = timeout
        self.recordings_dir = recordings_dir
        self.latest: dict = {}
        self.lock = threading.Lock()
        self._shutdown = asyncio.Event()
        self._serial: serial.Serial | None = None
        self._thread: threading.Thread | None = None
        self._log_file = None
        self._log_lock = threading.Lock()

    def start(self) -> None:
        self._serial = serial.Serial(self.port, self.baud, timeout=self.timeout)
        self._thread = threading.Thread(target=self._read_loop, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        if self._serial:
            self._serial.close()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._log_file:
            with self._log_lock:
                self._log_file.close()
            self._log_file = None

    def _start_log(self) -> None:
        """Open a UTC-timestamped JSONL recording file."""
        if not self.recordings_dir:
            return
        Path(self.recordings_dir).mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        log_path = Path(self.recordings_dir) / f"gps_{ts}.jsonl"
        self._log_file = open(log_path, "a", buffering=1)
        print(f"[gps] recording to {log_path}", flush=True)

    def _log(self, data: dict) -> None:
        """Append one timestamped JSON line to the recording file."""
        if not self._log_file:
            return
        entry = {"recorded_at_utc": datetime.now(timezone.utc).isoformat(), **data}
        with self._log_lock:
            self._log_file.write(json.dumps(entry, default=str) + "\n")

    def _read_loop(self) -> None:
        while self._serial and self._serial.is_open:
            try:
                raw = self._serial.readline()
                if not raw:
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if not line.startswith("$"):
                    continue
                msg = pynmea2.parse(line)
                self._handle_msg(msg)
            except pynmea2.ParseError:
                pass
            except Exception as exc:
                print(f"[gps] read error: {exc}", flush=True)

    def _handle_msg(self, msg: pynmea2.NMEASentence) -> None:
        stype = msg.sentence_type
        now = time.time()

        if stype == "GGA":
            fix_quality = self._int(getattr(msg, "gps_qual", None))
            num_sats = self._int(getattr(msg, "num_sats", None))
            hdop = self._float(getattr(msg, "horizontal_dil", None))
            altitude = self._float(getattr(msg, "altitude", None))
            lat = self._dm_to_dd(getattr(msg, "lat", None), getattr(msg, "lat_dir", None))
            lon = self._dm_to_dd(getattr(msg, "lon", None), getattr(msg, "lon_dir", None))

            with self.lock:
                self.latest.update({
                    "timestamp": now,
                    "fix_quality": fix_quality,
                    "fix_quality_str": self._fix_quality_str(fix_quality),
                    "satellites": num_sats,
                    "hdop": hdop,
                    "altitude_m": altitude,
                    "latitude": lat,
                    "longitude": lon,
                    "has_lock": fix_quality is not None and fix_quality > 0 and lat is not None and lon is not None,
                })
                latest_copy = self.latest.copy()
            self._log(latest_copy)

        elif stype == "RMC":
            status = getattr(msg, "status", "")
            lat = self._dm_to_dd(getattr(msg, "lat", None), getattr(msg, "lat_dir", None))
            lon = self._dm_to_dd(getattr(msg, "lon", None), getattr(msg, "lon_dir", None))
            speed_knots = self._float(getattr(msg, "spd_over_grnd", None))
            course = self._float(getattr(msg, "true_course", None))

            with self.lock:
                update = {
                    "timestamp": now,
                    "status": status,
                    "latitude": lat,
                    "longitude": lon,
                    "speed_knots": speed_knots,
                    "speed_kmh": speed_knots * 1.852 if speed_knots is not None else None,
                    "course": course,
                    "has_lock": status == "A" and lat is not None and lon is not None,
                }
                if msg.datestamp:
                    update["datestamp"] = msg.datestamp.isoformat()
                if msg.timestamp:
                    date = msg.datestamp if msg.datestamp else datetime.now(timezone.utc).date()
                    update["timestamp_utc"] = datetime.combine(
                        date, msg.timestamp, tzinfo=timezone.utc
                    ).isoformat()
                self.latest.update(update)
                latest_copy = self.latest.copy()
            self._log(latest_copy)

        elif stype == "GSA":
            mode = getattr(msg, "mode_fix_type", "")
            pdop = self._float(getattr(msg, "pdop", None))
            hdop = self._float(getattr(msg, "hdop", None))
            vdop = self._float(getattr(msg, "vdop", None))
            with self.lock:
                self.latest.update({
                    "timestamp": now,
                    "fix_mode": mode,
                    "pdop": pdop,
                    "hdop": hdop,
                    "vdop": vdop,
                })
                latest_copy = self.latest.copy()
            self._log(latest_copy)

        elif stype == "GSV":
            in_view = self._int(getattr(msg, "num_sv_in_view", None))
            with self.lock:
                self.latest.update({
                    "timestamp": now,
                    "satellites_in_view": in_view,
                })
                latest_copy = self.latest.copy()
            self._log(latest_copy)

        elif stype == "VTG":
            true_track = self._float(getattr(msg, "true_track", None))
            mag_track = self._float(getattr(msg, "mag_track", None))
            speed_knots = self._float(getattr(msg, "spd_over_grnd_kts", None))
            speed_kmh = self._float(getattr(msg, "spd_over_grnd_kmh", None))
            with self.lock:
                self.latest.update({
                    "timestamp": now,
                    "course": true_track,
                    "course_magnetic": mag_track,
                    "speed_knots": speed_knots,
                    "speed_kmh": speed_kmh,
                })
                latest_copy = self.latest.copy()
            self._log(latest_copy)

    @staticmethod
    def _dm_to_dd(value, direction) -> float | None:
        """Convert NMEA DDMM.MMMM coordinate to signed decimal degrees."""
        if value is None or direction is None:
            return None
        try:
            value = float(value)
        except (ValueError, TypeError):
            return None
        degrees = int(value / 100)
        minutes = value - degrees * 100
        decimal = degrees + minutes / 60.0
        if direction in ("S", "W"):
            decimal = -decimal
        return decimal

    @staticmethod
    def _int(value) -> int | None:
        if value is None:
            return None
        try:
            return int(value)
        except (ValueError, TypeError):
            return None

    @staticmethod
    def _float(value) -> float | None:
        if value is None:
            return None
        try:
            return float(value)
        except (ValueError, TypeError):
            return None

    @staticmethod
    def _fix_quality_str(q: int | None) -> str:
        if q is None:
            return "Unknown"
        return {
            0: "No fix",
            1: "GPS fix",
            2: "DGPS fix",
            3: "PPS fix",
            4: "RTK fixed",
            5: "RTK float",
            6: "Dead reckoning",
        }.get(q, "Unknown")

    def _copy_latest(self) -> dict:
        with self.lock:
            return self.latest.copy()

    async def handle_telemetry(self, request: web.Request) -> web.Response:
        """Full GPS state for the telemetry widget."""
        return web.json_response(self._copy_latest())

    async def handle_track(self, request: web.Request) -> web.Response:
        """Minimal track update for the map widget."""
        data = self._copy_latest()
        track = {
            "lat": data.get("latitude"),
            "lon": data.get("longitude"),
            "heading": data.get("course"),
        }
        if track["lat"] is not None and track["lon"] is not None:
            track["target"] = {"lat": track["lat"], "lon": track["lon"]}
        return web.json_response(track)

    async def handle_health(self, request: web.Request) -> web.Response:
        data = self._copy_latest()
        return web.json_response({"ok": bool(data.get("has_lock"))})


async def main() -> None:
    parser = argparse.ArgumentParser(description="ATGM336H / NMEA GPS Eventide service")
    parser.add_argument("--port", type=str, required=True, help="Serial port device.")
    parser.add_argument("--baud", type=int, required=True, help="Serial baud rate.")
    parser.add_argument("--http-port", type=int, required=True, help="HTTP API port.")
    parser.add_argument("--recordings-dir", type=str, default=None)
    args = parser.parse_args()

    service = GPSService(args.port, args.baud, recordings_dir=args.recordings_dir)
    service._start_log()
    service.start()

    app = web.Application()
    app.router.add_get("/api/telemetry", service.handle_telemetry)
    app.router.add_get("/api/track", service.handle_track)
    app.router.add_get("/api/health", service.handle_health)

    runner = web.AppRunner(app)
    await runner.setup()
    site = web.TCPSite(runner, "0.0.0.0", args.http_port)
    await site.start()

    print(f"[gps] serial {args.port} @ {args.baud} baud", flush=True)
    print(f"[gps] HTTP API on 0.0.0.0:{args.http_port}", flush=True)

    shutdown = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, shutdown.set)

    try:
        await shutdown.wait()
    finally:
        service.stop()
        await runner.cleanup()


if __name__ == "__main__":
    asyncio.run(main())
