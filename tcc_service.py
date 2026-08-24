#!/usr/bin/env python3
"""
tcc_service.py — Eventide service for the CMPS12 tilt-compensated compass.

Reads the sensor via I2C using tcc_i2c.py, exposes an HTTP API for the
dashboard's orientation3d and telemetry widgets, and streams line-delimited
JSON over the configured Unix domain socket.

Run directly for local testing:
    python3 tcc_service.py --http-port 8080 --unix-socket /tmp/tcc_stream.sock
"""

import argparse
import asyncio
import json
import os
import signal
import time
from datetime import datetime, timezone
from pathlib import Path

from aiohttp import web

from tcc_i2c import CMPS12


class CompassService:
    """Sensor reader + HTTP endpoints + Unix-socket streamer."""

    def __init__(
        self,
        bus_number: int,
        address_8bit: int,
        poll_interval: float,
        stream_interval: float,
        recordings_dir: str | None = None,
    ):
        self.bus_number = bus_number
        self.address_8bit = address_8bit
        self.poll_interval = poll_interval
        self.stream_interval = stream_interval
        self.recordings_dir = recordings_dir
        self.compass = CMPS12(bus_number=bus_number, address_8bit=address_8bit)
        self.latest: dict = {}
        self._stream_clients: set[asyncio.StreamWriter] = set()
        self._shutdown = asyncio.Event()
        self._poll_task: asyncio.Task | None = None
        self._stream_task: asyncio.Task | None = None
        self._log_file = None

    async def start(self) -> None:
        self._poll_task = asyncio.create_task(self._poll_loop())
        self._stream_task = asyncio.create_task(self._stream_loop())

    async def stop(self) -> None:
        self._shutdown.set()
        for task in (self._poll_task, self._stream_task):
            if task:
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass
        self.compass.close()
        if self._log_file:
            self._log_file.close()
            self._log_file = None

    def _start_log(self) -> None:
        """Open a UTC-timestamped JSONL recording file."""
        if not self.recordings_dir:
            return
        Path(self.recordings_dir).mkdir(parents=True, exist_ok=True)
        ts = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%SZ")
        log_path = Path(self.recordings_dir) / f"tcc_{ts}.jsonl"
        self._log_file = open(log_path, "a", buffering=1)
        print(f"[tcc] recording to {log_path}", flush=True)

    def _log(self, data: dict) -> None:
        """Append one timestamped JSON line to the recording file."""
        if not self._log_file:
            return
        entry = {"recorded_at_utc": datetime.now(timezone.utc).isoformat(), **data}
        self._log_file.write(json.dumps(entry, default=str) + "\n")

    def _read_sync(self) -> dict:
        return self.compass.read_all_fast()

    async def _poll_loop(self) -> None:
        """Poll the sensor in a thread pool and cache the latest reading."""
        loop = asyncio.get_running_loop()
        while not self._shutdown.is_set():
            try:
                data = await loop.run_in_executor(None, self._read_sync)
                ts = time.time()
                self.latest = {
                    "timestamp": ts,
                    "bearing": data["bearing"],
                    "pan": data["bearing"],
                    "pitch": data["pitch"],
                    "tilt": data["pitch"],
                    "roll": data["roll"],
                    "temperature": data["temperature"],
                    "mag_raw": {
                        "x": data["mag_raw"][0],
                        "y": data["mag_raw"][1],
                        "z": data["mag_raw"][2],
                    },
                    "accel_raw": {
                        "x": data["accel_raw"][0],
                        "y": data["accel_raw"][1],
                        "z": data["accel_raw"][2],
                    },
                    "gyro_raw": {
                        "x": data["gyro_raw"][0],
                        "y": data["gyro_raw"][1],
                        "z": data["gyro_raw"][2],
                    },
                    "bosch_bearing": data["bosch_bearing"],
                    "pitch_180": data["pitch_180"],
                    "calibration": data["calibration"],
                }
                self._log(self.latest)
            except Exception as exc:
                print(f"[tcc] poll error: {exc}", flush=True)

            try:
                await asyncio.wait_for(
                    self._shutdown.wait(), timeout=self.poll_interval
                )
            except asyncio.TimeoutError:
                pass

    async def _stream_loop(self) -> None:
        """Push the latest reading to every connected Unix-socket client."""
        while not self._shutdown.is_set():
            try:
                await asyncio.wait_for(
                    self._shutdown.wait(), timeout=self.stream_interval
                )
                return
            except asyncio.TimeoutError:
                pass

            if self.latest and self._stream_clients:
                await self._broadcast(self.latest)

    async def _broadcast(self, payload: dict) -> None:
        line = json.dumps(payload, default=str) + "\n"
        dead = set()
        for writer in list(self._stream_clients):
            try:
                writer.write(line.encode())
                await writer.drain()
            except Exception:
                dead.add(writer)
        for writer in dead:
            self._stream_clients.discard(writer)

    async def handle_stream(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        """Raw Unix-socket client handler; streams JSON lines."""
        self._stream_clients.add(writer)

        # Send the most recent sample immediately so the client isn't idle.
        if self.latest:
            try:
                line = json.dumps(self.latest, default=str) + "\n"
                writer.write(line.encode())
                await writer.drain()
            except Exception:
                self._stream_clients.discard(writer)
                writer.close()
                return

        try:
            # Wait for the client to close the connection or a shutdown signal.
            while not self._shutdown.is_set():
                data = await reader.read(1024)
                if not data:
                    break
        except asyncio.CancelledError:
            pass
        except Exception:
            pass
        finally:
            self._stream_clients.discard(writer)
            try:
                writer.close()
                await writer.wait_closed()
            except Exception:
                pass

    async def handle_telemetry(self, request: web.Request) -> web.Response:
        """Polled telemetry endpoint used by dashboard widgets."""
        return web.json_response(self.latest)

    async def handle_health(self, request: web.Request) -> web.Response:
        return web.json_response({"ok": bool(self.latest)})


async def main() -> None:
    parser = argparse.ArgumentParser(
        description="Tilt-compensated compass Eventide service"
    )
    parser.add_argument("--bus-number", type=int, default=1)
    parser.add_argument("--address", type=str, default="0xC0")
    parser.add_argument("--poll-interval", type=float, default=0.02)
    parser.add_argument("--stream-interval", type=float, default=0.1)
    parser.add_argument("--http-port", type=int, required=True)
    parser.add_argument("--unix-socket", type=str, required=True)
    parser.add_argument("--recordings-dir", type=str, default=None)
    args = parser.parse_args()

    address_8bit = int(args.address, 0)

    service = CompassService(
        bus_number=args.bus_number,
        address_8bit=address_8bit,
        poll_interval=args.poll_interval,
        stream_interval=args.stream_interval,
        recordings_dir=args.recordings_dir,
    )
    service._start_log()
    await service.start()

    app = web.Application()
    app.router.add_get("/api/telemetry", service.handle_telemetry)
    app.router.add_get("/api/health", service.handle_health)

    runner = web.AppRunner(app)
    await runner.setup()
    tcp_site = web.TCPSite(runner, "0.0.0.0", args.http_port)
    await tcp_site.start()

    sock_path = args.unix_socket
    Path(sock_path).parent.mkdir(parents=True, exist_ok=True)
    if os.path.exists(sock_path):
        os.unlink(sock_path)

    # Create the socket world-writable so any local consumer can connect.
    old_umask = os.umask(0o000)
    try:
        unix_server = await asyncio.start_unix_server(
            service.handle_stream, path=sock_path
        )
    finally:
        os.umask(old_umask)

    print(f"[tcc] HTTP API on 0.0.0.0:{args.http_port}", flush=True)
    print(f"[tcc] stream socket on {sock_path}", flush=True)

    loop = asyncio.get_running_loop()
    for sig in (signal.SIGTERM, signal.SIGINT):
        loop.add_signal_handler(sig, service._shutdown.set)

    try:
        await service._shutdown.wait()
    finally:
        unix_server.close()
        await unix_server.wait_closed()
        await runner.cleanup()
        await service.stop()


if __name__ == "__main__":
    asyncio.run(main())
