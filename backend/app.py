from __future__ import annotations

import asyncio
import csv
import json
import random
import time
from pathlib import Path
from typing import AsyncIterator

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles

from step_detection import StepDetector

try:
    import serial
except Exception:  # pyserial optional for simulation mode
    serial = None

ROOT = Path(__file__).resolve().parent.parent
STATIC = ROOT / "static"
LOG_FILE = ROOT / "imu_stream.csv"

app = FastAPI(title="ATmega328PB IMU Step Analytics")
app.mount("/static", StaticFiles(directory=STATIC), name="static")


class IMUStream:
    def __init__(self) -> None:
        self.detector = StepDetector()
        self.clients: set[WebSocket] = set()

    async def connect(self, ws: WebSocket) -> None:
        await ws.accept()
        self.clients.add(ws)

    def disconnect(self, ws: WebSocket) -> None:
        self.clients.discard(ws)

    async def publish(self, payload: dict) -> None:
        stale: list[WebSocket] = []
        for ws in self.clients:
            try:
                await ws.send_text(json.dumps(payload))
            except Exception:
                stale.append(ws)
        for ws in stale:
            self.disconnect(ws)

    async def run(self, serial_port: str | None, baudrate: int = 115200) -> None:
        LOG_FILE.parent.mkdir(parents=True, exist_ok=True)
        with LOG_FILE.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp_ms", "ax", "ay", "az", "steps", "threshold"])

            async for sample in stream_imu(serial_port=serial_port, baudrate=baudrate):
                timestamp_ms = sample["timestamp_ms"]
                ax, ay, az = sample["ax"], sample["ay"], sample["az"]

                event = self.detector.update(timestamp_ms, ax, ay, az)
                state = self.detector.debug_state()
                writer.writerow([
                    timestamp_ms,
                    f"{ax:.4f}",
                    f"{ay:.4f}",
                    f"{az:.4f}",
                    self.detector.step_count,
                    f"{state['threshold']:.4f}",
                ])
                f.flush()

                payload = {
                    "type": "imu",
                    "timestamp_ms": timestamp_ms,
                    "ax": ax,
                    "ay": ay,
                    "az": az,
                    "magnitude": (ax * ax + ay * ay + az * az) ** 0.5,
                    "threshold": state["threshold"],
                    "sigma_hp": state["sigma_hp"],
                    "steps": self.detector.step_count,
                    "step_event": bool(event),
                }
                if event:
                    payload["event"] = {
                        "peak": event.peak,
                        "threshold": event.threshold,
                    }

                await self.publish(payload)


imu_stream = IMUStream()


async def stream_imu(serial_port: str | None, baudrate: int) -> AsyncIterator[dict]:
    if serial_port and serial is not None:
        with serial.Serial(serial_port, baudrate=baudrate, timeout=1) as ser:
            while True:
                line = ser.readline().decode(errors="ignore").strip()
                # Expected format: timestamp_ms,ax,ay,az
                try:
                    ts, ax, ay, az = line.split(",")
                    yield {
                        "timestamp_ms": int(ts),
                        "ax": float(ax),
                        "ay": float(ay),
                        "az": float(az),
                    }
                except ValueError:
                    await asyncio.sleep(0)
    else:
        # Simulation mode, for front-end development before hardware is plugged in.
        t = 0
        base = 50  # Hz
        while True:
            phase = (t / base) % 1.0
            burst = 0.0
            if 0.10 < phase < 0.17:
                burst = 1.25 * (1 - abs(phase - 0.135) / 0.035)

            ax = 0.02 * random.uniform(-1, 1) + 0.05
            ay = 0.03 * random.uniform(-1, 1)
            az = 1.0 + burst + 0.04 * random.uniform(-1, 1)

            yield {
                "timestamp_ms": int(time.time() * 1000),
                "ax": ax,
                "ay": ay,
                "az": az,
            }
            t += 1
            await asyncio.sleep(1 / base)


@app.get("/")
async def index() -> FileResponse:
    return FileResponse(STATIC / "index.html")


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket) -> None:
    await imu_stream.connect(ws)
    try:
        while True:
            _ = await ws.receive_text()
    except WebSocketDisconnect:
        imu_stream.disconnect(ws)


@app.on_event("startup")
async def startup() -> None:
    serial_port = None
    # Set IMU_SERIAL_PORT for live ATmega feed, e.g. /dev/ttyUSB0
    import os

    serial_port = os.getenv("IMU_SERIAL_PORT")
    asyncio.create_task(imu_stream.run(serial_port=serial_port))
