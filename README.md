# ATmega328PB IMU Live Step Detection Dashboard

This project gives you a complete laptop-side front end + analysis pipeline and a board-side firmware implementation for ATmega328PB.

## What it does
- Pulls IMU samples from ATmega328PB over serial (`timestamp_ms,ax,ay,az`).
- Runs adaptive step detection in real-time.
- Streams live values to a browser dashboard.
- Shows live step count, magnitude, threshold, and noise sigma.
- Logs all incoming samples to `imu_stream.csv`.

## Architecture
- `backend/app.py`: FastAPI server + WebSocket broadcaster + serial reader.
- `backend/step_detection.py`: adaptive algorithm tuned for walking step events.
- `static/index.html`: live dashboard with Chart.js.
- `firmware/step_detector.[ch]`: C implementation for ATmega328PB firmware.
- `firmware/src/main.c`: deployable firmware (MPU6050 + UART stream + step prints).
- `docs/VSCode_ATmega328PB_Deployment.md`: end-to-end VS Code deployment instructions.
- `.vscode/tasks.json`: one-click build/flash/monitor/dashboard tasks.

## Quick start (host dashboard only)
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
uvicorn backend.app:app --reload
```
Open: <http://127.0.0.1:8000>

## Use with live ATmega328PB serial data
```bash
export IMU_SERIAL_PORT=/dev/ttyUSB0
uvicorn backend.app:app --reload
```

## VS Code board deployment
Follow: `docs/VSCode_ATmega328PB_Deployment.md`

Tasks available in VS Code:
- `avr:build`
- `avr:flash`
- `avr:monitor`
- `host:dashboard`

## ATmega serial output format
Your microcontroller should send one CSV line per sample:
```text
timestamp_ms,ax,ay,az
```
Example:
```text
215321,0.015,-0.023,1.068
```

When a step is found, firmware prints:
```text
STEP,215321,42,thr=0.307
```

## Algorithm summary
1. Compute acceleration magnitude `sqrt(ax^2+ay^2+az^2)`.
2. Low-pass filter to estimate gravity.
3. High-pass = magnitude - low-pass.
4. Dynamic threshold from rolling standard deviation of high-pass.
5. Declare step on upward threshold crossing, with refractory timer and peak validation.

Tune constants in both:
- `backend/step_detection.py`
- `firmware/step_detector.c`
