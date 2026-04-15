# VS Code Deployment Guide (ATmega328PB + IMU Step Counter)

This guide gets your board flashing + streaming IMU data to the web dashboard.

## 1) Prerequisites

Install these on your laptop:

- **VS Code**
- **Toolchain**: `avr-gcc`, `avr-libc`, `avrdude`
- **Python 3.10+**

### Linux example
```bash
sudo apt update
sudo apt install gcc-avr avr-libc avrdude python3-venv
```

### macOS (Homebrew)
```bash
brew install avr-gcc avrdude python
```

## 2) Wiring (MPU6050 to ATmega328PB)

- MPU6050 `VCC` -> `5V` (or 3.3V if your module requires)
- MPU6050 `GND` -> `GND`
- MPU6050 `SDA` -> `PC4` (SDA)
- MPU6050 `SCL` -> `PC5` (SCL)

## 3) Open project in VS Code

1. Open `/workspace/Testing` in VS Code.
2. Install recommended extensions when prompted.
3. Open **Terminal -> Run Task**.

## 4) Build and flash firmware

Run task:
- `avr:flash`

It will:
1. compile `firmware/src/main.c` + `firmware/step_detector.c`
2. generate `build/firmware.hex`
3. flash the board with `avrdude`

When prompted, enter your serial port:
- Linux: `/dev/ttyUSB0` or `/dev/ttyACM0`
- macOS: `/dev/cu.usbserial-*`
- Windows: `COM4` (example)

## 5) Check serial output

Run task:
- `avr:monitor`

Expected messages include:
- `BOOT,ATmega328PB step counter`
- IMU CSV lines: `timestamp_ms,ax,ay,az`
- Step events: `STEP,timestamp,count,thr=...`

## 6) Run live dashboard

In a Python venv:
```bash
python -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

Then run VS Code task:
- `host:dashboard`

Open <http://127.0.0.1:8000>.

The dashboard will now show:
- live magnitude
- adaptive threshold
- noise sigma
- step count + per-step event details

## 7) Tuning for your walking style

Start with constants in both files:
- `backend/step_detection.py`
- `firmware/step_detector.c`

Adjust in this order:
1. `MIN_THRESHOLD` / `min_threshold` to reject noise.
2. `THRESHOLD_SCALE` / `threshold_scale` to control sensitivity.
3. `REFRACTORY_MS` / `refractory_ms` to prevent double-counting.

## 8) Troubleshooting

- **No serial port visible**: install USB-UART drivers and replug board.
- **`avrdude` sync errors**: confirm programmer type/baud/port and bootloader.
- **No IMU data**: check SDA/SCL wiring and module address (`0x68`).
- **Counts too high**: raise threshold or refractory period.
- **Counts too low**: lower threshold scale or minimum threshold.
