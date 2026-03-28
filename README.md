# YAM Teleop Viewer

Bidirectional teleop bridge for the [YAM](https://github.com/i2rt/i2rt) arm with a browser-based MuJoCo viewer (via [viser](https://github.com/nerfstudio-project/viser) + [mjviser](https://pypi.org/project/mjviser/)).

Mirror the real arm in sim, or control the real arm from sim sliders.

## Setup

```bash
git clone --recursive https://github.com/quantbagel/can-mac.git
cd can-mac
uv sync
```

### CAN bridge (requires a CANable 2.0 adapter)

```bash
cd can-bridge && cargo build --release
./target/release/can-bridge  # listens on /tmp/can0.sock
```

## Run

```bash
uv run teleop_viewer.py
# Open http://localhost:8080
```
