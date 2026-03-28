# YAM Teleop Viewer

Bidirectional teleop bridge for the [YAM](https://github.com/i2rt/i2rt) arm with a browser-based MuJoCo viewer (via [viser](https://github.com/nerfstudio-project/viser) + [mjviser](https://pypi.org/project/mjviser/)).

Mirror the real arm in sim, or control the real arm from sim sliders.

## Setup

```bash
# CAN bridge (requires a CANable 2.0 adapter)
cd can-bridge && cargo build --release
./target/release/can-bridge  # listens on /tmp/can0.sock

# Python deps
uv pip install mujoco numpy viser mjviser
uv pip install -e i2rt
```

## Run

```bash
python teleop_viewer.py
# Open http://localhost:8080
```
