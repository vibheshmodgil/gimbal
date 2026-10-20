# Running on a Jetson Orin Nano (JetPack 6)

The Jetson replaces the Windows PC: it runs the tracker (YOLO11n-pose on the Jetson GPU) and, optionally, the
device service. The gimbal (ESP32) stays as it is, on the same Wi-Fi.

## 1. Get the code

```bash
git clone https://github.com/<you>/anvyxo-gimbal.git ~/anvyxo-gimbal
cd ~/anvyxo-gimbal
```

The model (`Tracker/yolo11n-pose.pt`) is not in git; it downloads automatically on the first run.

## 2a. Run the tracker in Docker (recommended)

Ultralytics' JetPack 6 image already has a CUDA build of PyTorch, so there is nothing to compile.

```bash
sudo docker run -it --rm --ipc=host --runtime=nvidia --network host \
  -v ~/anvyxo-gimbal:/app -w /app ultralytics/ultralytics:latest-jetson-jetpack6 \
  bash -c "pip install -q flask websocket-client && python Tracker/server.py --gimbal 192.168.1.6"
```

- Use the gimbal's **IP**, not `anvyxo.local`: the container has no mDNS. The IP is shown at the bottom of the OLED.
- The startup line `person detector on Orin` means the GPU is used. If it says `CPU`, the container was started without `--runtime=nvidia`.

## 2b. Or install natively

Plain `pip install ultralytics` installs a CPU-only PyTorch on ARM. Install NVIDIA's PyTorch and torchvision
wheels for JetPack 6 first, following the Ultralytics Jetson guide
(https://docs.ultralytics.com/guides/nvidia-jetson/). Then:

```bash
pip install -r Tracker/requirements.txt
python Tracker/server.py --selftest      # logic checks, no hardware
python Tracker/server.py --gimbal 192.168.1.6
```

## 3. Open the UI from your PC

The tracker listens on localhost only, because it has no login and can move the gimbal and fire the laser.
From the PC:

```bash
ssh -L 8000:localhost:8000 <user>@<jetson-ip>
```

Then open http://localhost:8000 on the PC.

On a trusted network you can skip the tunnel and start the tracker with `--host 0.0.0.0`, then open
`http://<jetson-ip>:8000`. Anyone on that Wi-Fi can then control the gimbal.

## 4. Device service (optional)

```bash
pip install -r host/requirements.txt
cd host && GIMBAL_HOST=192.168.1.6 GIMBAL_TOKEN=change-me python -m gimbal_service.api   # see host/README.md
```

## Retune after moving

- `latency` in `Tracker/params.json` (0.31 s) was measured with a GTX 1650. Detection time on the Orin differs, so retune it in the Tuning tab if the gimbal lags or overshoots.
- The brownout limits still apply: they come from the servo power supply, not from the computer.
