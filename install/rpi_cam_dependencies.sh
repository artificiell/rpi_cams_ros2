#!/bin/bash
set -e

echo "=== [1/9] Installing base build dependencies ==="
sudo systemctl --no-block restart netplan-wpa-wlan0.service ssh.service
sudo apt update
sudo apt install -y build-essential git meson ninja-build pkg-config python3-ply \
    libdrm-dev libudev-dev libjpeg-dev libtiff5-dev libpng-dev libboost-all-dev  \
    libavcodec-dev libavformat-dev libavdevice-dev libavfilter-dev libavutil-dev \
    libegl1-mesa-dev libgles2-mesa-dev libv4l-dev v4l-utils libcamera-dev ffmpeg \
    libexif-dev libcamera-tools ros-jazzy-cv-bridge ros-jazzy-camera-info-manager-py \
    ros-jazzy-image-geometry \
    python3-pybind11 pybind11-dev libcap-dev libopenexr-dev

echo "=== [2/9] Enabling camera auto-detect and overlay in config.txt ==="
CONFIG_FILE="/boot/firmware/config.txt"
OVERLAY="dtoverlay=imx219,cam0"

if grep -Fxq "camera_auto_detect=0" "$CONFIG_FILE"; then
    sudo sed -i 's/^camera_auto_detect=0/camera_auto_detect=1/' "$CONFIG_FILE"
    echo "Set camera_auto_detect=1 in $CONFIG_FILE"
elif ! grep -Fxq "camera_auto_detect=1" "$CONFIG_FILE"; then
    echo "camera_auto_detect=1" | sudo tee -a "$CONFIG_FILE"
    echo "Added camera_auto_detect=1 to $CONFIG_FILE"
else
    echo "camera_auto_detect=1 already present in $CONFIG_FILE"
fi

if ! grep -Fxq "$OVERLAY" "$CONFIG_FILE"; then
    echo "$OVERLAY" | sudo tee -a "$CONFIG_FILE"
    echo "Added $OVERLAY to $CONFIG_FILE"
else
    echo "Camera overlay already present in $CONFIG_FILE"
fi

echo "=== [3/9] Building libcamera from source (with Python bindings) ==="
mkdir -p ~/Downloads && cd ~/Downloads
if [ ! -d libcamera ]; then
    echo "Cloning libcamera..."
    git clone --recursive https://github.com/raspberrypi/libcamera.git
fi
cd libcamera
meson setup build --wipe -Dpycamera=enabled

if ! meson configure build | grep -q "pycamera.*enabled"; then
    echo "ERROR: pycamera bindings did not enable. Check that pybind11 is installed correctly."
    exit 1
fi

ninja -C build
sudo ninja -C build install
sudo ldconfig

echo "=== [4/9] Fixing Python path for libcamera bindings ==="
# libcamera's meson install places the Python bindings under the unversioned
# python3/dist-packages path, which is not on Python 3.12's default sys.path.
PYCAMERA_SO=$(sudo find /usr/local -iname "_libcamera*.so" -path "*dist-packages*" | head -n1)
if [ -z "$PYCAMERA_SO" ]; then
    echo "ERROR: could not find built libcamera Python bindings under /usr/local."
    exit 1
fi
PYCAMERA_DIR=$(dirname "$(dirname "$PYCAMERA_SO")")
echo "Found libcamera Python bindings dir: $PYCAMERA_DIR"

PYTHON_VER="python3.$(python3 -c 'import sys; print(sys.version_info[1])')"
PTH_FILE="/usr/lib/${PYTHON_VER}/dist-packages/local.pth"
if [ ! -f "$PTH_FILE" ] || ! grep -Fxq "$PYCAMERA_DIR" "$PTH_FILE" 2>/dev/null; then
    echo "$PYCAMERA_DIR" | sudo tee "$PTH_FILE" > /dev/null
    echo "Wrote $PTH_FILE -> $PYCAMERA_DIR"
else
    echo "$PTH_FILE already points at $PYCAMERA_DIR"
fi

if ! env -u PYTHONPATH python3 -c "import libcamera" 2>/dev/null; then
    echo "ERROR: 'import libcamera' still fails after path fix. Aborting."
    exit 1
fi
echo "libcamera Python bindings import OK."

echo "=== [5/9] Building rpicam-apps from source ==="
cd ~/Downloads
if [ ! -d rpicam-apps ]; then
    echo "Cloning rpicam-apps..."
    git clone https://github.com/raspberrypi/rpicam-apps.git
fi
cd rpicam-apps
meson setup build --wipe
ninja -C build
sudo ninja -C build install
sudo ldconfig

echo "=== [6/9] Installing picamera2 (without pip dependency resolution) ==="
pip install --break-system-packages --no-deps --upgrade picamera2

echo "=== [7/9] Installing picamera2 runtime dependencies ==="
# picamera2's module chain hard-imports these at load time (not lazily),
# so all are required just to 'import picamera2', regardless of which
# picamera2 features are actually used.
sudo -H pip3 install --break-system-packages \
    videodev2 python-prctl piexif tqdm libarchive-c simplejpeg av pidng OpenEXR

echo "=== [8/9] Stubbing out pykms (kms++ bindings, not built on this system) ==="
# pykms is only needed for DrmPreview (live display preview window), which a
# headless service never uses. drm_preview.py is imported unconditionally by
# picamera2 and reads PixelFormat.* constants at class-definition time, so
# the stub must tolerate arbitrary attribute access, not just raise on import.
STUB_DIR="/usr/local/lib/python3/dist-packages/pykms"
sudo mkdir -p "$STUB_DIR"
sudo tee "$STUB_DIR/__init__.py" > /dev/null << 'EOF'
# Stub: kms++ (pykms) is not built on this system. This system runs headless
# and never calls start_preview(Preview.DRM), so DrmPreview is imported but
# never instantiated. This stub exists only to satisfy import-time and
# class-definition-time attribute access (e.g. PixelFormat.RGB888 constants).
class _Unavailable:
    def __getattr__(self, name):
        # Attribute access (e.g. PixelFormat.RGB888) returns another stub,
        # so class-body constant lookups don't blow up at import time.
        return _Unavailable()

    def __call__(self, *a, **kw):
        raise RuntimeError(
            "pykms is a stub — DRM preview is not available on this system."
        )

    def __repr__(self):
        return "<pykms stub — unavailable>"


def __getattr__(name):
    return _Unavailable()
EOF

echo "=== [9/9] Verifying full stack ==="
if ! python3 -c "from picamera2 import Picamera2; print('picamera2 import OK')"; then
    echo "ERROR: picamera2 import failed. See traceback above."
    exit 1
fi

python3 - << 'EOF'
from picamera2 import Picamera2
picam2 = Picamera2()
config = picam2.create_video_configuration(main={'size': (640, 480), 'format': 'BGR888'})
picam2.configure(config)
picam2.start()
arr = picam2.capture_array()
assert arr.shape == (480, 640, 3), f"Unexpected frame shape: {arr.shape}"
print(f"Camera capture OK: shape={arr.shape}, dtype={arr.dtype}")
picam2.stop()
EOF

echo ""
echo "Installation complete!"
echo "If this was the first run (config.txt was just changed), reboot now:"
echo "  sudo reboot"
