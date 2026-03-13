# rpi_cams_ros2

ROS 2 package for capturing and processing video streams from the Raspberry Pi Camera Module (v2) and ArduCam Time-of-Flight camera. This package includes:

* A camera driver node using `rpicam-vid` to capture and publish image and camera info messages.
* An ASCII image viewer node for terminal-based video debugging.
* An image-to-laserscan node for emulating 2D laser scan messages from camera images.

---

## Prerequisites

* Ubuntu 24.04+ (64-bit)
* Raspberry Pi Camera Module v2 (tested with IMX219)
* `rpicam-apps` and `libcamera` from the official Raspberry Pi GitHub:
  * [rpicam-apps](https://github.com/raspberrypi/rpicam-apps)
  * [libcamera](https://github.com/raspberrypi/libcamera)
* ROS 2 (Humble or later)

---

## Installation

1. Clone the repository into your ROS 2 workspace:

   ```bash
   cd ~/ros2_ws/src/
   git clone https://github.com/artificiell/rpi_cam_ros2.git
   ```
   
2. Install ststem dependencies (_note, this will take some time!_):

   ```bash
   cd ~/ros2_ws/src/rpi_cam_ros2/scripts/
   ./install_camera_dependencies.sh
   ```

3. Build the workspace:

   ```bash
   cd ~/ros2_ws/
   colcon build
   ```

4. Source the workspace:

   ```bash
   source install/setup.bash
   ```

---

If encounter error:

   ```bash
   In file included from ../encoder/libav_encoder.cpp:20:
   ../encoder/libav_encoder.hpp:33:2: error: #error "Error: libavcodec API version is too old for the libav encoder!"
      33 | #error "Error: libavcodec API version is too old for the libav encoder!"
         |  ^~~~~
   ```

Go to the installation folder and edite file `libav_encoder.hpp`: 

   ```bash
   cd ~/Downloads/rpicam-apps
   emacs encoder/libav_encoder.hpp
   ```
Change the lines:
 
   ```bash
   #if LIBAVCODEC_VERSION_MAJOR < 61
   #error "Error: libavcodec API version is too old for the libav encoder!"
   #endif
   ```

 ...to reduce the `LIBAVCODEC_VERSION_MAJOR` dependecy, e.g.,: 

   ```bash
   #if LIBAVCODEC_VERSION_MAJOR < 60
   #error "Error: libavcodec API version is too old for the libav encoder!"
   #endif
   ```

## Nodes

### 📷 RPi Camera Sensor (`camera.py`)

Captures video frames using `rpicam-vid`, publishes captured frames as `sensor_msgs/msg/Image`, and loads optional calibration data.

#### Published Topics

* **`camera/image`** (`sensor_msgs/msg/Image`): Image frames.
* **`camera/info`** (`sensor_msgs/msg/CameraInfo`): Camera intrinsic parameters.

#### Parameters

* **`resolution`** (string, default: `"VGA"`): Supported options: `QVGA`, `VGA`, `180p`, `360p`.
* **`framerate`** (int, default: `30`): Frames per second.
* **`codec`** (string, default: `"mjpeg"`): Currently, only MJPEG is supported.
* **`flip`** (bool, default: `True`): Whether to vertically and horizontally flip the image.
* **`profile`** (string, default: `"best_effort"`): QoS profile: `best_effort` or `reliable`.

#### Calibration

The node attempts to load a YAML calibration file from the `config/` directory matching the selected resolution:

* `calibration_qvga.yaml`
* `calibration_vga.yaml`
* `calibration_180p.yaml`
* `calibration_360p.yaml`

If not found, default dummy parameters are used.

---

### 🖼 ASCII Image Viewer (`ascii_viewer.py`)

Subscribes to `camera/image` and displays grayscale images in terminal, using ASCII characters. TrueColor is supported in compatible terminals.

#### Subscribed Topics

* **`camera/image`** (`sensor_msgs/msg/Image`)

#### Parameters

* **`width`** (int, default: `80`): Width of the ASCII output in characters.

---

### 🧭 Image-to-LaserScan (`image_to_laserscan.py`)

Converts the camera view into a 1D `sensor_msgs/msg/LaserScan` by estimating floor contact points.

#### Subscribed Topics

* **`camera/image`** (`sensor_msgs/msg/Image`)
* **`camera/info`** (`sensor_msgs/msg/CameraInfo`)

#### Published Topics

* **`scan`** (`sensor_msgs/msg/LaserScan`): Simulated scan using perspective geometry.

#### Parameters

* **`height`** (float, default: `0.08`): Camera height above ground in meters.

---

## Launch Files

### `camera.launch.py`

Launches the `rpi_camera_sensor` node with optional arguments for resolution, frame rate, image flip, and namespace.

```bash
ros2 launch rpi_cam_ros2 camera.launch.py
```

#### Launch Arguments

* **`robot_ns`** (string, default: `"rp0"`): Namespace to apply to node and topics.
* **`resolution`** (string, default: `"VGA"`): Camera resolution.
* **`frame_rate`** (int, default: `15`): Frame rate in Hz.
* **`flip_image`** (bool, default: `True`): Flip vertically.

---

### `camera_laserscan.launch.py`

Launches both the camera and the image-to-laserscan node, emulating a downward 2D laser scan.

```bash
ros2 launch rpi_cam_ros2 camera_laserscan.launch.py
```

#### Launch Arguments

Same as above, plus:

* **`camera_height`** (float, default: `0.08`): Height of the camera in meters.

---

## Example Usage

**Run camera node only:**

```bash
ros2 launch rpi_cam_ros2 camera.launch.py resolution:=QVGA frame_rate:=15 robot_ns:=rp0
```

**Run camera + laserscan node:**

```bash
ros2 launch rpi_cam_ros2 camera_laserscan.launch.py camera_height:=0.1 resolution:=180p
```

**Visualize camera stream:**

```bash
rqt_image_view /rp0/camera/image
```

**View ASCII camera stream in terminal:**

```bash
ros2 run rpi_cam_ros2 ascii_viewer --ros-args -p width:=100
```

**View laser scan output:**

```bash
ros2 topic echo /rp0/scan
```

---

## Troubleshooting

* **Camera doesn't start:** Confirm `rpicam-hello` works independently.
* **No image topic:** Ensure the codec is supported (e.g., `mjpeg`) and the resolution is valid.
* **Calibration not found:** Add a YAML file to `config/` with correct camera intrinsics.
* **LaserScan is empty or noisy:** Make sure the camera is facing straight ahead and height is correctly set.

---

## License

This software is released under the MIT License. See the [LICENSE](LICENSE) file for more details.

---
