# Serial Communication Library

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-orange) ![License: MIT](https://img.shields.io/badge/License-MIT-yellow)

This is a ROS 2 port of [wjwwood/serial](https://github.com/wjwwood/serial), a cross-platform C++ library for interfacing with RS-232 serial ports. It provides a modern C++ interface designed to look and feel like PySerial, built with `ament_cmake` and targeting **ROS 2 Jazzy on Linux**.

`Serial` is a class that provides the basic interface common to serial libraries (`open`, `close`, `read`, `write`, etc.) and requires no extra dependencies. It also provides tight control over timeouts and handshaking lines.

This fork extends the upstream library with a sysfs-based **device discovery API** for finding serial and UVC video devices by USB PID:VID and serial number, along with an `autofind_devices` ROS 2 node that uses that API at startup.

> **Upstream references** (original wjwwood library, ROS 1 / catkin):
> - Website: http://wjwwood.github.com/serial/
> - API docs: http://wjwwood.github.com/serial/doc/1.1.0/index.html

---

### Dependencies

Required:
* [ament_cmake](https://docs.ros.org/en/jazzy/p/ament_cmake/) — ROS 2 CMake build system
* [rclcpp](https://docs.ros.org/en/jazzy/p/rclcpp/) — ROS 2 C++ client library (for `autofind_devices` node)
* [cmake](http://www.cmake.org) ≥ 3.5

Optional (for tests):
* [Boost](http://www.boost.org/) — Boost C++ libraries

Optional (for documentation):
* [Doxygen](http://www.doxygen.org/) — Documentation generation tool
* [graphviz](http://www.graphviz.org/) — Graph visualization software

---

### Build & Install

Clone into a colcon workspace and build:

```bash
cd ~/your_ws/src
git clone <this-repo-url> serial-ros2
cd ..
colcon build --packages-select serial
```

Source the workspace:

```bash
. install/setup.bash
```

---

### Identifying USB Devices on Linux

Before using the discovery API or the `autofind_devices` node you need to know the PID:VID and serial number of your device. The examples below are from a working example using an [Arducam B0322](https://www.google.com/url?sa=t&source=web&rct=j&opi=89978449&url=https://www.arducam.com/arducam-2mp-global-shutter-usb-camera-board-for-computer-50fps-ov2311-monochrome-uvc-webcam-module-with-low-distortion-m12-lens-without-microphones-compatible-with-windows-linux-android-and-mac-os.html). The commands below require the `usbutils` package:

```bash
sudo apt install usbutils
```

#### Find PID:VID for all connected USB devices

```bash
lsusb
```

Example output (the `ID` field is `VID:PID`):

```
Bus 001 Device 003: ID 0c45:6366 Microdia Webcam
Bus 001 Device 004: ID 16c0:0483 Van Ooijen Technische Informatica Teensyduino
```

#### Find the iSerial (USB serial number) for a specific device

```bash
sudo lsusb -v -d 0c45:6366 | grep iSerial
```

Example output:

```
  iSerial                 3 UC621
```

To see all identifiers for every connected device at once:

```bash
sudo lsusb -v 2>/dev/null | grep -E "idVendor|idProduct|iSerial"
```

#### Alternative: query a specific `/dev` node with `udevadm`

`udevadm` is part of `systemd` (pre-installed on Ubuntu) and works for both serial and video devices without requiring root:

```bash
# For a serial device
udevadm info /dev/ttyACM0 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT"

# For a video device
udevadm info /dev/video4 | grep -E "ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT"
```

| `udevadm` field | Equivalent |
|-----------------|------------|
| `ID_VENDOR_ID` | VID (first half of PID:VID) |
| `ID_MODEL_ID` | PID (second half of PID:VID) |
| `ID_SERIAL_SHORT` | iSerial / USB serial number |

---

### Device Discovery API

All functions live in the `serial::` namespace and are declared in `include/serial/serial.h`. They use Linux sysfs and the `list_ports()` enumeration to locate devices without requiring any device to be opened.

---

#### `findSerialMultipleSerialDevicePathsByPIDVID`

Finds all serial port paths whose `hardware_id` contains the given `PID:VID` string. If no serial ports match, automatically falls back to scanning sysfs for UVC `/dev/videoX` nodes with the same PID:VID.

```cpp
std::vector<std::string> paths =
    serial::findSerialMultipleSerialDevicePathsByPIDVID("16C0:0483");
// e.g. {"/dev/ttyACM0", "/dev/ttyACM1"}
```

Throws `std::runtime_error` if neither serial nor video devices are found.

---

#### `findSerialDevicePathByPIDVID`

Convenience wrapper around `findSerialMultipleSerialDevicePathsByPIDVID` that returns only the first match.

```cpp
std::string path = serial::findSerialDevicePathByPIDVID("16C0:0483");
// e.g. "/dev/ttyACM0"
```

Throws `std::runtime_error` if no matching device is found.

---

#### `findStringCharacteristicInDevicePath`

Checks whether a substring appears in the `hardware_id` of a serial device at the given path. For `/dev/videoX` paths the check is performed against the USB serial number read from sysfs instead.

```cpp
bool match = serial::findStringCharacteristicInDevicePath("/dev/ttyACM0", "SER001");
// true if "SER001" appears in hardware_id
```

---

#### `findSubstringInDevicesByPIDVID`

Finds all devices matching a PID:VID and checks each one for a substring in its `hardware_id` (or USB serial for video devices). Returns a `std::map<path, bool>`.

```cpp
// All matching devices (true = substring found, false = not found)
auto results = serial::findSubstringInDevicesByPIDVID("16C0:0483", "SER001");

// Only the devices where the substring was found
auto matched = serial::findSubstringInDevicesByPIDVID("16C0:0483", "SER001",
                                                       /*only_return_true_paths=*/true);
```

Throws `std::runtime_error` if no device matches the PID:VID.

---

#### `findVideoDevicesByPIDVID`

Walks `/sys/bus/usb/devices` to find all UVC video nodes for a given PID:VID. Returns only the lowest-numbered `/dev/videoX` node per USB interface (the capture node).

```cpp
std::vector<std::string> video_paths =
    serial::findVideoDevicesByPIDVID("0c45:6366");
// e.g. {"/dev/video4"}
```

Throws `std::runtime_error` if the PID:VID string is malformed.

---

#### `getUSBSerialForVideoDevice`

Reads the USB serial number for a `/dev/videoX` device by resolving its sysfs path and walking up the tree until a `serial` file is found.

```cpp
std::string usb_serial = serial::getUSBSerialForVideoDevice("/dev/video4");
// e.g. "UC621"
```

Returns an empty string if the serial number cannot be determined.

---

### `autofind_devices` ROS 2 Node

The `autofind_devices` node runs at startup, resolves up to two devices that share the same PID:VID by their individual USB serial numbers, and logs the resulting path-to-serial-number mapping. It exits after the mapping step.

**Parameters**

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `pidvid` | `string` | Yes | USB PID:VID to search for, e.g. `"0c45:6366"` |
| `serial_number_1` | `string` | No | Serial number substring to match against device 1 |
| `serial_number_2` | `string` | No | Serial number substring to match against device 2 |

**Usage**

```bash
ros2 run serial autofind_devices \
  --ros-args \
  -p pidvid:=0c45:6366 \
  -p serial_number_1:='"UC621"'
```

> **Note on quoting:** ROS 2 parameter type inference will treat a bare value like `UC621` as a string, but numeric-looking strings (e.g. `"123456"`) must be double-quoted inside the shell argument (`'"123456"'`) to prevent them from being coerced to integers.

**Behaviour**

- If only one device is found the node reports whether it matches either serial number.
- If two devices are found the node attempts to align each serial number to a distinct path and warns if both match the same device.
- If more than two devices are found the node lists all paths and flags any that could not be identified.
- If `pidvid` is empty the node exits with an error.

---

### License

The MIT License

Copyright (c) 2012 William Woodall, John Harrison

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

### Authors

William Woodall <wjwwood@gmail.com>
John Harrison <ash.gti@gmail.com>

This fork adds ROS 2 Jazzy support, `ament_cmake` build integration, sysfs-based multi-bus device discovery, and the `autofind_devices` node.

### Contact

William Woodall <william@osrfoundation.org>
