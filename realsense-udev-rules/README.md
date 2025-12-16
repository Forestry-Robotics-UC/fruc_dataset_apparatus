# RealSense Udev Rules Installer for Steam Deck

This script installs the official Intel RealSense udev rules on the Steam Deck, enabling the IMU functionality of RealSense cameras. Without these rules, the IMU will not work.

---

## **Prerequisites**
- `wget` installed
- `sudo` access
- The user needs to be in the `plugdev` group for the udev rules to work. For this, you need to:
  - Create the group if it does not exist yet:
    ```bash
    sudo groupadd plugdev
    ```
  - Add the group to the `deck` user:
    ```bash
    sudo usermod -aG plugdev deck
    ```
  *Note: You may need to log out and back in for the group changes to take effect.*

---

## **Script Workflow**

### 1. **Disable SteamOS Read-Only Mode**
The Steam Deck restricts modifications to system partitions by default. To install the udev rules, you must temporarily disable read-only mode:
```bash
sudo steamos-readonly disable
```
*Note: This does not affect user data or system stability.*

### 2. **Download Udev Rules**
The script downloads, using wget, the following udev rules from the [official librealsense repository](https://github.com/IntelRealSense/librealsense):
- [`99-realsense-libusb.rules`](https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-libusb.rules)
- [`99-realsense-d4xx-mipi-dfu.rules`](https://raw.githubusercontent.com/IntelRealSense/librealsense/master/config/99-realsense-d4xx-mipi-dfu.rules)

These files are downloaded to a temporary config folder within the script directory and later removed.

### 3. **Install Rules**
The script copies the downloaded rules to the system udev directory:
```bash
/etc/udev/rules.d/
```

### 4. **Reload Udev Rules**
After installation, the script restarts the `udevadm`(udev management tool) service to apply the new rules:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5. **Re-enable SteamOS Read-Only Mode**
For security, the script re-enables read-only mode after installation:
```bash
sudo steamos-readonly enable
```
