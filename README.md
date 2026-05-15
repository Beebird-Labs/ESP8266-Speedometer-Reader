[![PlatformIO CI & Release](https://github.com/cstoffel/ESP8266-Speedometer-Reader/actions/workflows/main.yml/badge.svg)](https://github.com/cstoffel/ESP8266-Speedometer-Reader/actions/workflows/main.yml)

# ESP8266 Speedometer Reader

This project uses an ESP8266 to read pulses from a speed sensor (such as a Vehicle Speed Sensor or wheel encoder), calculate the speed in MPH, and wirelessly transmit the data using the ESP-NOW protocol. This version specifically works for a 2000 Toyota Crown, but should work on other Toyotas of this vintage and should be easily adaptable to other manufacturers.

## Features

- **Interrupt-Driven Reading**: Accurately counts pulses and measures intervals using hardware interrupts (defaulting to Pin 5).
- **Advanced Filtering & Smoothing**:
  - **Physics-Based Glitch Filter**: Rejects impossible acceleration or deceleration (e.g., dropped or false double-pulses).
  - **Parametric Smoothing**: Applies an exponential moving average to stabilize speed readings.
- **Low Latency Wireless**: Uses ESP-NOW to broadcast the calculated speed to a receiver at 100ms intervals.
- **Binary Protocol**: Transmits speed as a compact packed struct rather than a formatted string for efficiency and precision.
- **Radio Watchdog**: Automatically reboots if no successful ESP-NOW send is confirmed within 10 seconds.
- **Built-in Test Mode**: Simulates speed profiles (acceleration/deceleration ramps) for testing the receiver without needing active sensor input.

## Hardware Requirements

- ESP8266 Development Board (e.g., Wemos D1 Mini)
- SPD signal connected to GPIO 5 (`D1` on most boards).
- A second ESP8266/ESP32 configured as an ESP-NOW receiver.

## Software Requirements

- PlatformIO (Recommended, as indicated by project configuration)
- ESP8266 Arduino Core

## Configuration

Before flashing the code to your ESP8266, make sure to update the following variables in [src/main.cpp](src/main.cpp) to match your specific setup:

### 1. ESP-NOW Receiver MAC Address

```cpp
static uint8_t receiver_mac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
```

**NOTE:** `{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF}` will broadcast to all receivers. If you want to target a single device, you must these values with the MAC addresss from that device.

### 2. Sensor Tuning

Adjust these macros and variables in `main.cpp` based on your specific vehicle or sensor:

- `K_SPEED_X10`: Precomputed speed conversion constant. Calculated as `10,000,000 / PULSE_FREQ_TO_MPH`. The default value of `8779631` corresponds to a `PULSE_FREQ_TO_MPH` of `1.139`. Adjust this for your vehicle's VSS output ratio.
- `SPEED_PIN`: The GPIO pin connected to your sensor (Default is `5`).
- `FILTER_WEIGHT`: Controls the amount of smoothing applied to the final output (Default `0.40f`).
- `SPEED_DEADZONE_US`: Hardware debounce duration in microseconds to prevent false triggers (Default `2000UL`).
- `OUTPUT_KPH`: Compile-time flag to select output unit. Set to `0` (default) for MPH or `1` for KPH.

## Usage

1. **Build and Upload**: Open the project in PlatformIO and upload it to your ESP8266.
2. **Monitor**: Open the Serial Monitor at `115200` baud.
3. **Test Mode**: To simulate speed data without a connected sensor, send a `t` or `T` via the Serial Monitor. This toggles the test mode on and off, cycling through predefined acceleration ramps and transmitting them over ESP-NOW.

## Protocol Payload

The data is transmitted via ESP-NOW as a packed binary struct (`SpeedPacket`):

```cpp
struct __attribute__((packed)) SpeedPacket {
  uint8_t  unit;  // 'M' for MPH, 'K' for KPH
  uint16_t speed; // speed in 0.1 unit increments (e.g. 657 = 65.7 MPH)
};
```

The total payload is **3 bytes**. On the receiver, cast the incoming data buffer directly to a `SpeedPacket*` to decode it:

```cpp
void on_receive(uint8_t *mac, uint8_t *data, uint8_t len) {
  if (len == sizeof(SpeedPacket)) {
    SpeedPacket *pkt = (SpeedPacket *)data;
    float speed = pkt->speed * 0.1f; // convert to float
    char unit = (char)pkt->unit;     // 'M' or 'K'
  }
}
```

## CI/CD

This project uses GitHub Actions for automated builds and releases:

- **On push to `main` or pull request**: Runs a full PlatformIO build to verify the firmware compiles successfully.
- **On tag push (`v*`)**: Builds production binaries and creates a GitHub Release with the compiled `firmware.bin` and `firmware.elf` artifacts attached.

To cut a release, push a version tag:

```sh
git tag v1.0.0
git push origin v1.0.0
```

## License

This project is licensed under the [GNU General Public License v3.0](LICENSE).
