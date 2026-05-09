# ESP8266 Speedometer Reader

This project uses an ESP8266 to read pulses from a speed sensor (such as a Vehicle Speed Sensor or wheel encoder), calculate the speed in MPH, and wirelessly transmit the data using the ESP-NOW protocol. This version specifically works for a 2000 Toyota Crown, but should work on other Toyotas of this vintage and should be easily adaptable to other manufacturers.

## Features

- **Interrupt-Driven Reading**: Accurately counts pulses and measures intervals using hardware interrupts (defaulting to Pin 5).
- **Advanced Filtering & Smoothing**:
  - **Physics-Based Glitch Filter**: Rejects impossible acceleration or deceleration (e.g., dropped or false double-pulses).
  - **Parametric Smoothing**: Applies an exponential moving average to stabilize speed readings.
- **Low Latency Wireless**: Uses ESP-NOW to broadcast the calculated speed to a receiver at 100ms intervals.
- **Built-in Test Mode**: Simulates speed profiles (acceleration/deceleration ramps) for testing the receiver without needing active sensor input.

## Hardware Requirements

- ESP8266 Development Board (e.g., NodeMCU, Wemos D1 Mini)
- SPD signal connected to GPIO 5 (`D1` on most boards).
- A second ESP8266/ESP32 configured as an ESP-NOW receiver.

## Software Requirements

- PlatformIO (Recommended, as indicated by project configuration)
- ESP8266 Arduino Core

## Configuration

Before flashing the code to your ESP8266, make sure to update the following variables in `src/main.cpp` to match your specific setup:

### 1. ESP-NOW Receiver MAC Address

You **must** update the `receiver_mac` array with the MAC address of the device receiving the speed data:

```cpp
static uint8_t receiver_mac = {0x98, 0x88, 0xE0, 0x76, 0x93, 0xEC};
```

### 2. Sensor Tuning

Adjust these macros and variables in `main.cpp` based on your specific vehicle or sensor:

- `PULSE_FREQ_TO_MPH`: The conversion factor from pulse frequency (Hz) to MPH.
- `SPEED_PIN`: The GPIO pin connected to your sensor (Default is `5`).
- `FILTER_WEIGHT`: Controls the amount of smoothing applied to the final output (Default `0.40f`).
- `SPEED_DEADZONE_US`: Hardware debounce duration in microseconds to prevent false triggers (Default `2000UL`).
- `s_output_kph`: A boolean flag to control the output unit. Set to `false` (default) for MPH or `true` for KPH.

## Usage

1. **Build and Upload**: Open the project in PlatformIO and upload it to your ESP8266.
2. **Monitor**: Open the Serial Monitor at `115200` baud.
3. **Test Mode**: To simulate speed data without a connected sensor, send a `t` or `T` via the Serial Monitor. This toggles the test mode on and off, cycling through predefined acceleration ramps and transmitting them over ESP-NOW.

## Protocol Payload

The data is transmitted via ESP-NOW as a standard formatted string:

- **MPH:** `"SP,M,<speed>"` (e.g., `"SP,M,55"`)
- **KPH:** `"SP,K,<speed>"` (e.g., `"SP,K,88"`)

Ensure your receiver reads incoming bytes as a string/character array to parse the payload properly.

## License

This project is provided as-is. Feel free to modify and adapt it for your specific speed-reading needs.
