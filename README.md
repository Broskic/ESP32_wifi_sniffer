# ESP32 Wi-Fi Sniffer

## Overview
This project implements an ESP32 in promiscuous mode to sniff Wi-Fi packets, extract MAC addresses from the sniffed packets, count unique devices, and display the count of unique devices every 10 seconds. It also filters out static or non-moving devices that haven't been seen for more than a specified timeout period.

## Requirements
- ESP32 Development Board
- Arduino IDE (or other suitable development environment for ESP32)
- USB cable to connect the ESP32 to your computer

## Libraries
This project uses the following libraries:
- `freertos/FreeRTOS.h`
- `freertos/task.h`
- `esp_wifi.h`
- `esp_wifi_types.h`
- `esp_system.h`
- `esp_event.h`
- `nvs_flash.h`
- `driver/gpio.h`
- `esp_log.h`
- Standard C++ libraries: `<set>`, `<string>`, `<sstream>`, `<map>`, `<chrono>`

## Setup
1. **Install Arduino IDE and ESP32 Board Support:**
   - Download and install the Arduino IDE from [Arduino's official website](https://www.arduino.cc/en/software).
   - Follow the instructions [here](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md) to install the ESP32 board support.

2. **Clone or Download this Repository:**
   - You can clone this repository using git:
     ```sh
     git clone https://github.com/Broskic/ESP32_wifi_sniffer.git
     ```
   - Or download the ZIP file from the repository page and extract it.

3. **Open the Project in Arduino IDE:**
   - Open the Arduino IDE and go to `File > Open` and select the `MAIN_CODE1.ino` file from the cloned or downloaded repository.

4. **Select the Board and Port:**
   - Go to `Tools > Board` and select `ESP32 Dev Module`.
   - Go to `Tools > Port` and select the port to which your ESP32 is connected.

5. **Upload the Code:**
   - Click on the upload button (right arrow) in the Arduino IDE to compile and upload the code to your ESP32.

## Running the Application
1. **Power Up the ESP32:**
   - Connect your ESP32 to your computer via USB. The board should power up and start running the application.

2. **Monitor Serial Output:**
   - Open the Serial Monitor in the Arduino IDE (`Tools > Serial Monitor`) or any serial terminal application.
   - Set the baud rate to 115200.
   - You should see the output of unique device counts and MAC addresses being printed every 10 seconds.

3. **LED Indicator:**
   - The LED connected to GPIO pin 5 will toggle its state every second.

## Code Explanation
The application is composed of several parts:

1. **Initialization Functions:**
   - `wifi_sniffer_init()`: Initializes the Wi-Fi sniffer.
   - `wifi_sniffer_set_channel(uint8_t channel)`: Sets the Wi-Fi channel.
   - `event_handler()`: Handles Wi-Fi events.

2. **Packet Handling:**
   - `wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)`: Processes incoming Wi-Fi packets to extract MAC addresses and update the last seen time for each device.

3. **Device Management:**
   - `remove_static_devices()`: Removes devices that have not been seen for more than 1 minute.
   - `print_unique_devices_task(void *pvParameter)`: Task that prints the count and MAC addresses of unique devices every 10 seconds.

4. **Main Functions:**
   - `setup()`: Initializes the serial communication, Wi-Fi sniffer, LED pin, and starts the device printing task.
   - `loop()`: Toggles the LED state and switches the Wi-Fi channel periodically.

