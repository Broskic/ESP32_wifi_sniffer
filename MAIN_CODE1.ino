#include "freertos/FreeRTOS.h"          // FreeRTOS library for ESP32
#include "freertos/task.h"              // FreeRTOS task management
#include "esp_wifi.h"                   // ESP32 Wi-Fi library
#include "esp_wifi_types.h"             // ESP32 Wi-Fi types
#include "esp_system.h"                 // ESP32 system functions
#include "esp_event.h"                  // ESP32 event handler
#include "nvs_flash.h"                  // ESP32 non-volatile storage
#include "driver/gpio.h"                // ESP32 GPIO driver
#include "esp_log.h"                    // ESP32 logging library
#include <set>                          // Standard C++ set container
#include <string>                       // Standard C++ string handling
#include <sstream>                      // Standard C++ string stream
#include <map>                          // Standard C++ map container
#include <chrono>                       // Standard C++ chrono library for time-related functionality

#define LED_GPIO_PIN                     (5)                   // GPIO pin for LED
#define WIFI_CHANNEL_SWITCH_INTERVAL  (500)                  // Interval for switching Wi-Fi channels in milliseconds
#define WIFI_CHANNEL_MAX               (13)                  // Maximum Wi-Fi channel number
#define PRINT_INTERVAL_MS             (10000)                // Interval for printing unique device count in milliseconds (10 seconds)
#define DEVICE_TIMEOUT_MS             (60000)                // Timeout in milliseconds for considering a device as static

uint8_t channel = 1;                                        // Current Wi-Fi channel being scanned

static wifi_country_t wifi_country = {.cc="CN", .schan = 1, .nchan = 13}; // Wi-Fi country configuration

typedef struct {                                            // Structure for Wi-Fi MAC header
    uint16_t frame_ctrl;                                 // Frame control field
    uint16_t duration_id;                                // Duration/ID field
    uint8_t addr1[6];                                       // Receiver address
    uint8_t addr2[6];                                       // Sender address
    uint8_t addr3[6];                                       // Filtering address
    uint16_t sequence_ctrl;                              // Sequence control field
    uint8_t addr4[6];                                       // Optional fourth address field
} wifi_ieee80211_mac_hdr_t;

typedef struct {                                            // Structure for full Wi-Fi packet
    wifi_ieee80211_mac_hdr_t hdr;                           // MAC header
    uint8_t payload[0];                                     // Network data with variable length, ended with 4 bytes checksum (CRC32)
} wifi_ieee80211_packet_t;

std::map<std::string, std::chrono::steady_clock::time_point> devices_last_seen; // Map to store last seen time of devices

static void wifi_sniffer_init(void);                        // Function prototype for initializing Wi-Fi sniffer
static void wifi_sniffer_set_channel(uint8_t channel);      // Function prototype for setting Wi-Fi channel
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type); // Function prototype for getting packet type as string
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type); // Function prototype for handling Wi-Fi packets
static void print_unique_devices_task(void *pvParameter);   // Function prototype for task to print unique device count
static void remove_static_devices();                        // Function prototype for removing static devices

void wifi_sniffer_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());                      // Initialize non-volatile storage
    ESP_ERROR_CHECK(esp_netif_init());                      // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_event_loop_create_default());       // Create the default event loop

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();    // Get default Wi-Fi configuration
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));                   // Initialize the Wi-Fi driver with the default configuration
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country));   // Set Wi-Fi country configuration
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));// Set Wi-Fi storage to RAM
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));     // Set Wi-Fi mode to NULL (promiscuous mode)
    ESP_ERROR_CHECK(esp_wifi_start());                      // Start the Wi-Fi driver
    esp_wifi_set_promiscuous(true);                         // Enable promiscuous mode
    esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler); // Set the packet handler callback function
}

void wifi_sniffer_set_channel(uint8_t channel)
{
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);   // Set Wi-Fi channel
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
    switch(type) {
        case WIFI_PKT_MGMT: return "MGMT";                   // Management packet
        case WIFI_PKT_DATA: return "DATA";                   // Data packet
        default:
        case WIFI_PKT_MISC: return "MISC";                   // Miscellaneous packet
    }
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT)                              // Only handle management packets
        return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff; // Cast the buffer to a Wi-Fi packet structure , c style cast / static_cast<>, dynamic_cast<>
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload; // Get the IEEE802.11 packet structure from the Wi-Fi packet
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;       // Get the MAC header from the IEEE802.11 packet

    // Extract MAC address // XX:XX:XX:XX:XX:XX
    char addr2_str[18];
    snprintf(addr2_str, sizeof(addr2_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
             hdr->addr2[3], hdr->addr2[4], hdr->addr2[5]);

    // Update last seen time
    auto now = std::chrono::steady_clock::now();            // Get the current time
    devices_last_seen[std::string(addr2_str)] = now;        // Update the last seen time for the device
}

void remove_static_devices() {
    auto now = std::chrono::steady_clock::now();            // Get the current time

    for(auto it = devices_last_seen.begin(); it != devices_last_seen.end(); ) {
        auto last_seen_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();

        // Check if the device has not been seen for more than DEVICE_TIMEOUT_MS
        if(last_seen_duration > DEVICE_TIMEOUT_MS) {
            it = devices_last_seen.erase(it);               // Erase the device from the map
        } else {
            ++it;
        }
    }
}

void print_unique_devices_task(void *pvParameter)
{
    while (true) {
        vTaskDelay(PRINT_INTERVAL_MS / portTICK_PERIOD_MS); // Delay for PRINT_INTERVAL_MS
        remove_static_devices();                            // Remove static devices from the map
        ESP_LOGI("Unique Devices", "Count: %d", devices_last_seen.size()); // Log the count of unique devices

        // Print each unique address
        for (const auto& device : devices_last_seen) {
            ESP_LOGI("Unique Device", "%s", device.first.c_str());
        }
    }
}

// the setup function runs once when you press reset or power the board
void setup() {
    // initialize digital pin 5 as an output.
    Serial.begin(115200);
    delay(10);
    wifi_sniffer_init();
    pinMode(LED_GPIO_PIN, OUTPUT);

    // Create a task to print the unique device count every 10 seconds
    xTaskCreate(&print_unique_devices_task, "print_unique_devices_task", 2048, NULL, 5, NULL);
}

// the loop function runs over and over again forever
void loop() {
    delay(1000); // wait for a second
    
    if (digitalRead(LED_GPIO_PIN) == LOW)
        digitalWrite(LED_GPIO_PIN, HIGH);
    else
        digitalWrite(LED_GPIO_PIN, LOW);
        
    vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS); // Delay Wi-Fi channel switch
    wifi_sniffer_set_channel(channel);  // Set the Wi-Fi channel
    channel = (channel % WIFI_CHANNEL_MAX) + 1; // Loop though all available Wi-Fi channels
}
