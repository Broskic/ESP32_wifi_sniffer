#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include <set>
#include <string>
#include <sstream>
#include <map>
#include <chrono>

#define LED_GPIO_PIN                     5
#define WIFI_CHANNEL_SWITCH_INTERVAL  (500)
#define WIFI_CHANNEL_MAX               (13)
#define PRINT_INTERVAL_MS             (10000)  // 10 seconds
#define DEVICE_TIMEOUT_MS             (60000)  // 1 minute

uint8_t channel = 1;

static wifi_country_t wifi_country = {.cc="CN", .schan = 1, .nchan = 13}; //Most recent esp32 library struct

typedef struct {
    unsigned frame_ctrl:16;
    unsigned duration_id:16;
    uint8_t addr1[6]; /* receiver address */
    uint8_t addr2[6]; /* sender address */
    uint8_t addr3[6]; /* filtering address */
    unsigned sequence_ctrl:16;
    uint8_t addr4[6]; /* optional */
} wifi_ieee80211_mac_hdr_t;

typedef struct {
    wifi_ieee80211_mac_hdr_t hdr;
    uint8_t payload[0]; /* network data ended with 4 bytes csum (CRC32) */
} wifi_ieee80211_packet_t;

std::map<std::string, std::chrono::steady_clock::time_point> devices_last_seen;

static void wifi_sniffer_init(void);
static void wifi_sniffer_set_channel(uint8_t channel);
static const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type);
static void wifi_sniffer_packet_handler(void *buff, wifi_promiscuous_pkt_type_t type);
static void print_unique_devices_task(void *pvParameter);
static void remove_static_devices();

static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    // Handle events here
}

void wifi_sniffer_init(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_country(&wifi_country)); /* set country for channel range [1, 13] */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_packet_handler);
}

void wifi_sniffer_set_channel(uint8_t channel)
{
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
}

const char *wifi_sniffer_packet_type2str(wifi_promiscuous_pkt_type_t type)
{
    switch(type) {
        case WIFI_PKT_MGMT: return "MGMT";
        case WIFI_PKT_DATA: return "DATA";
        default:
        case WIFI_PKT_MISC: return "MISC";
    }
}

void wifi_sniffer_packet_handler(void* buff, wifi_promiscuous_pkt_type_t type)
{
    if (type != WIFI_PKT_MGMT)
        return;

    const wifi_promiscuous_pkt_t *ppkt = (wifi_promiscuous_pkt_t *)buff;
    const wifi_ieee80211_packet_t *ipkt = (wifi_ieee80211_packet_t *)ppkt->payload;
    const wifi_ieee80211_mac_hdr_t *hdr = &ipkt->hdr;

    // Extract MAC address
    char addr2_str[18];
    snprintf(addr2_str, sizeof(addr2_str), "%02x:%02x:%02x:%02x:%02x:%02x",
             hdr->addr2[0], hdr->addr2[1], hdr->addr2[2],
             hdr->addr2[3], hdr->addr2[4], hdr->addr2[5]);

    // Update last seen time
    auto now = std::chrono::steady_clock::now();
    devices_last_seen[std::string(addr2_str)] = now;
}

void remove_static_devices() {
    auto now = std::chrono::steady_clock::now();
    
    for(auto it = devices_last_seen.begin(); it != devices_last_seen.end(); ) {
        auto last_seen_duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - it->second).count();
        
        // Check if the device has not been seen for more than DEVICE_TIMEOUT_MS
        if(last_seen_duration > DEVICE_TIMEOUT_MS) {
            // Erase the device from the map
            it = devices_last_seen.erase(it);
        } else {
            ++it;
        }
    }
}

void print_unique_devices_task(void *pvParameter)
{
    while (true) {
        vTaskDelay(PRINT_INTERVAL_MS / portTICK_PERIOD_MS);
        remove_static_devices();
        ESP_LOGI("Unique Devices", "Count: %d", devices_last_seen.size());

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
    vTaskDelay(WIFI_CHANNEL_SWITCH_INTERVAL / portTICK_PERIOD_MS);
    wifi_sniffer_set_channel(channel);
    channel = (channel % WIFI_CHANNEL_MAX) + 1;
}
