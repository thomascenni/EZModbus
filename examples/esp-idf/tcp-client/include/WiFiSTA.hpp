/**
 * @file WiFiSTA.hpp
 * @brief Header-only helper for ESP-IDF ≥ 5.4 (STA mode)
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include <cstring>

class WiFiSTA
{
public:
    // Ctor: store SSID / password
    constexpr WiFiSTA(const char *ssid,
                      const char *pass,
                      uint8_t     max_retry = 5,
                      wifi_auth_mode_t min_auth = WIFI_AUTH_OPEN) noexcept
        : ssid_{ssid},
          pass_{pass},
          max_retry_{max_retry},
          min_auth_{min_auth}
    {}

    // Disable copy/move to avoid UB
    WiFiSTA(const WiFiSTA&) = delete;
    WiFiSTA& operator=(const WiFiSTA&) = delete;
    WiFiSTA(WiFiSTA&&) = delete;
    WiFiSTA& operator=(WiFiSTA&&) = delete;

    // Launch STA and wait for connection (timeout in ms)
    esp_err_t begin(uint32_t timeout_ms = 10000)
    {
        // Avoid double initialization
        if (eg_ != nullptr) {
            return ESP_ERR_INVALID_STATE;
        }

        // Initialize NVS (required by ESP-IDF internals)
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        if (ret != ESP_OK) return ret;

        // init netif / event loop (idempotent)
        ESP_ERROR_CHECK(esp_netif_init());
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        esp_netif_create_default_wifi_sta();

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ret = esp_wifi_init(&cfg);
        if (ret != ESP_OK) return ret;

        // Event Group
        eg_ = xEventGroupCreate();
        if (!eg_) {
            esp_wifi_deinit();
            return ESP_ERR_NO_MEM;
        }

        // Reset retry counter (guard against max_retry_ == 0)
        retry_cnt_ = max_retry_;

        ret = esp_event_handler_instance_register(
            WIFI_EVENT, ESP_EVENT_ANY_ID,
            &WiFiSTA::staticEventHandler, this, &h_any_);
        if (ret != ESP_OK) {
            cleanup();
            return ret;
        }

        ret = esp_event_handler_instance_register(
            IP_EVENT, IP_EVENT_STA_GOT_IP,
            &WiFiSTA::staticEventHandler, this, &h_ip_);
        if (ret != ESP_OK) {
            cleanup();
            return ret;
        }

        // Config STA
        wifi_config_t wc = {};  // Modern C++ initialization
        strlcpy(reinterpret_cast<char*>(wc.sta.ssid), ssid_, sizeof(wc.sta.ssid));
        strlcpy(reinterpret_cast<char*>(wc.sta.password), pass_, sizeof(wc.sta.password));
        wc.sta.failure_retry_cnt   = max_retry_;
        wc.sta.threshold.authmode  = min_auth_;  // Configurable

        ret = esp_wifi_set_mode(WIFI_MODE_STA);
        if (ret != ESP_OK) {
            cleanup();
            return ret;
        }

        ret = esp_wifi_set_config(WIFI_IF_STA, &wc);
        if (ret != ESP_OK) {
            cleanup();
            return ret;
        }

        ret = esp_wifi_start();
        if (ret != ESP_OK) {
            cleanup();
            return ret;
        }

        EventBits_t bits = xEventGroupWaitBits(
            eg_, CONNECTED_BIT | FAIL_BIT, pdFALSE, pdFALSE,
            timeout_ms ? pdMS_TO_TICKS(timeout_ms) : portMAX_DELAY);

        // Clean bits to avoid ghost states
        xEventGroupClearBits(eg_, CONNECTED_BIT | FAIL_BIT);

        return (bits & CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
    }

    // Clean stop
    esp_err_t stop()
    {
        return cleanup();
    }

    // True if connected
    bool isConnected() const
    {
        wifi_ap_record_t ap;
        return esp_wifi_sta_get_ap_info(&ap) == ESP_OK;
    }

    // Destructor for automatic cleanup
    ~WiFiSTA()
    {
        cleanup();
    }

private:
    //—————————————————————————  Event plumbing  —————————————————————————
    static void staticEventHandler(void *arg,
                                   esp_event_base_t base,
                                   int32_t          id,
                                   void            *data)
    {
        static_cast<WiFiSTA*>(arg)->handleEvent(base, id, data);
    }

    void handleEvent(esp_event_base_t base,
                     int32_t          id,
                     void            *)
    {
        if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
            esp_wifi_connect();
        } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
            // Correction: protection contre underflow si max_retry_ == 0
            if (max_retry_ > 0 && retry_cnt_ > 0) {
                retry_cnt_--;
                ESP_LOGW(TAG_, "Retrying Wi-Fi… (%u left)", retry_cnt_);
                esp_wifi_connect();
            } else {
                ESP_LOGE(TAG_, "Wi-Fi connection failed after %u retries", max_retry_);
                xEventGroupSetBits(eg_, FAIL_BIT);
            }
        } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
            ESP_LOGI(TAG_, "Wi-Fi connected successfully");
            xEventGroupSetBits(eg_, CONNECTED_BIT);
        }
    }

    // Complete cleanup of resources
    esp_err_t cleanup()
    {
        esp_err_t ret = ESP_OK;

        // Unregister event handlers
        if (h_any_) {
            esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, h_any_);
            h_any_ = nullptr;
        }
        if (h_ip_) {
            esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, h_ip_);
            h_ip_ = nullptr;
        }

        // Clean event group
        if (eg_) {
            vEventGroupDelete(eg_);
            eg_ = nullptr;
        }

        // Stop WiFi only if initialized
        wifi_mode_t mode;
        if (esp_wifi_get_mode(&mode) == ESP_OK) {
            esp_err_t wifi_ret = esp_wifi_stop();
            if (wifi_ret != ESP_OK && ret == ESP_OK) ret = wifi_ret;

            wifi_ret = esp_wifi_deinit();
            if (wifi_ret != ESP_OK && ret == ESP_OK) ret = wifi_ret;
        }

        return ret;
    }

    //——————————————————————————  Data  ——————————————————————————
    const char *ssid_;
    const char *pass_;
    uint8_t     max_retry_;
    wifi_auth_mode_t min_auth_;
    uint8_t     retry_cnt_ {0};

    EventGroupHandle_t eg_ {nullptr};
    esp_event_handler_instance_t h_any_{};
    esp_event_handler_instance_t h_ip_ {};

    static constexpr int CONNECTED_BIT = BIT0;
    static constexpr int FAIL_BIT      = BIT1;
    static constexpr inline char TAG_[] = "WiFiSTA";
};