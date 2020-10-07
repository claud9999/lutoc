/** LuToC for ESP32 - CC0 public domain license. */
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <esp_sntp.h>

#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_system.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_eth.h"

#include "sdkconfig.h"
#include "esp_wifi_default.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/err.h"
#include "lwip/sys.h"

#include <ltc.h>

#include "driver/i2s.h"
#include "freertos/queue.h"

#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define WRITE_BIT I2C_MASTER_WRITE  /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ    /*!< I2C master read */
#define ACK_CHECK_EN 0x1            /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0           /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                 /*!< I2C ack value */
#define NACK_VAL 0x1                /*!< I2C nack value */

static const char *TAG = "lutoc";

static gpio_num_t i2c_gpio_sda = CONFIG_I2C_GPIO_SDA;
static gpio_num_t i2c_gpio_scl = CONFIG_I2C_GPIO_SCL;
static uint32_t i2c_frequency = CONFIG_I2C_FREQUENCY;
static i2c_port_t i2c_port = CONFIG_I2C_PORT_NUM;

#define GOT_IPV4_BIT BIT(0)

#define CONNECTED_BITS (GOT_IPV4_BIT)

static EventGroupHandle_t conn_evtgrp;
static esp_ip4_addr_t ip_addr;
static const char *conn_name;
static esp_netif_t *esp_netif = NULL;

static void on_got_ip(void *arg, esp_event_base_t event_base,
                      int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Got IP event!");
    ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
    memcpy(&ip_addr, &event->ip_info.ip, sizeof(ip_addr));
    xEventGroupSetBits(conn_evtgrp, GOT_IPV4_BIT);
}

static void on_wifi_disconnect(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    ESP_LOGI(TAG, "Wi-Fi disconnected, trying to reconnect...");
    esp_err_t err = esp_wifi_connect();
    if (err == ESP_ERR_WIFI_NOT_STARTED) return;
    ESP_ERROR_CHECK(err);
}

static void start(void) {
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_netif_config_t netif_config = ESP_NETIF_DEFAULT_WIFI_STA();

    esp_netif_t *netif = esp_netif_new(&netif_config);

    assert(netif);

    esp_netif_attach_wifi_station(netif);
    esp_wifi_set_default_wifi_sta_handlers();

    esp_netif = netif;

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_config_t wifi_config = { .sta = { .ssid = CONFIG_WIFI_SSID, .password = CONFIG_WIFI_PASSWD } };
    ESP_LOGI(TAG, "Connecting to %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    conn_name = CONFIG_WIFI_SSID;
}

static void stop(void) {
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &on_wifi_disconnect));
    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &on_got_ip));
    esp_err_t err = esp_wifi_stop();
    if (err == ESP_ERR_WIFI_NOT_INIT) return;
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(esp_netif));
    esp_netif_destroy(esp_netif);
    esp_netif = NULL;
}

esp_err_t wifi_connect(void) {
    if (conn_evtgrp != NULL) return ESP_ERR_INVALID_STATE;
    conn_evtgrp = xEventGroupCreate();
    start();
    ESP_ERROR_CHECK(esp_register_shutdown_handler(&stop));
    ESP_LOGI(TAG, "Waiting for IP");
    xEventGroupWaitBits(conn_evtgrp, CONNECTED_BITS, true, true, portMAX_DELAY);
    ESP_LOGI(TAG, "Connected to %s", conn_name);
    ESP_LOGI(TAG, "IPv4 address: " IPSTR, IP2STR(&ip_addr));
    return ESP_OK;
}

esp_err_t wifi_disconnect(void) {
    if (conn_evtgrp == NULL) return ESP_ERR_INVALID_STATE;
    vEventGroupDelete(conn_evtgrp);
    conn_evtgrp = NULL;
    stop();
    ESP_LOGI(TAG, "Disconnected from %s", conn_name);
    conn_name = NULL;
    return ESP_OK;
}

static esp_err_t i2c_master_driver_initialize(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_gpio_sda,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = i2c_gpio_scl,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = i2c_frequency
    };
    return i2c_param_config(i2c_port, &conf);
}

#define I82BCD(I) ((I) % 10 + ((I) / 10) * 0x10)
#define BCD2I8(B) ((B & 0xF) + ((B & 0xF0) / 0x10) * 10)

static void set_rtc_time(struct timeval *tv) {
    struct tm tm = { 0 };

    int chip_addr = 0x32;
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    uint8_t block[7];

    localtime_r(&tv->tv_sec, &tm);

    /* note that usec will be set to zero when sec is set */
    block[0] = I82BCD(tm.tm_sec);
    block[1] = I82BCD(tm.tm_min);
    block[2] = I82BCD(tm.tm_hour);
    block[3] = tm.tm_wday;
    block[4] = I82BCD(tm.tm_mday);
    block[5] = I82BCD(tm.tm_mon);
    block[6] = I82BCD(tm.tm_year - 100);

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x00, ACK_CHECK_EN);
    i2c_master_write(cmd, block, sizeof(block), ACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) ESP_LOGE(TAG, "i2c write err: %d", ret);
    i2c_driver_delete(i2c_port);
}

static void get_rtc_time(struct timeval *tv, struct tm *tm) {
    uint8_t chip_addr = 0x32;
    i2c_driver_install(i2c_port, I2C_MODE_MASTER, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    i2c_master_driver_initialize();
    uint8_t block[8];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, 0x10, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, chip_addr << 1 | READ_BIT, ACK_CHECK_EN);
    i2c_master_read(cmd, block, sizeof(block) - 1, ACK_VAL);
    i2c_master_read_byte(cmd, &block[sizeof(block) - 1], NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_port, cmd, 50 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) ESP_LOGE(TAG, "i2c cmd err: %d", ret);
    else {
        struct tm t;
        if (!tm) tm = &t;
        tv->tv_usec = BCD2I8(block[0]);
        tm->tm_sec = BCD2I8(block[1]);
        tm->tm_min = BCD2I8(block[2]);
        tm->tm_hour = BCD2I8(block[3]);
        tm->tm_wday = block[4];
        tm->tm_mday = BCD2I8(block[5]);
        tm->tm_mon = BCD2I8(block[6]);
        tm->tm_year = BCD2I8(block[7]) + 100;
        tv->tv_sec = mktime(tm);
    }
    i2c_driver_delete(i2c_port);
}

void time_sync_notification_cb(struct timeval *tv) {
    ESP_LOGI(TAG, "Notification of a time synchronization event");
    set_rtc_time(tv);
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_connect();

    ESP_LOGI(TAG, "Initializing SNTP");

    time_t now;
    struct tm timeinfo;

    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, CONFIG_NTP_SERVER);
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
    sntp_init();

    // wait for time to be set
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    time(&now);

    char strftime_buf[64];

    setenv("TZ", CONFIG_TZ, 1);
    tzset();
    localtime_r(&now, &timeinfo);
    ESP_LOGI(TAG, "The time zone: %s", CONFIG_TZ);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    ESP_LOGI(TAG, "The current date/time: %s", strftime_buf);

    /* set up i2s */
    i2s_config_t i2s_config = {
        .mode = I2S_MODE_MASTER | I2S_MODE_TX | I2S_MODE_DAC_BUILT_IN,
        .sample_rate = 48000,
        .bits_per_sample = 16,
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .intr_alloc_flags = 0,
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
    };
    i2s_driver_install(0, &i2s_config, 0, NULL);
    i2s_set_pin(0, NULL);
    i2s_set_sample_rates(0, 48000);

    /* set up ltc */
    double fps = 25;
    ltcsnd_sample_t *buf;
    LTCEncoder *encoder;
    encoder = ltc_encoder_create(48000, fps, fps==25?LTC_TV_625_50:LTC_TV_525_60, LTC_USE_DATE);
    buf = calloc(ltc_encoder_get_buffersize(encoder), sizeof(ltcsnd_sample_t));
    if (!buf) return;

    /* get current time from RTC */
    struct timeval tv;
    struct tm tm;
    get_rtc_time(&tv, &tm);

    /* set timecode time */
    SMPTETimecode st;
    const char timezone[6] = "+0800";
    strcpy(st.timezone, timezone);
    st.years = tm.tm_year;
    st.months = tm.tm_mon;
    st.days = tm.tm_mday;
    st.hours = tm.tm_hour;
    st.mins = tm.tm_min;
    st.secs = tm.tm_sec;
    st.frame = 0;
    ltc_encoder_set_timecode(encoder, &st);

    while(1) {
        ltc_encoder_encode_frame(encoder);

        size_t len = ltc_encoder_copy_buffer(encoder, buf);

        if (len > 0) {
            printf("writing %d bytes\n", len);
            i2s_write(0, buf, len, &len, portMAX_DELAY);
            printf("wrote %d bytes to i2s\n", len);
        }

        ltc_encoder_inc_timecode(encoder);
    }

    i2s_driver_uninstall(0);
    ltc_encoder_free(encoder);
    free(buf);
}
