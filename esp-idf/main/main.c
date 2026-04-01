/*
 * CyMouse Receiver - ESP-IDF 原生移植版本
 *
 * 原项目: ~/CyMouse/CyMouse_Receiver (PlatformIO + Arduino)
 * 目标平台: ESP32-S3, ESP-IDF v6.0
 *
 * 主要差异:
 *   - Arduino USB HID Mouse → TinyUSB (esp_tinyusb)
 *   - Serial.print → ESP_LOGI
 *   - millis() → esp_timer_get_time() / 1000
 *   - delay() → vTaskDelay(pdMS_TO_TICKS())
 *   - setup() + loop() → app_main() + FreeRTOS 任务
 *   - ESP-NOW 回调签名已更新为 IDF v5+ 新式签名
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_timer.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char *TAG = "CyMouseRcv";

// ─── 配置 ───────────────────────────────────────────────
#define WIFI_CHANNEL          13
#define CONNECTION_TIMEOUT_MS 3000LL
#define ESPNOW_PHY_MODE       WIFI_PHY_MODE_HT20
#define ESPNOW_PHY_RATE       WIFI_PHY_RATE_MCS0_LGI

static const char    *MY_DEVICE_NAME  = "CyMouseReceiver_V1";
static uint8_t        broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ─── 数据结构 ────────────────────────────────────────────
typedef enum {
    PACKET_TYPE_DISCOVERY = 0,
    PACKET_TYPE_MOUSE_DATA,
    PACKET_TYPE_HEARTBEAT,
} PacketType;

#pragma pack(push, 1)
typedef struct {
    PacketType type;
    char       deviceName[32];
    int16_t    deltaX;
    int16_t    deltaY;
    int8_t     wheel;
    uint8_t    buttons;
} UniversalPacket;
#pragma pack(pop)

typedef struct {
    uint8_t   mac_addr[6];
    PacketType type;
    int16_t   deltaX;
    int16_t   deltaY;
    int8_t    wheel;
    uint8_t   buttons;
} QueueItem_t;

// ─── USB HID 描述符 ───────────────────────────────────────
static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_MOUSE()
};

static const tusb_desc_device_t desc_device = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,   /* Espressif VID */
    .idProduct          = 0x4005,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

enum { ITF_NUM_HID = 0, ITF_NUM_TOTAL };

#define EPNUM_HID        0x81
#define HID_EP_SIZE      16
#define CONFIG_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN)

static const uint8_t desc_configuration[] = {
    /* 配置描述符: 配置号, 接口数, 字符串索引, 总长度, 属性, 电流(mA) */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN, 0x00, 100),
    /* HID 接口描述符: 接口号, 字符串索引, 协议, report描述符长度, EP地址, EP大小, 轮询间隔(ms) */
    TUD_HID_DESCRIPTOR(ITF_NUM_HID, 0, HID_ITF_PROTOCOL_MOUSE,
                       sizeof(hid_report_descriptor), EPNUM_HID, HID_EP_SIZE, 1),
};

static const char lang_id[]       = {0x09, 0x04};  /* English (0x0409) */
static const char *string_desc_arr[] = {
    lang_id,              /* 0: 语言 ID */
    "Cynix Tech",         /* 1: 制造商 */
    "CyMouse Receiver",   /* 2: 产品名 */
    "CyMouse-00001",      /* 3: 序列号 */
};

// ─── USB HID 流控信号量 ──────────────────────────────────────
// 用于等待上一个 report 被主机取走后再发下一个，防止端点忙时静默丢包
static SemaphoreHandle_t s_hid_sent_sem = NULL;

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
    (void)instance; (void)report; (void)len;
    /* 此回调从 TinyUSB 任务上下文（非硬件 ISR）触发，必须用 xSemaphoreGive
     * 而非 xSemaphoreGiveFromISR，这样 FreeRTOS 会立即调度等待的 mouseTask */
    if (s_hid_sent_sem) {
        xSemaphoreGive(s_hid_sent_sem);
    }
}

// ─── TinyUSB 必要回调 ─────────────────────────────────────
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void)instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type,
                                uint8_t *buffer, uint16_t reqlen)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type,
                            uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance; (void)report_id; (void)report_type;
    (void)buffer;   (void)bufsize;
}

// ─── 全局状态 ─────────────────────────────────────────────
static QueueHandle_t mouseDataQueue;
static volatile bool    isConnected      = false;
static volatile int64_t lastPacketTimeMs = 0;
static uint8_t          peerMacAddress[6] = {0};

static inline int64_t get_millis(void)
{
    return esp_timer_get_time() / 1000LL;
}

// ─── 前置声明 ─────────────────────────────────────────────
static void resetConnection(void);

// ─── ESP-NOW 接收回调 (ISR 上下文) ───────────────────────
// IDF v5+ 新式签名: esp_now_recv_info_t 替代 mac_addr 参数
static void IRAM_ATTR OnDataRecv(const esp_now_recv_info_t *recv_info,
                                  const uint8_t *data, int data_len)
{
    if (data_len != (int)sizeof(UniversalPacket)) {
        return;
    }
    const UniversalPacket *packet = (const UniversalPacket *)data;
    if (packet->type == PACKET_TYPE_MOUSE_DATA ||
        packet->type == PACKET_TYPE_HEARTBEAT) {
        QueueItem_t item;
        memcpy(item.mac_addr, recv_info->src_addr, 6);
        item.type    = packet->type;
        item.deltaX  = packet->deltaX;
        item.deltaY  = packet->deltaY;
        item.wheel   = packet->wheel;
        item.buttons = packet->buttons;
        xQueueSendFromISR(mouseDataQueue, &item, NULL);
    }
}

// ─── 鼠标数据处理任务 (核心 1) ────────────────────────────
static void mouseTask(void *pvParameters)
{
    QueueItem_t receivedItem;
    uint8_t     lastButtons = 0;
    (void)pvParameters;

    ESP_LOGI(TAG, "鼠标处理任务已启动。");

    for (;;) {
        BaseType_t res = xQueueReceive(mouseDataQueue, &receivedItem,
                                       pdMS_TO_TICKS(500));

        if (res != pdTRUE) {
            /* 500ms 内无数据 → 检查心跳超时 */
            if (isConnected &&
                (get_millis() - lastPacketTimeMs > CONNECTION_TIMEOUT_MS)) {
                resetConnection();
            }
            continue;
        }

        lastPacketTimeMs = get_millis();

        /* 首次收到数据包时建立连接 */
        if (!isConnected) {
            ESP_LOGI(TAG,
                     "连接建立！发送端 MAC: %02X:%02X:%02X:%02X:%02X:%02X",
                     receivedItem.mac_addr[0], receivedItem.mac_addr[1],
                     receivedItem.mac_addr[2], receivedItem.mac_addr[3],
                     receivedItem.mac_addr[4], receivedItem.mac_addr[5]);

            memcpy(peerMacAddress, receivedItem.mac_addr, 6);

            esp_now_peer_info_t peerInfo;
            memset(&peerInfo, 0, sizeof(peerInfo));
            memcpy(peerInfo.peer_addr, peerMacAddress, 6);
            peerInfo.channel = WIFI_CHANNEL;
            peerInfo.encrypt = false;
            peerInfo.ifidx   = WIFI_IF_STA;

            esp_err_t r = esp_now_add_peer(&peerInfo);
            if (r != ESP_OK) {
                if (esp_now_mod_peer(&peerInfo) == ESP_OK) {
                    ESP_LOGI(TAG, "对等设备已存在，更新信息成功。");
                } else {
                    ESP_LOGW(TAG, "添加/更新对等设备失败。");
                }
            } else {
                ESP_LOGI(TAG, "已将发送端添加为对等设备。");
            }

            /* 为该对等设备设置 ESP-NOW PHY 速率 (IDF v6 新接口) */
            esp_now_rate_config_t rate_cfg;
            memset(&rate_cfg, 0, sizeof(rate_cfg));
            rate_cfg.phymode = ESPNOW_PHY_MODE;
            rate_cfg.rate    = ESPNOW_PHY_RATE;
            esp_now_set_peer_rate_config(peerMacAddress, &rate_cfg);

            isConnected = true;
        }

        /* 只有鼠标数据包才驱动 USB HID */
        if (receivedItem.type == PACKET_TYPE_MOUSE_DATA) {
            bool has_movement   = (receivedItem.deltaX != 0 ||
                                   receivedItem.deltaY != 0 ||
                                   receivedItem.wheel  != 0);
            bool has_btn_change = (receivedItem.buttons != lastButtons);

            if ((has_movement || has_btn_change) && tud_ready()) {
                /* 清掉上次可能残留的完成信号 */
                xSemaphoreTake(s_hid_sent_sem, 0);

                tud_hid_mouse_report(0,
                                     receivedItem.buttons,
                                     (int8_t)receivedItem.deltaX,
                                     (int8_t)receivedItem.deltaY,
                                     (int8_t)receivedItem.wheel,
                                     0);
                lastButtons = receivedItem.buttons;

                /* 阻塞等待主机取走本报告，与 USB 1000Hz 轮询时钟同步
                 * 等价于 Arduino SendReport() 中等待 input_sem */
                xSemaphoreTake(s_hid_sent_sem, pdMS_TO_TICKS(2));
            }
        }
    }
}

// ─── WiFi + ESP-NOW 初始化 ────────────────────────────────
static void initWiFi(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable        = false;
    cfg.rx_ba_win         = 16;
    cfg.wifi_task_core_id = 0;

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    wifi_country_t country = {
        .cc     = "CN",
        .schan  = 1,
        .nchan  = 13,
        .policy = WIFI_COUNTRY_POLICY_AUTO,
    };

    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA,
        WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW20));

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
}

// ─── 重置连接状态 ─────────────────────────────────────────
static void resetConnection(void)
{
    ESP_LOGI(TAG, "--- 连接超时，重置状态 ---");
    esp_err_t result = esp_now_del_peer(peerMacAddress);
    if (result == ESP_OK) {
        ESP_LOGI(TAG, "已成功删除旧的对等设备。");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        ESP_LOGW(TAG, "尝试删除一个不存在的对等设备。");
    } else {
        ESP_LOGE(TAG, "删除对等设备失败。");
    }
    isConnected = false;
    memset(peerMacAddress, 0, 6);
    ESP_LOGI(TAG, "接收端已回到广播模式，等待新连接...");
}

// ─── 主入口 ───────────────────────────────────────────────
void app_main(void)
{
    ESP_LOGI(TAG, "CyMouse 接收端启动...");
    ESP_LOGI(TAG, "UniversalPacket 大小: %u 字节", (unsigned)sizeof(UniversalPacket));

    /* 1. 创建 HID 发送完成信号量 */
    s_hid_sent_sem = xSemaphoreCreateBinary();

    /* 2. 初始化 TinyUSB USB HID 鼠标 */
    tinyusb_config_t tusb_cfg = {
        .task = {
            .size     = 4096,
            .priority = 5,
            .xCoreID  = 1,   /* USB 任务固定在核心 1（与 mouseTask 同核）
                              * 核心 0 专供 WiFi（优先级 23），避免 WiFi 抢占
                              * TinyUSB（优先级 5）导致 report_complete_cb 延迟 */
        },
        .descriptor = {
            .device            = &desc_device,
            .string            = string_desc_arr,
            .string_count      = (int)(sizeof(string_desc_arr) /
                                       sizeof(string_desc_arr[0])),
            .full_speed_config = desc_configuration,
        },
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 3. 初始化 WiFi + ESP-NOW */
    initWiFi();
    vTaskDelay(pdMS_TO_TICKS(500));

    /* 4. 创建鼠标数据队列 */
    mouseDataQueue = xQueueCreate(1024, sizeof(QueueItem_t));
    if (mouseDataQueue == NULL) {
        ESP_LOGE(TAG, "创建鼠标数据队列失败！");
        return;
    }

    /* 5. 注册 ESP-NOW 接收回调 */
    ESP_ERROR_CHECK(esp_now_register_recv_cb(OnDataRecv));

    /* 6. 添加广播对等设备 */
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        ESP_LOGE(TAG, "添加广播对等设备失败。");
        return;
    }

    /* 7. 为广播对等设备设置 PHY 速率 */
    esp_now_rate_config_t rate_cfg;
    memset(&rate_cfg, 0, sizeof(rate_cfg));
    rate_cfg.phymode = ESPNOW_PHY_MODE;
    rate_cfg.rate    = ESPNOW_PHY_RATE;
    esp_now_set_peer_rate_config(broadcastAddress, &rate_cfg);

    /* 8. 鼠标任务固定在核心 1，USB 任务在核心 0 */
    xTaskCreatePinnedToCore(mouseTask, "MouseTask", 8192, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "初始化完成，开始广播身份...");

    /* 主循环: 未连接时定期广播发现包 */
    for (;;) {
        if (!isConnected) {
            UniversalPacket discoveryPacket;
            memset(&discoveryPacket, 0, sizeof(discoveryPacket));
            discoveryPacket.type = PACKET_TYPE_DISCOVERY;
            strncpy(discoveryPacket.deviceName, MY_DEVICE_NAME,
                    sizeof(discoveryPacket.deviceName) - 1);
            esp_now_send(broadcastAddress,
                         (const uint8_t *)&discoveryPacket,
                         sizeof(discoveryPacket));
            ESP_LOGI(TAG, "正在广播身份，等待配对...");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
