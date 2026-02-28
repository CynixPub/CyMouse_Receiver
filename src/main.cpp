#include <Arduino.h>
#include <esp_now.h>
#include <USB.h>
#include <USBHIDMouse.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// --- 配置定义 ---
#define WIFI_CHANNEL 13
#define CONNECTION_TIMEOUT 3000 // 3秒无数据则认为连接丢失
const char* MY_DEVICE_NAME = "CyMouseReceiver_V1";
uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// --- 数据结构定义 ---
typedef enum {
    PACKET_TYPE_DISCOVERY = 0,
    PACKET_TYPE_MOUSE_DATA,
    PACKET_TYPE_HEARTBEAT // 新增心跳包类型
} PacketType;

#pragma pack(push, 1)
typedef struct {
    PacketType type;
    char deviceName[32];
    int16_t deltaX;
    int16_t deltaY;
    int8_t wheel;
    uint8_t buttons;
} UniversalPacket;
#pragma pack(pop)

typedef struct {
    uint8_t mac_addr[6];
    PacketType type; // 新增type字段，用于区分包类型
    int16_t deltaX;
    int16_t deltaY;
    int8_t wheel;
    uint8_t buttons;
} QueueItem_t;

// --- 函数前置声明 ---
void resetConnection();

// --- 全局变量 ---
USBHIDMouse Mouse;
static QueueHandle_t mouseDataQueue;
static bool isConnected = false;
static unsigned long lastPacketTime = 0; // 用于心跳检测
static uint8_t peerMacAddress[6] = {0};   // 保存已连接的对端MAC地址


// ESP-NOW数据接收回调
// 职责：只负责接收数据包，验证类型和长度，然后快速送入队列。不做任何业务逻辑。
// IRAM_ATTR: 将函数放入内部SRAM，避免Flash缓存未命中带来的延迟
void IRAM_ATTR OnDataRecv(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    if (data_len != sizeof(UniversalPacket)) {
        return; // 长度不匹配，立即丢弃
    }

    UniversalPacket* packet = (UniversalPacket*)data;

    // 现在接收数据包或心跳包
    if (packet->type == PACKET_TYPE_MOUSE_DATA || packet->type == PACKET_TYPE_HEARTBEAT) {
        QueueItem_t item;
        memcpy(item.mac_addr, mac_addr, 6);
        item.type = packet->type; // 记录包类型
        item.deltaX = packet->deltaX;
        item.deltaY = packet->deltaY;
        item.wheel = packet->wheel;
        item.buttons = packet->buttons;

        xQueueSendFromISR(mouseDataQueue, &item, NULL);
    }
}

// 高优先级任务，用于处理鼠标数据和USB HID通信
// 职责：处理队列数据，执行配对逻辑，并控制USB HID。
void mouseTask(void *pvParameters) {
    QueueItem_t receivedItem;
    uint8_t lastButtons = 0;

    Serial.println("鼠标处理任务已启动。");

    for (;;) {
        // 使用有限超时代替无限等待，以便在无数据时检查心跳超时
        BaseType_t queueResult = xQueueReceive(mouseDataQueue, &receivedItem, pdMS_TO_TICKS(500));

        // 队列超时，没有收到数据，检查心跳
        if (queueResult != pdTRUE) {
            if (isConnected && (millis() - lastPacketTime > CONNECTION_TIMEOUT)) {
                resetConnection();
            }
            continue;
        }

        // 收到任何数据包都代表连接是活动的，更新心跳时间
        lastPacketTime = millis();

        // 当我们收到第一个鼠标数据包时，意味着发送端已经与我们配对成功。
        // 此时我们才需要将发送端添加为对等设备，并标记连接状态。
        if (!isConnected) {
            Serial.print("收到首个鼠标数据包，连接建立！发送端 MAC: ");
            for (int i = 0; i < 6; i++) {
                Serial.printf("%02X", receivedItem.mac_addr[i]);
                if (i < 5) Serial.print(":");
            }
            Serial.println();
            
            // 保存对端的MAC地址，以便断开连接时使用
            memcpy(peerMacAddress, receivedItem.mac_addr, 6);

            esp_now_peer_info_t peerInfo = {};
            memcpy(peerInfo.peer_addr, peerMacAddress, 6);
            peerInfo.channel = WIFI_CHANNEL;
            peerInfo.encrypt = false;
            peerInfo.ifidx = WIFI_IF_STA;
            
            // 尝试添加对等设备，如果已存在则尝试修改
            if (esp_now_add_peer(&peerInfo) != ESP_OK) {
                if (esp_now_mod_peer(&peerInfo) == ESP_OK) {
                    Serial.println("对等设备已存在，更新信息成功。");
                } else {
                    Serial.println("警告：添加或更新对等设备失败。");
                }
            } else {
                Serial.println("已将发送端添加为对等设备。");
            }
            isConnected = true; // 确认连接
        }
        
        // 只有当包类型是MOUSE_DATA时，才处理鼠标动作
        if (receivedItem.type == PACKET_TYPE_MOUSE_DATA) {
            // 处理鼠标移动和滚轮
            if (receivedItem.deltaX != 0 || receivedItem.deltaY != 0 || receivedItem.wheel != 0) {
                Mouse.move(receivedItem.deltaX, receivedItem.deltaY, receivedItem.wheel);
            }

            // 处理按键
            if (receivedItem.buttons != lastButtons) {
                uint8_t changed_buttons = receivedItem.buttons ^ lastButtons;
                if (changed_buttons & 0x01) { (receivedItem.buttons & 0x01) ? Mouse.press(MOUSE_LEFT) : Mouse.release(MOUSE_LEFT); }
                if (changed_buttons & 0x02) { (receivedItem.buttons & 0x02) ? Mouse.press(MOUSE_RIGHT) : Mouse.release(MOUSE_RIGHT); }
                if (changed_buttons & 0x04) { (receivedItem.buttons & 0x04) ? Mouse.press(MOUSE_MIDDLE) : Mouse.release(MOUSE_MIDDLE); }
                if (changed_buttons & 0x08) { (receivedItem.buttons & 0x08) ? Mouse.press(MOUSE_BACKWARD) : Mouse.release(MOUSE_BACKWARD); }
                if (changed_buttons & 0x10) { (receivedItem.buttons & 0x10) ? Mouse.press(MOUSE_FORWARD) : Mouse.release(MOUSE_FORWARD); }
                lastButtons = receivedItem.buttons;
            }
        }
    }
}

// 标准WiFi初始化函数
void initWiFi() {

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();// 這是 ESP-NOW 官方範例中推薦的步驟


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    cfg.nvs_enable = false;
    cfg.rx_ba_win = 16;
    cfg.wifi_task_core_id = 0;

    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    wifi_country_t country = {
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_AUTO,
    };

    // 必须先启动WiFi，再设置协议和其他运行时参数
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_country(&country));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); 
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_wake_window(65535));
    //ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_2M_S));
    ESP_ERROR_CHECK(esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_MCS0_LGI));

   // return true;
}



// 断开并重置连接状态
void resetConnection() {
    Serial.println("\n--- 连接超时，重置状态 ---");
    // 从ESP-NOW中删除旧的对等设备，这是保证重连成功的关键
    esp_err_t result = esp_now_del_peer(peerMacAddress);
    if (result == ESP_OK) {
        Serial.println("已成功删除旧的对等设备。");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
        Serial.println("警告：尝试删除一个不存在的对等设备。");
    } else {
        Serial.println("错误：删除对等设备失败。");
    }
    isConnected = false;
    memset(peerMacAddress, 0, 6); // 清空MAC地址
    Serial.println("接收端已回到广播模式，等待新的连接...");
    Serial.println("--------------------------\n");
}

void setup() {
    Serial.begin(115200);
    Serial.println("CyMouse接收端启动...");
    Serial.printf("Size of UniversalPacket: %u bytes\n", sizeof(UniversalPacket));

    USB.productName("CyMouse Receiver");      // 设置设备名
    USB.manufacturerName("Cynix Tech");  // 设置厂商名 (可选)
    
    USB.begin();
    Mouse.begin();
    initWiFi(); 
    vTaskDelay(pdMS_TO_TICKS(500)); // 确保WiFi稳定启动
    
    mouseDataQueue = xQueueCreate(1024, sizeof(QueueItem_t));
    if (mouseDataQueue == NULL) {
        Serial.println("错误：创建鼠标数据队列失败！");
        return;
    }
    

    esp_err_t cbErr = esp_now_register_recv_cb(OnDataRecv);
    if (cbErr != ESP_OK) {
        Serial.printf("错误：注册接收回调失败 (%s)\n", esp_err_to_name(cbErr));
        return;
    }

    // 添加广播地址为对等设备，以便我们可以发送广播包
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("错误：添加广播对等设备失败。");
        return;
    }

    // 将USB HID任务固定到核心1，与核心0的WiFi任务分离，降低延迟抖动
    xTaskCreatePinnedToCore(mouseTask, "MouseTask", 8192, NULL, 5, NULL, 1);
    
    Serial.println("初始化完成，开始广播身份...");
}

void loop() {
    // 职责：作为“灯塔”，在未连接时，坚持广播自己的身份信息。
    if (!isConnected) {
        static unsigned long lastBroadcastTime = 0;
        if (millis() - lastBroadcastTime >= 1000) {
            lastBroadcastTime = millis();
            
            UniversalPacket discoveryPacket = {}; // Zero-initialize
            discoveryPacket.type = PACKET_TYPE_DISCOVERY;
            strcpy(discoveryPacket.deviceName, MY_DEVICE_NAME);
            
            esp_now_send(broadcastAddress, (uint8_t *)&discoveryPacket, sizeof(discoveryPacket));
            Serial.println("正在广播身份，等待配对...");
        }
    } else {
        // 已连接时，mouseTask 负责心跳超时检测，loop 无需工作
        delay(1000);
    }
}
