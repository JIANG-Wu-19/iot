/*
 * Copyright (C) 2019 Javier FILEIV <javier.fileiv@gmail.com>
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file        main.c
 * @brief       Example using MQTT Paho package from RIOT
 *
 * @author      Javier FILEIV <javier.fileiv@gmail.com>
 *
 * @}
 */

#include <cassert>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "timex.h"
#include "ztimer.h"
#include "shell.h"
#include "thread.h"
#include "mutex.h"
#include "paho_mqtt.h"
#include "MQTTClient.h"
#include "xtimer.h"
#include <string>
#include "ledcontroller.hh"
#include "mpu6050.h"
#include "periph/gpio.h"  
#include "periph_conf.h"
#include "board.h"
#include "clk.h"
#include "msg.h"
extern "C" {
    #include "nimble_riot.h"
    #include "nimble_autoadv.h"
    #include "log.h"
    #include "host/ble_hs.h"
    #include "host/util/util.h"
    #include "host/ble_gatt.h"
    #include "services/gap/ble_svc_gap.h"
    #include "services/gatt/ble_svc_gatt.h"
    #include "lwip/netif.h"

}
using namespace std;
void setup();
int predict(float *imu_data, int data_len, float threashold, int class_num);

#define MAIN_QUEUE_SIZE     (8)
static msg_t _main_msg_queue[MAIN_QUEUE_SIZE];

#define BUF_SIZE                        1024
#define MQTT_VERSION_v311               4       /* MQTT v3.1.1 version is 4 */
#define COMMAND_TIMEOUT_MS              4000

string DEFAULT_MQTT_CLIENT_ID = "esp32_test";
string DEFAULT_MQTT_USER = "esp32";
string DEFAULT_MQTT_PWD = "esp32";
// Please enter the IP of the computer on which you have ThingsBoard installed.
string DEFAULT_IPV4 = "192.168.31.229";
string DEFAULT_TOPIC = "v1/devices/me/telemetry";



/**
 * @brief Default MQTT port
 */
#define DEFAULT_MQTT_PORT               1883

/**
 * @brief Keepalive timeout in seconds
 */
#define DEFAULT_KEEPALIVE_SEC           10

#ifndef MAX_LEN_TOPIC
#define MAX_LEN_TOPIC                   100
#endif

#ifndef MAX_TOPICS
#define MAX_TOPICS                      4
#endif

#define IS_CLEAN_SESSION                1
#define IS_RETAINED_MSG                 0

static MQTTClient client;
static Network network;
static int topic_cnt = 0;
// static int led_state;

#define LED_MSG_TYPE_ISR     (0x3456)
#define LED_MSG_TYPE_RED     (0x3111)
#define LED_MSG_TYPE_NONE    (0x3110)
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27
#define g_acc (9.8)
#define class_num (4)
#define SAMPLES_PER_GESTURE (10)
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_motion_thread[THREAD_STACKSIZE];
// the pid of led thread
static kernel_pid_t _led_pid;
// static kernel_pid_t _main_pid;
static kernel_pid_t _motion_pid;
int mqtt_interval_ms = 5000;
struct MPU6050Data
{
    float ax, ay, az; // acceler_x_axis, acceler_y_axis, acceler_z_axis
    float gx, gy, gz; // gyroscope_x_axis, gyroscope_y_axis, gyroscope_z_axis
};
void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
void *_led_thread(void *arg)
{
    (void) arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(0); // 默认关闭
    while(1){
        // Wait for a message to control the LED
        // Display different light colors based on the motion state of the device.
        msg_t msg;
        msg_receive(&msg);
        
        if (msg.type == LED_MSG_TYPE_MOTION) {
            // 根据运动状态显示不同颜色
            int motion_state = (int)msg.content.value;
            switch (motion_state) {
                case 0: // Stationary
                    led.change_led_color(0); // 关闭
                    break;
                case 1: // Tilted
                    led.change_led_color(1); // 红色
                    break;
                case 2: // Rotating
                    led.change_led_color(4); // 绿色
                    break;
                case 3: // Moving
                    led.change_led_color(2); // 蓝色
                    break;
                default:
                    led.change_led_color(0); // 关闭
                    break;
            }
        }
        delay_ms(10);    
    }
    return NULL;
}

float gyro_fs_convert = 1.0;
float accel_fs_convert;

void get_imu_data(MPU6050 mpu, float *imu_data){
    int16_t ax, ay, az, gx, gy, gz;
    for(int i = 0; i < SAMPLES_PER_GESTURE; ++i)
    {
        /* code */
        delay_ms(collect_interval_ms);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        imu_data[i*6 + 0] = ax / accel_fs_convert;
        imu_data[i*6 + 1] = ay / accel_fs_convert;
        imu_data[i*6 + 2] = az / accel_fs_convert;
        imu_data[i*6 + 3] = gx / gyro_fs_convert;
        imu_data[i*6 + 4] = gy / gyro_fs_convert;
        imu_data[i*6 + 5] = gz / gyro_fs_convert;
    }
} 

void *_motion_thread(void *arg)
{
    (void) arg;
    // Initialize MPU6050 sensor
    MPU6050 mpu;
    // get mpu6050 device id
    uint8_t device_id = mpu.getDeviceID();
    printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
    mpu.initialize();
    // Configure gyroscope and accelerometer full scale ranges
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;

    // Convert gyroscope full scale range to conversion factor
    if (gyro_fs == MPU6050_GYRO_FS_250)
        gyro_fs_convert = 131.0;
    else if (gyro_fs == MPU6050_GYRO_FS_500)
        gyro_fs_convert = 65.5;
    else if (gyro_fs == MPU6050_GYRO_FS_1000)
        gyro_fs_convert = 32.8;
    else if (gyro_fs == MPU6050_GYRO_FS_2000)
        gyro_fs_convert = 16.4;
    else
        printf("[IMU_THREAD] Unknown GYRO_FS: 0x%x\n", gyro_fs);

    // Convert accelerometer full scale range to real value
    if (accel_fs_g == MPU6050_ACCEL_FS_2)
        accel_fs_real = g_acc * 2;
    else if (accel_fs_g == MPU6050_ACCEL_FS_4)
        accel_fs_real = g_acc * 4;
    else if (accel_fs_g == MPU6050_ACCEL_FS_8)
        accel_fs_real = g_acc * 8;
    else if (accel_fs_g == MPU6050_ACCEL_FS_16)
        accel_fs_real = g_acc * 16;
    else
        printf("[IMU_THREAD] Unknown ACCEL_FS: 0x%x\n", accel_fs_g);

    // Calculate accelerometer conversion factor
    accel_fs_convert = 32768.0 / accel_fs_real;
    float imu_data[SAMPLES_PER_GESTURE * 6] = {0};
    int data_len = SAMPLES_PER_GESTURE * 6;
    delay_ms(200);
    // Main loop
    int ret = 0;
    string motions[class_num] = {"Stationary", "Tilted", "Rotating", "Moving"};
    while (1) {
        delay_ms(predict_interval_ms);    
        // Read sensor data
        get_imu_data(mpu, imu_data);
        ret = predict(imu_data, data_len, threshold, class_num);
        // 保存运动状态供GATT服务器使用
        current_motion_state = ret;
        // 发送消息给LED线程
        msg_t msg;
        msg.type = LED_MSG_TYPE_MOTION;
        msg.content.value = ret;
        msg_send(&msg, _led_pid);
        // Print result
        printf("Predict: %d, %s\n", ret, motions[ret].c_str());
    }
    return NULL;
}

// 自定义想要的UUID
/* UUID = 1bce38b3-d137-48ff-a13e-033e14c7a335 */
static const ble_uuid128_t gatt_svr_svc_rw_demo_uuid
        = {{128}, {0x15, 0xa3, 0xc7, 0x14, 0x3e, 0x03, 0x3e, 0xa1, 0xff,
                0x48, 0x37, 0xd1, 0xb3, 0x38, 0xce, 0x1b}};
/* UUID = 35f28386-3070-4f3b-ba38-27507e991762 */
static const ble_uuid128_t gatt_svr_chr_rw_demo_write_uuid
        = {{128}, {0x62, 0x17, 0x99, 0x7e, 0x50, 0x27, 0x38, 0xba, 0x3b,
                0x4f, 0x70, 0x30, 0x86, 0x83, 0xf2, 0x35}};
/* UUID = 16151413-1211-1009-0807-060504030201 */
static const ble_uuid128_t gatt_svr_chr_rw_demo_readonly_uuid
        = {{128}, {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16}};
/* UUID = 16151413-1211-1009-0807-060504030202 */
static const ble_uuid128_t gatt_svr_chr_threshold_uuid
        = {{128}, {0x02, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16}};
/* UUID = 16151413-1211-1009-0807-060504030203 */
static const ble_uuid128_t gatt_svr_chr_interval_uuid
        = {{128}, {0x03, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09,
                0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16}};

static int gatt_svr_chr_access_rw_demo(
        uint16_t conn_handle, uint16_t attr_handle,
        struct ble_gatt_access_ctxt *ctxt, void *arg);

/* define several bluetooth services for our device */
static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    /*
     * access_cb defines a callback for read and write access events on
     * given characteristics
     */
    {
        /* Service: Read/Write Demo */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = (ble_uuid_t*) &gatt_svr_svc_rw_demo_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) { {
            /* Characteristic: Read/Write Demo write */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_write_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Read/Write Demo read only */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_readonly_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ,
        }, {
            /* Characteristic: Threshold */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_threshold_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            /* Characteristic: Interval */
            .uuid = (ble_uuid_t*) &gatt_svr_chr_interval_uuid.u,
            .access_cb = gatt_svr_chr_access_rw_demo,
            .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
        }, {
            0, /* No more characteristics in this service */
        }, }
    },
    {
        0, /* No more services */
    },
};

static int gatt_svr_chr_access_rw_demo(
        uint16_t conn_handle, uint16_t attr_handle,
        struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    (void) conn_handle;
    (void) attr_handle;
    (void) arg;
    int rc = 0;
    
    // 获取各个特性的UUID
    ble_uuid_t* write_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_write_uuid.u;
    ble_uuid_t* readonly_uuid = (ble_uuid_t*) &gatt_svr_chr_rw_demo_readonly_uuid.u;
    ble_uuid_t* threshold_uuid = (ble_uuid_t*) &gatt_svr_chr_threshold_uuid.u;
    ble_uuid_t* interval_uuid = (ble_uuid_t*) &gatt_svr_chr_interval_uuid.u;
    
    // 判断是哪个特性被访问
    if (ble_uuid_cmp(ctxt->chr->uuid, write_uuid) == 0) {
        // 处理第一个可读写特性
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                /* send given data to the client */
                rc = os_mbuf_append(ctxt->om, &rm_demo_write_data,
                                    strlen(rm_demo_write_data));
                return rc;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                /* read sent data */
                rc = ble_hs_mbuf_to_flat(ctxt->om, &rm_demo_write_data,
                                         sizeof(rm_demo_write_data), &om_len);
                /* we need to null-terminate the received string */
                rm_demo_write_data[om_len] = '\0';
                return rc;
            default:
                return 1;
        }
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, readonly_uuid) == 0) {
        // 处理只读特性 - 返回当前运动状态
        if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
            string motions[class_num] = {"Stationary", "Tilted", "Rotating", "Moving"};
            snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "Motion State: %d (%s)", 
                     current_motion_state, motions[current_motion_state].c_str());
            rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
            return rc;
        }
        return 0;
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, threshold_uuid) == 0) {
        // 处理阈值特性
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                // 读取阈值
                snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "Threshold: %.2f", threshold);
                rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
                return rc;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                // 设置阈值
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                if (om_len >= sizeof(float)) {
                    rc = ble_hs_mbuf_to_flat(ctxt->om, &threshold,
                                             sizeof(threshold), &om_len);
                }
                return rc;
            default:
                return 1;
        }
    }
    else if (ble_uuid_cmp(ctxt->chr->uuid, interval_uuid) == 0) {
        // 处理间隔特性
        switch (ctxt->op) {
            case BLE_GATT_ACCESS_OP_READ_CHR:
                // 读取间隔
                snprintf(str_answer, STR_ANSWER_BUFFER_SIZE, "Interval: %d ms", predict_interval_ms);
                rc = os_mbuf_append(ctxt->om, &str_answer, strlen(str_answer));
                return rc;
            case BLE_GATT_ACCESS_OP_WRITE_CHR:
                // 设置间隔
                uint16_t om_len;
                om_len = OS_MBUF_PKTLEN(ctxt->om);
                if (om_len >= sizeof(int)) {
                    int new_interval;
                    rc = ble_hs_mbuf_to_flat(ctxt->om, &new_interval,
                                             sizeof(new_interval), &om_len);
                    if (rc == 0) {
                        predict_interval_ms = new_interval;
                    }
                }
                return rc;
            default:
                return 1;
        }
    }
    
    return 1;
}

int mqtt_disconnect(void)
{
    topic_cnt = 0;
    int res = MQTTDisconnect(&client);
    if (res < 0) {
        printf("mqtt_example: Unable to disconnect\n");
    }
    else {
        printf("mqtt_example: Disconnect successful\n");
    }

    NetworkDisconnect(&network);
    return res;
}

int mqtt_connect(void)
{
    const char *remote_ip;
    remote_ip = DEFAULT_IPV4.c_str();
    if (client.isconnected) {
        printf("mqtt_example: client already connected, disconnecting it\n");
        MQTTDisconnect(&client);
        NetworkDisconnect(&network);
    }
    int port = DEFAULT_MQTT_PORT;

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = MQTT_VERSION_v311;

    data.clientID.cstring = (char *)DEFAULT_MQTT_CLIENT_ID.c_str();
    data.username.cstring = (char *)DEFAULT_MQTT_USER.c_str();
    data.password.cstring = (char *)DEFAULT_MQTT_PWD.c_str();
    data.keepAliveInterval = DEFAULT_KEEPALIVE_SEC;
    data.cleansession = IS_CLEAN_SESSION;
    data.willFlag = 0;

    NetworkConnect(&network, (char *)remote_ip, port);
    int ret = MQTTConnect(&client, &data);
    if (ret < 0) {
        printf("mqtt_example: Unable to connect client %d\n", ret);
        mqtt_disconnect();
        return ret;
    }
    else {
        printf("mqtt_example: Connection successfully\n");
    }

    return (ret > 0) ? 0 : 1;
}
int mqtt_pub(void)
{
    int rc = 0;
    // input your code
    return rc;
}

void send(void)
{
    mqtt_connect();
    mqtt_pub();
    mqtt_disconnect();
}

static unsigned char buf[BUF_SIZE];
static unsigned char readbuf[BUF_SIZE];

int main(void)
{
    if (IS_USED(MODULE_GNRC_ICMPV6_ECHO)) {
        msg_init_queue(_main_msg_queue, MAIN_QUEUE_SIZE);
    }
#ifdef MODULE_LWIP
    /* let LWIP initialize */
    delay_ms(10);
#endif

    NetworkInit(&network);
    MQTTClientInit(&client, &network, COMMAND_TIMEOUT_MS, buf, BUF_SIZE,
                   readbuf,
                   BUF_SIZE);
    printf("Running mqtt paho example. Type help for commands info\n");

    MQTTStartTask(&client);
    // create led thread
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    else
    {
        printf("[MAIN] LED_PID: %d\n", _led_pid);
    }
    setup();
    // create motion thread
    _motion_pid = thread_create(stack_for_motion_thread, sizeof(stack_for_motion_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _motion_thread, NULL,
                            "motion_predict_thread");
    if (_motion_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    else
    {
        printf("[MAIN] MOTION_PID: %d\n", _motion_pid);
    }


    int rc = 0;
    (void)rc;

    /* verify and add our custom services */
    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    assert(rc == 0);
    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    assert(rc == 0);

    /* set the device name */
    ble_svc_gap_device_name_set(CONFIG_NIMBLE_AUTOADV_DEVICE_NAME);
    /* reload the GATT server to link our added services */
    ble_gatts_start();

    // 获取蓝牙设备的默认 MAC 地址
    uint8_t own_addr_type;
    uint8_t own_addr[6];
    ble_hs_id_infer_auto(0, &own_addr_type);
    ble_hs_id_copy_addr(own_addr_type, own_addr, NULL);

    // 打印 MAC 地址
    LOG_INFO("Default MAC address: %02X:%02X:%02X:%02X:%02X:%02X\n",
             own_addr[5], own_addr[4], own_addr[3],
             own_addr[2], own_addr[1], own_addr[0]);
    /* start to advertise this node */
    nimble_autoadv_start(NULL);

    // waiting for get IP 
    extern struct netif *netif_default;
    uint32_t addr;
    do
    {
        addr = netif_ip_addr4(netif_default)->addr;
        printf("Waiting for getting IP, current IP addr:");
        ip_addr_debug_print(LWIP_DBG_ON, netif_ip_addr4(netif_default));
        printf("\n");
        delay_ms(1000);
    }while (addr == 0x0);    


    while (1)
    {
        send();
        delay_ms(mqtt_interval_ms);
    }
    
    return 0;
}
