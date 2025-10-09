/*
 * Copyright (C) 2021 Otto-von-Guericke-Universität Magdeburg
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     examples
 * @{
 *
 * @file
 * @brief       Blinky application
 *
 * @author      Marian Buschsieweke <marian.buschsieweke@ovgu.de>
 *
 * @}
 */

#include <stdio.h>
#include <stdlib.h>
#include <cmath>
#include <string>
#include <log.h>
#include <errno.h>
#include "clk.h"
#include "board.h"
#include "periph_conf.h"
#include "timex.h"
#include "ztimer.h"
#include "periph/gpio.h"  
#include "thread.h"
#include "msg.h"
#include "shell.h"
// #include "xtimer.h"
#include "mpu6050.h"
#define LED_GPIO GPIO12
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_imu_thread[THREAD_STACKSIZE];

struct MPU6050Data
{
    float ax, ay, az; // acceler_x_axis, acceler_y_axis, acceler_z_axis
    float gx, gy, gz; // gyroscope_x_axis, gyroscope_y_axis, gyroscope_z_axis
};

// MPU6050Data StaticData[1000];
int length = 0;

// MPU6050Data SumData = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
// MPU6050Data AvgData = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

float sum_ax = 0, sum_ay = 0, sum_az = 0;
float sum_gx = 0, sum_gy = 0, sum_gz = 0;
float avg_ax = 0, avg_ay = 0, avg_az = 0;
float avg_gx = 0, avg_gy = 0, avg_gz = 0;

void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
#define g_acc (9.8)
void *_imu_thread(void *arg)
{
    (void) arg;
    // Initialize MPU6050 sensor
    MPU6050 mpu;
    // get mpu6050 device id
    uint8_t device_id = mpu.getDeviceID();
    // get mpu6050 device id
    printf("[IMU_THREAD] DEVICE_ID:0x%x\n", device_id);
    mpu.initialize();
    // Configure gyroscope and accelerometer full scale ranges
    uint8_t gyro_fs = mpu.getFullScaleGyroRange();
    uint8_t accel_fs_g = mpu.getFullScaleAccelRange();
    uint16_t accel_fs_real = 1;
    float gyro_fs_convert = 1.0;

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
    float accel_fs_convert = 32768.0 / accel_fs_real;

    // Initialize variables
    int16_t ax, ay, az, gx, gy, gz;

    delay_ms(100);
    // Main loop
    while (1) {
        // Read sensor data
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        printf("[IMU_THREAD] original data (X,Y,Z):(%.02f,%.02f,%.02f), (XG,YG,ZG):(%.02f,%.02f,%.02f)\n", (float)ax, (float)ay, (float)az, (float)gx, (float)gy, (float)gz);

        sum_ax += ax;
        sum_ay += ay;
        sum_az += az;
        sum_gx += gx;
        sum_gy += gy;
        sum_gz += gz;    
        length++;

        avg_ax = sum_ax / length;
        avg_ay = sum_ay / length;
        avg_az = sum_az / length;
        avg_gx = sum_gx / length;
        avg_gy = sum_gy / length;
        avg_gz = sum_gz / length;

        printf("[IMU_THREAD] AVG original data (X,Y,Z):(%.02f,%.02f,%.02f), (XG,YG,ZG):(%.02f,%.02f,%.02f)\n", avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz);

        delay_ms(10);    
        MPU6050Data data;
        // Convert raw sensor data to real values
        data.ax = (ax - 1743.67) / accel_fs_convert;
        data.ay = (ay + 1050.07) / accel_fs_convert;
        data.az = az / accel_fs_convert;
        data.gx = (gx + 96.85) / gyro_fs_convert;
        data.gy = (gy - 15.91) / gyro_fs_convert;
        data.gz = (gz - 36.77) / gyro_fs_convert;
        // Print sensor data and balance angle
        printf("----------------------------------------\n");
        printf("[IMU_THREAD] (X,Y,Z):(%.02f,%.02f,%.02f)(m/s^2), (XG,YG,ZG):(%.02f,%.02f,%.02f)(°/s)\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);

        // Store data in array
        // SumData.ax += data.ax;
        // SumData.ay += data.ay;
        // SumData.az += data.az;
        // SumData.gx += data.gx;
        // SumData.gy += data.gy;
        // SumData.gz += data.gz;
        // length++;

        // AvgData.ax = SumData.ax / length;
        // AvgData.ay = SumData.ay / length;
        // AvgData.az = SumData.az / length;
        // AvgData.gx = SumData.gx / length;
        // AvgData.gy = SumData.gy / length;
        // AvgData.gz = SumData.gz / length;

        // Calculate and print average data every 100 samples
        // printf("[IMU_THREAD] AVG (X,Y,Z):(%.02f,%.02f,%.02f)(m/s^2), (XG,YG,ZG):(%.02f,%.02f,%.02f)(°/s)\n", AvgData.ax, AvgData.ay, AvgData.az, AvgData.gx, AvgData.gy, AvgData.gz);
    }
    return NULL;
}

static const shell_command_t shell_commands[] = {
    { NULL, NULL, NULL }
};

int main(void)
{
    thread_create(stack_for_imu_thread, sizeof(stack_for_imu_thread), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _imu_thread, NULL,
                            "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");

    char line_buf[SHELL_DEFAULT_BUFSIZE];
    shell_run(shell_commands, line_buf, SHELL_DEFAULT_BUFSIZE);
    return 0;
}
