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
#include "ledcontroller.hh"
#include "mpu6050.h"
#define THREAD_STACKSIZE        (THREAD_STACKSIZE_IDLE)
static char stack_for_led_thread[THREAD_STACKSIZE];
static char stack_for_imu_thread[THREAD_STACKSIZE];

static kernel_pid_t _led_pid;
#define LED_MSG_TYPE_ISR     (0x3456)
#define LED_MSG_TYPE_NONE    (0x3110)
#define LED_MSG_TYPE_RED     (0x3111)
#define LED_MSG_TYPE_GREEN   (0x3112)
#define LED_MSG_TYPE_BLUE    (0x3113)
#define LED_MSG_TYPE_YELLOW  (0x3114)
#define LED_MSG_TYPE_PURPLE  (0x3115)
#define LED_MSG_TYPE_CYAN    (0x3116)
#define LED_GPIO_R GPIO26
#define LED_GPIO_G GPIO25
#define LED_GPIO_B GPIO27

#define ACC_THR 0.275
#define GYRO_THR 33.0
#define TILT_THR 0.985 // cos(45°) = 0.707

struct MPU6050Data
{
    float ax, ay, az;
    float gx, gy, gz;
};
enum MoveState{Stationary, Tilted, Rotating, Moving, MovingX, MovingY, MovingZ};

void delay_ms(uint32_t sleep_ms)
{
    ztimer_sleep(ZTIMER_USEC, sleep_ms * US_PER_MS);
    return;
}
/**
 * LED control thread function.
 * Then, it enters an infinite loop where it waits for messages to control the LED.
 * @param arg Unused argument.
 * @return NULL.
 */
void *_led_thread(void *arg)
{
    (void) arg;
    LEDController led(LED_GPIO_R, LED_GPIO_G, LED_GPIO_B);
    led.change_led_color(0);
    while(1){
        // Input your codes
        // Wait for a message to control the LED
        // Display different light colors based on the motion state of the device.

        printf("[LED_THREAD] WAIT\n");
        msg_t msg;
        // Wait for the message from OTHER thread
        msg_receive(&msg);
        if (msg.type == LED_MSG_TYPE_NONE)
        {
            // TURN OFF LIGHT
            led.change_led_color(0);
            printf("[LED_THREAD]: LED TURN OFF!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_RED)
        {
            // TURN ON RED LIGHT
            led.change_led_color(1);
            printf("[LED_THREAD]: LED RED!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_GREEN)
        {
            // TURN ON GREEN LIGHT
            led.change_led_color(2);
            printf("[LED_THREAD]: LED GREEN!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_BLUE)
        {
            // TURN ON BLUE LIGHT
            led.change_led_color(4);
            printf("[LED_THREAD]: LED BLUE!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_YELLOW)
        {
            // TURN ON YELLOW LIGHT
            led.change_led_color(3);
            printf("[LED_THREAD]: LED YELLOW!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_PURPLE)
        {
            // TURN ON PURPLE LIGHT
            led.change_led_color(5);
            printf("[LED_THREAD]: LED PURPLE!!\n");
        }
        else if (msg.type == LED_MSG_TYPE_CYAN)
        {
            // TURN ON CYAN LIGHT
            led.change_led_color(6);
            printf("[LED_THREAD]: LED CYAN!!\n");
        }
        else
        {
            // UNKNOWN MESSAGE
            printf("[LED_THREAD]: UNKNOWN MESSAGE!!\n");
        }
    }
    return NULL;
}

#define g_acc (9.8)
MoveState detectMovement(MPU6050Data &data)
{
    // Input your code
    // Please use your imagination or search online 
    // to determine the motion state of the device 
    // based on the data obtained from the MPU6050 sensor.

    float ax = data.ax;
    float ay = data.ay;
    float az = data.az;
    float gx = data.gx;
    float gy = data.gy;
    float gz = data.gz;

    float acc_magnitude = std::sqrt(ax * ax + ay * ay + az * az);
    float gyro_magnitude = std::sqrt(gx * gx + gy * gy + gz * gz);

    if (gyro_magnitude > GYRO_THR) {
        return Rotating;
    } else if (std::abs(acc_magnitude - g_acc) > ACC_THR) {
        // if (az / g_acc > 1.1 || az / g_acc < 0.9) {
        //     return MovingZ;
        // } else if (std::abs(ax) > std::abs(ay)) {
        //     return MovingX;
        // } else {
        //     return MovingY;
        // }

        return Moving;
    } else if (std::abs(az) / acc_magnitude < TILT_THR) { // cos(45°) = 0.707
        return Tilted;
    } else {
        return Stationary;
    }
}

void *_imu_thread(void *arg)
{
    (void) arg;
    // Input your code
    // 1. initial mpu6050 sensor
    // 2. Acquire sensor data every 100ms
    // 3. Determine the motion state
    // 4. notify the LED thread to display the light color through a message.
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

        // printf("[IMU_THREAD] original data (X,Y,Z):(%.02f,%.02f,%.02f), (XG,YG,ZG):(%.02f,%.02f,%.02f)\n", (float)ax, (float)ay, (float)az, (float)gx, (float)gy, (float)gz);

        // sum_ax += ax;
        // sum_ay += ay;
        // sum_az += az;
        // sum_gx += gx;
        // sum_gy += gy;
        // sum_gz += gz;    
        // length++;

        // avg_ax = sum_ax / length;
        // avg_ay = sum_ay / length;
        // avg_az = sum_az / length;
        // avg_gx = sum_gx / length;
        // avg_gy = sum_gy / length;
        // avg_gz = sum_gz / length;

        // printf("[IMU_THREAD] AVG original data (X,Y,Z):(%.02f,%.02f,%.02f), (XG,YG,ZG):(%.02f,%.02f,%.02f)\n", avg_ax, avg_ay, avg_az, avg_gx, avg_gy, avg_gz);

        delay_ms(100);    
        MPU6050Data data;
        // Convert raw sensor data to real values
        data.ax = (ax - 1028.68) / accel_fs_convert;
        data.ay = (ay + 605.5) / accel_fs_convert;
        data.az = az / accel_fs_convert;
        data.gx = (gx + 95.99) / gyro_fs_convert;
        data.gy = (gy - 15.42) / gyro_fs_convert;
        data.gz = (gz - 36.23) / gyro_fs_convert;
        // Print sensor data and balance angle
        printf("----------------------------------------\n");
        printf("[IMU_THREAD] (X,Y,Z):(%.02f,%.02f,%.02f)(m/s^2), (XG,YG,ZG):(%.02f,%.02f,%.02f)(°/s)\n", data.ax, data.ay, data.az, data.gx, data.gy, data.gz);

        // Detect movement state
        MoveState state = detectMovement(data);
        // Send message to LED thread based on movement state
        msg_t msg;
        if (state == Stationary) {
            msg.type = LED_MSG_TYPE_NONE;
            printf("[IMU_THREAD] State: Stationary\n");
        } else if (state == Tilted) {
            msg.type = LED_MSG_TYPE_RED;
            printf("[IMU_THREAD] State: Tilted\n");
        } else if (state == Rotating) {
            msg.type = LED_MSG_TYPE_BLUE;
            printf("[IMU_THREAD] State: Rotating\n");
        } else if (state == Moving) {
            msg.type = LED_MSG_TYPE_GREEN;
            printf("[IMU_THREAD] State: Moving\n");
        } else if (state == MovingX) {
            msg.type = LED_MSG_TYPE_YELLOW;
            printf("[IMU_THREAD] State: MovingX\n");
        } else if (state == MovingY) {
            msg.type = LED_MSG_TYPE_PURPLE;
            printf("[IMU_THREAD] State: MovingY\n");
        } else if (state == MovingZ) {
            msg.type = LED_MSG_TYPE_CYAN;
            printf("[IMU_THREAD] State: MovingZ\n");
        } else {
            msg.type = LED_MSG_TYPE_NONE;
            printf("[IMU_THREAD] State: Unknown\n");
        }
        msg_send(&msg, _led_pid);

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
    _led_pid = thread_create(stack_for_led_thread, sizeof(stack_for_led_thread), THREAD_PRIORITY_MAIN - 2,
                            THREAD_CREATE_STACKTEST, _led_thread, NULL,
                            "led_controller_thread");
    if (_led_pid <= KERNEL_PID_UNDEF) {
        printf("[MAIN] Creation of receiver thread failed\n");
        return 1;
    }
    thread_create(stack_for_imu_thread, sizeof(stack_for_imu_thread), THREAD_PRIORITY_MAIN - 1,
                            THREAD_CREATE_STACKTEST, _imu_thread, NULL,
                            "imu_read_thread");
    printf("[Main] Initialization successful - starting the shell now\n");
    while(1)
    {

    }
    return 0;
}
