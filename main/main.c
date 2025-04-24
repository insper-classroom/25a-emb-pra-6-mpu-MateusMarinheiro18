
#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>
#include <queue.h>

#include "pico/stdlib.h"
#include <stdio.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "Fusion.h"

#define SAMPLE_PERIOD    (0.01f)    
#define MPU_ADDRESS      0x68
#define I2C_SDA_GPIO     4
#define I2C_SCL_GPIO     5

#define UART_ID          uart0
#define UART_TX_PIN      0
#define UART_RX_PIN      1
#define UART_BAUDRATE    115200

#define QUEUE_LENGTH     10
#define QUEUE_ITEM_SIZE  sizeof(CsvData_t)

typedef struct {
    float roll;
    float pitch;
    float yaw;
    int   click;
} CsvData_t;

static QueueHandle_t    xQueueCsv;
static SemaphoreHandle_t xUartSem;

static void mpu6050_reset();
static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp);
static void mpu6050_task(void *p);
static void uart_task(void *p);

int main() {
    stdio_init_all();

    // inicializa UART
    uart_init(UART_ID, UART_BAUDRATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    // cria fila e semáforo binário
    xQueueCsv = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    xUartSem  = xSemaphoreCreateBinary();
    // libera o semáforo para começar
    xSemaphoreGive(xUartSem);

    // cria tasks
    xTaskCreate(mpu6050_task, "mpu6050_task", 4096, NULL, 2, NULL);
    xTaskCreate(uart_task,    "uart_task",    4096, NULL, 1, NULL);

    // inicia o scheduler
    vTaskStartScheduler();

    // não deve chegar aqui
    while (1) {
        tight_loop_contents();
    }
    return 0;
}

static void mpu6050_reset() {
    uint8_t buf[2] = { 0x6B, 0x00 };
    i2c_write_blocking(i2c_default, MPU_ADDRESS, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    uint8_t buffer[6], reg;

    reg = 0x3B;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        accel[i] = (int16_t)(buffer[2*i] << 8 | buffer[2*i+1]);

    reg = 0x43;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 6, false);
    for (int i = 0; i < 3; i++)
        gyro[i] = (int16_t)(buffer[2*i] << 8 | buffer[2*i+1]);

    reg = 0x41;
    i2c_write_blocking(i2c_default, MPU_ADDRESS, &reg, 1, true);
    i2c_read_blocking (i2c_default, MPU_ADDRESS, buffer, 2, false);
    *temp = (int16_t)(buffer[0] << 8 | buffer[1]);
}

static void mpu6050_task(void *p) {
    i2c_init(i2c_default, 400000);
    gpio_set_function(I2C_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_GPIO, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_GPIO);
    gpio_pull_up(I2C_SCL_GPIO);
    mpu6050_reset();

    FusionAhrs ahrs;
    FusionAhrsInitialise(&ahrs);

    int16_t rawAccel[3], rawGyro[3], rawTemp;
    CsvData_t data;

    while (1) {
        mpu6050_read_raw(rawAccel, rawGyro, &rawTemp);

        FusionVector gyr = {
            .axis.x = rawGyro[0] / 131.0f,
            .axis.y = rawGyro[1] / 131.0f,
            .axis.z = rawGyro[2] / 131.0f,
        };
        FusionVector acc = {
            .axis.x = rawAccel[0] / 16384.0f,
            .axis.y = rawAccel[1] / 16384.0f,
            .axis.z = rawAccel[2] / 16384.0f,
        };

        FusionAhrsUpdateNoMagnetometer(&ahrs, gyr, acc, SAMPLE_PERIOD);
        FusionEuler e = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

        data.roll  = e.angle.roll;
        data.pitch = e.angle.pitch;
        data.yaw   = e.angle.yaw;
        data.click = (acc.axis.z > 0.5f) ? 1 : 0;

        xQueueSend(xQueueCsv, &data, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

static void uart_task(void *p) {
    CsvData_t data;
    char outbuf[64];

    while (1) {
        if (xQueueReceive(xQueueCsv, &data, portMAX_DELAY) == pdTRUE) {
            int len = snprintf(outbuf, sizeof(outbuf),
                               "%.2f,%.2f,%.2f,%d\n",
                               data.roll, data.pitch, data.yaw, data.click);

            xSemaphoreTake(xUartSem, portMAX_DELAY);
            uart_write_blocking(UART_ID, (uint8_t *)outbuf, len);
            xSemaphoreGive(xUartSem);
        }
    }
}
