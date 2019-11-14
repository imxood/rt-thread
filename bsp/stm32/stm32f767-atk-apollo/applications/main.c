/*
 * Copyright (c) 2006-2018, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-11-06     BalanceTWK   first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include <rtdbg.h>
#include <board.h>
#include "oled_i2c.h"

/* defined the LED0 pin: PB1 */
#define LED0_PIN    GET_PIN(B, 1)

#define THREAD_PRIORITY     25
#define THREAD_STACK_SIZE   1024
#define THREAD_TIMESLICE    5

static rt_thread_t eep_tid = RT_NULL;
static rt_thread_t led_tid = RT_NULL;
static rt_thread_t oled_tid = RT_NULL;

static rt_sem_t dynamic_sem = NULL;



#ifndef EEP_I2CBUS_NAME
#define EEP_I2CBUS_NAME          "i2c2"  /* 连接的I2C总线设备名称 */
#endif

#define EEP_ADDR             0x50  //从设备芯片地址


#define BUFFER_LENGTH   8
static uint8_t buffer_rd[BUFFER_LENGTH] = {0};
static uint8_t buffer_wr[BUFFER_LENGTH] = {0};


static struct rt_i2c_bus_device *eep_i2c_bus = RT_NULL;

rt_err_t eeprom_iic_write(uint8_t write_addr, uint8_t *data, uint32_t number)
{
    uint8_t *buf = NULL;
    uint32_t size = number + 1;
    rt_err_t ret = RT_EOK;

    buf = rt_malloc(size);
    if (buf == NULL) {
        LOG_E("malloc failed");
        return RT_ERROR;
    }

    buf[0] = write_addr;
    rt_memcpy(buf + 1, data, number);

    ret = rt_i2c_master_send(eep_i2c_bus, EEP_ADDR, RT_I2C_WR, buf, size);
    rt_thread_mdelay(10);  //必须延时
    
    rt_free(buf);

    if (ret == size)
    {
        LOG_I("EEP write ok");
        return RT_EOK;
    }
    else
    {
        LOG_E("EEP write failed ,ERR is: %d", ret);
        return -RT_ERROR;
    }
}

rt_err_t eeprom_iic_read(uint8_t read_addr, uint8_t *buf, uint32_t len)
{
    rt_i2c_master_send(eep_i2c_bus, EEP_ADDR, RT_I2C_WR, &read_addr, 1);
    rt_i2c_master_recv(eep_i2c_bus, EEP_ADDR, RT_I2C_RD, buf, len);//地址读数据
    return RT_EOK;
}

static void eeprom_i2c_thread(void *param)
{
    eep_i2c_bus = rt_i2c_bus_device_find(EEP_I2CBUS_NAME);
    if (eep_i2c_bus == NULL) {
        LOG_E("can't find device[%s]", EEP_I2CBUS_NAME);
        return;
    }
    LOG_I("EEP set i2c bus to %s", EEP_I2CBUS_NAME);

    static uint8_t tmp = 0;

    while (1) {
        for (int i = 0; i < BUFFER_LENGTH; i++) {
            buffer_wr[i] = tmp + i;
        }
        if (eeprom_iic_write(0x00, buffer_wr, BUFFER_LENGTH)) {
            LOG_E("write operation failed");
            break;
        }
        if (eeprom_iic_read(0x00, buffer_rd, BUFFER_LENGTH)) {
            LOG_E("read operation failed");
            break;
        }
        ulog_hexdump("buffer_rd", 16, buffer_rd, BUFFER_LENGTH);
        rt_thread_delay(3000);
        tmp++;
    }

    rt_sem_release(dynamic_sem);
}

static void led_thread(void *param)
{
    /* set LED0 pin mode to output */
    rt_pin_mode(LED0_PIN, PIN_MODE_OUTPUT);

    while (1) {
        rt_pin_write(LED0_PIN, PIN_HIGH);
        rt_thread_mdelay(200);
        rt_pin_write(LED0_PIN, PIN_LOW);
        rt_thread_mdelay(200);
    }
}

int main(void)
{
    rt_err_t ret = RT_EOK;

    dynamic_sem = rt_sem_create("sem", 0, RT_IPC_FLAG_FIFO);

    if (dynamic_sem == NULL) {
        LOG_E("create dynamic semaphore failed.");
        return -1;
    }

    /* eeprom */
    eep_tid = rt_thread_create("eeprom_i2c_thread", eeprom_i2c_thread, NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (eep_tid == NULL) {
        LOG_E("create thread[eeprom_i2c_thread] failed.");
        return -1;
    }
    rt_thread_startup(eep_tid);


    /* led */
    led_tid = rt_thread_create("led_thread", led_thread, NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (led_tid == NULL) {
        LOG_E("create thread[led_thread] failed.");
        return -1;
    }
    rt_thread_startup(led_tid);


    /* oled */
    oled_tid = rt_thread_create("oled_thread", oled_thread, NULL, THREAD_STACK_SIZE, THREAD_PRIORITY, THREAD_TIMESLICE);

    if (oled_tid == NULL) {
        LOG_E("create thread[oled_tid] failed.");
        return -1;
    }
    rt_thread_startup(oled_tid);



    LOG_I("waiting for sempore ..");

    ret = rt_sem_take(dynamic_sem, RT_WAITING_FOREVER);

    if (ret) {
        LOG_E("run rt_sem_take failed, ret: %d", ret);
        return RT_ERROR;
    }

    LOG_I("ok");

    return RT_EOK;
}
