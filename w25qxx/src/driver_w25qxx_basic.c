/**
 * Copyright (c) 2015 - present LibDriver All rights reserved
 * 
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 *
 * @file      driver_w25qxx_basic.c
 * @brief     driver w25qxx basic source file
 * @version   1.0.0
 * @author    Shifeng Li
 * @date      2021-07-15
 *
 * <h3>history</h3>
 * <table>
 * <tr><th>Date        <th>Version  <th>Author      <th>Description
 * <tr><td>2021/07/15  <td>1.0      <td>Shifeng Li  <td>first upload
 * </table>
 */

#include "driver_w25qxx_basic.h"
#include "manual_spi_bus.h"
#include "esp_log.h"

static const char *TAG = "W25QXX_BASIC";

// Static variables for SPI configuration
static int spi_miso;
static int spi_mosi;
static int spi_sclk;
static int spi_cs;

static w25qxx_handle_t gs_handle;        /**< w25qxx handle */

/**
 * @brief     basic example init
 * @param[in] type chip type
 * @param[in] interface chip interface
 * @param[in] dual_quad_spi_enable bool value
 * @return    status code
 *            - 0 success
 *            - 1 init failed
 * @note      none
 */
uint8_t w25qxx_basic_init(int miso, int mosi, int sclk, int cs)
{
    spi_miso = miso;
    spi_mosi = mosi;
    spi_sclk = sclk;
    spi_cs = cs;

    esp_err_t ret = spi_init(miso, mosi, sclk, cs);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI");
        return 1;
    }

    ESP_LOGI(TAG, "W25QXX initialized with MISO=%d, MOSI=%d, SCLK=%d, CS=%d",
             miso, mosi, sclk, cs);
    return 0;
}

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t w25qxx_basic_deinit(void)
{
    if (w25qxx_deinit(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  basic example power down
 * @return status code
 *         - 0 success
 *         - 1 power down failed
 * @note   none
 */
uint8_t w25qxx_basic_power_down(void)
{
    if (w25qxx_power_down(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  basic example wake up
 * @return status code
 *         - 0 success
 *         - 1 wake up failed
 * @note   none
 */
uint8_t w25qxx_basic_wake_up(void)
{
    if (w25qxx_release_power_down(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief  basic example chip erase
 * @return status code
 *         - 0 success
 *         - 1 chip erase failed
 * @note   none
 */
uint8_t w25qxx_basic_chip_erase(void)
{
    if (w25qxx_chip_erase(&gs_handle) != 0)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief      basic example get the manufacturer && device id information
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get manufacturer device id failed
 * @note       none
 */
uint8_t w25qxx_basic_get_id(uint16_t *id)
{
    if (id == NULL) {
        ESP_LOGE(TAG, "ID buffer is NULL");
        return 1;
    }

    uint8_t manufacturer;
    uint8_t device_id;

    spi_begin();
    
    spi_write_byte(0x90);  // Read Manufacturer/Device ID command
    spi_write_byte(0x00);  // Dummy byte
    spi_write_byte(0x00);  // Dummy byte
    spi_write_byte(0x00);  // Dummy byte
    
    manufacturer = spi_read_byte();
    device_id = spi_read_byte();

    spi_end();

    *id = (manufacturer << 8) | device_id;
    return 0;
}

/**
 * @brief     basic example write
 * @param[in] addr written address
 * @param[in] *data pointer to a data buffer
 * @param[in] len data length
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t w25qxx_basic_write(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (data == NULL) {
        ESP_LOGE(TAG, "Data buffer is NULL");
        return 1;
    }

    // Write Enable
    spi_begin();
    spi_write_byte(0x06);  // Write Enable command
    spi_end();

    // Page Program
    uint8_t cmd[4];
    cmd[0] = 0x02;  // Page Program command
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    spi_begin();
    
    // Send command and address
    for (int i = 0; i < 4; i++) {
        spi_write_byte(cmd[i]);
    }

    // Write data
    for (uint32_t i = 0; i < len; i++) {
        spi_write_byte(data[i]);
    }

    spi_end();

    // Wait for write to complete
    uint8_t status;
    do {
        spi_begin();
        spi_write_byte(0x05);  // Read Status Register command
        status = spi_read_byte();
        spi_end();
    } while (status & 0x01);  // Wait while busy

    return 0;
}

/**
 * @brief     basic example enable write
 * @return    void
 * @note      none
 */
void w25qxx_basic_enable_write()
{
    w25qxx_enable_write(&gs_handle);
}

/**
 * @brief     basic example disable write
 * @return    void
 * @note      none
 */
void w25qxx_basic_disable_write()
{
    w25qxx_disable_write(&gs_handle);
}

/**
 * @brief      basic example read
 * @param[in]  addr read address
 * @param[out] *data pointer to a data buffer
 * @param[in]  len data length
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t w25qxx_basic_read(uint32_t addr, uint8_t *data, uint32_t len)
{
    if (data == NULL) {
        ESP_LOGE(TAG, "Data buffer is NULL");
        return 1;
    }

    uint8_t cmd[4];
    cmd[0] = 0x03;  // Read command
    cmd[1] = (addr >> 16) & 0xFF;
    cmd[2] = (addr >> 8) & 0xFF;
    cmd[3] = addr & 0xFF;

    spi_begin();
    
    // Send command and address
    for (int i = 0; i < 4; i++) {
        spi_write_byte(cmd[i]);
    }

    // Read data
    for (uint32_t i = 0; i < len; i++) {
        data[i] = spi_read_byte();
    }

    spi_end();
    return 0;
}
