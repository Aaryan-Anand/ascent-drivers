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
 * @file      driver_w25qxx_basic.h
 * @brief     driver w25qxx basic header file
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

#ifndef DRIVER_W25QXX_BASIC_H
#define DRIVER_W25QXX_BASIC_H

#include "driver_w25qxx_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup w25qxx_example_driver w25qxx example driver function
 * @brief    w25qxx example driver modules
 * @ingroup  w25qxx_driver
 * @{
 */

/**
 * @brief Initialize the W25QXX flash memory with SPI configuration
 * 
 * @param miso GPIO pin number for MISO
 * @param mosi GPIO pin number for MOSI
 * @param sclk GPIO pin number for SCLK
 * @param cs GPIO pin number for CS
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_init(int miso, int mosi, int sclk, int cs);

/**
 * @brief Read data from the flash memory
 * 
 * @param addr Starting address to read from
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_read(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Write data to the flash memory
 * 
 * @param addr Starting address to write to
 * @param data Data to write
 * @param len Number of bytes to write
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_write(uint32_t addr, uint8_t *data, uint32_t len);

/**
 * @brief Erase a sector of the flash memory
 * 
 * @param addr Address within the sector to erase
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_erase_sector(uint32_t addr);

/**
 * @brief Erase a block (64KB) of the flash memory
 * 
 * @param addr Address within the block to erase
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_erase_block_64k(uint32_t addr);

/**
 * @brief Get the chip ID
 * 
 * @param id Pointer to store the chip ID
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_get_id(uint16_t *id);

/**
 * @brief Deinitialize the W25QXX flash memory
 * 
 * @return uint8_t 0 if successful, 1 if failed
 */
uint8_t w25qxx_basic_deinit(void);

/**
 * @brief  basic example power down
 * @return status code
 *         - 0 success
 *         - 1 power down failed
 * @note   none
 */
uint8_t w25qxx_basic_power_down(void);

/**
 * @brief  basic example wake up
 * @return status code
 *         - 0 success
 *         - 1 wake up failed
 * @note   none
 */
uint8_t w25qxx_basic_wake_up(void);

/**
 * @brief  basic example chip erase
 * @return status code
 *         - 0 success
 *         - 1 chip erase failed
 * @note   none
 */
uint8_t w25qxx_basic_chip_erase(void);

/**
 * @brief     basic example enable write
 * @return    void
 * @note      none
 */
void w25qxx_basic_enable_write();

/**
 * @brief     basic example disable write
 * @return    void
 * @note      none
 */
void w25qxx_basic_disable_write();

/**
 * @brief      basic example get the manufacturer && device id information
 * @param[out] *manufacturer pointer to a manufacturer buffer
 * @param[out] *device_id pointer to a device id buffer
 * @return     status code
 *             - 0 success
 *             - 1 get manufacturer device id failed
 * @note       none
 */
uint8_t w25qxx_basic_get_id(uint8_t *manufacturer, uint8_t *device_id);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
